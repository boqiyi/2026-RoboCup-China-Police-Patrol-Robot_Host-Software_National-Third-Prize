"""只连接 STM32 的上下舵机交互标定；普通舵机没有机械角度反馈。"""

from __future__ import annotations

import codecs
import os
import queue
import re
import sys
import threading
import time

import config
import gimbal
import runlog
import selfcheck
from drivers.protocol_v2 import MessageType, StopReason
from mcu import McuError


_POLL_S = 0.02
_INPUT_JOIN_S = 0.1
_HELP = (
    "命令：set 数值；+数值 / -数值；down；up；center；mark；help；q。",
    "PWM 范围 500..2500 μs。建议先 +25 / -25，再 +5 / -5 微调；正负表示脉宽变化。",
    "左右始终居中。小车放平，摄像头正对车头且与地面平行时输入 mark。",
    "机械水平以实物观察为准；mark 只记录本轮结果，不修改正式配置。",
    "center 返回本轮已标记的水平值；未标记时返回原配置。输入 q 或 Ctrl-C 退出。",
)


def _parse_command(line: str, current: int, marked: int | None) -> tuple[str, int | None]:
    command = line.strip().lower()
    if command in ("q", "mark", "help", ""):
        return command, None
    if command == "down":
        target = config.SERVO_TILT_MIN_US
    elif command == "up":
        target = config.SERVO_TILT_MAX_US
    elif command == "center":
        target = config.SERVO_TILT_CENTER_US if marked is None else marked
    elif match := re.fullmatch(r"set\s+([0-9]+)", command):
        target = int(match.group(1))
    elif re.fullmatch(r"[+-][0-9]+", command):
        target = current + int(command)
    else:
        raise ValueError("无效命令：使用 set 整数、+整数、-整数，或输入 help。")
    if not config.SERVO_PWM_MIN_US <= target <= config.SERVO_PWM_MAX_US:
        raise ValueError(
            f"上下 PWM {target} μs 越界；允许 "
            f"{config.SERVO_PWM_MIN_US}..{config.SERVO_PWM_MAX_US} μs，未发送命令。"
        )
    return command, target


def _stdin_chunks(stopping: threading.Event):
    """轮询原始 fd；绝不让守护线程持有 sys.stdin 的 BufferedReader 锁。"""
    descriptor = sys.stdin.fileno()
    if os.name == "nt":
        import ctypes
        from ctypes import wintypes
        import msvcrt

        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.GetFileType.argtypes = (wintypes.HANDLE,)
        kernel.GetFileType.restype = wintypes.DWORD
        kernel.PeekNamedPipe.argtypes = (
            wintypes.HANDLE, wintypes.LPVOID, wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD), ctypes.POINTER(wintypes.DWORD),
            ctypes.POINTER(wintypes.DWORD),
        )
        kernel.PeekNamedPipe.restype = wintypes.BOOL
        handle = msvcrt.get_osfhandle(descriptor)
        file_type = kernel.GetFileType(handle)
        while not stopping.is_set():
            if file_type == 3:  # FILE_TYPE_PIPE：select 不支持 Windows stdin 管道。
                available = wintypes.DWORD()
                if not kernel.PeekNamedPipe(handle, None, 0, None, ctypes.byref(available), None):
                    error = ctypes.get_last_error()
                    if error in (109, 232):  # ERROR_BROKEN_PIPE / ERROR_NO_DATA
                        return
                    raise ctypes.WinError(error)
                if not available.value:
                    stopping.wait(_POLL_S)
                    continue
                count = min(available.value, 4096)
            elif file_type == 1:  # FILE_TYPE_DISK：重定向文件是有限输入。
                count = 4096
            elif file_type == 2:  # 非控制台字符设备（如 NUL）没有可读取的命令。
                return
            else:
                raise OSError("不支持的 Windows 标准输入类型")
            chunk = os.read(descriptor, count)
            if not chunk:
                return
            yield chunk
    else:
        import select

        while not stopping.is_set():
            readable, _writable, _exceptional = select.select([descriptor], [], [], _POLL_S)
            if not readable:
                continue
            chunk = os.read(descriptor, 4096)
            if not chunk:
                return
            yield chunk


def _read_console_commands(commands: queue.Queue, stopping: threading.Event) -> None:
    """Windows 控制台按键轮询，同样不用 Python 缓冲 stdin。"""
    import msvcrt

    line = ""
    while not stopping.is_set():
        if not msvcrt.kbhit():
            stopping.wait(_POLL_S)
            continue
        character = msvcrt.getwche()
        if character in ("\x00", "\xe0"):  # 功能键的第二个字符也不是命令文本。
            if msvcrt.kbhit():
                msvcrt.getwch()
            continue
        if character == "\x03":
            raise KeyboardInterrupt
        if character == "\x1a":
            commands.put(("quit", None))
            return
        if character == "\b":
            line = line[:-1]
        elif character in ("\r", "\n"):
            commands.put(("line", line))
            if line.strip().lower() == "q":
                return
            line = ""
        else:
            line += character


def _windows_stdin_is_console() -> bool:
    """NUL 也可能 isatty()==True；只有真实控制台才使用 kbhit。"""
    import ctypes
    from ctypes import wintypes
    import msvcrt

    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.GetConsoleMode.argtypes = (wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD))
    kernel.GetConsoleMode.restype = wintypes.BOOL
    mode = wintypes.DWORD()
    return bool(kernel.GetConsoleMode(msvcrt.get_osfhandle(sys.stdin.fileno()), ctypes.byref(mode)))


def _read_stdin(commands: queue.Queue, stopping: threading.Event) -> None:
    if os.name == "nt" and _windows_stdin_is_console():
        _read_console_commands(commands, stopping)
        return
    decoder = codecs.getincrementaldecoder(sys.stdin.encoding or "utf-8")(errors="replace")
    pending = ""
    for chunk in _stdin_chunks(stopping):
        pending += decoder.decode(chunk)
        while "\n" in pending:
            line, pending = pending.split("\n", 1)
            if stopping.is_set():
                return
            commands.put(("line", line.rstrip("\r")))
            if line.strip().lower() == "q":
                return
    if not stopping.is_set():
        pending += decoder.decode(b"", final=True)
        if pending:
            commands.put(("line", pending))
        commands.put(("quit", None))


def _read_commands(read_line, commands: queue.Queue, stopping: threading.Event) -> None:
    """输入线程只读终端并入队；全部设备操作留在主线程。"""
    if read_line is None or read_line is input:
        try:
            _read_stdin(commands, stopping)
        except BaseException as exc:
            commands.put(("error", exc))
        return
    while not stopping.is_set():
        try:
            line = read_line()
            if not isinstance(line, str):
                raise TypeError("输入读取器必须返回字符串")
        except EOFError:
            commands.put(("quit", None))
            return
        except BaseException as exc:  # 也将输入线程的 Ctrl-C 转交主线程清理。
            commands.put(("error", exc))
            return
        if stopping.is_set():
            return
        commands.put(("line", line))
        if line.strip().lower() == "q":
            return


def _check_link(mcu, session: tuple[int, int]) -> None:
    """静止标定只需要会话和新鲜 STATE，不套用激光/IMU 导航门禁。"""
    reason = getattr(mcu, "_hard_fault_reason", None)
    if reason is not None:
        raise McuError(str(reason))
    if not getattr(mcu, "connected", False):
        raise McuError("云台标定期间 STM32 连接丢失")
    if (mcu.session_nonce, mcu.boot_id) != session:
        raise McuError("云台标定期间 MCU 会话或 boot 变化，已中止")
    state = getattr(mcu, "state", None)
    if state is None:
        raise McuError("云台标定缺少 STATE 遥测")
    if (state.session_nonce, state.boot_id) != session:
        raise McuError("云台标定收到跨会话 STATE")
    fresh = getattr(mcu, "_sample_is_fresh", None)
    if not callable(fresh) or not fresh("state"):
        raise McuError("云台标定 STATE 遥测超龄或无法确认新鲜度")


def _summary(marked: int | None) -> str:
    if marked is None:
        return "本轮未输入 mark，未测得机械水平 PWM；正式配置保持原值。"
    return (
        "请复制反馈：\n"
        f"SERVO_TILT_CENTER_US = {marked}\n"
        "down 实际方向：下 / 上；up 实际方向：上 / 下（请按实物观察填写）。\n"
        "上述值为本轮机械水平记录，正式配置尚未修改。"
    )


def run(mcu, controller, *, read_line=None, write_line=print) -> None:
    """有界确认后交互调整；退出恢复并停车，调用方负责最终关闭串口。"""
    original_center = config.SERVO_TILT_CENTER_US
    pan_center = config.SERVO_PAN_CENTER_US
    current = original_center
    marked: int | None = None
    protected = False
    stopping = threading.Event()
    reader: threading.Thread | None = None
    failure: BaseException | None = None
    try:
        mcu.open()
        selfcheck._timed_call(mcu.hello, deadline=None, maximum=3.0)
        synchronize = getattr(mcu, "synchronize_clock", None)
        if callable(synchronize):
            selfcheck._timed_call(
                synchronize, deadline=None, maximum=config.MCU_COMMAND_CONFIRM_TIMEOUT_S
            )
        protection = selfcheck._check_motion_protection(mcu, deadline=None)
        if not protection.ok:
            raise McuError(f"云台测试未启动：{protection.message}")
        protected = True
        session = (mcu.session_nonce, mcu.boot_id)
        _check_link(mcu, session)

        def apply(target: int) -> None:
            nonlocal current
            _check_link(mcu, session)
            sequence = controller.set_pwm(pan_center, target)
            _check_link(mcu, session)
            current = target  # 只有 ACK、应用回读和到位等待全部完成才可 mark。
            write_line(
                f"已确认 PWM 应用：CH1 上下={current} μs；"
                f"CH2 左右={pan_center} μs；seq={sequence}（机械角度请观察实物）。"
            )

        apply(original_center)
        for line in _HELP:
            write_line(line)
        commands: queue.Queue = queue.Queue()
        reader = threading.Thread(
            target=_read_commands,
            args=(read_line, commands, stopping),
            daemon=True,
            name="gimbal-test-input",
        )
        reader.start()
        while True:
            tick = time.monotonic()
            mcu.pump()
            _check_link(mcu, session)
            try:
                kind, value = commands.get_nowait()
            except queue.Empty:
                kind, value = "idle", None
            if kind == "error":
                raise value
            if kind == "quit":
                break
            if kind == "line":
                try:
                    command, target = _parse_command(value, current, marked)
                except ValueError as exc:
                    write_line(str(exc))
                else:
                    if command == "q":
                        break
                    if command == "help":
                        for line in _HELP:
                            write_line(line)
                    elif command == "mark":
                        marked = current
                        runlog.emit(
                            "gimbal_test.mark", "记录本轮机械水平 PWM，未修改配置",
                            pan_us=pan_center, tilt_us=marked,
                            ch1_pwm_us=marked, ch2_pwm_us=pan_center,
                        )
                        write_line(_summary(marked))
                    elif target is not None:
                        apply(target)
            time.sleep(max(0.0, _POLL_S - (time.monotonic() - tick)))
    except BaseException as exc:
        failure = exc
        raise
    finally:
        stopping.set()
        errors: list[str] = []
        cleanup_interrupt: KeyboardInterrupt | None = None
        if protected:
            restore = original_center if marked is None else marked
            try:
                controller.set_pwm(pan_center, restore)
                runlog.emit(
                    "gimbal_test.restore", "退出时恢复本轮标记值或原配置中位",
                    pan_us=pan_center, tilt_us=restore, measured=marked is not None,
                )
            except BaseException as exc:
                if isinstance(exc, KeyboardInterrupt):
                    cleanup_interrupt = exc
                errors.append(f"云台恢复未确认: {exc}")
                runlog.exception("gimbal_test.restore_failed", "退出恢复失败", exc)
        if getattr(mcu, "connected", False):
            try:
                sequence = mcu.send_stop(StopReason.CLOSE)
                gimbal.wait_for_ack(
                    mcu, MessageType.STOP, sequence,
                    time.monotonic() + config.MCU_COMMAND_CONFIRM_TIMEOUT_S,
                )
            except BaseException as exc:
                if isinstance(exc, KeyboardInterrupt):
                    cleanup_interrupt = exc
                errors.append(f"退出 STOP 未确认: {exc}")
                runlog.exception("gimbal_test.stop_failed", "退出停车确认失败", exc)
        if reader is not None and reader.ident is not None:
            try:
                reader.join(timeout=_INPUT_JOIN_S)
            except BaseException as exc:
                if isinstance(exc, KeyboardInterrupt):
                    cleanup_interrupt = exc
                errors.append(f"输入线程收尾失败: {exc}")
                runlog.exception("gimbal_test.input_join_failed", "输入线程收尾失败", exc)
        for message in errors + [_summary(marked)]:
            try:
                write_line(message)
            except Exception as exc:
                runlog.exception("gimbal_test.output_failed", "退出提示输出失败", exc)
        if cleanup_interrupt is not None and failure is None:
            raise cleanup_interrupt
        if errors:
            if failure is None:
                raise McuError("；".join(errors))
            if hasattr(failure, "add_note"):
                failure.add_note("；".join(errors))
