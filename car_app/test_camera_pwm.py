#!/usr/bin/env python3
"""RDK 摄像头双轴 PWM 实物测试：每行输入左右、上下两个整数。"""

from __future__ import annotations

import argparse
from dataclasses import replace
from enum import IntEnum
import queue
import sys
import threading
import time

sys.dont_write_bytecode = True

import config
import gimbal
import gimbal_test
import runlog
import selfcheck
from drivers.protocol_v2 import (
    DIAGNOSTICS_PAYLOAD_LENGTH, MessageType, ParserIssue, StateFlag, StopReason,
    decode_diagnostics, decode_hello_response,
)
from mcu import McuError, McuLink


EXPECTED_BUILD_ID = "car12-single-wheel-v12"
EXPECTED_CAPABILITIES = 0xEDB


class _V12StopReason(IntEnum):
    TURN_SETTLE_ERROR = 12


def _decode_diagnostics(payload):
    # The shared v9 codec predates this v12 reason. Preserve the actual reason.
    if len(payload) == DIAGNOSTICS_PAYLOAD_LENGTH and payload[-1] == 12:
        sample = decode_diagnostics(payload[:-1] + b"\x00")
        return replace(sample, last_stop_reason=_V12StopReason.TURN_SETTLE_ERROR)
    return decode_diagnostics(payload)


class CameraPwmLink(McuLink):
    """Local v12 firmware adapter; shared v9 configuration stays unchanged."""

    def _complete_hello(self, payload):
        response = decode_hello_response(payload)  # Also validates protocol 2.3.
        if response.session_nonce != self._pending_hello_nonce or self._hello_response is not None:
            return
        if response.capabilities != EXPECTED_CAPABILITIES:
            raise McuError(f"固件能力不兼容 0x{response.capabilities:X}，需要 0xEDB")
        if response.boot_id == 0:
            raise McuError("固件 boot_id=0（身份初始化失败）")
        if response.build_id != EXPECTED_BUILD_ID:
            raise McuError(f"固件标识不兼容 {response.build_id!r}，需要 {EXPECTED_BUILD_ID}")
        self.session_nonce, self.boot_id = response.session_nonce, response.boot_id
        self.capabilities, self.build_id = response.capabilities, response.build_id
        self.connected = True
        self._reset_link_state()
        self._session_sent_at = self._pending_hello_sent_at
        self.negotiated_required_capabilities = EXPECTED_CAPABILITIES
        self._hello_response = response
        self._capture_call("update_metadata", build_id=self.build_id, boot_id=self.boot_id,
                           session_nonce=self.session_nonce,
                           negotiated_capabilities=EXPECTED_CAPABILITIES)
        self._capture_call("event", "hello_accepted", build_id=self.build_id,
                           boot_id=self.boot_id, session_nonce=self.session_nonce)

    def _start_capture(self):
        super()._start_capture()
        self._capture_call("update_metadata", expected_build_id=EXPECTED_BUILD_ID,
                           required_capabilities=EXPECTED_CAPABILITIES)

    def _handle(self, frame):
        if frame.message_type is not MessageType.DIAGNOSTICS:
            return super()._handle(frame)
        try:
            self._accept_diagnostics(_decode_diagnostics(frame.payload))
        except Exception as exc:
            self._retain_parse_issues((ParserIssue("PAYLOAD_ERROR", str(exc)),))

    def _capture_frame(self, direction, frame, host_ms, **wire_context):
        if frame.message_type is not MessageType.DIAGNOSTICS:
            return super()._capture_frame(direction, frame, host_ms, **wire_context)
        decoded, error = None, None
        try:
            decoded = _decode_diagnostics(frame.payload)
        except Exception as exc:
            error = str(exc)
        self._capture_call("record_frame", direction, frame, host_ms, decoded=decoded,
                           session_nonce=self.session_nonce, boot_id=self.boot_id,
                           mcu_uptime_reference_ms=getattr(self.state, "uptime_ms", None),
                           decode_allowed=True, decode_error=error, **wire_context)


_HELP = (
    "输入：左右PWM 上下PWM，例如 1500 700；范围均为500..2500 μs。",
    "例如 1600 700 调左右；1500 800 调上下；每行两个整数后按回车。",
    "CH1/PB6=上下，CH2/PB7=左右；实际方向和角度请观察实物。",
    "help 查看帮助；q / Ctrl-C 退出并发送STOP、关闭PWM。",
)


def parse_pwm(line: str) -> tuple[int, int]:
    parts = line.split()
    if len(parts) != 2 or any(not part.isascii() or not part.isdecimal() for part in parts):
        raise ValueError("请输入两个整数：左右PWM 上下PWM，例如 1500 700；未发送。")
    pan, tilt = map(int, parts)
    if any(not config.SERVO_PWM_MIN_US <= value <= config.SERVO_PWM_MAX_US
           for value in (pan, tilt)):
        raise ValueError(
            f"PWM越界：允许{config.SERVO_PWM_MIN_US}..{config.SERVO_PWM_MAX_US} μs；未发送。"
        )
    return pan, tilt


def _check_parked(link, session, *, pwm_enabled=False):
    gimbal_test._check_link(link, session)
    state = link.state
    if state.target_linear_mms or state.target_angular_mrads or state.state_flags & StateFlag.TURN_ACTIVE:
        raise McuError("STATE报告非零底盘目标或转弯仍活动，终止云台测试")
    if pwm_enabled and not state.state_flags & StateFlag.GIMBAL_PWM_ENABLED:
        raise McuError("STATE报告舵机PWM已关闭，停止测试；请检查故障和供电")


def run(link, controller, *, read_line=None, write_line=print):
    """主线程独占设备；输入线程仅入队，不发送任何非零底盘目标。"""
    stopping = threading.Event()
    reader = None
    failure = None
    try:
        write_line("请先退出比赛及其他串口程序；本测试只控制云台，不发送非零车轮目标。")
        link.open()
        link.hello(timeout_s=3.0)
        link.synchronize_clock(timeout_s=config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
        protection = selfcheck._check_motion_protection(link, deadline=None)
        if not protection.ok:
            raise McuError(protection.message)
        session = link.session_nonce, link.boot_id
        _check_parked(link, session)
        write_line(f"STM32已连接：{link.port}；固件={link.build_id}；底盘零目标与STOP已确认。")
        write_line("启动不自动设置姿态，请输入左右、上下PWM。")
        for line in _HELP:
            write_line(line)
        commands = queue.Queue()
        reader = threading.Thread(target=gimbal_test._read_commands,
                                  args=(read_line, commands, stopping),
                                  name="camera-pwm-input", daemon=True)
        reader.start()
        applied = False
        while True:
            link.pump()
            _check_parked(link, session, pwm_enabled=applied)
            try:
                kind, value = commands.get_nowait()
            except queue.Empty:
                stopping.wait(0.02)
                continue
            if kind == "error":
                raise value
            if kind == "quit" or (kind == "line" and value.strip().lower() == "q"):
                break
            command = value.strip().lower()
            if command == "help":
                for line in _HELP:
                    write_line(line)
                continue
            if not command:
                continue
            try:
                pan, tilt = parse_pwm(command)
            except ValueError as exc:
                write_line(str(exc))
                continue
            _check_parked(link, session, pwm_enabled=applied)
            sequence = controller.set_pwm(pan, tilt)
            _check_parked(link, session, pwm_enabled=True)
            applied = True
            write_line(f"已确认：左右={pan} μs，上下={tilt} μs；seq={sequence}；"
                       "ACK接受、STATE应用及PWM开启已确认，实际转动请观察。")
    except BaseException as exc:
        failure = exc
        raise
    finally:
        stopping.set()
        try:
            if link.connected:
                sequence = link.send_stop(StopReason.CLOSE)
                gimbal.wait_for_ack(link, MessageType.STOP, sequence,
                                    time.monotonic() + config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
                write_line(f"退出STOP已接受，seq={sequence}；下位机按协议关闭PWM。")
        except BaseException as exc:
            write_line(f"退出STOP未确认：{exc}")
            if failure is None:
                raise
        finally:
            try:
                if reader is not None and reader.ident is not None:
                    reader.join(timeout=0.1)
            finally:
                link.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.MCU_PORT, help="STM32串口，默认%(default)s")
    args = parser.parse_args(argv)
    with runlog.RunLog("camera_pwm_test", periodic_status=False) as log:
        print(f"日志目录：{log.directory}")
        link = CameraPwmLink(port=args.port)
        try:
            run(link, gimbal.Gimbal(link))
            return 0
        except KeyboardInterrupt:
            print("已退出摄像头PWM测试。")
            return 130
        except Exception as exc:
            runlog.exception("camera_pwm_test.failed", "摄像头PWM测试失败", exc)
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
