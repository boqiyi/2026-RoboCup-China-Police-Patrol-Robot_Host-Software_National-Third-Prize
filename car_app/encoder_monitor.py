"""HELLO-only encoder monitor for manually turning raised vehicle wheels."""

from __future__ import annotations

import sys

# The documented entrypoint does not need -B and must not populate car_app caches.
if __name__ == "__main__":
    sys.dont_write_bytecode = True

import argparse
from collections import deque
from dataclasses import dataclass, replace
from enum import IntEnum
import random
import time

import config
import mcu
from drivers.protocol_v2 import (
    DIAGNOSTICS_PAYLOAD_LENGTH, HelloRequest, MessageType, ParserIssue,
    PROTOCOL_MAJOR, PROTOCOL_MINOR, StateFlag, StateTelemetry,
    WireSafetyStopReason, decode_diagnostics, decode_hello_response,
    encode_hello_request,
)


EXPECTED_BUILD_ID = "car12-single-wheel-v12"
EXPECTED_CAPABILITIES = 0xEDB
PRINT_INTERVAL_S = 0.1
POLL_INTERVAL_S = 0.01
SPEED_WINDOW_MS = 500
LOCAL_STOP_FAULT_BIT = 1 << 20


class _V12StopReason(IntEnum):
    TURN_SETTLE_ERROR = 12


def _decode_diagnostics(payload: bytes):
    """The frozen shared codec predates the v12 turn-settle reason."""
    if len(payload) == DIAGNOSTICS_PAYLOAD_LENGTH and payload[-1] == 12:
        sample = decode_diagnostics(payload[:-1] + b"\x00")
        return replace(sample, last_stop_reason=_V12StopReason.TURN_SETTLE_ERROR)
    return decode_diagnostics(payload)


@dataclass(frozen=True)
class EncoderSnapshot:
    left_mm: int | None
    right_mm: int | None
    left_relative_mm: int | None
    right_relative_mm: int | None
    left_speed_mmps: float | None
    right_speed_mmps: float | None
    age_ms: float | None
    odom_valid: bool | None
    flags: int | None
    odom_status: int | None
    fault_bits: int | None
    status: str


class EncoderMeasurements:
    """Keep a fixed startup baseline and a bounded, source-timed speed window."""

    def __init__(self) -> None:
        self._identity = None
        self._baseline = None
        self._state = None
        self._source_time_s = None
        self._received_at_s = None
        self._uptime_ms = None
        self._samples = deque(maxlen=256)

    def clear_speed(self) -> None:
        self._samples.clear()

    def observe(self, state: StateTelemetry, source_time_s: float,
                received_at_s: float, expanded_uptime_ms: int,
                *, speed_blocked: bool = False) -> None:
        identity = (state.session_nonce, state.boot_id)
        if self._identity is not None and identity != self._identity:
            raise mcu.McuError("编码器监视会话或 MCU boot 已变化，请重新启动工具")
        self._identity = identity
        if self._uptime_ms is not None and expanded_uptime_ms <= self._uptime_ms:
            return
        if self._uptime_ms is not None and (
            expanded_uptime_ms - self._uptime_ms > config.TELEMETRY_STALE_MS
            or received_at_s - self._received_at_s > config.TELEMETRY_STALE_MS / 1000.0
        ):
            self.clear_speed()
        self._state = state
        self._source_time_s = source_time_s
        self._received_at_s = received_at_s
        self._uptime_ms = expanded_uptime_ms
        fresh = 0 <= received_at_s - source_time_s <= config.TELEMETRY_STALE_MS / 1000.0
        if fresh and self._baseline is None:
            self._baseline = (state.left_mm, state.right_mm)
        valid = bool(state.state_flags & StateFlag.ODOM_VALID) and state.odom_status == 0
        if not fresh or not valid or speed_blocked or state.fault_bits & LOCAL_STOP_FAULT_BIT:
            self.clear_speed()
            return
        self._samples.append((expanded_uptime_ms, state.left_mm, state.right_mm))
        # Retain the nearest endpoint at or before t-500ms, using actual MCU dt.
        while len(self._samples) > 2 and expanded_uptime_ms - self._samples[1][0] >= SPEED_WINDOW_MS:
            self._samples.popleft()

    def snapshot(self, now_s: float, *, speed_blocked: bool = False) -> EncoderSnapshot:
        state = self._state
        age_ms = None if state is None else (now_s - self._source_time_s) * 1000.0
        fresh = age_ms is not None and 0 <= age_ms <= config.TELEMETRY_STALE_MS
        if not fresh:
            self.clear_speed()
            return EncoderSnapshot(None, None, None, None, None, None, age_ms,
                                   None, None, None, None, "缺测（STATE 未到达或已超龄）")
        valid = bool(state.state_flags & StateFlag.ODOM_VALID) and state.odom_status == 0
        blocked = speed_blocked or bool(state.fault_bits & LOCAL_STOP_FAULT_BIT)
        left_speed = right_speed = None
        if not valid or blocked:
            self.clear_speed()
        elif len(self._samples) >= 2:
            first, last = self._samples[0], self._samples[-1]
            elapsed_ms = last[0] - first[0]
            if elapsed_ms >= SPEED_WINDOW_MS:
                left_speed = (last[1] - first[1]) * 1000.0 / elapsed_ms
                right_speed = (last[2] - first[2]) * 1000.0 / elapsed_ms
        baseline = self._baseline
        status = "正常"
        if not valid:
            status = "ODOM 无效（保留下位机原始 mm 字段）"
        elif blocked:
            status = "编码器/本地锁停故障（轮速不可用）"
        elif state.fault_bits:
            status = "下位机报告故障"
        if left_speed is None and valid and not blocked:
            status += "；轮速窗口建立中"
        return EncoderSnapshot(
            state.left_mm, state.right_mm,
            None if baseline is None else state.left_mm - baseline[0],
            None if baseline is None else state.right_mm - baseline[1],
            left_speed, right_speed, age_ms, valid, int(state.state_flags),
            state.odom_status, state.fault_bits, status,
        )


class EncoderMonitorLink(mcu.McuLink):
    """Receive ordinary protocol telemetry, with HELLO as the sole permitted TX."""

    def __init__(self, port: str | None = None) -> None:
        super().__init__(port=port)
        self.measurements = EncoderMeasurements()
        self._fatal_reason = None

    def _start_capture(self) -> None:
        pass

    def _send(self, message_type: MessageType, payload: bytes) -> int:
        if message_type is not MessageType.HELLO_REQ:
            raise mcu.McuError("编码器监视工具只允许发送 HELLO，禁止运动或恢复命令")
        return super()._send(message_type, payload)

    def hello(self, timeout_s: float = 3.0):
        if self._ser is None:
            raise mcu.McuError("串口未打开，先调用 open()")
        if (PROTOCOL_MAJOR, PROTOCOL_MINOR) != (2, 3):
            raise mcu.McuError("编码器监视需要 protocol 2.3 编解码器")
        if self._session_sent_at is not None:
            raise mcu.McuError("编码器监视不允许自动重新 HELLO，请重新启动工具")
        self._parser.clear()
        self.parse_issues.clear()
        self.last_parse_error = None
        self.connected = False
        self.negotiated_required_capabilities = 0
        nonce = random.getrandbits(64)
        self._pending_hello_nonce = nonce
        self._pending_hello_sent_at = time.monotonic()
        self._hello_response = self._hello_error = None
        try:
            request = HelloRequest(2, 3, nonce, EXPECTED_CAPABILITIES)
            self._send(MessageType.HELLO_REQ, encode_hello_request(request))
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                frames = self.pump()
                if self._hello_response is None and self._hello_error is None:
                    for frame in frames:
                        if frame.message_type is MessageType.HELLO_RESP:
                            self._complete_hello(frame.payload)
                if self._hello_error is not None:
                    raise self._hello_error
                if self._hello_response is not None:
                    return self._hello_response
                time.sleep(POLL_INTERVAL_S)
            raise mcu.McuError("v12 HELLO 握手超时，检查串口号、接线与固件")
        finally:
            self._pending_hello_nonce = None
            if self._hello_response is None or self._hello_error is not None:
                self.connected = False
                self.negotiated_required_capabilities = 0

    def _complete_hello(self, payload: bytes) -> None:
        response = decode_hello_response(payload)
        if response.session_nonce != self._pending_hello_nonce or self._hello_response is not None:
            return
        if response.capabilities != EXPECTED_CAPABILITIES or response.build_id != EXPECTED_BUILD_ID:
            raise mcu.McuError(
                f"固件身份/能力不匹配：{response.build_id!r}/0x{response.capabilities:X}，"
                f"需要 {EXPECTED_BUILD_ID}/0xEDB"
            )
        if response.boot_id == 0:
            raise mcu.McuError("固件 boot_id=0（身份初始化失败）")
        self.session_nonce, self.boot_id = response.session_nonce, response.boot_id
        self.capabilities, self.build_id = response.capabilities, response.build_id
        self.connected = True
        self._reset_link_state()
        self._session_sent_at = self._pending_hello_sent_at
        self.negotiated_required_capabilities = EXPECTED_CAPABILITIES
        self._hello_response = response

    def _latch_hard_fault(self, reason: str) -> None:
        if self._hard_fault_reason is None:
            self._hard_fault_reason = reason
        if reason == "MCU boot 变化（复位）":
            self._fatal_reason = reason

    def _retain_parse_issues(self, issues: tuple[ParserIssue, ...]) -> None:
        if issues:
            self.parse_issues.extend(issues)
            latest = issues[-1]
            self.last_parse_error = f"{latest.code}: {latest.detail}"

    def _handle(self, frame) -> None:
        if frame.message_type is not MessageType.DIAGNOSTICS:
            super()._handle(frame)
            return
        try:
            self._accept_diagnostics(_decode_diagnostics(frame.payload))
        except Exception as exc:
            self._retain_parse_issues((ParserIssue("PAYLOAD_ERROR", str(exc)),))

    def _speed_blocked(self) -> bool:
        return (
            self.diagnostics is not None
            and self.diagnostics.last_stop_reason == WireSafetyStopReason.ENCODER_FAULT
        )

    def _accept_state(self, state: StateTelemetry) -> None:
        previous = self.state
        super()._accept_state(state)
        if self.state is not previous:
            self.measurements.observe(
                state, self._sample_time("state"), self._last_state_recv,
                self._family_uptime_ms["state"], speed_blocked=self._speed_blocked(),
            )

    def _accept_diagnostics(self, sample) -> None:
        super()._accept_diagnostics(sample)
        if self._speed_blocked():
            self.measurements.clear_speed()

    def pump(self):
        frames = super().pump()
        if self.last_parse_error and self.last_parse_error.startswith("READ_ERROR:"):
            raise mcu.McuError(f"串口读取失败：{self.last_parse_error}")
        if self._fatal_reason is not None:
            raise mcu.McuError(f"{self._fatal_reason}，请重新启动编码器监视工具")
        return frames

    def snapshot(self, now_s: float | None = None) -> EncoderSnapshot:
        now_s = time.monotonic() if now_s is None else now_s
        result = self.measurements.snapshot(now_s, speed_blocked=self._speed_blocked())
        details = []
        if self.diagnostics is not None:
            reason = self.diagnostics.last_stop_reason
            age = (now_s - self._sample_time("diagnostics")) * 1000.0
            freshness = "新鲜" if 0 <= age <= config.TELEMETRY_STALE_MS else "超龄"
            details.append(f"stop={reason.name}({int(reason)})/{freshness}")
        else:
            details.append("stop=--（DIAG 缺测）")
        if self._hard_fault_reason:
            details.append(self._hard_fault_reason)
        if self.last_parse_error:
            # Malformed traffic cannot produce unlimited terminal lines or history.
            detail = self.last_parse_error.replace("\n", " ").replace("\r", " ")[:120]
            details.append(f"解析提示={detail}")
        return replace(result, status="；".join((result.status, *details)))


def format_snapshot(snapshot: EncoderSnapshot, elapsed_s: float) -> str:
    def signed(value, decimals=0):
        return "--" if value is None else f"{value:+.{decimals}f}"

    age = "--" if snapshot.age_ms is None else f"{snapshot.age_ms:.0f}"
    flags = "--" if snapshot.flags is None else f"0x{snapshot.flags:02X}"
    odom = "--" if snapshot.odom_status is None else f"0x{snapshot.odom_status:02X}"
    faults = "--" if snapshot.fault_bits is None else f"0x{snapshot.fault_bits:08X}"
    valid = "--" if snapshot.odom_valid is None else str(int(snapshot.odom_valid))
    return (
        f"t={elapsed_s:8.2f}s 累计mm L={signed(snapshot.left_mm)} R={signed(snapshot.right_mm)} "
        f"相对mm L={signed(snapshot.left_relative_mm)} R={signed(snapshot.right_relative_mm)} "
        f"估算mm/s L={signed(snapshot.left_speed_mmps, 1)} R={signed(snapshot.right_speed_mmps, 1)} "
        f"age={age}ms ODOM_VALID={valid} flags={flags} odom_status={odom} "
        f"fault_bits={faults} 状态={snapshot.status}"
    )


def run_monitor(link: EncoderMonitorLink, *, output=None) -> None:
    output = sys.stdout if output is None else output
    started = time.monotonic()
    next_print = started
    while True:
        link.pump()
        now = time.monotonic()
        if now >= next_print:
            print(format_snapshot(link.snapshot(now), now - started), file=output, flush=True)
            next_print += PRINT_INTERVAL_S
            if next_print <= now:
                next_print = now + PRINT_INTERVAL_S
        time.sleep(min(POLL_INTERVAL_S, max(0.0, next_print - time.monotonic())))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="架空车轮后手动转轮，终端读取左右编码器里程与估算轮速。")
    parser.add_argument("--port", help=f"临时覆盖 STM32 串口（默认 {config.MCU_PORT}）")
    args = parser.parse_args(argv)
    link = EncoderMonitorLink(port=args.port)
    result = 0
    try:
        print("手转编码器监视：先停止其他小车程序，架空两侧车轮，保持 MCU 和编码器供电。", flush=True)
        print("仅发送一次 HELLO 建立会话并关闭电机输出；运行和退出只接收数据、关闭串口。", flush=True)
        print("当前固件不能持续上报静止手转的原始计数；这里只显示 STATE 整数 mm，不反推计数。", flush=True)
        print("首个新鲜 STATE 固定为相对基线；估算轮速需约 500 ms 有效窗口，-- 表示不可用。Ctrl+C 退出。", flush=True)
        link.open()
        link.hello()
        print(f"已连接 {link.port}，固件 {link.build_id}，boot={link.boot_id}，每 100 ms 打印。", flush=True)
        run_monitor(link)
    except KeyboardInterrupt:
        result = 130
        try:
            print("编码器监视结束。", flush=True)
        except (BrokenPipeError, OSError, KeyboardInterrupt):
            pass
    except BrokenPipeError:
        result = 1
    except (mcu.McuError, OSError) as exc:
        result = 1
        print(f"编码器监视错误：{exc}", file=sys.stderr, flush=True)
    finally:
        # shutdown() would transmit STOP, so this tool deliberately uses close().
        try:
            link.close()
        except KeyboardInterrupt:
            result = 130
        except OSError as exc:
            result = 1
            print(f"串口关闭失败：{exc}", file=sys.stderr, flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
