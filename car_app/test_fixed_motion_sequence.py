"""Protocol 2.3/v12 bench test: STOP at 970 mm, then enter the turn speed.

Send firmware actions, check fresh telemetry and confirm encoder standstill.
Firmware already scales odometry, so its millimetres are used directly.
"""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable
from dataclasses import replace
from enum import IntEnum
import math
import queue
import sys
import threading
import time

import config
import mcu
import runlog
from motion_speed_input import TerminalSpeedInput
from drivers.protocol_v2 import (
    AckStatus, DIAGNOSTICS_PAYLOAD_LENGTH, MessageType, MotionAction,
    MotionActionCode, MotionActionFlag, ParserIssue, PROTOCOL_MAJOR,
    PROTOCOL_MINOR, StateFlag, StopReason, WireGyroStatus, WireSafetyStopReason,
    decode_ack, decode_diagnostics, decode_hello_response,
)


LINEAR_SPEED_MMPS = 150
STRAIGHT_DISTANCE_MM = 970.0
COMMAND_INTERVAL_S = 0.1
CONTROL_INTERVAL_S = 0.005
PREPARE_TIMEOUT_S = 5.0
TURN_TIMEOUT_S = 13.0
STEP_PAUSE_S = 1.0
STOP_TIMEOUT_S = 5.0
FAILURE_DIAG_TIMEOUT_S = 0.25
EXPECTED_CAPABILITIES = 0xEDB
EXPECTED_BUILD_ID = "car12-single-wheel-v12"
FIRMWARE_PACKAGE = "STM32-CAR12-MotionActions-v12-SingleWheelTurn"
FIRMWARE_HEX_SHA256 = "f43de06dae57f3d2ccbdf1535b101820830adf527117083cb9c0dbd401c48cd2"


class _V10StopReason(IntEnum):
    TURN_SETTLE_ERROR = 12

_STOP_REASON_TEXT = {
    WireSafetyStopReason.NONE: "未记录停止原因",
    WireSafetyStopReason.BOOT: "启动停车",
    WireSafetyStopReason.HOST: "上位机STOP",
    WireSafetyStopReason.FRONT_OBSTACLE: "前方障碍",
    WireSafetyStopReason.REAR_OBSTACLE: "后方障碍",
    WireSafetyStopReason.RANGE_INVALID: "测距不可用",
    WireSafetyStopReason.UART_FAULT: "串口故障",
    WireSafetyStopReason.LEASE_EXPIRED: "运动租约超时",
    WireSafetyStopReason.IMU_FAULT: "陀螺仪故障/转向角度进展超时",
    WireSafetyStopReason.FATAL: "固件致命故障",
    WireSafetyStopReason.ENCODER_FAULT: "编码器反馈异常",
    WireSafetyStopReason.CONTROL_TIMING: "控制时序异常",
    _V10StopReason.TURN_SETTLE_ERROR: "转向停稳失败",
}


def _remaining(deadline: float, message: str = "测试超时") -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise mcu.McuError(message)
    return remaining


def _decode_diagnostics(payload):
    if len(payload) == DIAGNOSTICS_PAYLOAD_LENGTH and payload[-1] == 12:
        # Validate every other field with the frozen codec; retain the original frame.
        sample = decode_diagnostics(payload[:-1] + b"\x00")
        return replace(sample, last_stop_reason=_V10StopReason.TURN_SETTLE_ERROR)
    return decode_diagnostics(payload)


class FixedMotionLink(mcu.McuLink):
    """Script-local v12 compatibility; shared modules retain their original contract."""

    def __init__(self, *args, **kwargs) -> None:
        self._samples = deque(maxlen=256)
        self.motion_checks_enabled = False
        self.motion_fault_state = None
        self.motion_fault_diagnostics = None
        self.stop_ack_times = {}
        super().__init__(*args, **kwargs)

    def _start_capture(self) -> None:
        super()._start_capture()
        self._capture_call("update_metadata", expected_build_id=EXPECTED_BUILD_ID,
                           required_capabilities=EXPECTED_CAPABILITIES,
                           firmware_package=FIRMWARE_PACKAGE,
                           firmware_hex_sha256=FIRMWARE_HEX_SHA256)

    def _complete_hello(self, payload: bytes) -> None:
        resp = decode_hello_response(payload)
        if resp.session_nonce != self._pending_hello_nonce or self._hello_response is not None:
            return
        if resp.capabilities != EXPECTED_CAPABILITIES:
            raise mcu.McuError(f"固件能力不兼容 0x{resp.capabilities:08X}，需要 0xEDB")
        if resp.boot_id == 0:
            raise mcu.McuError("固件 boot_id=0（身份初始化失败）")
        if resp.build_id != EXPECTED_BUILD_ID:
            raise mcu.McuError(f"固件标识不兼容 {resp.build_id!r}，需要 {EXPECTED_BUILD_ID}")
        self.session_nonce, self.boot_id = resp.session_nonce, resp.boot_id
        self.capabilities, self.build_id = resp.capabilities, resp.build_id
        self.connected = True
        self._reset_link_state()
        self._session_sent_at = self._pending_hello_sent_at
        self.negotiated_required_capabilities = EXPECTED_CAPABILITIES
        self._hello_response = resp
        self._capture_call("update_metadata", build_id=self.build_id,
                           boot_id=self.boot_id, session_nonce=self.session_nonce,
                           negotiated_capabilities=EXPECTED_CAPABILITIES)
        self._capture_call("event", "hello_accepted", session_nonce=self.session_nonce,
                           boot_id=self.boot_id, build_id=self.build_id)

    def _handle(self, frame) -> None:
        if frame.message_type is not MessageType.DIAGNOSTICS:
            super()._handle(frame)
            return
        try:
            self._accept_diagnostics(_decode_diagnostics(frame.payload))
        except Exception as exc:
            self._retain_parse_issues((ParserIssue("PAYLOAD_ERROR", str(exc)),))

    def _capture_frame(self, direction, frame, host_ms, **wire_context) -> None:
        if frame.message_type is not MessageType.DIAGNOSTICS:
            super()._capture_frame(direction, frame, host_ms, **wire_context)
            return
        decoded, error = None, None
        try:
            decoded = _decode_diagnostics(frame.payload)
        except Exception as exc:
            error = str(exc)
        self._capture_call("record_frame", direction, frame, host_ms, decoded=decoded,
                           session_nonce=self.session_nonce, boot_id=self.boot_id,
                           mcu_uptime_reference_ms=getattr(self.state, "uptime_ms", None),
                           decode_allowed=True, decode_error=error, **wire_context)

    def _reset_link_state(self) -> None:
        super()._reset_link_state()
        self._samples.clear()
        self.motion_checks_enabled = False
        self.motion_fault_state = self.motion_fault_diagnostics = None
        self.stop_ack_times.clear()

    def _accept_state(self, state) -> None:
        previous = self.state
        super()._accept_state(state)
        if self.state is state and previous is not state:
            self._samples.append(("state", state, time.monotonic()))
            reason = None
            if state.state_flags & StateFlag.TURN_ABORTED:
                reason = "转向被下位机中止"
            elif state.fault_bits & (1 << 20):
                reason = "下位机本地停机锁存"
            elif self.motion_checks_enabled:
                if state.fault_bits & ~config.MCU_IGNORED_RUNTIME_FAULT_BITS:
                    reason = "MCU 固件故障"
                elif state.odom_status != 0 or not state.state_flags & StateFlag.ODOM_VALID:
                    reason = "ODOM 无效（里程/航向不可用）"
            if reason is not None:
                if self.motion_fault_state is None:
                    self.motion_fault_state = state
                self._latch_hard_fault(reason)

    def _accept_diagnostics(self, sample) -> None:
        previous = self.diagnostics
        super()._accept_diagnostics(sample)
        if self.diagnostics is not sample or previous is sample or not self.motion_checks_enabled:
            return
        reason = None
        if sample.last_stop_reason not in (WireSafetyStopReason.NONE, WireSafetyStopReason.BOOT,
                                          WireSafetyStopReason.HOST):
            reason = f"下位机失败停车：{sample.last_stop_reason.name}({int(sample.last_stop_reason)})"
        elif sample.gyro_status is not WireGyroStatus.READY:
            reason = f"陀螺仪非 READY：{sample.gyro_status.name}"
        if reason is not None:
            if self.motion_fault_diagnostics is None:
                self.motion_fault_diagnostics = sample
                if self.motion_fault_state is None:
                    self.motion_fault_state = self.state
            self._latch_hard_fault(reason)

    def _accept_wheel_trace(self, sample, frame_sequence=None) -> None:
        previous = self.wheel_trace
        super()._accept_wheel_trace(sample, frame_sequence)
        if self.wheel_trace is sample and previous is not sample:
            self._samples.append(("trace", sample, time.monotonic()))

    def _accept_ack(self, ack) -> None:
        super()._accept_ack(ack)
        if self.last_ack is ack and ack.acknowledged_type is MessageType.STOP:
            self.stop_ack_times[ack.acknowledged_sequence] = time.monotonic() * 1000.0

    def drain_samples(self) -> list:
        samples = list(self._samples)
        self._samples.clear()
        return samples

    def source_sample(self, sample, received_at):
        if (not self._session_matches(sample.session_nonce, sample.boot_id)
                or self._uptime_reference_raw is None or self._clock_offset_s is None):
            return None
        delta = ((sample.uptime_ms - self._uptime_reference_raw + 0x80000000)
                 & 0xFFFFFFFF) - 0x80000000
        if delta == -0x80000000:
            return None
        uptime = self._uptime_reference_ms + delta
        generated_at = min(received_at, self._clock_offset_s + uptime / 1000.0)
        if not 0 <= time.monotonic() - generated_at <= config.TELEMETRY_STALE_MS / 1000.0:
            return None
        return uptime, generated_at

    def motion_health_reason(self):
        reason = super().motion_health_reason()
        if reason is not None:
            return reason
        if self.state.odom_status != 0:
            return "ODOM 无效（里程/航向不可用）"
        diag = self.diagnostics
        if diag is None or not self._sample_is_fresh("diagnostics"):
            return "DIAGNOSTICS 遥测超龄"
        if diag.gyro_status is not WireGyroStatus.READY:
            return f"陀螺仪非 READY：{diag.gyro_status.name}"
        gyro_age = diag.gyro_sample_age_ms + max(0.0, time.monotonic() - self._sample_time("diagnostics")) * 1000
        if gyro_age > config.TELEMETRY_STALE_MS:
            return "陀螺仪样本超龄"
        return None


class WheelSpeedEstimator:
    """Signed wheel speeds from approximately 100 ms of new MCU STATE data."""

    def __init__(self) -> None:
        self._samples = deque()
        self._raw_uptime: int | None = None
        self._uptime_ms = 0
        self._received_at: float | None = None
        self._speeds: tuple[float, float] | None = None

    def observe(self, state, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        raw = getattr(state, "uptime_ms", None)
        wheels = (getattr(state, "left_mm", None), getattr(state, "right_mm", None))
        if (isinstance(raw, bool) or not isinstance(raw, int) or not 0 <= raw <= 0xFFFFFFFF
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in wheels)):
            self._samples.clear()
            self._speeds = None
            return
        if self._raw_uptime is not None:
            delta = (raw - self._raw_uptime) & 0xFFFFFFFF
            if not 0 < delta < 0x80000000:
                return  # Duplicated or reordered data cannot refresh the speed.
            if delta > config.TELEMETRY_STALE_MS:
                self._samples.clear()
                self._speeds = None
            self._uptime_ms += delta
        self._raw_uptime = raw
        self._received_at = now
        self._samples.append((self._uptime_ms, wheels[0], wheels[1]))
        # Retain the closest older sample at least 100 ms before this sample.
        while len(self._samples) > 2 and self._uptime_ms - self._samples[1][0] >= 100:
            self._samples.popleft()
        first = self._samples[0]
        elapsed_ms = self._uptime_ms - first[0]
        if elapsed_ms >= 100:
            self._speeds = ((wheels[0] - first[1]) * 1000.0 / elapsed_ms,
                            (wheels[1] - first[2]) * 1000.0 / elapsed_ms)

    def speeds(self, now: float | None = None) -> tuple[float, float] | None:
        now = time.monotonic() if now is None else now
        if (self._received_at is None
                or not 0 <= now - self._received_at <= config.TELEMETRY_STALE_MS / 1000.0):
            return None
        return self._speeds


class AsyncTerminal:
    """A bounded, terminal-only queue keeps slow stdout off the control path."""

    def __init__(self, write: Callable[[str], None] | None = None) -> None:
        self.queue: queue.Queue[str] = queue.Queue(maxsize=4)
        self._write = write or (lambda line: print(line, flush=True))
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, name="fixed-motion-terminal", daemon=True)
        self._thread.start()

    def __call__(self, line: str) -> None:
        try:
            self.queue.put_nowait(line)
        except queue.Full:
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(line)
            except queue.Full:
                pass

    def _run(self) -> None:
        while not self._stopping.is_set() or not self.queue.empty():
            try:
                line = self.queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                self._write(line)
            except Exception:
                pass  # Output failures must not block lease renewal or STOP.
            finally:
                self.queue.task_done()

    def close(self) -> None:
        self._stopping.set()
        self._thread.join(timeout=0.3)


class FixedMotionDisplay:
    def __init__(self, link: mcu.McuLink, emitter: Callable[[str], None]) -> None:
        self.link, self.emitter = link, emitter
        self.phase = "准备"
        self.speed = WheelSpeedEstimator()
        self.straight_distance_mm = 0.0
        self.next_output = time.monotonic()

    def emit(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now < self.next_output:
            return
        self.next_output = now + COMMAND_INTERVAL_S
        state = self.link.state
        fresh = state is not None and self.link._sample_is_fresh("state", now)
        speeds = self.speed.speeds(now) if fresh else None
        wheels = (f"{speeds[0]:+.1f}", f"{speeds[1]:+.1f}") if speeds is not None else ("等待数据", "等待数据")
        yaw = f"{state.yaw_tenth_deg / 10.0:.1f}°" if fresh else "等待数据"
        odom = f"{state.left_mm}/{state.right_mm} mm" if fresh else "等待数据"
        summary = getattr(self.link, "wheel_trace_summary", None)
        trace = summary() if callable(summary) else "轮速追踪：等待数据"
        self.emitter(f"阶段={self.phase} | 编码器估算左速={wheels[0]} 右速={wheels[1]} mm/s | "
                     f"航向={yaw} | 里程L/R={odom} | "
                     f"直行={self.straight_distance_mm:.1f} mm ({self.straight_distance_mm / 10.0:.1f} cm) | {trace}")


class _Sequence:
    def __init__(self, link: mcu.McuLink, display: FixedMotionDisplay) -> None:
        self.link, self.display = link, display
        self.deadline = float("inf")
        self.issued = set()
        self.straight_origin = None
        self.last_action = None
        self.last_sequence = None
        self.failure_state = None
        self.failure_speed = None
        self.aborted_sample = None
        self.observations = []
        self.current_action = None
        self.action_started_at = None
        self.trace_moving_seen = False
        self.first_zero_trace = None
        self.stop_completed_at = None
        self.stop_sequence = None
        self.stop_records = {}

    def record(self, event, **fields) -> None:
        self.link._capture_call("event", event, phase=self.display.phase,
                                action=getattr(self.current_action, "name", None),
                                session_nonce=self.link.session_nonce, boot_id=self.link.boot_id,
                                **fields)

    def begin_action(self, action) -> None:
        self.current_action = action
        self.action_started_at = time.monotonic()
        self.trace_moving_seen = False
        self.first_zero_trace = None
        self.record("fixed_action_start", state=self.link.state,
                    threshold_mm=STRAIGHT_DISTANCE_MM if action is MotionActionCode.STRAIGHT else None)

    def send_action(self, action: MotionActionCode, speed: int = 0) -> int:
        turning = action in (MotionActionCode.TURN_LEFT, MotionActionCode.TURN_RIGHT, MotionActionCode.UTURN_LEFT)
        command = MotionAction(self.link.session_nonce, action,
                               MotionActionFlag.BRAKE_AFTER_TURN if turning else MotionActionFlag(0),
                               speed, 1800 if action is MotionActionCode.UTURN_LEFT else 900 if turning else 0)
        self.last_action, self.last_sequence = command, None
        sequence = self.link.send_motion_action(action, speed, require_ranges=False, require_upper=False)
        self.last_sequence = sequence
        self.issued.add(sequence)
        return sequence

    def receive(self, *, check_health: bool = True) -> list:
        previous_state = self.link.state
        previous_speeds = self.display.speed.speeds()
        frames = self.link.pump()
        states = []
        self.observations = []
        self.aborted_sample = None
        for kind, sample, received_at in self.link.drain_samples():
            source = self.link.source_sample(sample, received_at)
            if kind == "state" and sample is self.link.motion_fault_state and self.failure_state is None:
                self.failure_state = sample
            if source is None:
                continue
            self.observations.append((*source, kind, sample))
            if kind == "state":
                states.append(sample)
                self.display.speed.observe(sample)
                if sample is self.failure_state:
                    self.failure_speed = self.display.speed.speeds()
                if sample.state_flags & StateFlag.TURN_ABORTED and self.aborted_sample is None:
                    self.aborted_sample = (sample, self.display.speed.speeds())
            elif self.action_started_at is not None and source[1] >= self.action_started_at:
                if sample.left_encoder_delta or sample.right_encoder_delta:
                    self.trace_moving_seen = True
                elif self.trace_moving_seen and self.first_zero_trace is None:
                    self.first_zero_trace = sample
                    self.record("fixed_first_zero", trace=sample)
        if self.failure_state is None and self.link.motion_fault_state is not None:
            self.failure_state = self.link.motion_fault_state
            self.failure_speed = previous_speeds if self.failure_state is previous_state else None
        error = self.link.last_parse_error
        if error and error.startswith("READ_ERROR:"):
            if self.aborted_sample is not None:
                self.failure_state, self.failure_speed = self.aborted_sample
            raise mcu.McuError(f"串口读取失败：{error}")
        for frame in frames:
            if frame.message_type is MessageType.ACK:
                ack = decode_ack(frame.payload)
                if ack.session_nonce != self.link.session_nonce or ack.boot_id != self.link.boot_id:
                    continue
                if ack.acknowledged_type is MessageType.STOP and ack.acknowledged_sequence in self.stop_records:
                    reference = self.stop_records.pop(ack.acknowledged_sequence)
                    self.record("fixed_stop_ack", frame_seq=ack.acknowledged_sequence,
                                host_rx_monotonic_ms=self.link.stop_ack_times.get(ack.acknowledged_sequence),
                                status=ack.status, reason=ack.reason, stop_reference=reference)
                if (ack.acknowledged_type is MessageType.MOTION_ACTION
                        and ack.acknowledged_sequence in self.issued and ack.status is not AckStatus.ACCEPTED):
                    if self.aborted_sample is not None:
                        self.failure_state, self.failure_speed = self.aborted_sample
                    raise mcu.McuError(f"动作被下位机拒绝：seq={ack.acknowledged_sequence} "
                                       f"状态={ack.status.name} detail=0x{ack.reason:02X}")
        if self.link._hard_fault_reason is not None:
            raise mcu.McuError(self.link._hard_fault_reason)
        if check_health:
            self.link._check_motion_lease()
            reason = self.link.motion_health_reason()
            if reason is not None:
                raise mcu.McuError(reason)
        return states

    def update_distance(self) -> None:
        state = self.link.state
        if self.straight_origin is not None and state is not None:
            if (state.session_nonce, state.boot_id) != self.straight_session:
                raise mcu.McuError("直行期间会话或 boot 变化")
            reason = self.link.motion_health_reason()
            if reason is not None:
                raise mcu.McuError(reason)
            self.display.straight_distance_mm = (
                state.left_mm - self.straight_origin[0] + state.right_mm - self.straight_origin[1]) / 2.0

    def stop(self, *, trigger_state=None) -> None:
        state = self.link.state if trigger_state is None else trigger_state
        sent_at = time.monotonic() * 1000.0
        self.stop_sequence = self.link.send_stop(StopReason.MISSION)
        reference = dict(state=state, host_tx_monotonic_ms=sent_at,
                         host_tx_completed_ms=time.monotonic() * 1000.0,
                         threshold_mm=STRAIGHT_DISTANCE_MM if self.current_action is MotionActionCode.STRAIGHT else None)
        self.stop_records[self.stop_sequence] = reference
        self.record("fixed_stop_sent", frame_seq=self.stop_sequence, **reference)
        self.send_action(MotionActionCode.BRAKE)
        self.stop_completed_at = time.monotonic()

    def pause(self, *, update_straight: bool = False) -> None:
        self.display.phase = "直行停稳确认" if update_straight else "转向停稳确认"
        deadline = min(self.deadline, time.monotonic() + STOP_TIMEOUT_S)
        previous = None
        stable_since = None
        stable_state = None
        while True:
            _remaining(deadline, "停车超时：未确认双轮停稳1秒")
            self.receive()
            _remaining(deadline, "停车超时：未确认双轮停稳1秒")
            if update_straight:
                self.update_distance()
            for uptime, generated_at, kind, sample in sorted(
                    self.observations, key=lambda item: (item[0], item[2] != "trace")):
                if generated_at < self.stop_completed_at:
                    continue
                if kind == "trace":
                    if sample.left_encoder_delta or sample.right_encoder_delta:
                        if stable_since is None or uptime >= stable_since:
                            stable_since = uptime
                            stable_state = None
                    continue
                wheels = (sample.left_mm, sample.right_mm)
                if (previous is None or uptime - previous[0] > config.TELEMETRY_STALE_MS
                        or wheels != previous[1]):
                    stable_since = uptime
                    stable_state = None
                previous = (uptime, wheels)
                if uptime - stable_since >= STEP_PAUSE_S * 1000:
                    stable_state = sample
            if stable_state is not None:
                self.record("fixed_stopped", state=stable_state,
                            straight_distance_mm=self.display.straight_distance_mm if update_straight else None,
                            first_zero_trace=self.first_zero_trace)
                if self.stop_sequence in self.stop_records:
                    self.record("fixed_stop_ack_missing", frame_seq=self.stop_sequence)
                return
            self.display.emit()
            time.sleep(CONTROL_INTERVAL_S)

    def straight(self) -> None:
        self.display.phase = "直行97cm"
        state = self.link.state
        self.straight_origin = (state.left_mm, state.right_mm)
        self.straight_session = (state.session_nonce, state.boot_id)
        self.begin_action(MotionActionCode.STRAIGHT)
        next_send = time.monotonic()
        while True:
            _remaining(self.deadline)
            states = self.receive()
            _remaining(self.deadline)
            self.update_distance()
            trigger = next((state for state in states
                            if (state.left_mm - self.straight_origin[0] + state.right_mm - self.straight_origin[1]) / 2.0
                            >= STRAIGHT_DISTANCE_MM), None)
            if trigger is not None:
                self.stop(trigger_state=trigger)
                self.pause(update_straight=True)
                return
            now = time.monotonic()
            if now >= next_send:
                next_send = now + COMMAND_INTERVAL_S
                self.send_action(MotionActionCode.STRAIGHT, LINEAR_SPEED_MMPS)
            self.display.emit()
            time.sleep(CONTROL_INTERVAL_S)

    def wait_turn_speed(self, input_source: TerminalSpeedInput) -> int:
        _remaining(self.deadline)
        self.display.phase = "等待输入转弯速度"
        wait_started = time.monotonic()
        prompt = "车辆已停稳。请输入三个转弯共用的轮速（31–310 mm/s，只输入整数后回车；q或Ctrl+C退出）："
        input_source.begin_request(prompt)
        while True:
            self.receive()
            line = input_source.poll()
            if line is not None:
                value = line.strip()
                if value.lower() == "q":
                    raise KeyboardInterrupt("用户取消速度输入")
                if value.isascii() and value.isdecimal():
                    speed_mmps = int(value)
                    if 31 <= speed_mmps <= 310:
                        input_source.end_request()
                        self.deadline += time.monotonic() - wait_started
                        self.record("fixed_turn_speed_selected", speed_mmps=speed_mmps)
                        self.display.emitter(f"三个转弯共用轮速：{speed_mmps} mm/s")
                        return speed_mmps
                input_source.begin_request(f"输入无效，必须是31–310的整数。{prompt}")
            time.sleep(CONTROL_INTERVAL_S)

    def turn(self, action: MotionActionCode, phase: str, speed_mmps: int, *, pause: bool) -> None:
        self.display.phase = phase
        self.begin_action(action)
        deadline = min(self.deadline, time.monotonic() + TURN_TIMEOUT_S)
        next_send = time.monotonic()
        issued = set()
        active = False
        while True:
            _remaining(deadline, "转向超时")
            states = self.receive()
            _remaining(deadline, "转向超时")
            completed = False
            for state in states:
                if (not state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                        or state.applied_control_sequence not in issued):
                    continue
                if state.state_flags & StateFlag.TURN_ABORTED:
                    self.failure_state = state
                    if self.aborted_sample is not None and self.aborted_sample[0] is state:
                        self.failure_speed = self.aborted_sample[1]
                    raise mcu.McuError("转向被下位机中止")
                if state.state_flags & StateFlag.TURN_ACTIVE:
                    active, completed = True, False
                elif active and state.state_flags & StateFlag.TURN_COMPLETED:
                    completed = True
                    completed_state = state
            if completed:
                self.record("fixed_turn_completed", state=completed_state)
                self.stop(trigger_state=completed_state)
                if pause:
                    self.pause()
                return
            now = time.monotonic()
            if now >= next_send:
                next_send = now + COMMAND_INTERVAL_S
                issued.add(self.send_action(action, speed_mmps))
            self.display.emit()
            time.sleep(CONTROL_INTERVAL_S)


def _collect_failure_diagnostics(link, state, initial_diag=None):
    """Receive only after STOP; an older DIAG cannot explain the failing STATE."""
    if not link.connected or link._ser is None:
        return None
    nonce, boot = (state.session_nonce, state.boot_id) if state is not None else (link.session_nonce, link.boot_id)
    deadline = time.monotonic() + FAILURE_DIAG_TIMEOUT_S
    diag = initial_diag
    while True:
        if time.monotonic() >= deadline:
            return None
        if (diag is not None and (diag.session_nonce, diag.boot_id) == (nonce, boot)
                and link._sample_is_fresh("diagnostics")
                and (state is None or (diag.uptime_ms - state.uptime_ms) & 0xFFFFFFFF < 0x80000000)):
            return diag
        try:
            link.pump()
        except Exception:
            return None
        error = link.last_parse_error
        if error and error.startswith("READ_ERROR:"):
            return None
        diag = link.diagnostics
        time.sleep(min(CONTROL_INTERVAL_S, max(0.0, deadline - time.monotonic())))


def _print_failure_diagnostics(sequence, state, speeds, diag, emit) -> None:
    command = sequence.last_action
    command_text = (f"指令={command.action.name} seq={sequence.last_sequence} "
                    f"轮速目标={command.speed_mms} mm/s" if command is not None else "尚未发送动作")
    if state is None:
        emit(f"故障现场：STATE缺失；{command_text}")
    else:
        latch = " LOCAL_STOP_LATCHED（本地停机锁存）" if state.fault_bits & (1 << 20) else ""
        emit(f"故障现场：MCU时间={state.uptime_ms} ms state_flags=0x{int(state.state_flags):04X} "
             f"fault_bits=0x{state.fault_bits:08X}{latch} odom_status={int(state.odom_status)} "
             f"applied_seq={state.applied_control_sequence}；{command_text}")
        wheel_text = f"编码器估算左速={speeds[0]:+.1f} 右速={speeds[1]:+.1f} mm/s" if speeds is not None else "编码器估算轮速=等待数据"
        emit(f"故障数据：{wheel_text} 航向={state.yaw_tenth_deg / 10.0:.1f}° "
             f"里程L/R={state.left_mm}/{state.right_mm} mm "
             f"目标v/w={state.target_linear_mms}/{state.target_angular_mrads} (mm/s,mrad/s)")
    if diag is not None:
        reason = diag.last_stop_reason
        text = _STOP_REASON_TEXT.get(reason, "未知停止原因")
        if reason in (WireSafetyStopReason.HOST, WireSafetyStopReason.NONE, WireSafetyStopReason.BOOT):
            text += "；仅供参考，原始停止原因未确定（可能为本次STOP或历史状态）"
        emit(f"下位机诊断：停止原因={reason.name}({int(reason)}) {text}；MCU时间={diag.uptime_ms} ms；"
             f"陀螺仪={diag.gyro_status.name} 样本年龄={diag.gyro_sample_age_ms} ms")
    else:
        latest = sequence.link.diagnostics
        reference = "DIAG缺失"
        if latest is not None:
            why = "陈旧/会话不匹配"
            if state is not None and (latest.uptime_ms - state.uptime_ms) & 0xFFFFFFFF >= 0x80000000:
                why = "早于故障STATE"
            reference = f"最近DIAG={latest.last_stop_reason.name}({int(latest.last_stop_reason)})（{why}，仅供参考）"
        emit(f"下位机诊断：未取得故障同期的新鲜DIAG，原始停止原因未确定；{reference}")


def run_fixed_motion_sequence(duration_s: float = 60.0, *, link: FixedMotionLink | None = None,
                              emitter: Callable[[str], None] | None = None) -> bool:
    """Run the four firmware actions once, always stopping and closing on exit."""
    link = FixedMotionLink() if link is None else link
    terminal = AsyncTerminal() if emitter is None else None
    emit = terminal if terminal is not None else emitter
    display = FixedMotionDisplay(link, emit)
    sequence = _Sequence(link, display)
    input_source = None
    success = False
    interrupted = None
    try:
        if isinstance(duration_s, bool) or not isinstance(duration_s, (int, float)) or not math.isfinite(duration_s) or duration_s <= 0:
            raise mcu.McuError("duration 必须是有限正数")
        if (PROTOCOL_MAJOR, PROTOCOL_MINOR) != (2, 3) or config.MCU_REQUIRED_CAPABILITIES != EXPECTED_CAPABILITIES:
            raise mcu.McuError("需要协议2.3/v12和能力掩码0xEDB")
        preparation_deadline = time.monotonic() + PREPARE_TIMEOUT_S
        display.emit()
        link.open()
        link.hello(timeout_s=min(3.0, _remaining(preparation_deadline)))
        if link.build_id != EXPECTED_BUILD_ID or link.negotiated_required_capabilities != EXPECTED_CAPABILITIES:
            raise mcu.McuError("固件标识或能力不匹配")
        sequence.stop()
        while True:
            _remaining(preparation_deadline, "准备超时：等待有效STATE、READY陀螺和STOP后的新鲜诊断")
            sequence.receive(check_health=False)
            diag = link.diagnostics
            if (link.motion_health_reason() is None and diag.last_stop_reason is WireSafetyStopReason.HOST
                    and link._sample_time("diagnostics") > sequence.stop_completed_at):
                break
            display.emit()
            time.sleep(CONTROL_INTERVAL_S)
        _remaining(preparation_deadline, "准备超时")
        link.motion_checks_enabled = True
        sequence.deadline = time.monotonic() + duration_s
        sequence.straight()
        input_source = TerminalSpeedInput(write_line=emit)
        turn_speed_mmps = sequence.wait_turn_speed(input_source)
        input_source.close()
        input_source = None
        sequence.turn(MotionActionCode.TURN_LEFT, "左90°", turn_speed_mmps, pause=True)
        sequence.turn(MotionActionCode.TURN_RIGHT, "右90°", turn_speed_mmps, pause=True)
        sequence.turn(MotionActionCode.UTURN_LEFT, "左180°", turn_speed_mmps, pause=True)
        success = True
    except BaseException as exc:
        # Freeze the failing sample before STOP changes targets/applied sequence.
        state = sequence.failure_state if sequence.failure_state is not None else link.state
        speeds = sequence.failure_speed if sequence.failure_state is not None else display.speed.speeds()
        initial_diag = link.motion_fault_diagnostics or link.diagnostics
        sequence.record("fixed_failure", state=state, diagnostics=initial_diag,
                        command=sequence.last_action, frame_seq=sequence.last_sequence, error=str(exc))
        reason = (str(exc) or "Ctrl+C") if isinstance(exc, KeyboardInterrupt) else str(exc)
        try:
            link.send_stop(StopReason.USER if isinstance(exc, KeyboardInterrupt) else StopReason.FAULT)
        except Exception:
            reason += "；STOP发送失败"
        emit(f"测试未完成：{reason}；阶段={display.phase}")
        if isinstance(exc, Exception):
            try:
                diag = _collect_failure_diagnostics(link, state, initial_diag)
                _print_failure_diagnostics(sequence, state, speeds, diag, emit)
            except Exception as diag_error:
                emit(f"下位机诊断：读取失败（{diag_error}），原始停止原因未确定")
        if not isinstance(exc, Exception):
            interrupted = exc
    finally:
        try:
            link.shutdown()
        except Exception:
            success = False
            emit("测试未完成：串口关闭失败")
        if success:
            display.phase = "完成"
            display.emit(force=True)
            emit(f"测试完成；直行停车后距离={display.straight_distance_mm:.1f} mm")
        if input_source is not None:
            input_source.close()
        if terminal is not None:
            terminal.close()
    if interrupted is not None:
        raise interrupted
    return success


def _duration(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("duration 必须是秒数") from exc
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("duration 必须是有限正数")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="v12：150mm/s直行，970mm触发STOP；停车后输入31–310mm/s轮速，左90°、右90°、左180°")
    parser.add_argument("--port", default=None, help="STM32 串口，默认使用 config.MCU_PORT")
    parser.add_argument("--duration", type=_duration, default=60.0,
                        help="准备完成后的运动总时限（秒，默认60，包含每步停稳确认，不含等待速度输入）")
    args = parser.parse_args(argv)
    try:
        return 0 if run_fixed_motion_sequence(args.duration, link=FixedMotionLink(port=args.port)) else 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    with runlog.session("fixed_motion_sequence", periodic_status=False):
        sys.exit(main())
