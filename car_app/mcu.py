"""STM32 串口链路（protocol 2.3）。复用 ``drivers/protocol_v2`` 的字节编解码。

单线程设计：主循环反复调用 :meth:`pump` 收遥测、:meth:`send_*` 发命令。
运动租约是 500ms，主循环必须在 :data:`config.HEARTBEAT_INTERVAL_S` 内发
CONTROL_TARGET 或 MOTION_ACTION 续租。

链路安全：本模块只接受「当前会话 / 当前 boot / 单调周期序号」的新遥测；
超龄、倒序、MCU 复位、持续 fault_bits 或命令被拒都会进入故障态，由
:meth:`telemetry_ok` 上报，顶层据此停车。启动阶段的 live fault_bits 允许
在有界去抖窗口内自行恢复，避免 IMU/传感器刚初始化就把会话永久锁死。
"""

from __future__ import annotations

from collections import OrderedDict, deque
from collections.abc import Callable
import math
import random
import time

import config
import runlog
from drivers.protocol_v2 import (
    AckMessage,
    AckStatus,
    ControlFlag,
    ControlTarget,
    CulvertUpper2Telemetry,
    DiagnosticsTelemetry,
    DirectionalRange4Telemetry,
    HelloRequest,
    HelloResponse,
    LightControl,
    LightState,
    MessageType,
    MotionAction,
    MotionActionCode,
    MotionActionFlag,
    PROTOCOL_MAJOR,
    PROTOCOL_MINOR,
    ParserIssue,
    ProtocolV2Parser,
    SequenceDisposition,
    StateFlag,
    StateTelemetry,
    StopCommand,
    StopReason,
    Capability,
    WireGimbalPose,
    WireLightState,
    WireRangeStatus,
    WheelTraceTelemetry,
    ProtocolFrame,
    classify_sequence,
    decode_ack,
    decode_culvert_upper2,
    decode_diagnostics,
    decode_directional_range4,
    decode_hello_response,
    decode_light_state,
    decode_state,
    decode_wheel_trace,
    decode_hello_request,
    decode_control_target,
    decode_motion_action,
    decode_stop,
    encode_control_target,
    encode_frame,
    encode_hello_request,
    encode_light_control,
    encode_motion_action,
    encode_stop,
)


class McuError(RuntimeError):
    """STM32 链路错误。"""


_ACK_CACHE_LIMIT = 64
_PARSE_ISSUE_CACHE_LIMIT = 64
_RX_CHUNK_BYTES = 512
_RX_PUMP_MAX_BYTES = 4096
_RX_PUMP_BUDGET_S = 0.02
_LEASE_SCHEDULING_MARGIN_S = 0.03


class McuLink:
    """打开串口、HELLO 握手、收发 protocol 2.3 帧、缓存并校验最新遥测。"""

    def __init__(self, port: str | None = None, baud: int | None = None) -> None:
        self.port = port or config.MCU_PORT
        self.baud = baud or config.MCU_BAUD
        self._ser: serial.Serial | None = None
        self._parser = ProtocolV2Parser()
        self._seq = 0
        self.session_nonce = 0
        self.boot_id = 0
        # HELLO_RESP 广告的固件能力，仅用于诊断和展示；运行时要求以协商掩码为准。
        self.capabilities = 0
        self.negotiated_required_capabilities = 0
        self.build_id = ""
        self.connected = False
        # 最新遥测
        self.state: StateTelemetry | None = None
        self.directional: DirectionalRange4Telemetry | None = None
        self.culvert_upper: CulvertUpper2Telemetry | None = None
        self.light_state: LightState | None = None
        self.diagnostics: DiagnosticsTelemetry | None = None
        self.last_ack: AckMessage | None = None
        self.last_parse_error: str | None = None
        self.parse_issues: deque[ParserIssue] = deque(maxlen=_PARSE_ISSUE_CACHE_LIMIT)
        # 遥测健康状态
        self._last_state_recv = 0.0
        self._last_directional_recv = 0.0
        self._last_upper_recv = 0.0
        self._last_light_recv = 0.0
        self._last_diagnostics_recv = 0.0
        self._last_state_uptime_ms: int | None = None
        self._last_directional_seq: int | None = None
        self._last_upper_seq: int | None = None
        self._session_sent_at: float | None = None
        self._uptime_reference_raw: int | None = None
        self._uptime_reference_ms = 0
        self._clock_offset_s: float | None = None
        self._clock_sync_attempted = False
        self._family_uptime_ms: dict[str, int] = {}
        self._motion_deadline: float | None = None
        self._active_motion_sent_at: float | None = None
        self._ack_cache: OrderedDict[tuple[MessageType, int], AckMessage] = OrderedDict()
        self._action_sends: OrderedDict[int, tuple[MotionAction, float, int | None, int, int]] = OrderedDict()
        self._hard_fault_reason: str | None = None
        self._consecutive_bad = 0
        self._capture = None
        self._capture_error_reported = False
        self._rx_monotonic_ms = None
        self.wheel_trace: WheelTraceTelemetry | None = None
        self._last_trace_recv = None
        self._last_trace_seq = None
        self.trace_sample_gaps = 0
        self.trace_ignored_samples = 0
        self._last_capture_status = {}
        self._pending_hello_nonce = None
        self._pending_hello_sent_at = None
        self._hello_response = None
        self._hello_error = None

    # ------------------------------------------------------------------
    # 连接
    # ------------------------------------------------------------------
    def open(self) -> None:
        import serial

        self._start_capture()

        # 重开串口时不能让上一连接遗留的半帧与新输入拼接。
        self._parser.clear()
        self.parse_issues.clear()
        self.last_parse_error = None
        try:
            self._ser = serial.Serial(
                self.port,
                self.baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=min(config.MCU_TIMEOUT_S, 0.02),
                write_timeout=config.MCU_WRITE_TIMEOUT_S,
            )
        except serial.SerialException as exc:
            raise McuError(f"无法打开 STM32 串口 {self.port}: {exc}") from exc
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._parser.clear()

    def close(self) -> None:
        try:
            if self._ser is not None:
                self._ser.close()
        finally:
            self._ser = None
            self.connected = False
            self.negotiated_required_capabilities = 0
            self._active_motion_sent_at = None
            if self._capture is not None:
                capture, self._capture = self._capture, None
                errors = []
                for operation in (
                    lambda: capture.update_metadata(trace_sample_gaps=self.trace_sample_gaps,
                                                    trace_ignored_samples=self.trace_ignored_samples),
                    lambda: capture.close(timeout=1.0),
                ):
                    try:
                        operation()
                    except (Exception, KeyboardInterrupt) as exc:
                        errors.append(f"{type(exc).__name__}: {exc}")
                        runlog.exception("wire_capture_error", "MCU采集收尾失败，控制清理继续", exc)
                try:
                    self._last_capture_status = capture.status()
                except (Exception, KeyboardInterrupt) as exc:
                    errors.append(f"{type(exc).__name__}: {exc}")
                if errors:
                    self._last_capture_status = dict(self._last_capture_status,
                                                    complete=False, cleanup_errors=errors)
                    try:
                        capture.update_metadata(capture_close_interrupted=True)
                    except (Exception, KeyboardInterrupt):
                        pass
                runlog.emit("wire_capture_end", "MCU原始采集结束", **self._last_capture_status)

    def shutdown(self) -> None:
        """STOP first, then receive the bounded wheel-trace tail without renewal."""
        try:
            if self._ser is not None and self.connected:
                try:
                    self.send_stop(StopReason.CLOSE)
                except (Exception, KeyboardInterrupt) as exc:
                    runlog.exception("stop_failed", "退出STOP发送失败，继续有界收尾", exc)
                self.receive_shutdown_tail()
        finally:
            self.close()

    def receive_shutdown_tail(self) -> None:
        """Receive only; absence of trace never extends this deadline."""
        if self._ser is None or not self.has_negotiated_capability(Capability.WHEEL_TRACE):
            return
        deadline = time.monotonic() + config.WHEEL_TRACE_TAIL_S
        complete = False
        try:
            while time.monotonic() < deadline:
                self.pump()
                time.sleep(min(0.005, max(0.0, deadline - time.monotonic())))
            complete = True
        except (Exception, KeyboardInterrupt) as exc:
            runlog.exception("wire_tail_error", "停车尾段接收中断，继续释放资源", exc)
        finally:
            self._capture_call("update_metadata", shutdown_tail_complete=complete)
            self._capture_call("event", "shutdown_tail_end", complete=complete)

    def _start_capture(self) -> None:
        if self._capture is not None:
            return
        try:
            from wire_capture import WireCapture
            self._capture = WireCapture(directory=runlog.current_directory(), metadata={
                "port": self.port, "baud": self.baud,
                "expected_build_id": config.MCU_EXPECTED_BUILD_ID,
                "required_capabilities": config.MCU_REQUIRED_CAPABILITIES,
            })
            self._capture_error_reported = False
            runlog.emit("wire_capture_start", "MCU原始数据采集已启用", **self._capture.status())
        except Exception as exc:
            runlog.exception("wire_capture_error", "MCU采集初始化失败，控制继续", exc)

    def capture_status(self) -> dict:
        return self._capture.status() if self._capture is not None else self._last_capture_status

    def _capture_call(self, method: str, *args, **kwargs) -> None:
        if self._capture is None:
            return
        try:
            getattr(self._capture, method)(*args, **kwargs)
            status = self._capture.status()
            if (status.get("errors_count") or status.get("dropped_records")) and not self._capture_error_reported:
                self._capture_error_reported = True
                runlog.emit("wire_capture_incomplete", "MCU采集不完整，控制继续", **status)
        except Exception as exc:
            if not self._capture_error_reported:
                self._capture_error_reported = True
                runlog.exception("wire_capture_error", "MCU采集失败，控制继续", exc)

    def _capture_frame(self, direction: str, frame, host_ms: float, **wire_context) -> None:
        decoders = {
            MessageType.HELLO_REQ: decode_hello_request,
            MessageType.HELLO_RESP: decode_hello_response,
            MessageType.CONTROL_TARGET: decode_control_target,
            MessageType.MOTION_ACTION: decode_motion_action,
            MessageType.STOP: decode_stop,
            MessageType.ACK: decode_ack,
            MessageType.STATE: decode_state,
            MessageType.DIAGNOSTICS: decode_diagnostics,
            MessageType.DIRECTIONAL_RANGE4: decode_directional_range4,
            MessageType.CULVERT_UPPER2: decode_culvert_upper2,
            MessageType.LIGHT_STATE: decode_light_state,
        }
        if self.has_negotiated_capability(Capability.WHEEL_TRACE):
            decoders[MessageType.WHEEL_TRACE] = decode_wheel_trace
        decoder = decoders.get(frame.message_type)
        decoded, error = None, None
        if decoder is not None:
            try:
                decoded = decoder(frame.payload)
            except Exception as exc:
                error = str(exc)
        self._capture_call("record_frame", direction, frame, host_ms, decoded=decoded,
                           session_nonce=self.session_nonce, boot_id=self.boot_id,
                           mcu_uptime_reference_ms=getattr(self.state, "uptime_ms", None),
                           decode_allowed=(frame.message_type is not MessageType.WHEEL_TRACE
                                           or self.has_negotiated_capability(Capability.WHEEL_TRACE)),
                           decode_error=error, **wire_context)

    def _reset_link_state(self) -> None:
        """新会话开始时清空遥测健康状态。"""
        self.state = None
        self.directional = None
        self.culvert_upper = None
        self.light_state = None
        self.diagnostics = None
        self.last_ack = None
        self._last_state_uptime_ms = None
        self._last_directional_seq = None
        self._last_upper_seq = None
        self._last_state_recv = 0.0
        self._last_directional_recv = 0.0
        self._last_upper_recv = 0.0
        self._last_light_recv = 0.0
        self._last_diagnostics_recv = 0.0
        self._session_sent_at = None
        self._uptime_reference_raw = None
        self._uptime_reference_ms = 0
        self._clock_offset_s = None
        self._clock_sync_attempted = False
        self._family_uptime_ms.clear()
        self._ack_cache.clear()
        self._action_sends.clear()
        self.wheel_trace = None
        self._last_trace_recv = None
        self._last_trace_seq = None
        self.trace_sample_gaps = 0
        self.trace_ignored_samples = 0
        self._hard_fault_reason = None
        self._consecutive_bad = 0
        self._active_motion_sent_at = None

    def _latch_hard_fault(self, reason: str) -> None:
        """锁存不可由普通清理解除的链路/控制故障，保留首个根因。"""
        if self._hard_fault_reason is None:
            self._hard_fault_reason = reason
            runlog.emit("mcu_hard_fault", "STM32链路故障已锁存", reason=reason)

    def pop_parse_issues(self) -> tuple[ParserIssue, ...]:
        """返回并清空近期帧解析问题，供自检和日志一次性消费。"""
        issues = tuple(self.parse_issues)
        self.parse_issues.clear()
        return issues

    def _retain_parse_issues(self, issues: tuple[ParserIssue, ...]) -> None:
        if not issues:
            return
        self.parse_issues.extend(issues)
        latest = issues[-1]
        self.last_parse_error = f"{latest.code}: {latest.detail}"
        for issue in issues:
            runlog.emit("mcu_parse_error", "STM32数据解析异常", code=issue.code, detail=issue.detail)

    def has_negotiated_capability(self, capability: Capability) -> bool:
        """当前 HELLO 会话是否实际协商了给定能力。"""
        return bool(self.negotiated_required_capabilities & int(capability))

    def hello(self, timeout_s: float = 3.0) -> HelloResponse:
        """Establish the v9 session before handling subsequent frames in its batch."""
        if self._ser is None:
            raise McuError("串口未打开，先调用 open()")
        self._parser.clear()
        self.parse_issues.clear()
        self.last_parse_error = None
        nonce = random.getrandbits(64)
        self.connected = False
        self.negotiated_required_capabilities = 0
        required = int(config.MCU_REQUIRED_CAPABILITIES)
        if required != 0xEDB:
            raise McuError("v9轮速追踪会话必须使用能力0xEDB")
        req = HelloRequest(PROTOCOL_MAJOR, PROTOCOL_MINOR, nonce, required)
        self._pending_hello_nonce = nonce
        self._pending_hello_sent_at = time.monotonic()
        self._hello_response = None
        self._hello_error = None
        try:
            self._send(MessageType.HELLO_REQ, encode_hello_request(req))
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                frames = self.pump()
                # Also support device adapters that return frames without invoking
                # _handle themselves. Real pump handles HELLO in wire order.
                if self._hello_response is None and self._hello_error is None:
                    for frame in frames:
                        if frame.message_type is MessageType.HELLO_RESP:
                            self._complete_hello(frame.payload)
                            if self._hello_response is not None:
                                break
                if self._hello_error is not None:
                    raise self._hello_error
                if self._hello_response is not None:
                    return self._hello_response
            raise McuError("HELLO 握手超时，未收到 HELLO_RESP（检查串口号/接线/固件）")
        finally:
            self._pending_hello_nonce = None
            if self._hello_response is None or self._hello_error is not None:
                self.connected = False
                self.negotiated_required_capabilities = 0

    def _complete_hello(self, payload: bytes) -> None:
        resp = decode_hello_response(payload)
        if resp.session_nonce != self._pending_hello_nonce:
            return
        if self._hello_response is not None:
            return
        required = int(config.MCU_REQUIRED_CAPABILITIES)
        if resp.capabilities != required:
            raise McuError(f"固件能力不兼容 0x{resp.capabilities:08X}，需要 0x{required:08X}")
        if resp.boot_id == 0:
            raise McuError("固件 boot_id=0（身份初始化失败）")
        if resp.build_id != config.MCU_EXPECTED_BUILD_ID:
            raise McuError(f"固件标识不兼容 {resp.build_id!r}，需要 {config.MCU_EXPECTED_BUILD_ID}")
        self.session_nonce = resp.session_nonce
        self.boot_id = resp.boot_id
        self.capabilities = resp.capabilities
        self.build_id = resp.build_id
        self.connected = True
        self._reset_link_state()
        self._session_sent_at = self._pending_hello_sent_at
        self.negotiated_required_capabilities = required
        self._hello_response = resp
        self._capture_call("update_metadata", build_id=self.build_id,
                           boot_id=self.boot_id, session_nonce=self.session_nonce,
                           negotiated_capabilities=required)
        self._capture_call("event", "hello_accepted", session_nonce=self.session_nonce,
                           boot_id=self.boot_id, build_id=self.build_id)

    # ------------------------------------------------------------------
    # 命令
    # ------------------------------------------------------------------
    def _send(self, message_type: MessageType, payload: bytes) -> int:
        import serial

        if self._ser is None:
            raise McuError("串口未打开")
        self._seq = (self._seq + 1) & 0xFFFF
        data = encode_frame(message_type, self._seq, payload)
        host_ms = time.monotonic() * 1000.0 if self._capture is not None else None
        try:
            written = self._ser.write(data)
            if host_ms is not None:
                self._capture_call("record_bytes", "tx", data[:max(0, written or 0)], host_ms,
                                   requested_bytes=len(data), written_bytes=written,
                                   host_tx_completed_ms=time.monotonic() * 1000.0)
                self._capture_frame("tx", ProtocolFrame(message_type, self._seq, payload), host_ms,
                                    transmission_complete=written == len(data))
            if written != len(data):
                reason = f"串口写入不完整 {written}/{len(data)}"
                self._capture_call("event", "tx_partial_write", message_type=int(message_type),
                                   frame_seq=self._seq, requested_bytes=len(data), written_bytes=written)
                self._latch_hard_fault(reason)
                raise McuError(reason)
        except (serial.SerialTimeoutException, serial.SerialException, OSError) as exc:
            if host_ms is not None:
                self._capture_frame("tx", ProtocolFrame(message_type, self._seq, payload), host_ms,
                                    transmission_complete=False, write_error=str(exc))
            self._capture_call("event", "tx_failed", message_type=int(message_type),
                               frame_seq=self._seq, error=str(exc))
            self._latch_hard_fault(f"串口写入失败: {exc}")
            raise McuError(f"串口写入失败: {exc}") from exc
        return self._seq

    def send_motion(
        self,
        linear_mmps: float,
        angular_mradps: float,
        *,
        require_ranges: bool = True,
        require_upper: bool = True,
    ) -> int:
        """发运动命令（线速度 mm/s、角速度 mrad/s，正=左转）。

        直行始终保持侧向测距门禁；普通巡逻和不依赖涵洞的测试可关闭上层测距
        门禁，涵洞测试保留该门禁。纯原地旋转可显式关闭全部测距门禁，但仍要求新鲜 STATE、
        有效里程与航向、通信健康和未忽略的固件故障位。
        """
        if getattr(self, "_turn_waiting", False):
            raise McuError("固定转向执行中不能发送 CONTROL_TARGET；请先 STOP")
        linear = int(round(linear_mmps))
        angular = int(round(angular_mradps))
        if not require_ranges and linear:
            raise McuError("仅原地转向可跳过测距门禁")
        if linear or angular:
            self._check_motion_deadline()
            if not self.connected or self._hard_fault_reason is not None:
                raise McuError(self._hard_fault_reason or "未建立STM32会话，拒绝运动")
            if self.odom_mm is None or self.state is None or self._active_fault_bits():
                raise McuError("缺少新鲜有效里程或固件故障，拒绝运动")
            if not require_ranges and linear == 0 and angular and self.yaw_deg is None:
                raise McuError("缺少新鲜有效航向，拒绝原地转向")
            families = [("diagnostics", Capability.DIAGNOSTICS)]
            if require_ranges:
                families.insert(0, ("directional", Capability.DIRECTIONAL_RANGE4))
                if require_upper:
                    families.insert(1, ("upper", Capability.CULVERT_UPPER2))
            for family, capability in families:
                if self.has_negotiated_capability(capability) and not self._sample_is_fresh(family):
                    raise McuError(f"{family} 遥测超龄，拒绝运动")
            stale_laser = (
                self._laser_stale_reason(require_upper=require_upper)
                if require_ranges else None
            )
            if stale_laser is not None:
                raise McuError(stale_laser)
        cmd = ControlTarget(
            session_nonce=self.session_nonce,
            flags=ControlFlag.MOTION_VALID,
            linear_mms=linear,
            angular_mrads=angular,
            aim_id=0,
            pose_id=WireGimbalPose.UNKNOWN,
            pan_pwm_us=0,
            tilt_pwm_us=0,
        )
        payload = encode_control_target(cmd)
        if linear or angular:
            self._check_motion_deadline()
        sent_at = time.monotonic() if linear or angular else None
        sequence = self._send(MessageType.CONTROL_TARGET, payload)
        self._active_motion_sent_at = sent_at
        return sequence

    def send_rotation(self, angular_mradps: float) -> int:
        """发送仅依赖 STATE/yaw 的原地旋转命令，不要求测距样本。"""
        return self.send_motion(0, angular_mradps, require_ranges=False)

    def send_motion_action(
        self,
        action: MotionActionCode,
        speed_mms: int = 0,
        *,
        brake_after_turn: bool = True,
        require_ranges: bool = True,
        require_upper: bool = True,
    ) -> int:
        """发送固件固定动作；同参数的新序号帧可安全续约。"""
        if not isinstance(action, MotionActionCode):
            raise McuError("未知固定动作")
        if not self.connected or self._ser is None:
            raise McuError("未建立 STM32 会话")
        if not self.has_negotiated_capability(Capability.MOTION_ACTIONS):
            raise McuError("MOTION_ACTIONS 未在当前 HELLO 会话协商")
        if action is not MotionActionCode.BRAKE:
            self._check_motion_deadline()
            self._check_motion_lease()
            reason = self.motion_health_reason()
            if reason is not None:
                raise McuError(f"{reason}，拒绝动作")
            if require_ranges:
                if not self._sample_is_fresh("directional"):
                    raise McuError("directional 遥测超龄，拒绝动作")
                if require_upper and not self._sample_is_fresh("upper"):
                    raise McuError("upper 遥测超龄，拒绝动作")
                reason = self._laser_stale_reason(require_upper=require_upper)
                if reason is not None:
                    raise McuError(reason)
        is_turn = action in (MotionActionCode.TURN_LEFT, MotionActionCode.TURN_RIGHT,
                             MotionActionCode.UTURN_LEFT)
        command = MotionAction(
            session_nonce=self.session_nonce,
            action=action,
            flags=(MotionActionFlag.BRAKE_AFTER_TURN if is_turn and brake_after_turn
                   else MotionActionFlag(0)),
            speed_mms=speed_mms,
            turn_angle_tenth_deg=(1800 if action is MotionActionCode.UTURN_LEFT else
                                  900 if is_turn else 0),
        )
        payload = encode_motion_action(command)
        sent_at = time.monotonic()
        sequence = self._send(MessageType.MOTION_ACTION, payload)
        self._ack_cache.pop((MessageType.MOTION_ACTION, sequence), None)
        self._action_sends[sequence] = (
            command, sent_at, self._family_uptime_ms.get("state"),
            self.session_nonce, self.boot_id,
        )
        self._action_sends.move_to_end(sequence)
        while len(self._action_sends) > _ACK_CACHE_LIMIT:
            self._action_sends.popitem(last=False)
        self._active_motion_sent_at = sent_at if action is not MotionActionCode.BRAKE else None
        return sequence

    def wait_for_motion_action_applied(
        self, sequence: int, timeout_s: float | None = None,
    ) -> StateTelemetry:
        """分别确认动作 ACK 已接受及新 STATE 已应用该序号。"""
        record = self._action_sends.get(sequence)
        if record is None:
            raise McuError(f"未知 MOTION_ACTION seq={sequence}")
        command, sent_at, baseline_uptime, nonce, boot = record
        deadline = sent_at + self._confirm_timeout(timeout_s)
        while True:
            now = time.monotonic()
            if (self.session_nonce, self.boot_id) != (nonce, boot) or not self.connected:
                self._stop_failed_confirmation("动作确认期间 MCU 会话变化")
                raise McuError("动作确认期间 MCU 会话变化")
            if command.action is not MotionActionCode.BRAKE:
                self._check_motion_lease(now)
                reason = self.motion_health_reason()
                if reason is not None:
                    self._stop_failed_confirmation(reason)
                    raise McuError(reason)
            ack = self._cached_ack(MessageType.MOTION_ACTION, sequence)
            if ack is not None and ack.status is not AckStatus.ACCEPTED:
                reason = f"MOTION_ACTION seq={sequence} 被拒: {ack.status.name}"
                self._stop_failed_confirmation(reason)
                raise McuError(reason)
            state = self.state
            uptime = self._family_uptime_ms.get("state")
            if (
                ack is not None and ack.status is AckStatus.ACCEPTED
                and state is not None
                and self._session_matches(state.session_nonce, state.boot_id)
                and state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                and state.applied_control_sequence == sequence
                and self._last_state_recv >= sent_at
                and (baseline_uptime is None or uptime is not None and uptime > baseline_uptime)
                and self._sample_is_fresh("state", now)
                and now < deadline
            ):
                return state
            if now >= deadline:
                self._stop_failed_confirmation(f"MOTION_ACTION seq={sequence} 未在确认窗口内应用")
                raise McuError(self._hard_fault_reason)
            self.pump()
            time.sleep(0.005)

    def wait_for_motion_action_complete(
        self,
        sequence: int,
        *,
        timeout_s: float = 13.0,
        require_ranges: bool = True,
        require_upper: bool = True,
        on_poll: Callable[[], None] | None = None,
    ) -> StateTelemetry:
        """每 100 ms 续约并等待当前转向的 ACTIVE→COMPLETED 新 STATE。"""
        record = self._action_sends.get(sequence)
        if record is None:
            raise McuError(f"未知 MOTION_ACTION seq={sequence}")
        command, sent_at, baseline_uptime, nonce, boot = record
        if command.action not in (MotionActionCode.TURN_LEFT, MotionActionCode.TURN_RIGHT,
                                  MotionActionCode.UTURN_LEFT):
            raise McuError("只有固定转向动作可等待完成；直行按里程停车")
        if not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise McuError("动作超时必须是有限正数")
        deadline = min(time.monotonic() + timeout_s,
                       self._motion_deadline if self._motion_deadline is not None else float("inf"))
        next_renew = sent_at + 0.1
        issued_sequences = {sequence}
        active_uptime: int | None = None
        self._turn_waiting = True
        try:
            while True:
                self.pump()
                now = time.monotonic()
                if (self.session_nonce, self.boot_id) != (nonce, boot) or not self.connected:
                    raise McuError("转向期间 MCU 会话变化")
                self._check_motion_lease(now)
                health = self.motion_health_reason()
                if health is not None:
                    raise McuError(health)
                if on_poll is not None:
                    on_poll()
                health = self.telemetry_ok(require_ranges=require_ranges,
                                           require_upper=require_upper)
                if health is not None:
                    raise McuError(health)
                state = self.state
                uptime = self._family_uptime_ms.get("state")
                if (state is not None and uptime is not None
                    and (baseline_uptime is None or uptime > baseline_uptime)
                    and self._last_state_recv >= sent_at
                    and self._sample_is_fresh("state", now)
                    and state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                    and state.applied_control_sequence in issued_sequences):
                    ack = self._cached_ack(MessageType.MOTION_ACTION,
                                           state.applied_control_sequence)
                    if ack is not None and ack.status is AckStatus.ACCEPTED:
                        if state.state_flags & StateFlag.TURN_ACTIVE:
                            active_uptime = uptime
                        elif state.state_flags & StateFlag.TURN_ABORTED:
                            raise McuError("固件转向已中止")
                        elif (state.state_flags & StateFlag.TURN_COMPLETED
                              and active_uptime is not None and uptime > active_uptime):
                            self._active_motion_sent_at = None
                            return state
                if now >= deadline:
                    raise McuError("固件转向超时或比赛截止时间已到")
                if now >= next_renew:
                    renewal = self.send_motion_action(
                        command.action, command.speed_mms,
                        brake_after_turn=bool(command.flags & MotionActionFlag.BRAKE_AFTER_TURN),
                        require_ranges=require_ranges, require_upper=require_upper,
                    )
                    issued_sequences.add(renewal)
                    next_renew = time.monotonic() + 0.1
                time.sleep(min(0.02, max(0.001, next_renew - time.monotonic())))
        except BaseException:
            try:
                self.send_stop(StopReason.FAULT)
            except Exception as exc:
                runlog.exception("mcu_action_stop_failed", "转向失败后 STOP 发送失败", exc)
            raise
        finally:
            self._turn_waiting = False

    def _motion_lease_deadline(self) -> float | None:
        if self._active_motion_sent_at is None:
            return None
        # Leave room for STOP, an in-progress receive pass and scheduling jitter.
        reserve = config.MCU_WRITE_TIMEOUT_S + _RX_PUMP_BUDGET_S + _LEASE_SCHEDULING_MARGIN_S
        return self._active_motion_sent_at + config.LEASE_MS / 1000.0 - reserve

    def _stop_failed_confirmation(self, reason: str) -> None:
        self._latch_hard_fault(reason)
        if self._active_motion_sent_at is not None:
            try:
                self.send_stop(StopReason.FAULT)
            except Exception as exc:
                runlog.exception("mcu_confirmation_stop_failed", "确认超时后的 STOP 发送失败", exc)

    def _check_motion_lease(self, now: float | None = None) -> None:
        deadline = self._motion_lease_deadline()
        if deadline is not None and (time.monotonic() if now is None else now) >= deadline:
            self._stop_failed_confirmation("运动控制间隔已用尽安全租约预算，禁止自动恢复运动")
            raise McuError(self._hard_fault_reason)

    def set_motion_deadline(self, deadline: float | None) -> None:
        """Set the absolute match deadline; HELLO must not reset the match clock."""
        if deadline is not None and (not isinstance(deadline, (int, float)) or not math.isfinite(deadline)):
            raise McuError("运动截止时间必须是有限单调时刻或 None")
        self._motion_deadline = deadline

    def _check_motion_deadline(self) -> None:
        if self._motion_deadline is not None and time.monotonic() >= self._motion_deadline:
            raise McuError("比赛截止时间已到，拒绝非零运动")

    def synchronize_clock(self, timeout_s: float | None = None) -> StateTelemetry:
        """Tighten the clock lower bound using one new zero-motion application.

        This active probe belongs immediately after HELLO. Ordinary telemetry and
        command confirmations cannot advance the clock anchor from receipt time.
        """
        timeout = min(self._confirm_timeout(timeout_s), config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
        if not self.connected or self._hard_fault_reason is not None:
            raise McuError(self._hard_fault_reason or "时钟同步前必须建立 HELLO 会话")
        if self._clock_sync_attempted:
            raise McuError("当前会话已经尝试零速时钟同步，不重复发送")
        self._clock_sync_attempted = True
        nonce, boot = self.session_nonce, self.boot_id
        previous_state = self.state
        previous_uptime = self._family_uptime_ms.get("state")
        sent_at = time.monotonic()
        deadline = sent_at + timeout
        sequence = self.send_motion(0, 0)
        self._ack_cache.pop((MessageType.CONTROL_TARGET, sequence), None)
        while True:
            now = time.monotonic()
            if self._hard_fault_reason is not None:
                raise McuError(self._hard_fault_reason)
            if (self.session_nonce, self.boot_id) != (nonce, boot) or not self.connected:
                raise McuError("零速时钟同步期间 MCU 会话变化")
            if now >= deadline:
                self._latch_hard_fault(f"零速时钟同步 seq={sequence} 未在确认窗口内应用")
                raise McuError(self._hard_fault_reason)
            state = self.state
            uptime = self._family_uptime_ms.get("state")
            if (
                state is not None and state is not previous_state
                and self._session_matches(state.session_nonce, state.boot_id)
                and state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                and state.applied_control_sequence == sequence
                and state.target_linear_mms == 0 and state.target_angular_mrads == 0
                and sent_at <= self._last_state_recv <= now
                and uptime is not None and (previous_uptime is None or uptime > previous_uptime)
            ):
                # The matching STATE was generated after this exact command was
                # sent, so sent_at - uptime is a tighter causal lower bound.
                lower_bound = sent_at - uptime / 1000.0
                previous_offset = self._clock_offset_s
                self._clock_offset_s = (lower_bound if previous_offset is None
                                        else max(previous_offset, lower_bound))
                runlog.emit("mcu_clock_synchronized", "零速应用回读已收紧 MCU 采集时间下界",
                            sequence=sequence, round_trip_s=now - sent_at,
                            issued_monotonic=sent_at, confirmation_window_s=timeout,
                            reply_uptime_ms=state.uptime_ms, reply_uptime_expanded_ms=uptime,
                            reply_received_monotonic=self._last_state_recv,
                            previous_offset_s=previous_offset, offset_s=self._clock_offset_s,
                            anchor_adjustment_s=(self._clock_offset_s - previous_offset
                                                 if previous_offset is not None else 0.0))
                return state
            self.pump()
            remaining = deadline - time.monotonic()
            if self.state is state and remaining > 0:
                time.sleep(min(0.005, remaining))

    def send_gimbal(self, pose: WireGimbalPose, pan_us: int, tilt_us: int) -> int:
        """发纯云台命令；pan 是左右、tilt 是上下，不续运动租约。"""
        for name, pulse_us in (("pan_us", pan_us), ("tilt_us", tilt_us)):
            if (isinstance(pulse_us, bool) or not isinstance(pulse_us, int)
                    or not config.SERVO_PWM_MIN_US <= pulse_us <= config.SERVO_PWM_MAX_US):
                raise McuError(
                    f"{name} 必须是 {config.SERVO_PWM_MIN_US}.."
                    f"{config.SERVO_PWM_MAX_US} μs 的整数: {pulse_us!r}"
                )
        cmd = ControlTarget(
            session_nonce=self.session_nonce,
            flags=ControlFlag.GIMBAL_VALID | ControlFlag.GIMBAL_PWM_ENABLE,
            linear_mms=0,
            angular_mrads=0,
            aim_id=0,
            pose_id=pose,
            # 保持协议字段/顺序：第一路为 CH1 上下，第二路为 CH2 左右。
            pan_pwm_us=tilt_us,
            tilt_pwm_us=pan_us,
        )
        return self._send(MessageType.CONTROL_TARGET, encode_control_target(cmd))

    def send_stop(self, reason: StopReason = StopReason.MISSION) -> int:
        sequence = self._send(MessageType.STOP, encode_stop(StopCommand(self.session_nonce, int(reason))))
        self._active_motion_sent_at = None
        return sequence

    def send_light(self, duty_permille: int) -> int:
        if not self.has_negotiated_capability(Capability.LIGHT_CONTROL):
            raise McuError("LIGHT_CONTROL 未在当前 HELLO 会话协商")
        return self._send(
            MessageType.LIGHT_CONTROL,
            encode_light_control(LightControl(self.session_nonce, duty_permille)),
        )

    # ------------------------------------------------------------------
    # 接收
    # ------------------------------------------------------------------
    def pump(self):
        """读串口字节、解析帧、更新并校验遥测缓存，返回本轮的帧列表。"""
        import serial

        frames = []
        if self._ser is None:
            return frames
        try:
            remaining = min(self._ser.in_waiting, _RX_PUMP_MAX_BYTES)
            deadline = time.monotonic() + _RX_PUMP_BUDGET_S
            while remaining > 0 and time.monotonic() < deadline:
                # A parser tail is at most one incomplete frame. Feed small chunks
                # so a valid UART backlog never trips its 1024-byte noise bound.
                data = self._ser.read(min(remaining, _RX_CHUNK_BYTES))
                if not data:
                    break
                remaining -= len(data)
                if self._capture is not None:
                    self._rx_monotonic_ms = time.monotonic() * 1000.0
                    self._capture_call("record_bytes", "rx", data, self._rx_monotonic_ms)
                chunk_frames = self._parser.feed(data)
                self._retain_parse_issues(self._parser.pop_issues())
                for frame in chunk_frames:
                    if self._capture is not None:
                        self._capture_frame("rx", frame, self._rx_monotonic_ms)
                    self._handle(frame)
                frames.extend(chunk_frames)
        except (serial.SerialException, OSError) as exc:
            # 单次读失败不立即中止：记录并返回空，交给 telemetry_ok 的超龄容错门控
            # （瞬时 USB 抖动自行恢复；链路真断则由 _send 的写失败锁存故障）
            self.last_parse_error = f"READ_ERROR: {exc}"
            runlog.exception("mcu_read_error", "STM32串口读取失败", exc)
        return frames

    def _session_matches(self, session_nonce: int, boot_id: int) -> bool:
        return (
            self.connected
            and session_nonce == self.session_nonce
            and boot_id == self.boot_id
        )

    def _accept_uptime(self, family: str, raw_uptime_ms: int) -> bool:
        """Unwrap one shared MCU clock, retaining each family's sample ordering."""
        if self._uptime_reference_raw is None:
            expanded = raw_uptime_ms
        else:
            delta = ((raw_uptime_ms - self._uptime_reference_raw + 0x80000000) & 0xFFFFFFFF) - 0x80000000
            if delta == -0x80000000:
                return False
            expanded = self._uptime_reference_ms + delta
        previous = self._family_uptime_ms.get(family)
        if previous is not None and expanded <= previous:
            return False
        received_at = time.monotonic()
        if self._clock_offset_s is None:
            # A new nonce cannot be echoed before HELLO was sent. Using this lower
            # bound keeps even the first queued frame old instead of re-dating it.
            anchor = received_at if self._session_sent_at is None else self._session_sent_at
            self._clock_offset_s = anchor - expanded / 1000.0
        self._clock_offset_s = min(self._clock_offset_s, received_at - expanded / 1000.0)
        if self._uptime_reference_raw is None or expanded > self._uptime_reference_ms:
            self._uptime_reference_raw = raw_uptime_ms
            self._uptime_reference_ms = expanded
        self._family_uptime_ms[family] = expanded
        setattr(self, f"_last_{family}_recv", received_at)
        return True

    def _sample_time(self, family: str) -> float:
        received_at = getattr(self, f"_last_{family}_recv")
        uptime = self._family_uptime_ms.get(family)
        if uptime is None or self._clock_offset_s is None:
            return received_at  # Explicit receive timestamps also support injected bench samples.
        return min(received_at, self._clock_offset_s + uptime / 1000.0)

    def _sample_is_fresh(self, family: str, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        age = now - self._sample_time(family)
        return 0.0 <= age <= config.TELEMETRY_STALE_MS / 1000.0

    def _handle(self, frame) -> None:
        mt = frame.message_type
        if mt is MessageType.HELLO_RESP and self._pending_hello_nonce is not None:
            try:
                self._complete_hello(frame.payload)
            except Exception as exc:
                self._hello_error = exc
            return
        try:
            if mt is MessageType.STATE:
                self._accept_state(decode_state(frame.payload))
            elif mt is MessageType.DIRECTIONAL_RANGE4:
                self._accept_directional(decode_directional_range4(frame.payload))
            elif mt is MessageType.CULVERT_UPPER2:
                self._accept_upper(decode_culvert_upper2(frame.payload))
            elif mt is MessageType.LIGHT_STATE:
                self._accept_light_state(decode_light_state(frame.payload))
            elif mt is MessageType.DIAGNOSTICS:
                self._accept_diagnostics(decode_diagnostics(frame.payload))
            elif mt is MessageType.ACK:
                self._accept_ack(decode_ack(frame.payload))
            elif mt is MessageType.WHEEL_TRACE:
                if self.has_negotiated_capability(Capability.WHEEL_TRACE):
                    self._accept_wheel_trace(decode_wheel_trace(frame.payload), frame.sequence)
        except Exception as exc:  # 坏帧只记录，不让它打断主循环
            self._retain_parse_issues((ParserIssue("PAYLOAD_ERROR", str(exc)),))

    def _accept_wheel_trace(self, sample: WheelTraceTelemetry, frame_sequence: int | None = None) -> None:
        # Optional observations never update the safety clock or health caches.
        if not self._session_matches(sample.session_nonce, sample.boot_id):
            self.trace_ignored_samples += 1
            return
        if self.wheel_trace is not None:
            delta = (sample.uptime_ms - self.wheel_trace.uptime_ms) & 0xFFFFFFFF
            if (classify_sequence(sample.sample_sequence, self._last_trace_seq)
                    is not SequenceDisposition.NEWER or not 0 < delta < 0x80000000):
                self.trace_ignored_samples += 1
                return
            # Header SEQ is shared by all MCU messages and may advance by more
            # than half its range while trace is idle. Only sample SEQ orders
            # this stream; header SEQ remains intact in the raw/frame logs.
            gap = ((sample.sample_sequence - self._last_trace_seq) & 0xFFFF) - 1
            if gap:
                self.trace_sample_gaps += gap
                self._capture_call("event", "trace_sample_gap", missing=gap,
                                   sample_sequence=sample.sample_sequence, frame_seq=frame_sequence)
        self.wheel_trace = sample
        self._last_trace_seq = sample.sample_sequence
        self._last_trace_recv = time.monotonic()

    def wheel_trace_summary(self) -> str:
        sample = self.wheel_trace
        if sample is None or not self.connected:
            return "轮速追踪：等待数据"
        now = time.monotonic()
        if self._last_trace_recv is None or not 0 <= now - self._last_trace_recv <= config.TELEMETRY_STALE_MS / 1000.0:
            return "轮速追踪：已过期"
        if self._clock_offset_s is None or self._uptime_reference_raw is None:
            return "轮速追踪：等待时钟数据"
        if self._clock_offset_s is not None and self._uptime_reference_raw is not None:
            delta = ((sample.uptime_ms - self._uptime_reference_raw + 0x80000000) & 0xFFFFFFFF) - 0x80000000
            generated = self._clock_offset_s + (self._uptime_reference_ms + delta) / 1000.0
            if not 0.0 <= now - generated <= config.TELEMETRY_STALE_MS / 1000.0:
                return "轮速追踪：已过期"
        left = sample.left_encoder_delta * 10.0 / sample.elapsed_ms
        right = sample.right_encoder_delta * 10.0 / sample.elapsed_ms
        return (f"轮速追踪：实测/目标L={left:.2f}/{sample.left_target_counts} "
                f"R={right:.2f}/{sample.right_target_counts} 计数/10ms；"
                f"目标命令L/R={sample.left_target_command}/{sample.right_target_command}；"
                f"PI L/R={sample.left_pi_output}/{sample.right_pi_output}；"
                f"实际输出L/R={sample.left_applied_output}/{sample.right_applied_output}；"
                f"参考/航向={sample.reference_yaw_tenth_deg / 10:.1f}/{sample.yaw_tenth_deg / 10:.1f}°；"
                f"航向修正={sample.heading_correction}；flags=0x{int(sample.flags):02X}")

    def _accept_state(self, st: StateTelemetry) -> None:
        if not self._session_matches(st.session_nonce, st.boot_id):
            if self.connected and st.boot_id != self.boot_id:
                self._latch_hard_fault("MCU boot 变化（复位）")
            return
        # STATE 的 range_cycle_seq 在当前固件恒为 0（遗留字段）。改用 32 位 uptime
        # 拒绝重复/倒退缓存帧，并允许固件运行计时正常回绕。
        if self._last_state_uptime_ms is not None:
            delta = (st.uptime_ms - self._last_state_uptime_ms) & 0xFFFFFFFF
            if not 0 < delta < 0x80000000:
                return
        if not self._accept_uptime("state", st.uptime_ms):
            return
        self._last_state_uptime_ms = st.uptime_ms
        self.state = st
        # STATE.fault_bits 是固件的实时健康快照，其中包括 IMU 零偏标定、
        # ToF 初始化等可恢复启动状态。不能首次看到非零就永久锁死会话；
        # telemetry_ok() 会对持续非零值去抖后 fail-closed。

    def _accept_directional(self, d: DirectionalRange4Telemetry) -> None:
        if not self._session_matches(d.session_nonce, d.boot_id):
            if self.connected and d.boot_id != self.boot_id:
                self._latch_hard_fault("MCU boot 变化（复位）")
            return
        if self._last_directional_seq is not None and classify_sequence(
            d.directional_cycle_seq, self._last_directional_seq
        ) is not SequenceDisposition.NEWER:
            return
        if not self._accept_uptime("directional", d.uptime_ms):
            return
        self._last_directional_seq = d.directional_cycle_seq
        self.directional = d

    def _accept_upper(self, u: CulvertUpper2Telemetry) -> None:
        if not self._session_matches(u.session_nonce, u.boot_id):
            if self.connected and u.boot_id != self.boot_id:
                self._latch_hard_fault("MCU boot 变化（复位）")
            return
        if self._last_upper_seq is not None and classify_sequence(
            u.upper_cycle_seq, self._last_upper_seq
        ) is not SequenceDisposition.NEWER:
            return
        if not self._accept_uptime("upper", u.uptime_ms):
            return
        self._last_upper_seq = u.upper_cycle_seq
        self.culvert_upper = u

    def _accept_light_state(self, state: LightState) -> None:
        if not self._session_matches(state.session_nonce, state.boot_id):
            if self.connected and state.boot_id != self.boot_id:
                self._latch_hard_fault("MCU boot 变化（复位）")
            return
        if not self._accept_uptime("light", state.uptime_ms):
            return
        self.light_state = state
        if state.state is WireLightState.FAULT or state.fault_bits != 0:
            self._latch_hard_fault(
                f"LIGHT_STATE 故障 state={state.state.name} bits=0x{state.fault_bits:08X}"
            )

    def _accept_diagnostics(self, diagnostics: DiagnosticsTelemetry) -> None:
        if not self._session_matches(diagnostics.session_nonce, diagnostics.boot_id):
            if self.connected and diagnostics.boot_id != self.boot_id:
                self._latch_hard_fault("MCU boot 变化（复位）")
            return
        if not self._accept_uptime("diagnostics", diagnostics.uptime_ms):
            return
        self.diagnostics = diagnostics

    def _accept_ack(self, ack: AckMessage) -> None:
        if not self._session_matches(ack.session_nonce, ack.boot_id):
            if self.connected and ack.boot_id != self.boot_id:
                self._latch_hard_fault("MCU boot 变化（复位）")
            return  # 陈旧/跨会话 ACK 直接丢弃，不污染当前会话确认状态
        self.last_ack = ack
        key = (ack.acknowledged_type, ack.acknowledged_sequence)
        self._ack_cache[key] = ack
        self._ack_cache.move_to_end(key)
        while len(self._ack_cache) > _ACK_CACHE_LIMIT:
            self._ack_cache.popitem(last=False)
        if ack.status is not AckStatus.ACCEPTED:
            self._latch_hard_fault(
                f"命令被拒: {ack.acknowledged_type.name} {ack.status.name}"
            )

    def _cached_ack(self, message_type: MessageType, sequence: int) -> AckMessage | None:
        return self._ack_cache.get((message_type, sequence))

    def _confirm_timeout(self, timeout_s: float | None) -> float:
        timeout = config.MCU_COMMAND_CONFIRM_TIMEOUT_S if timeout_s is None else timeout_s
        if timeout <= 0 or timeout >= config.LEASE_MS / 1000.0:
            raise McuError("命令确认超时必须大于 0 且短于 MCU 运动租约")
        return timeout

    def wait_for_control_applied(
        self, sequence: int, timeout_s: float | None = None
    ) -> StateTelemetry:
        """有界等待指定 CONTROL_TARGET 在 STATE 中回读为已应用。"""
        deadline = time.monotonic() + self._confirm_timeout(timeout_s)
        while True:
            now = time.monotonic()
            if self._hard_fault_reason is not None:
                raise McuError(self._hard_fault_reason)
            self._check_motion_lease(now)
            ack = self._cached_ack(MessageType.CONTROL_TARGET, sequence)
            if ack is not None and ack.status is not AckStatus.ACCEPTED:
                raise McuError(
                    f"CONTROL_TARGET seq={sequence} 被拒: {ack.status.name}"
                )
            if (
                self.state is not None
                and (self.state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT)
                and self.state.applied_control_sequence == sequence
                and now < deadline
                and self._sample_is_fresh("state", now)
            ):
                return self.state
            if now >= deadline:
                self._stop_failed_confirmation(
                    f"CONTROL_TARGET seq={sequence} 未在确认窗口内应用"
                )
                raise McuError(self._hard_fault_reason)
            self.pump()
            time.sleep(0.005)

    def wait_for_light_applied(
        self, sequence: int, duty_permille: int, timeout_s: float | None = None
    ) -> LightState:
        """有界等待 LIGHT_CONTROL 的匹配 ACK 与 LIGHT_STATE 实际回读。"""
        expected_state = (
            WireLightState.APPLIED if duty_permille > 0 else WireLightState.DISABLED
        )
        deadline = time.monotonic() + self._confirm_timeout(timeout_s)
        while True:
            now = time.monotonic()
            if self._hard_fault_reason is not None:
                raise McuError(self._hard_fault_reason)
            self._check_motion_lease(now)
            ack = self._cached_ack(MessageType.LIGHT_CONTROL, sequence)
            if ack is not None and ack.status is not AckStatus.ACCEPTED:
                raise McuError(
                    f"LIGHT_CONTROL seq={sequence} 被拒: {ack.status.name}"
                )
            state = self.light_state
            if (
                ack is not None
                and ack.status is AckStatus.ACCEPTED
                and state is not None
                and state.applied_light_sequence == sequence
                and state.duty_permille == duty_permille
                and state.state is expected_state
                and now < deadline
                and self._sample_is_fresh("light", now)
            ):
                return state
            if now >= deadline:
                self._stop_failed_confirmation(
                    f"LIGHT_CONTROL seq={sequence} 未在确认窗口内应用"
                )
                raise McuError(self._hard_fault_reason)
            self.pump()
            time.sleep(0.005)

    # ------------------------------------------------------------------
    # 健康
    # ------------------------------------------------------------------
    def motion_health_snapshot(self) -> str:
        """保留失效 STATE 原值，并标明诊断停车原因是否仍新鲜。"""
        now = time.monotonic()
        state = self.state
        flags = getattr(state, "state_flags", None)
        odom_status = getattr(state, "odom_status", None)
        state_age = (f"{max(0, round((now - self._sample_time('state')) * 1000))} ms"
                     if state is not None else "unknown")
        diagnostics = self.diagnostics
        stop_reason = getattr(diagnostics, "last_stop_reason", None)
        if stop_reason is None:
            stop_text = "unknown"
        else:
            stop_text = f"{getattr(stop_reason, 'name', stop_reason)}({int(stop_reason)})"
        if diagnostics is None:
            diag_age = "unknown（缺失）"
        else:
            age_ms = max(0, round((now - self._sample_time('diagnostics')) * 1000))
            freshness = "新鲜" if self._sample_is_fresh("diagnostics", now) else "超龄"
            diag_age = f"{age_ms} ms（{freshness}）"
        if not self.has_negotiated_capability(Capability.DIAGNOSTICS):
            diag_age += "（未协商）"
        return (
            f"state_flags={f'0x{int(flags):04X}' if flags is not None else 'unknown'} "
            f"odom_status={f'0x{int(odom_status):02X}' if odom_status is not None else 'unknown'} "
            f"STATE年龄={state_age} "
            f"fault_bits(raw)=0x{int(getattr(state, 'fault_bits', 0)):08X} "
            f"fault_bits(active)=0x{self._active_fault_bits():08X} "
            f"left_mm={getattr(state, 'left_mm', 'unknown')} "
            f"right_mm={getattr(state, 'right_mm', 'unknown')} "
            f"yaw_tenth_deg={getattr(state, 'yaw_tenth_deg', 'unknown')} "
            f"applied_control_sequence={getattr(state, 'applied_control_sequence', 'unknown')} "
            f"target_linear_mms={getattr(state, 'target_linear_mms', 'unknown')} "
            f"target_angular_mrads={getattr(state, 'target_angular_mrads', 'unknown')} "
            f"DIAG停止原因={stop_text} DIAG年龄={diag_age}"
        )

    def motion_health_reason(self) -> str | None:
        """即时动作门禁；不使用遥测去抖，也不检查独立的激光门禁。"""
        reason = None
        if self._hard_fault_reason is not None:
            reason = self._hard_fault_reason
        elif not self.connected or self._ser is None:
            reason = "未建立 STM32 会话"
        elif self.state is None:
            reason = "缺少 STATE 遥测"
        elif not self._sample_is_fresh("state"):
            reason = "STATE 遥测超龄"
        elif self._active_fault_bits():
            reason = "MCU 固件故障"
        elif not (self.state.state_flags & StateFlag.ODOM_VALID):
            reason = "ODOM 无效（里程/航向不可用）"
        elif self.has_negotiated_capability(Capability.DIAGNOSTICS):
            reason = self.diagnostics_ok()
        return f"{reason}；{self.motion_health_snapshot()}" if reason is not None else None

    def diagnostics_ok(self) -> str | None:
        """已协商的诊断帧是否保持新鲜。"""
        if not self.connected or self._ser is None:
            return "未连接"
        if not self.has_negotiated_capability(Capability.DIAGNOSTICS):
            return "DIAGNOSTICS 未协商"
        if (
            self.diagnostics is None
            or not self._sample_is_fresh("diagnostics")
        ):
            return "DIAGNOSTICS 遥测超龄"
        return None

    def _effective_range_status(self, raw_status, age_ms, max_age_ms: int):
        """把超龄样本统一收敛为 STALE，禁止上层继续使用旧距离。"""
        if raw_status is None:
            return None
        if isinstance(age_ms, bool) or not isinstance(age_ms, int) or not 0 <= age_ms <= max_age_ms:
            return WireRangeStatus.STALE
        return raw_status

    @property
    def directional_age_ms(self) -> tuple[int, int, int, int] | None:
        directional = self.directional
        if directional is None:
            return None
        ages = tuple(getattr(directional, "age_ms", ()))
        elapsed = max(0, int((time.monotonic() - self._sample_time("directional")) * 1000))
        return tuple(age + elapsed for age in ages) if len(ages) == 4 else None

    @property
    def upper_age_ms(self) -> tuple[int, int] | None:
        upper = self.culvert_upper
        if upper is None:
            return None
        ages = getattr(upper, "age_ms", ())
        if (not isinstance(ages, (list, tuple)) or len(ages) != 2
                or any(isinstance(age, bool) or not isinstance(age, int) or age < 0 for age in ages)):
            return None
        elapsed = max(0, int((time.monotonic() - self._sample_time("upper")) * 1000))
        return tuple(age + elapsed for age in ages)

    @property
    def directional_effective_statuses(self) -> tuple[WireRangeStatus, ...]:
        directional = self.directional
        ages = self.directional_age_ms
        if directional is None or ages is None:
            return ()
        return tuple(
            self._effective_range_status(status, ages[index], config.DIRECTIONAL_LASER_MAX_AGE_MS)
            for index, status in enumerate(directional.status)
        )

    @property
    def upper_effective_statuses(self) -> tuple[WireRangeStatus, ...]:
        upper = self.culvert_upper
        ages = self.upper_age_ms
        if upper is None or ages is None:
            return ()
        statuses = getattr(upper, "status", ())
        if not isinstance(statuses, (list, tuple)) or len(statuses) != 2:
            return ()
        distances = getattr(upper, "distance_mm", ())
        if not isinstance(distances, (list, tuple)):
            distances = ()
        effective = []
        for index, status in enumerate(statuses):
            status = self._effective_range_status(
                status, ages[index], config.CULVERT_UPPER_LASER_MAX_AGE_MS
            )
            if status is WireRangeStatus.VALID:
                distance = distances[index] if index < len(distances) else None
                if (isinstance(distance, bool) or not isinstance(distance, (int, float))
                        or not math.isfinite(distance) or not 0 <= distance < 65535):
                    status = WireRangeStatus.SIGNAL_ERROR
            effective.append(status)
        return tuple(effective)

    def _active_fault_bits(self) -> int:
        """返回仍应阻止上位机运动的固件故障位。"""
        state = self.state
        raw = int(getattr(state, "fault_bits", 0)) if state is not None else 0
        ignored = int(config.MCU_IGNORED_RUNTIME_FAULT_BITS)
        return raw & ~ignored

    def _laser_stale_reason(self, *, require_upper: bool = True) -> str | None:
        # 当前固件只实装 CH1/CH2/CH4/CH5；协议空槽不参与健康门禁。
        directional_channels = ((1, "左下1"), (2, "右下2"))
        ages = self.directional_age_ms
        statuses = self.directional_effective_statuses
        if ages is not None and len(statuses) == 4:
            for index, name in directional_channels:
                age, status = ages[index], statuses[index]
                if status not in (WireRangeStatus.VALID, WireRangeStatus.NO_TARGET,
                                  WireRangeStatus.OUT_OF_RANGE):
                    return (
                        f"{name}激光故障 status={status.name} age_ms={age}，"
                        f"年龄上限={config.DIRECTIONAL_LASER_MAX_AGE_MS}"
                    )
        upper_ages = self.upper_age_ms
        upper_statuses = self.upper_effective_statuses
        if (require_upper and self.has_negotiated_capability(Capability.CULVERT_UPPER2)
                and (upper_ages is None or len(upper_statuses) != 2)):
            return "上层涵洞激光缺少有效状态或样本年龄证据"
        if (
            require_upper
            and
            self.has_negotiated_capability(Capability.CULVERT_UPPER2)
            and upper_ages is not None
            and len(upper_statuses) == 2
            and all(status not in (WireRangeStatus.VALID, WireRangeStatus.NO_TARGET,
                                   WireRangeStatus.OUT_OF_RANGE)
                    for status in upper_statuses)
        ):
            # 上右故障时允许健康的上左备援；无目标状态仍是有效观测。
            return (
                "上左4/上右5涵洞激光均故障 "
                f"status={[status.name for status in upper_statuses]} "
                f"age_ms={upper_ages}，年龄上限={config.CULVERT_UPPER_LASER_MAX_AGE_MS}"
            )
        return None
    def telemetry_ok(
        self,
        *,
        require_ranges: bool = True,
        require_upper: bool = True,
    ) -> str | None:
        """返回 None=正常，或故障原因字符串。

        硬故障（命令被拒、跨 boot、链路写失败）仅能由新 HELLO 会话解除；
        实时 fault_bits、遥测超龄与 ODOM 无效会容忍连续
        TELEMETRY_FAULT_TOLERANCE 次后才上报，但始终 fail-closed。
        """
        if self._hard_fault_reason is not None:
            return self._hard_fault_reason
        if not self.connected or self._ser is None:
            return "未连接"
        now = time.monotonic()
        reason: str | None = None
        diagnostics_reason = (
            self.diagnostics_ok()
            if self.has_negotiated_capability(Capability.DIAGNOSTICS)
            else None
        )
        if not self._sample_is_fresh("state", now):
            reason = "STATE 遥测超龄"
        elif require_ranges and not self._sample_is_fresh("directional", now):
            reason = "DIRECTIONAL_RANGE4 遥测超龄"
        elif require_ranges and require_upper and (
            self.has_negotiated_capability(Capability.CULVERT_UPPER2)
            and not self._sample_is_fresh("upper", now)
        ):
            reason = "CULVERT_UPPER2 遥测超龄"
        elif diagnostics_reason is not None:
            reason = diagnostics_reason
        elif require_ranges and self._laser_stale_reason(require_upper=require_upper) is not None:
            reason = self._laser_stale_reason(require_upper=require_upper)
        elif self._active_fault_bits() != 0:
            reason = f"MCU fault_bits=0x{self.state.fault_bits:08X}"
        elif self.state is not None and not (self.state.state_flags & StateFlag.ODOM_VALID):
            reason = "ODOM 无效（IMU 未就绪）"
        if reason is None:
            self._consecutive_bad = 0
            return None
        self._consecutive_bad += 1
        if self._consecutive_bad >= config.TELEMETRY_FAULT_TOLERANCE:
            return reason
        return None

    def clear_fault(self) -> None:
        """仅清理软性遥测去抖；硬锁存必须通过新 HELLO 会话恢复。"""
        self._consecutive_bad = 0

    # ------------------------------------------------------------------
    # 便捷取数（单位：mm / 度 / 状态）
    # ------------------------------------------------------------------
    @property
    def left_mm(self) -> int | None:
        return self._dir(1)

    @property
    def right_mm(self) -> int | None:
        return self._dir(2)

    @property
    def rear_mm(self) -> int | None:
        return self._dir(3)

    def _dir(self, idx: int) -> int | None:
        directional = self.directional
        if directional is None or self._dir_status(idx) is not WireRangeStatus.VALID:
            return None
        return directional.distance_mm[idx]

    @property
    def left_status(self) -> WireRangeStatus | None:
        return self._dir_status(1)

    @property
    def right_status(self) -> WireRangeStatus | None:
        return self._dir_status(2)

    @property
    def rear_status(self) -> WireRangeStatus | None:
        return self._dir_status(3)

    def _dir_status(self, idx: int) -> WireRangeStatus | None:
        statuses = self.directional_effective_statuses
        if len(statuses) != 4 or not 0 <= idx < len(statuses):
            return None
        return statuses[idx]

    @property
    def upper_left_mm(self) -> int | None:
        return self._upper(0)

    @property
    def upper_right_mm(self) -> int | None:
        return self._upper(1)

    def _upper(self, idx: int) -> int | None:
        upper = self.culvert_upper
        if upper is None or self._upper_status(idx) is not WireRangeStatus.VALID:
            return None
        return upper.distance_mm[idx]

    @property
    def upper_left_status(self) -> WireRangeStatus | None:
        return self._upper_status(0)

    @property
    def upper_right_status(self) -> WireRangeStatus | None:
        return self._upper_status(1)

    def _upper_status(self, idx: int) -> WireRangeStatus | None:
        statuses = self.upper_effective_statuses
        if len(statuses) != 2 or not 0 <= idx < len(statuses):
            return None
        return statuses[idx]
    @property
    def yaw_deg(self) -> float | None:
        s = self.state
        if s is None:
            return None
        if not (s.state_flags & StateFlag.ODOM_VALID):
            return None
        if not self._sample_is_fresh("state"):
            return None  # 遥测超龄，不返回陈旧 yaw（转弯据此原地等待/中止）
        return s.yaw_tenth_deg / 10.0

    @property
    def odom_mm(self) -> tuple[int, int] | None:
        """双轮编码器里程 (left_mm, right_mm)，仅 ODOM_VALID 且遥测新鲜时非 None。

        固件在 STATE 帧里上报双轮绝对里程，RDK 解码为 ``StateTelemetry.left_mm/right_mm``
        （单位 mm）。注意与本类 :attr:`left_mm`/:attr:`right_mm`（侧向激光距离）命名相同、
        语义不同：本属性读的是 ``self.state.left_mm/right_mm``（里程）。
        只有固件 profile 已核验（ODOM_VALID 置位）且里程非陈旧时本属性才非 None，否则
        上层应回退死推。
        """
        s = self.state
        if s is None:
            return None
        if not (s.state_flags & StateFlag.ODOM_VALID):
            return None
        if not self._sample_is_fresh("state"):
            return None  # 遥测超龄，不返回陈旧里程
        return (s.left_mm, s.right_mm)

    @property
    def state_sample_token(self) -> int | None:
        """当前 STATE 的新鲜度令牌；只用于与前次令牌比较，32 位 uptime 会回绕。"""
        state = self.state
        if self.odom_mm is None or state is None:
            return None
        token = getattr(state, "uptime_ms", None)
        return token if isinstance(token, int) else None



