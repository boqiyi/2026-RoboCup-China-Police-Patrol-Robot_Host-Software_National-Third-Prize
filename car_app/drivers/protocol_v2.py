"""STM32 protocol-major-2 的纯字节编解码器。

本模块不打开串口、不启动线程，也不尝试兼容旧 XOR 帧。所有多字节字段
均为小端；帧长度和 CRC 的权威定义见 ``docs/STM32F103C8T6-protocol-v2-
firmware-handoff.md``。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag


SYNC = b"\xAA\x55"
MIN_LENGTH = 3
MAX_LENGTH = 64
MAX_PAYLOAD = MAX_LENGTH - 3
MAX_PARSER_BUFFER = 1024
PROTOCOL_MAJOR = 2
PROTOCOL_MINOR = 3
HAZARD_RANGE4_MIN_MINOR = 1
DIRECTIONAL_RANGE4_MIN_MINOR = 2
CULVERT_UPPER2_MIN_MINOR = 2
LIGHT_CONTROL_MIN_MINOR = 2
DIAGNOSTICS_MIN_MINOR = 3
INVALID_DISTANCE_MM = 0xFFFF
AGE_UNIT_MS = 10
YAW_TENTH_DEG_MIN = -1800
YAW_TENTH_DEG_MAX = 1799
ODOM_STATUS_ALLOWED_MASK = 0x07
ODOM_STATUS_IMU_NOT_READY = 0x04
DIAGNOSTICS_TOF_COUNT = 6
DIAGNOSTICS_PAYLOAD_FORMAT = "<QIIBBHhi6IB3s6BB"
DIAGNOSTICS_PAYLOAD_LENGTH = struct.calcsize(DIAGNOSTICS_PAYLOAD_FORMAT)
WHEEL_TRACE_PAYLOAD_FORMAT = "<QIIHHB13hH"
WHEEL_TRACE_PAYLOAD_LENGTH = struct.calcsize(WHEEL_TRACE_PAYLOAD_FORMAT)


class ProtocolV2Error(Exception):
    """协议 v2 的具名错误基类。"""


class ProtocolValueError(ProtocolV2Error, ValueError):
    """调用方试图编码超出线协议范围的值。"""


class ProtocolPayloadError(ProtocolV2Error):
    """CRC 正确的帧仍包含非法或矛盾 payload。"""


class MessageType(IntEnum):
    HELLO_REQ = 0x10
    CONTROL_TARGET = 0x11
    STOP = 0x12
    LIGHT_CONTROL = 0x13
    MOTION_ACTION = 0x14
    HELLO_RESP = 0x90
    STATE = 0x91
    ACK = 0x92
    HAZARD_STATE = 0x93
    DIRECTIONAL_RANGE4 = 0x94
    CULVERT_UPPER2 = 0x95
    LIGHT_STATE = 0x96
    DIAGNOSTICS = 0x97
    WHEEL_TRACE = 0x98


class Capability(IntFlag):
    MOTION = 1 << 0
    ODOM = 1 << 1
    RANGE4 = 1 << 2
    GIMBAL = 1 << 3
    STOP_ACK = 1 << 4
    HAZARD_RANGE4 = 1 << 5
    DIRECTIONAL_RANGE4 = 1 << 6
    CULVERT_UPPER2 = 1 << 7
    LIGHT_CONTROL = 1 << 8
    DIAGNOSTICS = 1 << 9
    MOTION_ACTIONS = 1 << 10
    WHEEL_TRACE = 1 << 11


BASE_REQUIRED_CAPABILITIES = (
    Capability.MOTION
    | Capability.ODOM
    | Capability.RANGE4
    | Capability.GIMBAL
    | Capability.STOP_ACK
)

# 历史 2.2 能力档位仅供旧日志/测试数据解释，不参与当前应用握手。
V22_DIRECTIONAL4_CAPABILITIES = (
    Capability.MOTION
    | Capability.ODOM
    | Capability.STOP_ACK
    | Capability.DIRECTIONAL_RANGE4
)
V22_MOTION_BRINGUP_CAPABILITIES = V22_DIRECTIONAL4_CAPABILITIES

V22_CULVERT_UPPER2_CAPABILITIES = (
    V22_DIRECTIONAL4_CAPABILITIES | Capability.CULVERT_UPPER2
)

# 历史定义保留原始位值，不参与当前握手。
V22_REQUIRED_CAPABILITIES = (
    Capability.MOTION
    | Capability.ODOM
    | Capability.GIMBAL
    | Capability.STOP_ACK
    | Capability.DIRECTIONAL_RANGE4
    | Capability.CULVERT_UPPER2
    | Capability.LIGHT_CONTROL
)
V22_ALLOWED_STAGE_CAPABILITIES = frozenset(
    {
        int(V22_DIRECTIONAL4_CAPABILITIES),
        int(V22_CULVERT_UPPER2_CAPABILITIES),
        int(V22_REQUIRED_CAPABILITIES),
    }
)
# v9 保留普通 2.3 会话 0x6DB 的线格式；应用统一协商轮速诊断 0xEDB。
# 补光 bit8 不可用，动作 bit10 必须协商；bit11 不增加运动健康门禁。
V23_BASE_CAPABILITIES = (
    Capability.MOTION | Capability.ODOM | Capability.GIMBAL
    | Capability.STOP_ACK | Capability.DIRECTIONAL_RANGE4
    | Capability.CULVERT_UPPER2 | Capability.DIAGNOSTICS
    | Capability.MOTION_ACTIONS
)
V23_REQUIRED_CAPABILITIES = V23_BASE_CAPABILITIES | Capability.WHEEL_TRACE
REQUIRED_CAPABILITIES = V23_REQUIRED_CAPABILITIES


class ControlFlag(IntFlag):
    """CONTROL_TARGET 的独立字段有效位。

    Gimbal-only 帧不得置 MOTION_VALID，因此不会更新底盘目标，也不得给
    MCU 的运动看门狗续期。
    """

    MOTION_VALID = 1 << 0
    GIMBAL_VALID = 1 << 1
    GIMBAL_PWM_ENABLE = 1 << 2


class StateFlag(IntFlag):
    ODOM_VALID = 1 << 0
    GIMBAL_PRESENT = 1 << 1
    GIMBAL_PWM_ENABLED = 1 << 2
    ACK_PRESENT = 1 << 3
    APPLIED_CONTROL_PRESENT = 1 << 4
    TURN_ACTIVE = 1 << 5
    TURN_COMPLETED = 1 << 6
    TURN_ABORTED = 1 << 7


class WheelTraceFlag(IntFlag):
    HEADING_ACTIVE = 1 << 0
    HEADING_LIMITED = 1 << 1
    LEFT_PI_LIMITED = 1 << 2
    RIGHT_PI_LIMITED = 1 << 3
    DIRECTION_DEADTIME = 1 << 4
    DRIVER_ARMED = 1 << 5
    MOTION_ALLOWED = 1 << 6


class MotionActionCode(IntEnum):
    STRAIGHT = 1
    BRAKE = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4
    UTURN_LEFT = 5


class MotionActionFlag(IntFlag):
    BRAKE_AFTER_TURN = 1 << 0


class AckStatus(IntEnum):
    ACCEPTED = 0
    REJECTED_SESSION = 1
    REJECTED_SEQUENCE = 2
    REJECTED_FLAGS = 3
    REJECTED_LIMIT = 4
    REJECTED_STATE = 5
    REJECTED_PAYLOAD = 6


class StopReason(IntEnum):
    UNSPECIFIED = 0
    MISSION = 1
    CULVERT = 2
    FAULT = 3
    LINK_LOSS = 4
    CLOSE = 5
    USER = 6


class WireRangeStatus(IntEnum):
    VALID = 0
    NO_TARGET = 1
    OUT_OF_RANGE = 2
    SIGNAL_ERROR = 3
    TIMEOUT = 4
    BUS_ERROR = 5
    STALE = 6


class WireGyroStatus(IntEnum):
    """0x97 的原始陀螺健康状态；与固件 GyroStatus 的数值冻结一致。"""

    NOT_INITIALIZED = 0
    WHO_AM_I_ERROR = 1
    UNCALIBRATED = 2
    CALIBRATING = 3
    READY = 4
    BUS_ERROR = 5
    SAMPLE_GAP = 6


class WireTofInternalStatus(IntEnum):
    """0x97 的内部 ToF 状态；NO_DEVICE/UNCALIBRATED 不会压缩为旧 range 状态。"""

    VALID = 0
    NO_TARGET = 1
    OUT_OF_RANGE = 2
    SIGNAL_ERROR = 3
    TIMEOUT = 4
    BUS_ERROR = 5
    STALE = 6
    NO_DEVICE = 7
    UNCALIBRATED = 8


class WireSafetyStopReason(IntEnum):
    """0x97 的 SafetyStopReason；与上位机发出的 STOP.reason 是两个独立域。"""

    NONE = 0
    BOOT = 1
    HOST = 2
    FRONT_OBSTACLE = 3
    REAR_OBSTACLE = 4
    RANGE_INVALID = 5
    UART_FAULT = 6
    LEASE_EXPIRED = 7
    IMU_FAULT = 8
    FATAL = 9
    ENCODER_FAULT = 10
    CONTROL_TIMING = 11
    TURN_SETTLE_ERROR = 12


class WireGimbalPose(IntEnum):
    NAV_45 = 0
    LEVEL_CENTER = 1
    LEVEL_LEFT = 2
    LEVEL_RIGHT = 3
    UNKNOWN = 0xFF


class WireGimbalMotionState(IntEnum):
    UNKNOWN = 0
    DISABLED = 1
    ESTIMATING = 2
    ESTIMATED_DWELL_COMPLETE = 3
    FAULT = 4
    STALE = 5


class WireLightState(IntEnum):
    """0x96 的实际补光状态；非 APPLIED 状态必须回读 duty=0。"""

    DISABLED = 0
    APPLIED = 1
    FAULT = 2
    STALE = 3


class SequenceDisposition(IntEnum):
    DUPLICATE = 0
    NEWER = 1
    OLDER_OR_AMBIGUOUS = 2


@dataclass(frozen=True, slots=True)
class ProtocolFrame:
    message_type: MessageType
    sequence: int
    payload: bytes

    def __post_init__(self) -> None:
        if not isinstance(self.message_type, MessageType):
            raise ProtocolValueError("ProtocolFrame.message_type 必须是已冻结的 MessageType")
        _uint(self.sequence, 16, "ProtocolFrame.sequence")
        if not isinstance(self.payload, bytes):
            raise ProtocolValueError("ProtocolFrame.payload 必须是 bytes")
        if len(self.payload) > MAX_PAYLOAD:
            raise ProtocolValueError("ProtocolFrame.payload 超过 61 字节")


@dataclass(frozen=True, slots=True)
class ParserIssue:
    code: str
    detail: str


@dataclass(frozen=True, slots=True)
class HelloRequest:
    major: int
    minor: int
    session_nonce: int
    required_capabilities: int


@dataclass(frozen=True, slots=True)
class HelloResponse:
    session_nonce: int
    major: int
    minor: int
    capabilities: int
    boot_id: int
    reset_reason: int
    state_period_ms: int
    build_id: str


@dataclass(frozen=True, slots=True)
class ControlTarget:
    session_nonce: int
    flags: ControlFlag
    linear_mms: int
    angular_mrads: int
    aim_id: int
    pose_id: WireGimbalPose
    pan_pwm_us: int
    tilt_pwm_us: int


@dataclass(frozen=True, slots=True)
class StopCommand:
    session_nonce: int
    reason: int


@dataclass(frozen=True, slots=True)
class MotionAction:
    session_nonce: int
    action: MotionActionCode
    flags: MotionActionFlag
    speed_mms: int
    turn_angle_tenth_deg: int


@dataclass(frozen=True, slots=True)
class AckMessage:
    session_nonce: int
    acknowledged_type: MessageType
    acknowledged_sequence: int
    status: AckStatus
    reason: int
    boot_id: int


@dataclass(frozen=True, slots=True)
class StateTelemetry:
    session_nonce: int
    boot_id: int
    uptime_ms: int
    left_mm: int
    right_mm: int
    yaw_tenth_deg: int
    odom_status: int
    range_cycle_seq: int
    range_distance_mm: tuple[int, int, int, int]
    range_status: tuple[WireRangeStatus, WireRangeStatus, WireRangeStatus, WireRangeStatus]
    range_age_ms: tuple[int, int, int, int]
    gimbal_aim_id: int
    gimbal_pose: WireGimbalPose
    gimbal_motion_state: WireGimbalMotionState
    state_flags: StateFlag
    ack_sequence: int | None
    applied_control_sequence: int | None
    target_linear_mms: int
    target_angular_mrads: int
    fault_bits: int


@dataclass(frozen=True, slots=True)
class HazardStateTelemetry:
    """独立 0x93 帧；固定顺序为 FL/FR/RL/RR。"""

    session_nonce: int
    boot_id: int
    uptime_ms: int
    hazard_cycle_seq: int
    distance_mm: tuple[int, int, int, int]
    status: tuple[WireRangeStatus, WireRangeStatus, WireRangeStatus, WireRangeStatus]
    age_ms: tuple[int, int, int, int]


@dataclass(frozen=True, slots=True)
class DirectionalRange4Telemetry:
    """0x94 四槽：前空槽、左下 CH1、右下 CH2、后空槽。"""

    session_nonce: int
    boot_id: int
    uptime_ms: int
    directional_cycle_seq: int
    calibration_profile_version: int
    distance_mm: tuple[int, int, int, int]
    status: tuple[WireRangeStatus, WireRangeStatus, WireRangeStatus, WireRangeStatus]
    age_ms: tuple[int, int, int, int]


@dataclass(frozen=True, slots=True)
class CulvertUpper2Telemetry:
    """0x95 两槽：左上 CH4、右上 CH5。"""

    session_nonce: int
    boot_id: int
    uptime_ms: int
    upper_cycle_seq: int
    calibration_profile_version: int
    distance_mm: tuple[int, int]
    status: tuple[WireRangeStatus, WireRangeStatus]
    age_ms: tuple[int, int]


@dataclass(frozen=True, slots=True)
class LightControl:
    """RDK→STM32 补光命令；duty_permille 的闭区间为 0..1000。"""

    session_nonce: int
    duty_permille: int


@dataclass(frozen=True, slots=True)
class LightState:
    """STM32→RDK 补光回读；ACK 与实际回读必须分别验证。"""

    session_nonce: int
    boot_id: int
    uptime_ms: int
    applied_light_sequence: int
    duty_permille: int
    state: WireLightState
    fault_bits: int


@dataclass(frozen=True, slots=True)
class DiagnosticsTelemetry:
    """protocol 2.3 的 0x97 原始诊断帧，ToF 顺序固定为前、左下、右下、后、左上、右上。"""

    session_nonce: int
    boot_id: int
    uptime_ms: int
    gyro_status: WireGyroStatus
    gyro_calibration_count: int
    gyro_sample_age_ms: int
    gyro_raw_z: int
    gyro_bias_lsb: int
    i2c_transaction_count: int
    i2c_retry_count: int
    i2c_recovery_count: int
    i2c_nack_count: int
    i2c_timeout_count: int
    i2c_bus_error_count: int
    tof_initialized_mask: int
    tof_internal_status: tuple[WireTofInternalStatus, ...]
    tof_raw_status: tuple[int, ...]
    last_stop_reason: WireSafetyStopReason


@dataclass(frozen=True, slots=True)
class WheelTraceTelemetry:
    """v9 的 0x98 原始采样；目标计数按 10 ms，输出是归一化 PWM 命令。"""

    session_nonce: int
    boot_id: int
    uptime_ms: int
    sample_sequence: int
    applied_control_sequence: int
    elapsed_ms: int
    left_encoder_delta: int
    right_encoder_delta: int
    left_target_counts: int
    right_target_counts: int
    left_target_command: int
    right_target_command: int
    left_pi_output: int
    right_pi_output: int
    left_applied_output: int
    right_applied_output: int
    reference_yaw_tenth_deg: int
    yaw_tenth_deg: int
    heading_correction: int
    flags: WheelTraceFlag


def _uint(value: object, bits: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolValueError(f"{name} 必须是 uint{bits}")
    maximum = (1 << bits) - 1
    if not 0 <= value <= maximum:
        raise ProtocolValueError(f"{name} 必须是 uint{bits}")
    return value


def _int16(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not -32768 <= value <= 32767:
        raise ProtocolValueError(f"{name} 必须是 int16")
    return value


def _int32(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not -2147483648 <= value <= 2147483647:
        raise ProtocolValueError(f"{name} 必须是 int32")
    return value


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE；标准向量 ``123456789`` 必须得到 0x29B1。"""

    if not isinstance(data, bytes):
        raise TypeError("CRC 输入必须是 bytes")
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(
    message_type: MessageType,
    sequence: int,
    payload: bytes = b"",
) -> bytes:
    if not isinstance(message_type, MessageType):
        raise ProtocolValueError("未知 TYPE；protocol v2 不允许扩展前静默发送")
    seq = _uint(sequence, 16, "sequence")
    if not isinstance(payload, bytes):
        raise ProtocolValueError("payload 必须是 bytes")
    length = 3 + len(payload)
    if not MIN_LENGTH <= length <= MAX_LENGTH:
        raise ProtocolValueError("LEN 必须在 3..64；请检查 payload 大小")
    body = bytes((length, int(message_type))) + struct.pack("<H", seq) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt_false(body))


def classify_sequence(candidate: int, reference: int) -> SequenceDisposition:
    """按 16 位半范围规则比较序号；差值 0x8000 视为不可信旧帧。"""

    current = _uint(candidate, 16, "candidate")
    previous = _uint(reference, 16, "reference")
    delta = (current - previous) & 0xFFFF
    if delta == 0:
        return SequenceDisposition.DUPLICATE
    if delta < 0x8000:
        return SequenceDisposition.NEWER
    return SequenceDisposition.OLDER_OR_AMBIGUOUS


class ProtocolV2Parser:
    """可增量恢复的有界解析器；坏帧只产生 issue，不会变成业务帧。"""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._issues: list[ParserIssue] = []

    def clear(self) -> None:
        self._buffer.clear()
        self._issues.clear()

    def pop_issues(self) -> tuple[ParserIssue, ...]:
        result = tuple(self._issues)
        self._issues.clear()
        return result

    def _issue(self, code: str, detail: str) -> None:
        self._issues.append(ParserIssue(code, detail))
        if len(self._issues) > 64:
            del self._issues[:-64]

    def feed(self, data: bytes) -> tuple[ProtocolFrame, ...]:
        if not isinstance(data, bytes):
            raise TypeError("ProtocolV2Parser.feed 只接受 bytes")
        if data:
            self._buffer.extend(data)
        if len(self._buffer) > MAX_PARSER_BUFFER:
            dropped = len(self._buffer) - MAX_PARSER_BUFFER
            del self._buffer[:dropped]
            self._issue("BUFFER_LIMIT", f"解析缓存超限，丢弃 {dropped} 字节")

        frames: list[ProtocolFrame] = []
        while True:
            marker = self._buffer.find(SYNC)
            if marker < 0:
                keep = 1 if self._buffer.endswith(SYNC[:1]) else 0
                dropped = len(self._buffer) - keep
                if dropped:
                    del self._buffer[:dropped]
                    self._issue("NOISE", f"同步头前丢弃 {dropped} 字节")
                break
            if marker:
                del self._buffer[:marker]
                self._issue("NOISE", f"同步头前丢弃 {marker} 字节")
            if len(self._buffer) < 3:
                break

            length = self._buffer[2]
            if not MIN_LENGTH <= length <= MAX_LENGTH:
                self._issue("BAD_LEN", f"LEN={length} 不在 3..64")
                del self._buffer[0]
                continue
            total = 2 + 1 + length + 2
            if len(self._buffer) < total:
                break

            raw = bytes(self._buffer[:total])
            body = raw[2:-2]
            received_crc = struct.unpack("<H", raw[-2:])[0]
            expected_crc = crc16_ccitt_false(body)
            if received_crc != expected_crc:
                self._issue(
                    "BAD_CRC",
                    f"CRC不匹配 received=0x{received_crc:04X} expected=0x{expected_crc:04X}",
                )
                del self._buffer[0]
                continue

            type_value = raw[3]
            try:
                message_type = MessageType(type_value)
            except ValueError:
                self._issue("UNKNOWN_TYPE", f"TYPE=0x{type_value:02X} 未冻结")
                del self._buffer[:total]
                continue
            sequence = struct.unpack("<H", raw[4:6])[0]
            frames.append(ProtocolFrame(message_type, sequence, raw[6:-2]))
            del self._buffer[:total]
        return tuple(frames)


def _validate_current_hello_request(value: HelloRequest) -> None:
    major = _uint(value.major, 8, "HELLO_REQ.major")
    minor = _uint(value.minor, 8, "HELLO_REQ.minor")
    _uint(value.session_nonce, 64, "HELLO_REQ.session_nonce")
    caps = _uint(value.required_capabilities, 32, "HELLO_REQ.required_capabilities")
    if (major, minor) != (PROTOCOL_MAJOR, PROTOCOL_MINOR):
        raise ProtocolValueError(
            f"HELLO_REQ 只接受 protocol {PROTOCOL_MAJOR}.{PROTOCOL_MINOR}"
        )
    if caps not in (int(V23_BASE_CAPABILITIES), int(REQUIRED_CAPABILITIES)):
        raise ProtocolValueError("HELLO_REQ.required_capabilities 必须是固件能力 0x6DB 或 0xEDB")


def encode_hello_request(value: HelloRequest) -> bytes:
    _validate_current_hello_request(value)
    return struct.pack(
        "<BBQI",
        value.major,
        value.minor,
        value.session_nonce,
        value.required_capabilities,
    )


def decode_hello_request(payload: bytes) -> HelloRequest:
    if len(payload) != struct.calcsize("<BBQI"):
        raise ProtocolPayloadError("HELLO_REQ payload 长度必须是 14")
    major, minor, nonce, caps = struct.unpack("<BBQI", payload)
    value = HelloRequest(major, minor, nonce, caps)
    try:
        _validate_current_hello_request(value)
    except ProtocolValueError as exc:
        raise ProtocolPayloadError(f"HELLO_REQ 字段非法：{exc}") from exc
    return value


def _validate_current_hello_response(value: HelloResponse) -> None:
    major = _uint(value.major, 8, "HELLO_RESP.major")
    minor = _uint(value.minor, 8, "HELLO_RESP.minor")
    _uint(value.session_nonce, 64, "HELLO_RESP.session_nonce")
    caps = _uint(value.capabilities, 32, "HELLO_RESP.capabilities")
    _uint(value.boot_id, 32, "HELLO_RESP.boot_id")
    _uint(value.reset_reason, 8, "HELLO_RESP.reset_reason")
    _uint(value.state_period_ms, 16, "HELLO_RESP.state_period_ms")
    if (major, minor) != (PROTOCOL_MAJOR, PROTOCOL_MINOR):
        raise ProtocolValueError(
            f"HELLO_RESP 固件版本 {major}.{minor} 不兼容；"
            f"当前仅支持 protocol {PROTOCOL_MAJOR}.{PROTOCOL_MINOR}，"
            "其他版本须依据实际固件确认协议后接入"
        )
    if caps not in (int(V23_BASE_CAPABILITIES), int(REQUIRED_CAPABILITIES)):
        raise ProtocolValueError(
            f"HELLO_RESP.capabilities=0x{caps:08X} 不属于固件能力 0x6DB / 0xEDB"
        )


def encode_hello_response(value: HelloResponse) -> bytes:
    _validate_current_hello_response(value)
    if not isinstance(value.build_id, str) or not value.build_id or value.build_id != value.build_id.strip():
        raise ProtocolValueError("HELLO_RESP.build_id 必须是非空无首尾空白文本")
    try:
        build = value.build_id.encode("utf-8")
    except UnicodeEncodeError as exc:
        raise ProtocolValueError("HELLO_RESP.build_id 不是有效 UTF-8") from exc
    if b"\x00" in build or len(build) > 24:
        raise ProtocolValueError("HELLO_RESP.build_id 最多 24 字节且不得含 NUL")
    prefix = struct.pack(
        "<QBBIIBHB",
        value.session_nonce,
        value.major,
        value.minor,
        value.capabilities,
        value.boot_id,
        value.reset_reason,
        value.state_period_ms,
        len(build),
    )
    return prefix + build


def decode_hello_response(payload: bytes) -> HelloResponse:
    prefix_size = struct.calcsize("<QBBIIBHB")
    if len(payload) < prefix_size:
        raise ProtocolPayloadError("HELLO_RESP payload 缺固定头")
    nonce, major, minor, caps, boot_id, reset, period, build_len = struct.unpack(
        "<QBBIIBHB", payload[:prefix_size]
    )
    if build_len == 0 or build_len > 24 or len(payload) != prefix_size + build_len:
        raise ProtocolPayloadError("HELLO_RESP build_id 长度字段非法")
    try:
        build = payload[prefix_size:].decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ProtocolPayloadError("HELLO_RESP build_id 不是 UTF-8") from exc
    if "\x00" in build or not build or build != build.strip():
        raise ProtocolPayloadError("HELLO_RESP build_id 含 NUL/空白或为空")
    value = HelloResponse(nonce, major, minor, caps, boot_id, reset, period, build)
    try:
        _validate_current_hello_response(value)
    except ProtocolValueError as exc:
        raise ProtocolPayloadError(f"HELLO_RESP 字段非法：{exc}") from exc
    return value


def _validate_control(value: ControlTarget) -> None:
    _uint(value.session_nonce, 64, "CONTROL_TARGET.session_nonce")
    try:
        flags = ControlFlag(value.flags)
    except (TypeError, ValueError) as exc:
        raise ProtocolValueError("CONTROL_TARGET.flags 非法") from exc
    allowed = int(ControlFlag.MOTION_VALID | ControlFlag.GIMBAL_VALID | ControlFlag.GIMBAL_PWM_ENABLE)
    if int(flags) == 0 or int(flags) & ~allowed:
        raise ProtocolValueError("CONTROL_TARGET.flags 含保留位或没有有效字段")
    _int16(value.linear_mms, "CONTROL_TARGET.linear_mms")
    _int16(value.angular_mrads, "CONTROL_TARGET.angular_mrads")
    _uint(value.aim_id, 16, "CONTROL_TARGET.aim_id")
    if not isinstance(value.pose_id, WireGimbalPose):
        raise ProtocolValueError("CONTROL_TARGET.pose_id 非法")
    _uint(value.pan_pwm_us, 16, "CONTROL_TARGET.pan_pwm_us")
    _uint(value.tilt_pwm_us, 16, "CONTROL_TARGET.tilt_pwm_us")
    if not flags & ControlFlag.MOTION_VALID and (value.linear_mms or value.angular_mrads):
        raise ProtocolValueError("未置 MOTION_VALID 时 v/w 必须为零")
    if flags & ControlFlag.GIMBAL_PWM_ENABLE and not flags & ControlFlag.GIMBAL_VALID:
        raise ProtocolValueError("GIMBAL_PWM_ENABLE 必须与 GIMBAL_VALID 同时出现")
    if not flags & ControlFlag.GIMBAL_VALID:
        if value.aim_id or value.pose_id is not WireGimbalPose.UNKNOWN or value.pan_pwm_us or value.tilt_pwm_us:
            raise ProtocolValueError("未置 GIMBAL_VALID 时云台字段必须为零/UNKNOWN")
    elif flags & ControlFlag.GIMBAL_PWM_ENABLE:
        if value.pose_id is WireGimbalPose.UNKNOWN or value.pan_pwm_us == 0 or value.tilt_pwm_us == 0:
            raise ProtocolValueError("启用云台 PWM 时必须给出已知姿态和非零脉宽")
    elif value.pose_id is not WireGimbalPose.UNKNOWN or value.pan_pwm_us or value.tilt_pwm_us:
        raise ProtocolValueError("关闭云台 PWM 时姿态必须 UNKNOWN 且脉宽必须为零")


def encode_control_target(value: ControlTarget) -> bytes:
    _validate_control(value)
    return struct.pack(
        "<QBhhHBHH",
        value.session_nonce,
        int(value.flags),
        value.linear_mms,
        value.angular_mrads,
        value.aim_id,
        int(value.pose_id),
        value.pan_pwm_us,
        value.tilt_pwm_us,
    )


def decode_control_target(payload: bytes) -> ControlTarget:
    if len(payload) != struct.calcsize("<QBhhHBHH"):
        raise ProtocolPayloadError("CONTROL_TARGET payload 长度必须是 20")
    nonce, flags_raw, linear, angular, aim_id, pose_raw, pan, tilt = struct.unpack(
        "<QBhhHBHH", payload
    )
    try:
        flags = ControlFlag(flags_raw)
        pose = WireGimbalPose(pose_raw)
        value = ControlTarget(nonce, flags, linear, angular, aim_id, pose, pan, tilt)
        _validate_control(value)
    except (ProtocolValueError, ValueError) as exc:
        raise ProtocolPayloadError(f"CONTROL_TARGET 字段非法：{exc}") from exc
    return value


def _validate_motion_action(value: MotionAction) -> None:
    _uint(value.session_nonce, 64, "MOTION_ACTION.session_nonce")
    if not isinstance(value.action, MotionActionCode):
        raise ProtocolValueError("MOTION_ACTION.action 非法")
    flags = _uint(value.flags, 8, "MOTION_ACTION.flags")
    if flags & ~int(MotionActionFlag.BRAKE_AFTER_TURN):
        raise ProtocolValueError("MOTION_ACTION.flags 含保留位")
    speed = _uint(value.speed_mms, 16, "MOTION_ACTION.speed_mms")
    angle = _uint(value.turn_angle_tenth_deg, 16, "MOTION_ACTION.turn_angle_tenth_deg")
    if value.action is MotionActionCode.BRAKE:
        valid = flags == 0 and speed == 0 and angle == 0
    elif value.action is MotionActionCode.STRAIGHT:
        valid = flags == 0 and speed > 0 and angle == 0
    else:
        expected = 1800 if value.action is MotionActionCode.UTURN_LEFT else 900
        valid = speed > 0 and angle == expected
    if not valid:
        raise ProtocolValueError("MOTION_ACTION 参数与动作不匹配")


def encode_motion_action(value: MotionAction) -> bytes:
    _validate_motion_action(value)
    return struct.pack(
        "<QBBHH", value.session_nonce, int(value.action), int(value.flags),
        value.speed_mms, value.turn_angle_tenth_deg,
    )


def decode_motion_action(payload: bytes) -> MotionAction:
    if len(payload) != 14:
        raise ProtocolPayloadError("MOTION_ACTION payload 长度必须是 14")
    nonce, action_raw, flags_raw, speed, angle = struct.unpack("<QBBHH", payload)
    try:
        action = MotionActionCode(action_raw)
        value = MotionAction(nonce, action, MotionActionFlag(flags_raw), speed, angle)
        _validate_motion_action(value)
    except (ValueError, ProtocolValueError) as exc:
        raise ProtocolPayloadError(f"MOTION_ACTION 字段非法：{exc}") from exc
    return value


def encode_stop(value: StopCommand) -> bytes:
    return struct.pack(
        "<QB",
        _uint(value.session_nonce, 64, "STOP.session_nonce"),
        _uint(value.reason, 8, "STOP.reason"),
    )


def decode_stop(payload: bytes) -> StopCommand:
    if len(payload) != struct.calcsize("<QB"):
        raise ProtocolPayloadError("STOP payload 长度必须是 9")
    nonce, reason = struct.unpack("<QB", payload)
    return StopCommand(nonce, reason)


def encode_ack(value: AckMessage) -> bytes:
    if not isinstance(value.acknowledged_type, MessageType):
        raise ProtocolValueError("ACK.acknowledged_type 必须是已知 TYPE")
    if value.acknowledged_type not in {
        MessageType.CONTROL_TARGET,
        MessageType.STOP,
        MessageType.LIGHT_CONTROL,
        MessageType.MOTION_ACTION,
    }:
        raise ProtocolValueError("ACK 只允许确认已知命令")
    if not isinstance(value.status, AckStatus):
        raise ProtocolValueError("ACK.status 必须是 AckStatus")
    return struct.pack(
        "<QBHBBI",
        _uint(value.session_nonce, 64, "ACK.session_nonce"),
        int(value.acknowledged_type),
        _uint(value.acknowledged_sequence, 16, "ACK.acknowledged_sequence"),
        int(value.status),
        _uint(value.reason, 8, "ACK.reason"),
        _uint(value.boot_id, 32, "ACK.boot_id"),
    )


def decode_ack(payload: bytes) -> AckMessage:
    if len(payload) != struct.calcsize("<QBHBBI"):
        raise ProtocolPayloadError("ACK payload 长度必须是 17")
    nonce, type_raw, sequence, status_raw, reason, boot_id = struct.unpack("<QBHBBI", payload)
    try:
        message_type = MessageType(type_raw)
        status = AckStatus(status_raw)
    except ValueError as exc:
        raise ProtocolPayloadError("ACK 含未知 TYPE/status") from exc
    if message_type not in {
        MessageType.CONTROL_TARGET,
        MessageType.STOP,
        MessageType.LIGHT_CONTROL,
        MessageType.MOTION_ACTION,
    }:
        raise ProtocolPayloadError("ACK 只允许确认已知命令")
    return AckMessage(nonce, message_type, sequence, status, reason, boot_id)


def _pack_tof_statuses(
    statuses: tuple[WireRangeStatus, ...],
    *,
    count: int,
    label: str,
) -> int:
    if len(statuses) != count:
        raise ProtocolValueError(f"{label}.status 必须恰含 {count} 路")
    packed = 0
    for index, status in enumerate(statuses):
        if not isinstance(status, WireRangeStatus):
            raise ProtocolValueError(f"{label}.status[{index}] 非法")
        packed |= int(status) << (index * 3)
    return packed


def _unpack_tof_statuses(
    packed: int,
    *,
    count: int,
    label: str,
) -> tuple[WireRangeStatus, ...]:
    used_bits = count * 3
    if packed >> used_bits:
        raise ProtocolPayloadError(f"{label} status 保留高位必须为零")
    result: list[WireRangeStatus] = []
    for index in range(count):
        raw = (packed >> (index * 3)) & 0x07
        try:
            result.append(WireRangeStatus(raw))
        except ValueError as exc:
            raise ProtocolPayloadError(
                f"{label} 第 {index} 路 status=7 未定义"
            ) from exc
    return tuple(result)


def _validate_tof_samples(
    *,
    distance_mm: tuple[int, ...],
    status: tuple[WireRangeStatus, ...],
    age_ms: tuple[int, ...],
    count: int,
    label: str,
) -> None:
    if len(distance_mm) != count or len(status) != count or len(age_ms) != count:
        raise ProtocolValueError(f"{label} 距离/status/age 必须各恰含 {count} 路")
    _pack_tof_statuses(status, count=count, label=label)
    for index, (distance, sample_status, sample_age_ms) in enumerate(
        zip(distance_mm, status, age_ms, strict=True)
    ):
        _uint(distance, 16, f"{label}.distance_mm[{index}]")
        _uint(sample_age_ms, 16, f"{label}.age_ms[{index}]")
        if sample_status is WireRangeStatus.VALID and distance == INVALID_DISTANCE_MM:
            raise ProtocolValueError(f"{label} 有效测距不得使用 0xFFFF")
        if sample_status is not WireRangeStatus.VALID and distance != INVALID_DISTANCE_MM:
            raise ProtocolValueError(
                f"{label} 无效测距必须使用 0xFFFF，且由 status 解释原因"
            )


def _validate_directional_range4(value: DirectionalRange4Telemetry) -> None:
    _uint(value.session_nonce, 64, "DIRECTIONAL_RANGE4.session_nonce")
    _uint(value.boot_id, 32, "DIRECTIONAL_RANGE4.boot_id")
    _uint(value.uptime_ms, 32, "DIRECTIONAL_RANGE4.uptime_ms")
    _uint(value.directional_cycle_seq, 16, "DIRECTIONAL_RANGE4.directional_cycle_seq")
    _uint(
        value.calibration_profile_version,
        32,
        "DIRECTIONAL_RANGE4.calibration_profile_version",
    )
    _validate_tof_samples(
        distance_mm=value.distance_mm,
        status=value.status,
        age_ms=value.age_ms,
        count=4,
        label="DIRECTIONAL_RANGE4",
    )


def encode_directional_range4(value: DirectionalRange4Telemetry) -> bytes:
    _validate_directional_range4(value)
    payload = struct.pack(
        "<QIIHI4HH4H",
        value.session_nonce,
        value.boot_id,
        value.uptime_ms,
        value.directional_cycle_seq,
        value.calibration_profile_version,
        *value.distance_mm,
        _pack_tof_statuses(value.status, count=4, label="DIRECTIONAL_RANGE4"),
        *value.age_ms,
    )
    if len(payload) != 40:
        raise AssertionError("DIRECTIONAL_RANGE4 payload 内部布局必须冻结为 40 字节")
    return payload


def decode_directional_range4(payload: bytes) -> DirectionalRange4Telemetry:
    if len(payload) != 40:
        raise ProtocolPayloadError("DIRECTIONAL_RANGE4 payload 长度必须是 40")
    unpacked = struct.unpack("<QIIHI4HH4H", payload)
    value = DirectionalRange4Telemetry(
        session_nonce=unpacked[0],
        boot_id=unpacked[1],
        uptime_ms=unpacked[2],
        directional_cycle_seq=unpacked[3],
        calibration_profile_version=unpacked[4],
        distance_mm=tuple(unpacked[5:9]),  # type: ignore[arg-type]
        status=_unpack_tof_statuses(
            unpacked[9], count=4, label="DIRECTIONAL_RANGE4"
        ),  # type: ignore[arg-type]
        age_ms=tuple(unpacked[10:14]),  # type: ignore[arg-type]
    )
    try:
        _validate_directional_range4(value)
    except ProtocolValueError as exc:
        raise ProtocolPayloadError(f"DIRECTIONAL_RANGE4 字段非法：{exc}") from exc
    return value


def _validate_culvert_upper2(value: CulvertUpper2Telemetry) -> None:
    _uint(value.session_nonce, 64, "CULVERT_UPPER2.session_nonce")
    _uint(value.boot_id, 32, "CULVERT_UPPER2.boot_id")
    _uint(value.uptime_ms, 32, "CULVERT_UPPER2.uptime_ms")
    _uint(value.upper_cycle_seq, 16, "CULVERT_UPPER2.upper_cycle_seq")
    _uint(
        value.calibration_profile_version,
        32,
        "CULVERT_UPPER2.calibration_profile_version",
    )
    _validate_tof_samples(
        distance_mm=value.distance_mm,
        status=value.status,
        age_ms=value.age_ms,
        count=2,
        label="CULVERT_UPPER2",
    )


def encode_culvert_upper2(value: CulvertUpper2Telemetry) -> bytes:
    _validate_culvert_upper2(value)
    payload = struct.pack(
        "<QIIHI2HH2H",
        value.session_nonce,
        value.boot_id,
        value.uptime_ms,
        value.upper_cycle_seq,
        value.calibration_profile_version,
        *value.distance_mm,
        _pack_tof_statuses(value.status, count=2, label="CULVERT_UPPER2"),
        *value.age_ms,
    )
    if len(payload) != 32:
        raise AssertionError("CULVERT_UPPER2 payload 内部布局必须冻结为 32 字节")
    return payload


def decode_culvert_upper2(payload: bytes) -> CulvertUpper2Telemetry:
    if len(payload) != 32:
        raise ProtocolPayloadError("CULVERT_UPPER2 payload 长度必须是 32")
    unpacked = struct.unpack("<QIIHI2HH2H", payload)
    value = CulvertUpper2Telemetry(
        session_nonce=unpacked[0],
        boot_id=unpacked[1],
        uptime_ms=unpacked[2],
        upper_cycle_seq=unpacked[3],
        calibration_profile_version=unpacked[4],
        distance_mm=tuple(unpacked[5:7]),  # type: ignore[arg-type]
        status=_unpack_tof_statuses(
            unpacked[7], count=2, label="CULVERT_UPPER2"
        ),  # type: ignore[arg-type]
        age_ms=tuple(unpacked[8:10]),  # type: ignore[arg-type]
    )
    try:
        _validate_culvert_upper2(value)
    except ProtocolValueError as exc:
        raise ProtocolPayloadError(f"CULVERT_UPPER2 字段非法：{exc}") from exc
    return value


def encode_light_control(value: LightControl) -> bytes:
    nonce = _uint(value.session_nonce, 64, "LIGHT_CONTROL.session_nonce")
    duty_permille = _uint(
        value.duty_permille, 16, "LIGHT_CONTROL.duty_permille"
    )
    if duty_permille > 1000:
        raise ProtocolValueError("LIGHT_CONTROL.duty_permille 必须在 0..1000")
    return struct.pack("<QH", nonce, duty_permille)


def decode_light_control(payload: bytes) -> LightControl:
    if len(payload) != struct.calcsize("<QH"):
        raise ProtocolPayloadError("LIGHT_CONTROL payload 长度必须是 10")
    nonce, duty_permille = struct.unpack("<QH", payload)
    if duty_permille > 1000:
        raise ProtocolPayloadError("LIGHT_CONTROL duty_permille 必须在 0..1000")
    return LightControl(nonce, duty_permille)


def _validate_light_state(value: LightState) -> None:
    _uint(value.session_nonce, 64, "LIGHT_STATE.session_nonce")
    _uint(value.boot_id, 32, "LIGHT_STATE.boot_id")
    _uint(value.uptime_ms, 32, "LIGHT_STATE.uptime_ms")
    _uint(value.applied_light_sequence, 16, "LIGHT_STATE.applied_light_sequence")
    _uint(value.duty_permille, 16, "LIGHT_STATE.duty_permille")
    if value.duty_permille > 1000:
        raise ProtocolValueError("LIGHT_STATE.duty_permille 必须在 0..1000")
    if not isinstance(value.state, WireLightState):
        raise ProtocolValueError("LIGHT_STATE.state 必须是 WireLightState")
    if value.state is not WireLightState.APPLIED and value.duty_permille != 0:
        raise ProtocolValueError("LIGHT_STATE 非 APPLIED 状态必须回读 duty_permille=0")
    _uint(value.fault_bits, 32, "LIGHT_STATE.fault_bits")
    if value.state is WireLightState.FAULT and value.fault_bits == 0:
        raise ProtocolValueError("LIGHT_STATE FAULT 状态必须给出非零 fault_bits")
    if value.state is not WireLightState.FAULT and value.fault_bits != 0:
        raise ProtocolValueError("LIGHT_STATE 非 FAULT 状态不得携带 fault_bits")


def encode_light_state(value: LightState) -> bytes:
    _validate_light_state(value)
    payload = struct.pack(
        "<QIIHHBI",
        value.session_nonce,
        value.boot_id,
        value.uptime_ms,
        value.applied_light_sequence,
        value.duty_permille,
        int(value.state),
        value.fault_bits,
    )
    if len(payload) != 25:
        raise AssertionError("LIGHT_STATE payload 内部布局必须冻结为 25 字节")
    return payload


def decode_light_state(payload: bytes) -> LightState:
    if len(payload) != 25:
        raise ProtocolPayloadError("LIGHT_STATE payload 长度必须是 25")
    nonce, boot_id, uptime_ms, applied, duty, state_raw, faults = struct.unpack(
        "<QIIHHBI", payload
    )
    try:
        state = WireLightState(state_raw)
    except ValueError as exc:
        raise ProtocolPayloadError("LIGHT_STATE 含未知 state") from exc
    value = LightState(nonce, boot_id, uptime_ms, applied, duty, state, faults)
    try:
        _validate_light_state(value)
    except ProtocolValueError as exc:
        raise ProtocolPayloadError(f"LIGHT_STATE 字段非法：{exc}") from exc
    return value


def _pack_diagnostics_tof_statuses(
    statuses: tuple[WireTofInternalStatus, ...],
) -> bytes:
    if len(statuses) != DIAGNOSTICS_TOF_COUNT:
        raise ProtocolValueError("DIAGNOSTICS.tof_internal_status 必须恰含六路")
    packed = bytearray(3)
    for index, status in enumerate(statuses):
        if not isinstance(status, WireTofInternalStatus):
            raise ProtocolValueError(f"DIAGNOSTICS.tof_internal_status[{index}] 非法")
        packed[index // 2] |= int(status) << (4 * (index % 2))
    return bytes(packed)


def _unpack_diagnostics_tof_statuses(
    packed: bytes,
) -> tuple[WireTofInternalStatus, ...]:
    if len(packed) != 3:
        raise ProtocolPayloadError("DIAGNOSTICS ToF packed status 长度必须是 3")
    result: list[WireTofInternalStatus] = []
    for index in range(DIAGNOSTICS_TOF_COUNT):
        raw = (packed[index // 2] >> (4 * (index % 2))) & 0x0F
        try:
            result.append(WireTofInternalStatus(raw))
        except ValueError as exc:
            raise ProtocolPayloadError(
                f"DIAGNOSTICS 第 {index} 路内部 ToF status={raw} 未定义"
            ) from exc
    return tuple(result)


def _validate_diagnostics(value: DiagnosticsTelemetry) -> None:
    _uint(value.session_nonce, 64, "DIAGNOSTICS.session_nonce")
    _uint(value.boot_id, 32, "DIAGNOSTICS.boot_id")
    _uint(value.uptime_ms, 32, "DIAGNOSTICS.uptime_ms")
    if not isinstance(value.gyro_status, WireGyroStatus):
        raise ProtocolValueError("DIAGNOSTICS.gyro_status 必须是 WireGyroStatus")
    _uint(value.gyro_calibration_count, 8, "DIAGNOSTICS.gyro_calibration_count")
    _uint(value.gyro_sample_age_ms, 16, "DIAGNOSTICS.gyro_sample_age_ms")
    _int16(value.gyro_raw_z, "DIAGNOSTICS.gyro_raw_z")
    _int32(value.gyro_bias_lsb, "DIAGNOSTICS.gyro_bias_lsb")
    _uint(value.i2c_transaction_count, 32, "DIAGNOSTICS.i2c_transaction_count")
    _uint(value.i2c_retry_count, 32, "DIAGNOSTICS.i2c_retry_count")
    _uint(value.i2c_recovery_count, 32, "DIAGNOSTICS.i2c_recovery_count")
    _uint(value.i2c_nack_count, 32, "DIAGNOSTICS.i2c_nack_count")
    _uint(value.i2c_timeout_count, 32, "DIAGNOSTICS.i2c_timeout_count")
    _uint(value.i2c_bus_error_count, 32, "DIAGNOSTICS.i2c_bus_error_count")
    _uint(value.tof_initialized_mask, 8, "DIAGNOSTICS.tof_initialized_mask")
    if value.tof_initialized_mask & 0xC0:
        raise ProtocolValueError("DIAGNOSTICS.tof_initialized_mask 保留高两位必须为零")
    _pack_diagnostics_tof_statuses(value.tof_internal_status)
    if len(value.tof_raw_status) != DIAGNOSTICS_TOF_COUNT:
        raise ProtocolValueError("DIAGNOSTICS.tof_raw_status 必须恰含六路")
    for index, raw_status in enumerate(value.tof_raw_status):
        _uint(raw_status, 8, f"DIAGNOSTICS.tof_raw_status[{index}]")
    if not isinstance(value.last_stop_reason, WireSafetyStopReason):
        raise ProtocolValueError(
            "DIAGNOSTICS.last_stop_reason 必须是 WireSafetyStopReason"
        )


def encode_diagnostics(value: DiagnosticsTelemetry) -> bytes:
    _validate_diagnostics(value)
    payload = struct.pack(
        DIAGNOSTICS_PAYLOAD_FORMAT,
        value.session_nonce,
        value.boot_id,
        value.uptime_ms,
        int(value.gyro_status),
        value.gyro_calibration_count,
        value.gyro_sample_age_ms,
        value.gyro_raw_z,
        value.gyro_bias_lsb,
        value.i2c_transaction_count,
        value.i2c_retry_count,
        value.i2c_recovery_count,
        value.i2c_nack_count,
        value.i2c_timeout_count,
        value.i2c_bus_error_count,
        value.tof_initialized_mask,
        _pack_diagnostics_tof_statuses(value.tof_internal_status),
        *value.tof_raw_status,
        int(value.last_stop_reason),
    )
    if len(payload) != DIAGNOSTICS_PAYLOAD_LENGTH:
        raise AssertionError("DIAGNOSTICS payload 内部布局必须冻结为 61 字节")
    return payload


def decode_diagnostics(payload: bytes) -> DiagnosticsTelemetry:
    if len(payload) != DIAGNOSTICS_PAYLOAD_LENGTH:
        raise ProtocolPayloadError("DIAGNOSTICS payload 长度必须是 61")
    unpacked = struct.unpack(DIAGNOSTICS_PAYLOAD_FORMAT, payload)
    try:
        gyro_status = WireGyroStatus(unpacked[3])
        tof_status = _unpack_diagnostics_tof_statuses(unpacked[15])
        last_stop_reason = WireSafetyStopReason(unpacked[22])
        value = DiagnosticsTelemetry(
            session_nonce=unpacked[0],
            boot_id=unpacked[1],
            uptime_ms=unpacked[2],
            gyro_status=gyro_status,
            gyro_calibration_count=unpacked[4],
            gyro_sample_age_ms=unpacked[5],
            gyro_raw_z=unpacked[6],
            gyro_bias_lsb=unpacked[7],
            i2c_transaction_count=unpacked[8],
            i2c_retry_count=unpacked[9],
            i2c_recovery_count=unpacked[10],
            i2c_nack_count=unpacked[11],
            i2c_timeout_count=unpacked[12],
            i2c_bus_error_count=unpacked[13],
            tof_initialized_mask=unpacked[14],
            tof_internal_status=tof_status,
            tof_raw_status=tuple(unpacked[16:22]),
            last_stop_reason=last_stop_reason,
        )
        _validate_diagnostics(value)
    except (ProtocolValueError, ValueError) as exc:
        raise ProtocolPayloadError(f"DIAGNOSTICS 字段非法：{exc}") from exc
    return value


def _validate_wheel_trace(value: WheelTraceTelemetry) -> None:
    _uint(value.session_nonce, 64, "WHEEL_TRACE.session_nonce")
    _uint(value.boot_id, 32, "WHEEL_TRACE.boot_id")
    _uint(value.uptime_ms, 32, "WHEEL_TRACE.uptime_ms")
    _uint(value.sample_sequence, 16, "WHEEL_TRACE.sample_sequence")
    _uint(value.applied_control_sequence, 16, "WHEEL_TRACE.applied_control_sequence")
    elapsed = _uint(value.elapsed_ms, 8, "WHEEL_TRACE.elapsed_ms")
    if elapsed == 0:
        raise ProtocolValueError("WHEEL_TRACE.elapsed_ms 必须大于零")
    for name in (
        "left_encoder_delta", "right_encoder_delta",
        "left_target_counts", "right_target_counts",
    ):
        _int16(getattr(value, name), f"WHEEL_TRACE.{name}")
    for name in (
        "left_target_command", "right_target_command",
        "left_pi_output", "right_pi_output",
        "left_applied_output", "right_applied_output",
    ):
        command = _int16(getattr(value, name), f"WHEEL_TRACE.{name}")
        if not -1000 <= command <= 1000:
            raise ProtocolValueError(f"WHEEL_TRACE.{name} 必须在 -1000..1000")
    for name in ("reference_yaw_tenth_deg", "yaw_tenth_deg"):
        yaw = _int16(getattr(value, name), f"WHEEL_TRACE.{name}")
        if not YAW_TENTH_DEG_MIN <= yaw <= YAW_TENTH_DEG_MAX:
            raise ProtocolValueError(f"WHEEL_TRACE.{name} 必须在 -1800..1799")
    correction = _int16(value.heading_correction, "WHEEL_TRACE.heading_correction")
    if not -80 <= correction <= 80:
        raise ProtocolValueError("WHEEL_TRACE.heading_correction 必须在 -80..80")
    flags = _uint(value.flags, 16, "WHEEL_TRACE.flags")
    if flags & 0xFF80:
        raise ProtocolValueError("WHEEL_TRACE.flags 保留位必须为零")


def encode_wheel_trace(value: WheelTraceTelemetry) -> bytes:
    _validate_wheel_trace(value)
    return struct.pack(
        WHEEL_TRACE_PAYLOAD_FORMAT,
        value.session_nonce, value.boot_id, value.uptime_ms,
        value.sample_sequence, value.applied_control_sequence, value.elapsed_ms,
        value.left_encoder_delta, value.right_encoder_delta,
        value.left_target_counts, value.right_target_counts,
        value.left_target_command, value.right_target_command,
        value.left_pi_output, value.right_pi_output,
        value.left_applied_output, value.right_applied_output,
        value.reference_yaw_tenth_deg, value.yaw_tenth_deg,
        value.heading_correction, int(value.flags),
    )


def decode_wheel_trace(payload: bytes) -> WheelTraceTelemetry:
    if len(payload) != WHEEL_TRACE_PAYLOAD_LENGTH:
        raise ProtocolPayloadError("WHEEL_TRACE payload 长度必须是 49")
    unpacked = struct.unpack(WHEEL_TRACE_PAYLOAD_FORMAT, payload)
    value = WheelTraceTelemetry(*unpacked[:-1], WheelTraceFlag(unpacked[-1]))
    try:
        _validate_wheel_trace(value)
    except ProtocolValueError as exc:
        raise ProtocolPayloadError(f"WHEEL_TRACE 字段非法：{exc}") from exc
    return value


def _pack_hazard_statuses(statuses: tuple[WireRangeStatus, ...]) -> int:
    if len(statuses) != 4:
        raise ProtocolValueError("HAZARD_STATE.status 必须恰含 FL/FR/RL/RR 四路")
    packed = 0
    for index, status in enumerate(statuses):
        if not isinstance(status, WireRangeStatus):
            raise ProtocolValueError(f"HAZARD_STATE.status[{index}] 非法")
        packed |= int(status) << (index * 3)
    return packed


def _unpack_hazard_statuses(
    packed: int,
) -> tuple[WireRangeStatus, WireRangeStatus, WireRangeStatus, WireRangeStatus]:
    if packed & 0xF000:
        raise ProtocolPayloadError("HAZARD_STATE status 保留高四位必须为零")
    result: list[WireRangeStatus] = []
    for index in range(4):
        raw = (packed >> (index * 3)) & 0x07
        try:
            result.append(WireRangeStatus(raw))
        except ValueError as exc:
            raise ProtocolPayloadError(
                f"HAZARD_STATE 第 {index} 路 status=7 未定义"
            ) from exc
    return tuple(result)  # type: ignore[return-value]


def _validate_hazard_state(value: HazardStateTelemetry) -> None:
    _uint(value.session_nonce, 64, "HAZARD_STATE.session_nonce")
    _uint(value.boot_id, 32, "HAZARD_STATE.boot_id")
    _uint(value.uptime_ms, 32, "HAZARD_STATE.uptime_ms")
    _uint(value.hazard_cycle_seq, 16, "HAZARD_STATE.hazard_cycle_seq")
    if len(value.distance_mm) != 4 or len(value.status) != 4 or len(value.age_ms) != 4:
        raise ProtocolValueError("HAZARD_STATE 距离/status/age 必须各恰含四路")
    _pack_hazard_statuses(value.status)
    for index, (distance, status, age_ms) in enumerate(
        zip(value.distance_mm, value.status, value.age_ms, strict=True)
    ):
        _uint(distance, 16, f"HAZARD_STATE.distance_mm[{index}]")
        if (
            isinstance(age_ms, bool)
            or not isinstance(age_ms, int)
            or age_ms < 0
            or age_ms > 2550
            or age_ms % AGE_UNIT_MS
        ):
            raise ProtocolValueError(
                f"HAZARD_STATE.age_ms[{index}] 必须是 0..2550 的 10ms 倍数"
            )
        if status is WireRangeStatus.VALID and distance == INVALID_DISTANCE_MM:
            raise ProtocolValueError("HAZARD_STATE 有效测距不得使用 0xFFFF")
        if status is not WireRangeStatus.VALID and distance != INVALID_DISTANCE_MM:
            raise ProtocolValueError(
                "HAZARD_STATE 无效测距必须使用 0xFFFF，且由 status 解释原因"
            )


def encode_hazard_state(value: HazardStateTelemetry) -> bytes:
    _validate_hazard_state(value)
    statuses = _pack_hazard_statuses(value.status)
    ages = tuple(age // AGE_UNIT_MS for age in value.age_ms)
    payload = struct.pack(
        "<QIIH4HH4B",
        value.session_nonce,
        value.boot_id,
        value.uptime_ms,
        value.hazard_cycle_seq,
        *value.distance_mm,
        statuses,
        *ages,
    )
    if len(payload) != 32:
        raise AssertionError("HAZARD_STATE payload 内部布局必须冻结为 32 字节")
    return payload


def decode_hazard_state(payload: bytes) -> HazardStateTelemetry:
    if len(payload) != 32:
        raise ProtocolPayloadError("HAZARD_STATE payload 长度必须是 32")
    unpacked = struct.unpack("<QIIH4HH4B", payload)
    value = HazardStateTelemetry(
        session_nonce=unpacked[0],
        boot_id=unpacked[1],
        uptime_ms=unpacked[2],
        hazard_cycle_seq=unpacked[3],
        distance_mm=tuple(unpacked[4:8]),  # type: ignore[arg-type]
        status=_unpack_hazard_statuses(unpacked[8]),
        age_ms=tuple(item * AGE_UNIT_MS for item in unpacked[9:13]),  # type: ignore[arg-type]
    )
    try:
        _validate_hazard_state(value)
    except ProtocolValueError as exc:
        raise ProtocolPayloadError(f"HAZARD_STATE 字段非法：{exc}") from exc
    return value


def _pack_range_statuses(statuses: tuple[WireRangeStatus, ...]) -> int:
    if len(statuses) != 4:
        raise ProtocolValueError("STATE.range_status 必须恰含四路")
    packed = 0
    for index, status in enumerate(statuses):
        if not isinstance(status, WireRangeStatus):
            raise ProtocolValueError(f"STATE.range_status[{index}] 非法")
        packed |= int(status) << (index * 3)
    return packed


def _unpack_range_statuses(packed: int) -> tuple[WireRangeStatus, WireRangeStatus, WireRangeStatus, WireRangeStatus]:
    if packed & 0xF000:
        raise ProtocolPayloadError("STATE range status 保留高四位必须为零")
    result: list[WireRangeStatus] = []
    for index in range(4):
        raw = (packed >> (index * 3)) & 0x07
        try:
            result.append(WireRangeStatus(raw))
        except ValueError as exc:
            raise ProtocolPayloadError(f"STATE 第 {index} 路 range status=7 未定义") from exc
    return tuple(result)  # type: ignore[return-value]


def _validate_state(value: StateTelemetry) -> None:
    _uint(value.session_nonce, 64, "STATE.session_nonce")
    _uint(value.boot_id, 32, "STATE.boot_id")
    _uint(value.uptime_ms, 32, "STATE.uptime_ms")
    if isinstance(value.left_mm, bool) or not isinstance(value.left_mm, int) or not -(1 << 31) <= value.left_mm < (1 << 31):
        raise ProtocolValueError("STATE.left_mm 必须是 int32")
    if isinstance(value.right_mm, bool) or not isinstance(value.right_mm, int) or not -(1 << 31) <= value.right_mm < (1 << 31):
        raise ProtocolValueError("STATE.right_mm 必须是 int32")
    _int16(value.yaw_tenth_deg, "STATE.yaw_tenth_deg")
    if not YAW_TENTH_DEG_MIN <= value.yaw_tenth_deg <= YAW_TENTH_DEG_MAX:
        raise ProtocolValueError("STATE.yaw_tenth_deg 必须在 -1800..1799（0.1°）")
    odom_status = _uint(value.odom_status, 8, "STATE.odom_status")
    if odom_status & ~ODOM_STATUS_ALLOWED_MASK:
        raise ProtocolValueError("STATE.odom_status 含保留位 0xF8")
    _uint(value.range_cycle_seq, 16, "STATE.range_cycle_seq")
    if len(value.range_distance_mm) != 4 or len(value.range_status) != 4 or len(value.range_age_ms) != 4:
        raise ProtocolValueError("STATE 测距值/status/age 必须各恰含四路")
    _pack_range_statuses(value.range_status)
    for index, (distance, status, age_ms) in enumerate(
        zip(value.range_distance_mm, value.range_status, value.range_age_ms, strict=True)
    ):
        _uint(distance, 16, f"STATE.range_distance_mm[{index}]")
        if isinstance(age_ms, bool) or not isinstance(age_ms, int) or age_ms < 0 or age_ms > 2550 or age_ms % AGE_UNIT_MS:
            raise ProtocolValueError(f"STATE.range_age_ms[{index}] 必须是 0..2550 的 10ms 倍数")
        if status is WireRangeStatus.VALID and distance == INVALID_DISTANCE_MM:
            raise ProtocolValueError("VALID 测距不得使用 0xFFFF")
        if status is not WireRangeStatus.VALID and distance != INVALID_DISTANCE_MM:
            raise ProtocolValueError("无效测距必须使用 0xFFFF，且由 status 解释原因")
    _uint(value.gimbal_aim_id, 16, "STATE.gimbal_aim_id")
    if not isinstance(value.gimbal_pose, WireGimbalPose):
        raise ProtocolValueError("STATE.gimbal_pose 非法")
    if not isinstance(value.gimbal_motion_state, WireGimbalMotionState):
        raise ProtocolValueError("STATE.gimbal_motion_state 非法")
    try:
        flags = StateFlag(value.state_flags)
    except (TypeError, ValueError) as exc:
        raise ProtocolValueError("STATE.state_flags 非法") from exc
    allowed = int(
        StateFlag.ODOM_VALID
        | StateFlag.GIMBAL_PRESENT
        | StateFlag.GIMBAL_PWM_ENABLED
        | StateFlag.ACK_PRESENT
        | StateFlag.APPLIED_CONTROL_PRESENT
        | StateFlag.TURN_ACTIVE
        | StateFlag.TURN_COMPLETED
        | StateFlag.TURN_ABORTED
    )
    if int(flags) & ~allowed:
        raise ProtocolValueError("STATE.state_flags 含保留位")
    if sum(bool(flags & flag) for flag in (
        StateFlag.TURN_ACTIVE, StateFlag.TURN_COMPLETED, StateFlag.TURN_ABORTED
    )) > 1:
        raise ProtocolValueError("STATE 转向结果位必须互斥")
    odom_valid = bool(flags & StateFlag.ODOM_VALID)
    if odom_valid != (odom_status == 0):
        raise ProtocolValueError("ODOM_VALID 仅可与 odom_status=0 同时成立")
    if flags & StateFlag.GIMBAL_PWM_ENABLED and not flags & StateFlag.GIMBAL_PRESENT:
        raise ProtocolValueError("GIMBAL_PWM_ENABLED 必须与 GIMBAL_PRESENT 同时出现")
    if value.ack_sequence is None:
        if flags & StateFlag.ACK_PRESENT:
            raise ProtocolValueError("ACK_PRESENT 已置位但 ack_sequence=None")
    else:
        _uint(value.ack_sequence, 16, "STATE.ack_sequence")
        if not flags & StateFlag.ACK_PRESENT:
            raise ProtocolValueError("ack_sequence 非空但 ACK_PRESENT 未置位")
    if value.applied_control_sequence is None:
        if flags & StateFlag.APPLIED_CONTROL_PRESENT:
            raise ProtocolValueError("APPLIED_CONTROL_PRESENT 已置位但 sequence=None")
    else:
        _uint(value.applied_control_sequence, 16, "STATE.applied_control_sequence")
        if not flags & StateFlag.APPLIED_CONTROL_PRESENT:
            raise ProtocolValueError("applied_control_sequence 非空但标志未置位")
    _int16(value.target_linear_mms, "STATE.target_linear_mms")
    _int16(value.target_angular_mrads, "STATE.target_angular_mrads")
    _uint(value.fault_bits, 32, "STATE.fault_bits")


def encode_state(value: StateTelemetry) -> bytes:
    _validate_state(value)
    statuses = _pack_range_statuses(value.range_status)
    ages = tuple(age // AGE_UNIT_MS for age in value.range_age_ms)
    ack = 0 if value.ack_sequence is None else value.ack_sequence
    applied = 0 if value.applied_control_sequence is None else value.applied_control_sequence
    prefix = struct.pack(
        "<QIIiihBH",
        value.session_nonce,
        value.boot_id,
        value.uptime_ms,
        value.left_mm,
        value.right_mm,
        value.yaw_tenth_deg,
        value.odom_status,
        value.range_cycle_seq,
    )
    ranges = struct.pack("<4HH4B", *value.range_distance_mm, statuses, *ages)
    suffix = struct.pack(
        "<HBBBHHhhI",
        value.gimbal_aim_id,
        int(value.gimbal_pose),
        int(value.gimbal_motion_state),
        int(value.state_flags),
        ack,
        applied,
        value.target_linear_mms,
        value.target_angular_mrads,
        value.fault_bits,
    )
    payload = prefix + ranges + suffix
    if len(payload) != 60:
        raise AssertionError("STATE payload 内部布局必须冻结为 60 字节")
    return payload


def decode_state(payload: bytes) -> StateTelemetry:
    if len(payload) != 60:
        raise ProtocolPayloadError("STATE payload 长度必须是 60")
    prefix_size = struct.calcsize("<QIIiihBH")
    range_size = struct.calcsize("<4HH4B")
    prefix = struct.unpack("<QIIiihBH", payload[:prefix_size])
    distances_and_status = struct.unpack(
        "<4HH4B", payload[prefix_size : prefix_size + range_size]
    )
    distances = tuple(distances_and_status[:4])
    statuses = _unpack_range_statuses(distances_and_status[4])
    ages = tuple(int(value) * AGE_UNIT_MS for value in distances_and_status[5:])
    aim_id, pose_raw, motion_raw, flags_raw, ack_raw, applied_raw, target_v, target_w, faults = struct.unpack(
        "<HBBBHHhhI", payload[prefix_size + range_size :]
    )
    try:
        pose = WireGimbalPose(pose_raw)
        motion = WireGimbalMotionState(motion_raw)
        flags = StateFlag(flags_raw)
    except ValueError as exc:
        raise ProtocolPayloadError("STATE 含未知云台姿态/状态或 flags") from exc
    value = StateTelemetry(
        session_nonce=prefix[0],
        boot_id=prefix[1],
        uptime_ms=prefix[2],
        left_mm=prefix[3],
        right_mm=prefix[4],
        yaw_tenth_deg=prefix[5],
        odom_status=prefix[6],
        range_cycle_seq=prefix[7],
        range_distance_mm=distances,  # type: ignore[arg-type]
        range_status=statuses,
        range_age_ms=ages,  # type: ignore[arg-type]
        gimbal_aim_id=aim_id,
        gimbal_pose=pose,
        gimbal_motion_state=motion,
        state_flags=flags,
        ack_sequence=ack_raw if flags & StateFlag.ACK_PRESENT else None,
        applied_control_sequence=(applied_raw if flags & StateFlag.APPLIED_CONTROL_PRESENT else None),
        target_linear_mms=target_v,
        target_angular_mrads=target_w,
        fault_bits=faults,
    )
    try:
        _validate_state(value)
    except ProtocolValueError as exc:
        raise ProtocolPayloadError(f"STATE 字段非法：{exc}") from exc
    return value
