"""开机自检：逐项检查硬件链路，结果写日志 + 打印。全过才放行启动。

先 STM32 握手与零速 / STOP 保护，再执行舵机检查；
MCU 自检必须收到新鲜且 ODOM_VALID 的遥测才算过；声卡自检不得播放比赛播报素材。
"""

from __future__ import annotations

import json
import math
import os
import platform
import shutil
import stat
import sys
import time
import wave
from dataclasses import dataclass, fields, is_dataclass
from datetime import datetime, timezone
from enum import Enum, IntEnum, IntFlag
from pathlib import Path

import config
import coverage
import gimbal as gimbal_mod
import runlog
from drivers.protocol_v2 import AckStatus, Capability, MessageType, StateFlag, StopReason


# 与 STM32 User/firmware_app.c 的 APP_FAULT_* 位一一对应。未知位仍保留十六进制，
# 防止固件新增诊断时被上位机静默忽略。
_FIRMWARE_FAULT_LABELS = (
    (1 << 0, "uart_overrun"),
    (1 << 1, "uart_framing"),
    (1 << 2, "uart_noise"),
    (1 << 3, "uart_parity"),
    (1 << 4, "uart_rx_overflow"),
    (1 << 5, "uart_tx_overflow"),
    (1 << 6, "uart_dma"),
    (1 << 8, "protocol_length"),
    (1 << 9, "protocol_crc"),
    (1 << 10, "protocol_type"),
    (1 << 11, "protocol_payload"),
    (1 << 12, "motion_profile_invalid"),
    (1 << 13, "servo_profile_invalid"),
    (1 << 14, "light_profile_invalid"),
    (1 << 15, "encoder_not_ready"),
    (1 << 16, "gyro_not_ready"),
    (1 << 17, "legacy_bit17_ignored"),
    (1 << 18, "tof_guard_invalid"),
    (1 << 19, "watchdog_not_ready"),
    (1 << 20, "local_stop_latched"),
    (1 << 21, "identity_not_ready"),
    (1 << 22, "clock_fault"),
)

_GYRO_STATUS_LABELS = {
    0: "not_initialized",
    1: "who_am_i_error",
    2: "uncalibrated",
    3: "calibrating",
    4: "ready",
    5: "bus_error",
    6: "sample_gap",
}
_RANGE_STATUS_LABELS = {
    0: "valid",
    1: "no_target",
    2: "out_of_range",
    3: "signal_error",
    4: "timeout",
    5: "bus_error",
    6: "stale",
}
_TOF_INTERNAL_STATUS_LABELS = {
    **_RANGE_STATUS_LABELS,
    7: "no_device",
    8: "uncalibrated",
}
_SIDE_CHANNELS = ((1, "left"), (2, "right"))
_UPPER_NAMES = ("upper_left", "upper_right")
_TOF_OPTIONAL_CLEAR_STATUSES = frozenset({"VALID", "NO_TARGET", "OUT_OF_RANGE"})


def _active_fault_bits(state) -> int:
    """仅忽略旧固件保留的 bit17，其他位继续参与安全门禁。"""
    bits = _as_int(_object_value(state, "fault_bits", 0))
    ignored = int(config.MCU_IGNORED_RUNTIME_FAULT_BITS)
    return bits & ~ignored


def _enum_name(value) -> str:
    """兼容协议 Enum、固件整数和测试替身，返回稳定的大写状态名。"""
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name.upper()
    if isinstance(value, str):
        return value.upper()
    try:
        return _RANGE_STATUS_LABELS.get(int(value), str(value)).upper()
    except (TypeError, ValueError):
        return str(value).upper()


def _internal_tof_name(value) -> str:
    """将 v2.3 DIAGNOSTICS 的内部 ToF 状态转成稳定名称。"""
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name.upper()
    if isinstance(value, str):
        return value.upper()
    try:
        return _TOF_INTERNAL_STATUS_LABELS.get(int(value), str(value)).upper()
    except (TypeError, ValueError):
        return str(value).upper()


def _object_value(value, name: str, default=None):
    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


def _diagnostics_health(mcu) -> tuple[bool, str]:
    """当前 v2.3 会话要求诊断帧新鲜。"""
    diagnostics_capability = getattr(Capability, "DIAGNOSTICS", None)
    if diagnostics_capability is None:
        return False, "协议层未提供 DIAGNOSTICS capability"
    try:
        negotiated = bool(mcu.has_negotiated_capability(diagnostics_capability))
    except Exception as exc:  # noqa: BLE001 - 自检必须把链接口不完整转成结果
        return False, f"无法确认诊断能力协商: {exc}"
    if not negotiated:
        return False, "DIAGNOSTICS 未在本次 HELLO 协商（protocol 2.3）"

    # 兼容本轮接口落地前后的命名；最终 McuLink 使用 diagnostics_ok()。
    diagnostics_ok = getattr(mcu, "diagnostics_ok", None)
    if callable(diagnostics_ok):
        try:
            reason = diagnostics_ok()
        except Exception as exc:  # noqa: BLE001
            return False, f"诊断新鲜度检查失败: {exc}"
        return (reason is None, "" if reason is None else str(reason))
    diagnostics_fresh = getattr(mcu, "diagnostics_fresh", None)
    if callable(diagnostics_fresh):
        try:
            return (bool(diagnostics_fresh()), "诊断遥测超龄")
        except Exception as exc:  # noqa: BLE001
            return False, f"诊断新鲜度检查失败: {exc}"
    return False, "上位机未实现 diagnostics_ok()"


def _directional_tof_health(mcu) -> tuple[bool, str]:
    directional = getattr(mcu, "directional", None)
    if directional is None:
        return False, "DIRECTIONAL_RANGE4 未收到"
    effective = getattr(mcu, "directional_effective_statuses", None)
    statuses = tuple(effective) if effective is not None else tuple(getattr(directional, "status", ()))
    if len(statuses) != 4:
        return False, f"DIRECTIONAL_RANGE4 状态数量异常: {len(statuses)}"

    problems: list[str] = []
    # 实装 CH1/CH2 在索引 1/2；冻结协议的空槽不参与业务。
    for index, name in _SIDE_CHANNELS:
        status = _enum_name(statuses[index])
        if status not in _TOF_OPTIONAL_CLEAR_STATUSES:
            problems.append(f"{name}={status}（需 VALID/NO_TARGET/OUT_OF_RANGE）")
    return (not problems, "; ".join(problems) or "ok")


def _upper_tof_health(mcu, *, required: bool) -> tuple[bool, str]:
    if not required:
        return True, "上层激光单独检查，不参与基础运动门禁"
    try:
        if not mcu.has_negotiated_capability(Capability.CULVERT_UPPER2):
            return False, "CULVERT_UPPER2 未协商"
    except Exception as exc:  # noqa: BLE001 - 缺少接口不能冒充健康观测
        return False, f"无法确认 CULVERT_UPPER2 能力: {exc}"
    upper = getattr(mcu, "culvert_upper", None)
    if upper is None:
        return False, "CULVERT_UPPER2 未收到"
    try:
        effective = getattr(mcu, "upper_effective_statuses", None)
        ages = getattr(mcu, "upper_age_ms", None)
        if effective is None or ages is None:
            return False, "CULVERT_UPPER2 缺少有效状态或样本年龄证据"
        statuses = tuple(effective)
        ages = tuple(ages)
    except Exception as exc:  # noqa: BLE001
        return False, f"CULVERT_UPPER2 新鲜度检查失败: {exc}"
    if len(statuses) != len(_UPPER_NAMES):
        return False, f"CULVERT_UPPER2 状态数量异常: {len(statuses)}"
    if len(ages) != len(_UPPER_NAMES):
        return False, f"CULVERT_UPPER2 年龄数量异常: {len(ages)}"
    raw_distances = getattr(upper, "distance_mm", ())
    distances = tuple(raw_distances) if isinstance(raw_distances, (list, tuple)) else ()
    names = [_enum_name(status) for status in statuses]
    healthy = []
    for index, name in enumerate(names):
        age = ages[index]
        fresh = (isinstance(age, int) and not isinstance(age, bool)
                 and 0 <= age <= config.CULVERT_UPPER_LASER_MAX_AGE_MS)
        numeric_valid = True
        if name == "VALID":
            distance = distances[index] if index < len(distances) else None
            numeric_valid = (isinstance(distance, (int, float)) and not isinstance(distance, bool)
                             and math.isfinite(distance) and 0 <= distance < 65535)
        healthy.append(fresh and name in _TOF_OPTIONAL_CLEAR_STATUSES and numeric_valid)
        if not fresh:
            names[index] = "STALE"
        elif not numeric_valid:
            names[index] = "INVALID_VALUE"
    left_status, right_status = names
    ages_detail = f" age_ms={ages}，年龄上限={config.CULVERT_UPPER_LASER_MAX_AGE_MS}"
    # CH5 的 0/1/2 均为健康观测；只有故障才备援 CH4。
    if healthy[1]:
        return True, f"upper_right={right_status}（主用）；upper_left={left_status}" + ages_detail
    if healthy[0]:
        return True, f"upper_right={right_status}（故障）；upper_left={left_status}（备援）" + ages_detail
    return False, (
        f"upper_right={right_status}（故障）；"
        f"upper_left={left_status}（备援需 VALID/NO_TARGET/OUT_OF_RANGE）"
    ) + ages_detail


def _diagnostic_root_causes(state, diagnostics) -> list[str]:
    """把原始故障位和诊断计数压缩成现场可执行的排查方向。"""
    causes: list[str] = []
    fault_bits = _as_int(_object_value(state, "fault_bits", 0))
    if fault_bits & (1 << 22):
        causes.append("clock_fault: 检查 HSE/PLL/SysTick 与板载晶振配置")
    if fault_bits & (1 << 16):
        causes.append("gyro_not_ready: 检查 MPU6050 供电、AD0=0、I2C2 PB10/PB11")
    if fault_bits & (1 << 18):
        causes.append("tof_guard_invalid: 检查实装 ToF 及 TCA9548A 选通")

    if diagnostics is None:
        return causes
    gyro_status = _as_int(_object_value(diagnostics, "gyro_status", -1), -1)
    gyro_label = _GYRO_STATUS_LABELS.get(gyro_status)
    if gyro_label == "who_am_i_error":
        causes.append("gyro_who_am_i_error: 核对 MPU6050 型号/地址/AD0")
    elif gyro_label == "bus_error":
        causes.append("gyro_bus_error: 检查 MPU6050 I2C2 接线与上拉")
    elif gyro_label in {"uncalibrated", "calibrating"}:
        causes.append("gyro_calibrating: 保持车辆静止直到完成零偏标定")
    elif gyro_label == "sample_gap":
        causes.append("gyro_sample_gap: 检查 I2C 超时/恢复是否挤占控制周期")

    timeout_count = _as_int(_object_value(diagnostics, "i2c_timeout_count", 0))
    recovery_count = _as_int(_object_value(diagnostics, "i2c_recovery_count", 0))
    bus_error_count = _as_int(_object_value(diagnostics, "i2c_bus_error_count", 0))
    if timeout_count or recovery_count or bus_error_count:
        causes.append(
            "i2c_fault_history: "
            f"timeout={timeout_count} recovery={recovery_count} bus_error={bus_error_count}"
        )
    return causes


def _format_fault_bits(bits: int) -> str:
    if bits == 0:
        return "ok"
    labels = [label for bit, label in _FIRMWARE_FAULT_LABELS if bits & bit]
    known_mask = sum(bit for bit, _label in _FIRMWARE_FAULT_LABELS)
    unknown = bits & ~known_mask
    if unknown:
        labels.append(f"unknown=0x{unknown:08X}")
    return ",".join(labels) or "uart_or_unknown"


@dataclass
class CheckResult:
    name: str
    ok: bool
    critical: bool
    message: str

    def mark(self) -> str:
        return "[OK]" if self.ok else "[FAIL]"


def _check_upper(mcu) -> CheckResult:
    """上层测距单独报告；不可用会降级涵洞触发，不伪装成通过。"""
    if not getattr(mcu, "connected", False):
        ok, reason = False, "STM32 未连接，未取得上层测距证据"
    else:
        ok, reason = _upper_tof_health(mcu, required=True)
    message = reason if ok else reason + "；涵洞触发降级，允许继续其他巡逻任务"
    return CheckResult("上层激光", ok, False, message)


def _format_mcu_diagnostics(mcu) -> str:
    """把实时状态和 v2.3 诊断带到自检结果，避免只看到“遥测不全”。"""
    state = mcu.state
    build = getattr(mcu, "build_id", "") or "?"
    if state is None:
        return f" build={build} state=missing diagnostics={getattr(mcu, 'diagnostics', None) is not None}"

    odom_labels = []
    if state.odom_status & 0x01:
        odom_labels.append("left_encoder")
    if state.odom_status & 0x02:
        odom_labels.append("right_encoder")
    if state.odom_status & 0x04:
        odom_labels.append("gyro_not_ready")
    odom_detail = ",".join(odom_labels) if odom_labels else "ok"
    details = [
        f"build={build}",
        f"odom_status=0x{state.odom_status:02X}({odom_detail})",
        f"state_flags=0x{int(state.state_flags):02X}",
        f"fault_bits=0x{state.fault_bits:08X}({_format_fault_bits(state.fault_bits)})",
    ]
    directional = getattr(mcu, "directional", None)
    if directional is not None:
        effective = getattr(mcu, "directional_effective_statuses", None)
        status_values = tuple(effective) if effective is not None else tuple(directional.status)
        details.append("side_status=" + ",".join(
            f"{name}={_enum_name(status_values[index])}"
            for index, name in _SIDE_CHANNELS if index < len(status_values)
        ))
        ages = getattr(mcu, "directional_age_ms", None)
        if ages is not None:
            details.append("side_age_ms=" + ",".join(
                f"{name}={ages[index]}" for index, name in _SIDE_CHANNELS if index < len(ages)
            ))
    upper = getattr(mcu, "culvert_upper", None)
    if upper is not None:
        effective = getattr(mcu, "upper_effective_statuses", None)
        status_values = tuple(effective) if effective is not None else tuple(upper.status)
        details.append("upper_status=" + ",".join(_enum_name(item) for item in status_values))
        ages = getattr(mcu, "upper_age_ms", None)
        if ages is not None:
            details.append("upper_age_ms=" + ",".join(str(item) for item in ages))
    diagnostics = getattr(mcu, "diagnostics", None)
    diagnostics_ok, diagnostics_reason = _diagnostics_health(mcu)
    if diagnostics is None:
        details.append("diagnostics=missing")
    else:
        gyro_status = _as_int(_object_value(diagnostics, "gyro_status", -1), -1)
        gyro_label = _GYRO_STATUS_LABELS.get(gyro_status, f"unknown={gyro_status}")
        calibration_count = _as_int(
            _object_value(diagnostics, "gyro_calibration_count", 0)
        )
        sample_age_ms = _as_int(_object_value(diagnostics, "gyro_sample_age_ms", 0))
        tof_mask = _as_int(
            _object_value(
                diagnostics,
                "tof_initialized_mask",
                _object_value(diagnostics, "tof_init_mask", 0),
            )
        )
        details.append(
            "diagnostics="
            f"gyro={gyro_label} cal={calibration_count} age_ms={sample_age_ms} "
            f"tof_init=0x{tof_mask:02X}"
        )
        details.append(
            "i2c="
            f"tx={_as_int(_object_value(diagnostics, 'i2c_transaction_count', 0))} "
            f"retry={_as_int(_object_value(diagnostics, 'i2c_retry_count', 0))} "
            f"recover={_as_int(_object_value(diagnostics, 'i2c_recovery_count', 0))} "
            f"nack={_as_int(_object_value(diagnostics, 'i2c_nack_count', 0))} "
            f"timeout={_as_int(_object_value(diagnostics, 'i2c_timeout_count', 0))} "
            f"bus={_as_int(_object_value(diagnostics, 'i2c_bus_error_count', 0))}"
        )
    details.append("diagnostics_fresh=" + ("true" if diagnostics_ok else f"false({diagnostics_reason})"))
    causes = _diagnostic_root_causes(state, diagnostics)
    if causes:
        details.append("root_cause=" + " | ".join(causes))
    return " " + " ".join(details)


def _remaining(deadline: float | None, maximum: float) -> float:
    if deadline is not None and not math.isfinite(deadline):
        raise ValueError("截止时间必须是有限的单调时钟值")
    remaining = maximum if deadline is None else min(maximum, deadline - time.monotonic())
    if remaining <= 0:
        raise TimeoutError("检查已到截止时间")
    return remaining


def _timed_call(method, *args, deadline: float | None, maximum: float):
    timeout = _remaining(deadline, maximum)
    if gimbal_mod._accepts_keyword(method, "timeout_s"):
        result = method(*args, timeout_s=timeout)
    else:
        result = method(*args)
    _remaining(deadline, maximum)
    return result


def _confirm_control(mcu, sequence: int, deadline: float | None) -> None:
    timeout = _remaining(deadline, config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
    command_deadline = time.monotonic() + timeout
    gimbal_mod.wait_for_ack(mcu, MessageType.CONTROL_TARGET, sequence, command_deadline)
    _timed_call(mcu.wait_for_control_applied, sequence,
                deadline=command_deadline, maximum=config.MCU_COMMAND_CONFIRM_TIMEOUT_S)


def _servo_command(mcu, gimbal, method_name: str, deadline: float | None) -> int:
    _remaining(deadline, config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
    method = getattr(gimbal, method_name)
    if gimbal_mod._accepts_keyword(method, "deadline"):
        sequence = method(deadline=deadline)
    else:
        sequence = method()
    _confirm_control(mcu, sequence, deadline)
    return sequence


def _check_servo_look(mcu, gimbal, deadline: float | None = None) -> CheckResult:
    """正式启动只确认水平居中导航姿态，不执行调试用左右往返。"""
    if not getattr(mcu, "connected", False):
        return CheckResult("舵机水平居中", False, True, "跳过：STM32 自检未通过，串口已关闭")
    try:
        sequence = _servo_command(mcu, gimbal, "tilt_look", deadline)
        return CheckResult("舵机水平居中", True, True,
                           f"水平导航 PWM 应用已确认 seq={sequence}；未验证机械角度")
    except Exception as exc:  # noqa: BLE001
        runlog.exception("selfcheck.gimbal", "水平导航姿态确认失败", exc)
        return CheckResult("舵机水平居中", False, True, f"失败: {exc}")


def _check_servo_sweep(mcu, gimbal, deadline: float | None) -> CheckResult:
    if not getattr(mcu, "connected", False):
        return CheckResult("云台往返", False, True, "跳过：STM32 未连接")
    completed: list[str] = []
    errors: list[str] = []
    restore_confirmed = False
    try:
        for method in ("pan_left", "pan_right", "center", "tilt_down", "tilt_up", "tilt_look"):
            sequence = _servo_command(mcu, gimbal, method, deadline)
            completed.append(f"{method}:seq={sequence}")
            if method == "tilt_look":
                restore_confirmed = True
    except Exception as exc:  # noqa: BLE001
        errors.append(str(exc))
        runlog.exception("selfcheck.gimbal_sweep", "云台往返中断", exc,
                         completed=completed)
    finally:
        if not restore_confirmed:
            try:
                # 即使检查截止，也尝试恢复水平导航 PWM，但超时不能记作恢复已确认。
                method = getattr(gimbal, "tilt_look")
                sequence = method(deadline=deadline) if gimbal_mod._accepts_keyword(method, "deadline") else method()
                _confirm_control(mcu, sequence, deadline)
                completed.append(f"tilt_look:seq={sequence}")
            except Exception as exc:  # noqa: BLE001
                errors.append(f"水平导航恢复失败: {exc}")
                runlog.exception("selfcheck.gimbal_restore", "自检后水平导航姿态未确认", exc)
    detail = "; ".join(completed + errors)
    return CheckResult("云台往返", not errors, True,
                       detail + "；仅确认 PWM 应用，不证明机械角度或实际转动")


def _check_motion_protection(mcu, deadline: float | None) -> CheckResult:
    if not getattr(mcu, "connected", False):
        return CheckResult("零速与STOP保护", False, True, "跳过：STM32 未连接")
    errors: list[str] = []
    try:
        _remaining(deadline, config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
        sequence = mcu.send_motion(0, 0)
        _confirm_control(mcu, sequence, deadline)
    except Exception as exc:  # noqa: BLE001
        errors.append(f"零速应用未确认: {exc}")
        runlog.exception("selfcheck.zero_motion", "零速保护检查失败", exc)
    finally:
        try:
            sequence = mcu.send_stop(StopReason.MISSION)
            timeout = _remaining(deadline, config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
            gimbal_mod.wait_for_ack(mcu, MessageType.STOP, sequence, time.monotonic() + timeout)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"STOP 未确认: {exc}")
            runlog.exception("selfcheck.stop", "STOP 保护检查失败", exc)
    return CheckResult("零速与STOP保护", not errors, True,
                       "; ".join(errors) if errors else "零速应用与 STOP ACK 已确认；未驱动车轮验证机械制动或失联保护")


def _close_safely(device, *, active: bool = False) -> None:
    try:
        shutdown = getattr(device, "shutdown", None)
        if active and callable(shutdown):
            shutdown()
        else:
            device.close()
    except Exception as exc:  # noqa: BLE001
        runlog.exception("selfcheck.close", "检查后关闭设备失败", exc)


def _check_mcu(
    mcu,
    deadline: float | None = None,
    *,
    passive: bool = False,
    require_upper: bool = True,
) -> CheckResult:
    """第 1 步：STM32（ToF、ODOM、无故障及可选诊断）。"""
    ports = [config.MCU_PORT]
    last = ""
    for port in ports:
        mcu.port = port
        try:
            _remaining(deadline, config.MCU_SELFCHECK_READY_TIMEOUT_S)
            mcu.open()
            _timed_call(mcu.hello, deadline=deadline, maximum=3.0)
            synchronize = getattr(mcu, "synchronize_clock", None)
            if not passive and callable(synchronize):
                _timed_call(synchronize, deadline=deadline,
                            maximum=config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
            ready_deadline = time.monotonic() + _remaining(deadline, config.MCU_SELFCHECK_READY_TIMEOUT_S)
            got_directional = False
            directional_ok = False
            directional_reason = "DIRECTIONAL_RANGE4 未收到"
            got_odom = False
            got_upper = False
            upper_ok = False
            upper_reason = "CULVERT_UPPER2 未收到"
            got_fault_free = False
            diagnostics_ok = False
            diagnostics_reason = "DIAGNOSTICS 未收到"
            need_upper = require_upper
            diagnostics_capability = getattr(Capability, "DIAGNOSTICS", None)
            diagnostics_negotiated = (
                diagnostics_capability is not None
                and mcu.has_negotiated_capability(diagnostics_capability)
            )
            while time.monotonic() < ready_deadline:
                mcu.pump()
                if mcu.directional is not None:
                    got_directional = True
                    directional_ok, directional_reason = _directional_tof_health(mcu)
                if mcu.state is not None:
                    got_odom = bool(mcu.state.state_flags & StateFlag.ODOM_VALID) and mcu.yaw_deg is not None
                    got_fault_free = _active_fault_bits(mcu.state) == 0
                if mcu.culvert_upper is not None:
                    got_upper = True
                    upper_ok, upper_reason = _upper_tof_health(mcu, required=need_upper)
                diagnostics_ok, diagnostics_reason = _diagnostics_health(mcu)
                if (
                    got_directional
                    and directional_ok
                    and got_odom
                    and got_fault_free
                    and (got_upper and upper_ok if need_upper else True)
                    and (not diagnostics_negotiated or diagnostics_ok)
                ):
                    break
                time.sleep(0.02)
            ready = (
                got_directional
                and directional_ok
                and got_odom
                and got_fault_free
                and (got_upper and upper_ok if need_upper else True)
                and (not diagnostics_negotiated or diagnostics_ok)
            )
            if not ready:
                diag = _format_mcu_diagnostics(mcu)
                _close_safely(mcu, active=not passive)
                last = (
                    f"端口 {port} 握手成功但设备未就绪"
                    f"（directional_received={got_directional} directional_ok={directional_ok} "
                    f"odom={got_odom} upper_required={need_upper} upper_received={got_upper} upper_ok={upper_ok} "
                    f"fault_free={got_fault_free} diagnostics_negotiated={diagnostics_negotiated} "
                    f"diagnostics_fresh={diagnostics_ok} timeout={config.MCU_SELFCHECK_READY_TIMEOUT_S:.1f}s；"
                    f"directional={directional_reason}；upper={upper_reason}；"
                    f"diagnostics={diagnostics_reason}）{diag}"
                )
                # HELLO 已经证明这是 STM32 端口；不要再把 RFID 端口的
                # HELLO 超时覆盖掉真正的遥测诊断。
                break
            fb = _active_fault_bits(mcu.state)
            if fb != 0:
                _close_safely(mcu, active=not passive)
                last = f"端口 {port} 固件 fault_bits=0x{fb:08X}（硬件未核验或故障，需台架标定/排查）"
                break
            msg = (
                f"端口={port} 握手成功 build={mcu.build_id} "
                f"advertised=0x{mcu.capabilities:08X} "
                f"negotiated=0x{mcu.negotiated_required_capabilities:08X}"
            )
            if mcu.directional is not None:
                msg += f" 侧向测距=[左下1:{mcu.left_mm},右下2:{mcu.right_mm}]"
            if mcu.yaw_deg is not None:
                msg += f" yaw={mcu.yaw_deg:.1f}°"
            return CheckResult("STM32 串口", True, True, msg)
        except Exception as exc:  # noqa: BLE001
            runlog.exception("selfcheck.mcu", "STM32 检查失败", exc, port=port)
            _close_safely(mcu, active=not passive)
            last = f"端口 {port} 失败: {exc}"
    if "握手成功" in last:
        return CheckResult(
            "STM32 串口",
            False,
            True,
            last
            + "；请保持车辆静止，按 fault_bits、诊断字段和 ToF 状态排查；"
            "建议运行 python3 main.py diagnose --duration 20",
        )
    return CheckResult("STM32 串口", False, True, last + "；请确认接线/波特率/哪个 USB 口")


def _check_camera(camera, deadline: float | None = None, frames: list | None = None) -> CheckResult:
    try:
        import numpy as np

        _timed_call(camera.open, deadline=deadline,
                    maximum=getattr(config, "CAMERA_OPEN_TIMEOUT_S", 3.0))
        frame_deadline = time.monotonic() + _remaining(deadline, 3.0)
        problem = "未收到画面"
        while time.monotonic() < frame_deadline:
            grab = getattr(camera, "grab_frame", None)
            if callable(grab):
                captured = _timed_call(grab, deadline=frame_deadline, maximum=0.02)
                frame = getattr(captured, "bgr", None)
                fresh = getattr(captured, "is_fresh", None)
                if not callable(fresh) or not fresh(config.CAMERA_FRAME_MAX_AGE_S):
                    frame = None
                    problem = "帧缺少有效采集时间或已过期"
            else:
                frame = _timed_call(camera.grab_bgr, deadline=frame_deadline, maximum=0.02)
            if (
                isinstance(frame, np.ndarray) and frame.dtype == np.uint8
                and frame.ndim == 3 and frame.shape[2] == 3
                and frame.shape[0] > 0 and frame.shape[1] > 0
            ):
                if frames is not None:
                    frames.append(frame)
                h, w = frame.shape[:2]
                return CheckResult("摄像头", True, True,
                                   f"有效 BGR 帧 {w}x{h}；未验证镜头遮挡或现场识别效果")
            if frame is not None:
                problem = "画面不是非空 uint8 三通道 BGR"
            time.sleep(max(0.0, min(0.02, frame_deadline - time.monotonic())))
        return CheckResult("摄像头", False, True, problem)
    except Exception as exc:  # noqa: BLE001
        runlog.exception("selfcheck.camera", "摄像头检查失败", exc)
        return CheckResult("摄像头", False, True, f"失败: {exc}")


def _check_model(detector, deadline: float | None = None, frame=None, *, infer: bool = True) -> CheckResult:
    try:
        _timed_call(detector.load, deadline=deadline,
                    maximum=getattr(config, "DETECTOR_LOAD_TIMEOUT_S", 15.0))
        if not infer:
            return CheckResult("BPU 模型", True, False, "运行模型已加载；启动未重复执行完整推理自检")
        synthetic = frame is None
        if synthetic:
            import numpy as np
            frame = np.zeros((config.YOLO_INPUT_H, config.YOLO_INPUT_W, 3), dtype=np.uint8)
        detections = _timed_call(detector.infer_bgr, frame, deadline=deadline, maximum=1.0)
        if not isinstance(detections, (list, tuple)):
            raise ValueError("推理没有返回检测结果列表")
        source = "合成 BGR（相机不可用）" if synthetic else "当前相机帧"
        return CheckResult("BPU 模型", True, True,
                           f"{source}完成一次真实推理，返回 {len(detections)} 个目标；未验证识别准确率")
    except Exception as exc:  # noqa: BLE001
        runlog.exception("selfcheck.model", "模型加载或推理检查失败", exc)
        return CheckResult("BPU 模型", False, True, f"失败: {exc}")


def _check_map(deadline: float | None = None) -> CheckResult:
    try:
        _remaining(deadline, 1.0)
        field_map = coverage.load_map()
        _remaining(deadline, 1.0)
        return CheckResult("导航地图", True, True,
                           f"地图校验通过：{len(field_map.nodes)} 节点、{len(field_map.roads)} 道路，起点 {field_map.start_node_id}")
    except Exception as exc:  # noqa: BLE001
        runlog.exception("selfcheck.map", "导航地图检查失败", exc)
        return CheckResult("导航地图", False, True, str(exc))


def _check_clues(clues, deadline: float | None = None) -> CheckResult:
    for class_id in range(1, 21):
        _remaining(deadline, 1.0)
        clue = clues.detection(class_id)
        category = "SUSPECT" if class_id <= 10 else "KNIFE"
        number = class_id if class_id <= 10 else class_id - 10
        if (clue is None or clue.category != category or clue.number != number
                or not isinstance(clue.text, str) or not clue.text.strip()
                or not isinstance(clue.audio, str) or not clue.audio.strip()
                or clues.detection_audio_path(class_id) is None):
            raise ValueError(f"模型类别 {class_id} 的线索或音频映射无效")
    for number in range(1, 13):
        _remaining(deadline, 1.0)
        clue = clues.patrol(number)
        if (clue is None or clue.number != number
                or not isinstance(clue.text, str) or not clue.text.strip()
                or not isinstance(clue.audio, str) or not clue.audio.strip()
                or clues.patrol_audio_path(number) is None):
            raise ValueError(f"巡逻点 {number} 的线索或音频映射无效")
    return CheckResult("线索映射", True, True, "20 类目标与 12 个巡逻号的运行映射可用；未验证录音内容")


def _check_audio_assets(deadline: float | None, clues=None) -> None:
    if clues is None:
        names = [f"patrol_{n:02d}.wav" for n in range(1, 13)]
        names += [f"clue_{kind}_{n:02d}.wav" for kind in ("suspect", "knife") for n in range(1, 11)]
        paths = [config.AUDIO_DIR / name for name in names]
    else:
        paths = [clues.patrol_audio_path(n) for n in range(1, 13)]
        paths += [clues.detection_audio_path(n) for n in range(1, 21)]
    for path in paths:
        _remaining(deadline, 10.0)
        if path is None:
            raise ValueError("比赛音频映射缺少文件路径")
        with wave.open(str(path), "rb") as wav:
            if wav.getframerate() <= 0 or wav.getnframes() <= 0 or not wav.readframes(1):
                raise ValueError(f"无有效音频样本: {path}")


def _check_audio(audio, deadline: float | None = None, clues=None) -> CheckResult:
    try:
        _check_audio_assets(deadline, clues)
        if _remaining(deadline, 10.0) < 10.0:
            raise TimeoutError("剩余检查时间不足以完成有界声卡测试")
        ok, msg = audio.test()
        return CheckResult("声卡", ok, True, "32 项比赛 WAV 素材可读；" + msg + "；未验证录音内容或扬声器实际可听音量")
    except Exception as exc:  # noqa: BLE001
        runlog.exception("selfcheck.audio", "声卡检查失败", exc)
        return CheckResult("声卡", False, True, f"失败: {exc}")


def _initialize_audio(audio, deadline: float | None = None, clues=None) -> CheckResult:
    _remaining(deadline, 10.0)
    if audio.mode != "aplay" or not audio.device or shutil.which("aplay") is None:
        return CheckResult("声卡", False, False, "aplay、设备选项或音频模式不可用")
    _check_audio_assets(deadline, clues)
    return CheckResult("声卡", True, False, "播放器配置和 WAV 素材可读；启动未播放测试音，未验证设备输出")


def _check_rfid(rfid, deadline: float | None = None, *, probe: bool = True) -> CheckResult:
    from rfid import parse_patrol_no

    try:
        _remaining(deadline, 1.0)
        rfid.open()
        _remaining(deadline, 1.0)
        if not probe:
            return CheckResult("RFID", True, False, "读卡串口已打开，等待读卡事件；未重复完整读卡自检")
        event = rfid.read_patrol_card()
        _remaining(deadline, 1.0)
    except Exception as exc:  # noqa: BLE001
        runlog.exception("selfcheck.rfid", "RFID 检查失败", exc)
        if not probe:
            _close_safely(rfid)
        return CheckResult("RFID", False, True, f"失败: {exc}")
    finally:
        if probe:
            _close_safely(rfid)
    if event.error:
        return CheckResult("RFID", False, True,
                           f"读块失败: {event.error}；响应状态={event.reader_status}")
    if event.reader_status == 0xFF:
        return CheckResult("RFID", True, True, "设备响应正常（当前无卡，属正常）")
    no = parse_patrol_no(event.block_data)
    if (event.reader_status == 0 and event.block_no == config.RFID_PATROL_BLOCK
            and isinstance(event.uid, bytes) and len(event.uid) == 4 and no is not None):
        return CheckResult("RFID", True, True,
                           f"读到卡 UID={event.uid.hex()}；0扇区1块第0字节="
                           f"{event.block_data[config.RFID_PATROL_BYTE_INDEX]:02X}；巡逻号={no}")
    return CheckResult("RFID", False, True, "没有合法巡逻读块证据；UID响应不能证明巡逻编号")



def _component_check(c: dict, key: str, name: str, critical: bool, check) -> CheckResult:
    runlog.status(phase="selfcheck", component=key)
    errors = c.get("_init_errors", {})
    if c.get(key) is None:
        detail = errors.get(key, "未创建组件") if isinstance(errors, dict) else "未创建组件"
        result = CheckResult(name, False, critical, f"{key}: {detail}")
    else:
        try:
            result = check(c[key])
            result.critical = critical
        except Exception as exc:  # noqa: BLE001
            runlog.exception("selfcheck.component_exception", "组件检查抛出异常", exc, component=key)
            result = CheckResult(name, False, critical, str(exc))
    runlog.emit("selfcheck.component", result.message, component=key,
                check=result.name, ok=result.ok, critical=result.critical)
    return result


def _refresh_runtime_ready(
    mcu,
    deadline: float | None,
    *,
    require_upper: bool = True,
) -> CheckResult:
    """耗时外设初始化后重新采集运行证据；STOP 不启动新的运动租约。"""
    if not getattr(mcu, "connected", False):
        return CheckResult("启动遥测刷新", False, True, "STM32 未连接")
    try:
        refresh_deadline = time.monotonic() + _remaining(deadline, 1.0)
        previous_token = getattr(mcu, "state_sample_token", None)
        requires_token = hasattr(mcu, "state_sample_token")
        stop_sequence = mcu.send_stop(StopReason.MISSION)
        ack_deadline = min(refresh_deadline, time.monotonic() + config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
        gimbal_mod.wait_for_ack(mcu, MessageType.STOP, stop_sequence, ack_deadline)
        reason = "未收到新鲜运行遥测"
        while time.monotonic() < refresh_deadline:
            mcu.pump()
            state = getattr(mcu, "state", None)
            directional_ok, directional_reason = _directional_tof_health(mcu)
            need_upper = require_upper
            upper_ok, upper_reason = _upper_tof_health(mcu, required=need_upper)
            token = getattr(mcu, "state_sample_token", None)
            token_ok = not requires_token or (token is not None and token != previous_token)
            telemetry_check = getattr(mcu, "telemetry_ok", None)
            telemetry_reason = (
                telemetry_check(require_upper=False)
                if callable(telemetry_check) and not require_upper
                and gimbal_mod._accepts_keyword(telemetry_check, "require_upper")
                else telemetry_check() if callable(telemetry_check) else None
            )
            odom_ok = not hasattr(mcu, "odom_mm") or mcu.odom_mm is not None
            if (
                token_ok and state is not None and state.state_flags & StateFlag.ODOM_VALID
                and _active_fault_bits(state) == 0 and mcu.yaw_deg is not None and odom_ok
                and directional_ok and (not need_upper or upper_ok) and telemetry_reason is None
            ):
                return CheckResult("启动遥测刷新", True, True,
                                   "外设初始化后 STOP 已确认，运行里程、航向和测距已重新接收")
            reason = f"state_new={token_ok} odom={odom_ok} directional={directional_reason} upper={upper_reason} telemetry={telemetry_reason}"
            time.sleep(max(0.0, min(0.02, refresh_deadline - time.monotonic())))
        return CheckResult("启动遥测刷新", False, True, reason)
    except Exception as exc:  # noqa: BLE001
        runlog.exception("selfcheck.runtime_refresh", "初始化后的运行遥测刷新失败", exc)
        return CheckResult("启动遥测刷新", False, True, str(exc))


def _run_checks(c: dict, *, full: bool, deadline: float | None) -> list[CheckResult]:
    results: list[CheckResult] = []
    runlog.status(phase="selfcheck", component="map")
    map_result = _check_map(deadline)
    results.append(map_result)
    runlog.emit("selfcheck.component", map_result.message, component="map",
                check=map_result.name, ok=map_result.ok, critical=True)
    results.append(_component_check(c, "clues", "线索映射", full,
                   lambda obj: _check_clues(obj, deadline)))
    conflict = c.get("rfid") is not None and getattr(c["rfid"], "port", None) == config.MCU_PORT
    if conflict:
        results.append(CheckResult("端口冲突", False, True, f"MCU/RFID 配置争用 {config.MCU_PORT}"))
    mcu_result = _component_check(c, "mcu", "STM32 串口", True,
                                 lambda obj: CheckResult("STM32 串口", False, True, "端口冲突，未打开设备")
                                 if conflict else _check_mcu(obj, deadline, require_upper=False))
    results.append(mcu_result)
    protection = _component_check(c, "mcu", "零速与STOP保护", True,
                                 lambda obj: _check_motion_protection(obj, deadline)
                                 if mcu_result.ok else CheckResult("零速与STOP保护", False, True, "MCU 检查未通过"))
    results.append(protection)
    results.append(_component_check(c, "gimbal", "云台往返" if full else "舵机水平居中", True,
                   lambda obj: (_check_servo_sweep(c["mcu"], obj, deadline) if full
                                else _check_servo_look(c["mcu"], obj, deadline))
                   if protection.ok else CheckResult("云台", False, True, "运动保护未确认，未开始云台检查")))
    frames: list = []
    results.append(_component_check(c, "camera", "摄像头", True,
                   lambda obj: _check_camera(obj, deadline, frames)))
    results.append(_component_check(c, "detector", "BPU 模型", full,
                   lambda obj: _check_model(obj, deadline, frames[0] if frames else None, infer=full)))
    results.append(_component_check(c, "audio", "声卡", full,
                   lambda obj: _check_audio(obj, deadline, c.get("clues")) if full
                   else _initialize_audio(obj, deadline, c.get("clues"))))
    results.append(_component_check(c, "rfid", "RFID", full,
                   lambda obj: CheckResult("RFID", False, full, "端口冲突，未打开设备")
                   if conflict else _check_rfid(obj, deadline, probe=full)))
    results.append(_component_check(c, "mcu", "启动遥测刷新", True,
                   lambda obj: _refresh_runtime_ready(obj, deadline, require_upper=False) if mcu_result.ok
                   else CheckResult("启动遥测刷新", False, True, "MCU 检查未通过")))
    results.append(_component_check(c, "mcu", "上层激光", False, _check_upper))
    return results


def run_selfcheck(*, components: dict | None = None, passive: bool = False) -> list[CheckResult]:
    """调试自检逐项执行；被动模式只发送 HELLO 并接收遥测。"""
    c = components or {}
    deadline = time.monotonic() + getattr(config, "SELFCHECK_TIME_LIMIT_S", 300.0)
    if passive:
        return [_component_check(c, "mcu", "STM32 串口", True,
                                 lambda obj: _check_mcu(obj, deadline, passive=True, require_upper=False)),
                _component_check(c, "mcu", "上层激光", False, _check_upper)]
    return _run_checks(c, full=True, deadline=deadline)


def run_startup_checks(*, components: dict | None = None, deadline: float | None = None) -> list[CheckResult]:
    """正式启动仅初始化外设并确认运动安全，任务性组件失败允许降级。"""
    return _run_checks(components or {}, full=False, deadline=deadline)


def write_log(results: list[CheckResult], log_dir: Path | None = None) -> Path:
    log_dir = Path(log_dir) if log_dir is not None else config.LOG_DIR
    log_dir.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = log_dir / f"selfcheck_{ts}.log"
    lines = [f"自检时间: {ts}"]
    for r in results:
        lines.append(f"{r.mark()} [{r.name}] {'关键' if r.critical else '一般'}: {r.message}")
    all_ok = all(r.ok for r in results if r.critical)
    lines.append("结论: " + ("全部关键项通过，可启动" if all_ok else "存在关键项失败，请排查后重试"))
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def print_results(results: list[CheckResult]) -> None:
    for r in results:
        print(f"{r.mark()} {r.name}: {r.message}")

_DIAG_FRAME_SAMPLE_LIMIT = 256
_DIAG_MAX_DURATION_S = 120.0


def _jsonable(value):
    """把协议 dataclass / Enum 转成可直接写入 JSON 的值。"""
    # IntEnum/IntFlag 是线上冻结的数值字段。保留整数使 JSON 可被诊断文本和
    # 离线分析程序无歧义地重放；普通 Enum 则保留可读名称。
    if isinstance(value, (IntEnum, IntFlag)):
        return int(value)
    if is_dataclass(value):
        return {item.name: _jsonable(getattr(value, item.name)) for item in fields(value)}
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes(value).hex()
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, Path):
        return str(value)
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return repr(value)


def _as_int(value, default: int = 0) -> int:
    """诊断遥测允许字段缺失；缺失值显示为默认值而不丢失整份报告。"""
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return default


def _read_text_if_present(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return None


def _process_cmdline(proc_path: Path) -> str:
    try:
        raw = (proc_path / "cmdline").read_bytes()
    except OSError:
        raw = b""
    if raw:
        return raw.replace(b"\0", b" ").decode("utf-8", "replace").strip()
    return _read_text_if_present(proc_path / "comm") or "?"


def _device_openers(device: str) -> list[dict[str, object]]:
    """只读枚举当前打开指定设备节点的进程。"""
    if os.name != "posix":
        return []
    real_device = os.path.realpath(device)
    openers: list[dict[str, object]] = []
    proc_root = Path("/proc")
    if not proc_root.exists():
        return openers
    for proc_path in proc_root.iterdir():
        if not proc_path.name.isdigit():
            continue
        fd_dir = proc_path / "fd"
        try:
            fds = list(fd_dir.iterdir())
        except OSError:
            continue
        matching_fds: list[str] = []
        for fd_path in fds:
            try:
                target = os.readlink(fd_path)
            except OSError:
                continue
            if os.path.realpath(target.removesuffix(" (deleted)")) == real_device:
                matching_fds.append(fd_path.name)
        if matching_fds:
            openers.append({
                "pid": _as_int(proc_path.name),
                "fds": matching_fds,
                "command": _process_cmdline(proc_path),
            })
    return openers


def _serial_diagnostics(configured_ports: list[str]) -> dict[str, object]:
    """只读记录 USB-TTY 映射和占用，定位 CH340 换口、掉线或串口争用。"""
    report: dict[str, object] = {
        "configured_ports": configured_ports,
        "ports": [],
        "by_id_links": [],
    }
    if os.name != "posix":
        report["reason"] = "当前不是 Linux/RDK，未探测 USB-TTY"
        return report

    by_id_for_device: dict[str, list[str]] = {}
    by_id_root = Path("/dev/serial/by-id")
    if by_id_root.exists():
        for link in sorted(by_id_root.iterdir()):
            try:
                target = os.path.realpath(link)
            except OSError:
                continue
            record = {"link": str(link), "target": target}
            links = report["by_id_links"]
            assert isinstance(links, list)
            links.append(record)
            by_id_for_device.setdefault(target, []).append(str(link))

    listed: dict[str, dict[str, object]] = {}
    try:
        from serial.tools import list_ports

        for info in list_ports.comports():
            device = str(info.device)
            listed[device] = {
                "device": device,
                "description": str(info.description or ""),
                "hwid": str(info.hwid or ""),
                "vid": info.vid,
                "pid": info.pid,
                "serial_number": info.serial_number,
                "location": info.location,
                "manufacturer": info.manufacturer,
                "product": info.product,
                "interface": info.interface,
            }
    except Exception as exc:  # noqa: BLE001 - 诊断不能因枚举库异常中断
        report["enumeration_error"] = str(exc)

    tracked = list(dict.fromkeys([*configured_ports, *sorted(listed)]))
    port_records: list[dict[str, object]] = []
    for device in tracked:
        device_path = Path(device)
        record = dict(listed.get(device, {"device": device}))
        exists = device_path.exists()
        record["exists"] = exists
        record["realpath"] = os.path.realpath(device)
        record["by_id"] = by_id_for_device.get(str(record["realpath"]), [])
        record["openers"] = _device_openers(device) if exists else []
        if exists:
            try:
                device_stat = device_path.stat()
                record["mode"] = f"0{stat.S_IMODE(device_stat.st_mode):o}"
                record["uid"] = device_stat.st_uid
                record["gid"] = device_stat.st_gid
            except OSError as exc:
                record["stat_error"] = str(exc)
        port_records.append(record)
    report["ports"] = port_records
    return report



def _system_diagnostics() -> dict[str, object]:
    local_now = datetime.now().astimezone()
    return {
        "wall_time_local": local_now.isoformat(),
        "wall_time_utc": datetime.now(timezone.utc).isoformat(),
        "unix_epoch_s": time.time(),
        "monotonic_s": time.monotonic(),
        "clock_looks_unset": local_now.year <= 2020,
        "platform": platform.platform(),
        "python": sys.version.split()[0],
    }


def _frame_record(frame, received_monotonic_s: float) -> dict[str, object]:
    message_type = getattr(frame, "message_type", None)
    payload = bytes(getattr(frame, "payload", b""))
    return {
        "received_monotonic_s": round(received_monotonic_s, 6),
        "message_type": getattr(message_type, "name", str(message_type)),
        "sequence": _as_int(getattr(frame, "sequence", 0)),
        "payload_length": len(payload),
        "payload_hex": payload.hex(),
    }


def _take_parser_issues(mcu) -> list[dict[str, object]]:
    """读取并清空本轮解析问题；诊断模式必须保留坏帧证据而非只留最后一条。"""
    getter = getattr(mcu, "pop_parse_issues", None)
    if not callable(getter):
        getter = getattr(mcu, "parser_issues", None)
    if not callable(getter):
        return []
    try:
        issues = getter()
    except Exception as exc:  # noqa: BLE001 - 诊断采集不能因附加证据失败而中断
        return [{"code": "ISSUE_READ_ERROR", "detail": str(exc)}]
    result: list[dict[str, object]] = []
    for issue in issues or ():
        item = _jsonable(issue)
        if isinstance(item, dict):
            result.append(item)
        else:
            # ParserIssue 是 dataclass；同时接受串口适配层常见的轻量 code/detail 对象，
            # 这样诊断报告不会把 BAD_CRC 等具体根因降级成 UNKNOWN。
            code = getattr(issue, "code", None)
            detail = getattr(issue, "detail", None)
            if code is not None:
                result.append(
                    {
                        "code": str(code),
                        "detail": str(detail) if detail is not None else str(issue),
                    }
                )
            else:
                result.append({"code": "UNKNOWN", "detail": str(item)})
    return result


def _append_parser_issues(
    report: dict[str, object], mcu, *, limit: int = _DIAG_FRAME_SAMPLE_LIMIT
) -> None:
    issues = _take_parser_issues(mcu)
    if not issues:
        return
    all_issues = report["parser_issues"]
    counts = report["parser_issue_counts"]
    assert isinstance(all_issues, list)
    assert isinstance(counts, dict)
    for issue in issues:
        code = str(issue.get("code", "UNKNOWN"))
        counts[code] = _as_int(counts.get(code, 0)) + 1
        if len(all_issues) < limit:
            all_issues.append(issue)


def _mcu_snapshot(mcu) -> dict[str, object]:
    capabilities = _as_int(getattr(mcu, "capabilities", 0))
    negotiated = _as_int(getattr(mcu, "negotiated_required_capabilities", 0))
    diagnostics_ok, diagnostics_reason = _diagnostics_health(mcu)
    return {
        "connected": bool(getattr(mcu, "connected", False)),
        "port": str(getattr(mcu, "port", "")),
        "baud": _as_int(getattr(mcu, "baud", 0)),
        "build_id": str(getattr(mcu, "build_id", "")),
        "boot_id": _as_int(getattr(mcu, "boot_id", 0)),
        "capabilities": capabilities,
        "capabilities_hex": f"0x{capabilities:08X}",
        "negotiated_capabilities": negotiated,
        "negotiated_capabilities_hex": f"0x{negotiated:08X}",
        "last_parse_error": getattr(mcu, "last_parse_error", None),
        "state": _jsonable(getattr(mcu, "state", None)),
        "directional": _jsonable(getattr(mcu, "directional", None)),
        "culvert_upper": _jsonable(getattr(mcu, "culvert_upper", None)),
        "light_state": _jsonable(getattr(mcu, "light_state", None)),
        "diagnostics": _jsonable(getattr(mcu, "diagnostics", None)),
        "wheel_trace": _jsonable(getattr(mcu, "wheel_trace", None)),
        "trace_sample_gaps": _as_int(getattr(mcu, "trace_sample_gaps", 0)),
        "capture": mcu.capture_status() if callable(getattr(mcu, "capture_status", None)) else {},
        "diagnostics_fresh": diagnostics_ok,
        "diagnostics_reason": diagnostics_reason or None,
    }


def _diagnostic_ports() -> list[str]:
    """被动诊断只能写入已配置的 MCU 串口，绝不探测 RFID/备用口。"""
    return [config.MCU_PORT]


def run_passive_diagnose(mcu, *, duration_s: float = 20.0) -> dict[str, object]:
    """采集 MCU 证据，不发送 CONTROL、STOP、云台或灯光命令。

    该函数唯一会写串口的操作是 ``mcu.hello()`` 内部的 HELLO_REQ；关闭时直接
    ``close()``，特意不走 ``shutdown()``，以保证诊断阶段不额外发送 STOP。
    """
    duration_s = float(duration_s)
    if not (0.0 < duration_s <= _DIAG_MAX_DURATION_S):
        raise ValueError(f"duration_s 必须在 (0, {_DIAG_MAX_DURATION_S:g}] 秒内")

    start_monotonic = time.monotonic()
    ports = _diagnostic_ports()
    report: dict[str, object] = {
        "schema_version": 1,
        "run_id": f"mono{time.monotonic_ns()}",
        "mode": "passive_diagnose",
        "safety": {
            "sent_commands": ["HELLO_REQ"],
            "not_sent": ["CONTROL_TARGET", "STOP", "LIGHT_CONTROL", "audio"],
        },
        "requested_duration_s": duration_s,
        "system": _system_diagnostics(),
        "serial": _serial_diagnostics(ports),
        "mcu": {
            "attempts": [],
            "hello_ok": False,
            "frame_counts": {},
            "frame_samples": [],
            "frames_total": 0,
            "parser_issues": [],
            "parser_issue_counts": {},
        },
    }
    mcu_report = report["mcu"]
    assert isinstance(mcu_report, dict)
    attempts = mcu_report["attempts"]
    assert isinstance(attempts, list)

    hello_ok = False
    try:
        for port in ports:
            mcu.port = port
            attempt: dict[str, object] = {"port": port}
            attempts.append(attempt)
            try:
                mcu.open()
                mcu.hello()
                _append_parser_issues(mcu_report, mcu)
                attempt["result"] = "hello_ok"
                hello_ok = True
                break
            except Exception as exc:  # noqa: BLE001 - 诊断必须把故障写进报告
                attempt["result"] = "error"
                attempt["error"] = str(exc)
                mcu.close()

        mcu_report["hello_ok"] = hello_ok
        if hello_ok:
            frame_counts: dict[str, int] = {}
            frame_samples: list[dict[str, object]] = []
            end_monotonic = time.monotonic() + duration_s
            while time.monotonic() < end_monotonic:
                try:
                    frames = mcu.pump()
                except Exception as exc:  # noqa: BLE001
                    mcu_report["pump_error"] = str(exc)
                    break
                received_monotonic = time.monotonic()
                for frame in frames:
                    record = _frame_record(frame, received_monotonic)
                    message_type = str(record["message_type"])
                    frame_counts[message_type] = frame_counts.get(message_type, 0) + 1
                    if len(frame_samples) < _DIAG_FRAME_SAMPLE_LIMIT:
                        frame_samples.append(record)
                _append_parser_issues(mcu_report, mcu)
                time.sleep(min(0.02, max(0.0, end_monotonic - time.monotonic())))
            mcu_report["frame_counts"] = frame_counts
            mcu_report["frame_samples"] = frame_samples
            mcu_report["frames_total"] = sum(frame_counts.values())
            _append_parser_issues(mcu_report, mcu)
        mcu_report["snapshot"] = _mcu_snapshot(mcu)
    finally:
        # 被动诊断不能调用 mcu.shutdown()，后者会发送 STOP(CLOSE)。
        mcu.close()

    report["elapsed_monotonic_s"] = round(time.monotonic() - start_monotonic, 3)
    return report


def _diagnostic_text(report: dict[str, object]) -> str:
    mcu = report.get("mcu", {})
    assert isinstance(mcu, dict)
    snapshot = mcu.get("snapshot", {})
    if not isinstance(snapshot, dict):
        snapshot = {}
    system = report.get("system", {})
    serial = report.get("serial", {})
    if not isinstance(system, dict):
        system = {}
    if not isinstance(serial, dict):
        serial = {}

    lines = [
        "RDK 被动诊断报告（可直接转发给 AI）",
        f"运行编号: {report.get('run_id', '?')}",
        "安全说明: 仅发送 HELLO_REQ 并读取遥测；未发送运动/STOP/云台/灯光，也未播放音频。",
        f"本地时间: {system.get('wall_time_local', '?')}",
        f"单调时钟: {system.get('monotonic_s', '?')}",
        f"系统时间疑似未校准: {'是' if system.get('clock_looks_unset') else '否'}",
        "",
        "[STM32]",
        f"HELLO: {'成功' if mcu.get('hello_ok') else '失败'}",
        f"端口: {snapshot.get('port', '?')}  波特率: {snapshot.get('baud', '?')}",
        f"固件 build: {snapshot.get('build_id') or '?'}",
        f"能力: advertised={snapshot.get('capabilities_hex', '?')} negotiated={snapshot.get('negotiated_capabilities_hex', '?')}",
        f"帧统计: {mcu.get('frame_counts', {})}",
    ]
    for attempt in mcu.get("attempts", []):
        if isinstance(attempt, dict):
            detail = attempt.get("error") or attempt.get("result")
            lines.append(f"串口尝试: {attempt.get('port', '?')} -> {detail}")

    state = snapshot.get("state")
    if isinstance(state, dict):
        fault_bits = _as_int(state.get("fault_bits", 0))
        lines.extend([
            f"STATE: odom_status=0x{_as_int(state.get('odom_status', 0)):02X} "
            f"state_flags=0x{_as_int(state.get('state_flags', 0)):02X}",
            f"fault_bits=0x{fault_bits:08X} ({_format_fault_bits(fault_bits)})",
        ])
    else:
        lines.append("STATE: 未收到")

    directional = snapshot.get("directional")
    if isinstance(directional, dict):
        statuses = directional.get("status", ())
        side_status = ", ".join(
            f"{name}={statuses[index]}" for index, name in _SIDE_CHANNELS
            if isinstance(statuses, (list, tuple)) and index < len(statuses)
        )
        lines.append(f"侧向 ToF 状态: {side_status or '?'}")
    else:
        lines.append("侧向 ToF: 未收到")
    upper = snapshot.get("culvert_upper")
    if isinstance(upper, dict):
        lines.append(f"上方 ToF 状态: {upper.get('status', '?')}")
    else:
        lines.append("上方 ToF: 未收到")
    diagnostics = snapshot.get("diagnostics")
    if isinstance(diagnostics, dict):
        gyro_status = _as_int(diagnostics.get("gyro_status", -1), -1)
        gyro_label = _GYRO_STATUS_LABELS.get(gyro_status, f"unknown={gyro_status}")
        tof_mask = _as_int(
            diagnostics.get("tof_initialized_mask", diagnostics.get("tof_init_mask", 0))
        )
        lines.extend([
            "DIAGNOSTICS: "
            f"fresh={snapshot.get('diagnostics_fresh', False)} "
            f"gyro={gyro_label} cal={_as_int(diagnostics.get('gyro_calibration_count', 0))} "
            f"sample_age_ms={_as_int(diagnostics.get('gyro_sample_age_ms', 0))}",
            "I2C: "
            f"tx={_as_int(diagnostics.get('i2c_transaction_count', 0))} "
            f"retry={_as_int(diagnostics.get('i2c_retry_count', 0))} "
            f"recovery={_as_int(diagnostics.get('i2c_recovery_count', 0))} "
            f"nack={_as_int(diagnostics.get('i2c_nack_count', 0))} "
            f"timeout={_as_int(diagnostics.get('i2c_timeout_count', 0))} "
            f"bus_error={_as_int(diagnostics.get('i2c_bus_error_count', 0))} "
            f"tof_initialized_mask=0x{tof_mask:02X}",
        ])
        internal_status = diagnostics.get("tof_internal_status", diagnostics.get("tof_status"))
        raw_status = diagnostics.get("tof_raw_status")
        if internal_status is not None:
            if isinstance(internal_status, (list, tuple)):
                internal_text = ",".join(
                    f"CH{index}={_internal_tof_name(internal_status[index])}"
                    for index in (1, 2, 4, 5) if index < len(internal_status)
                )
            else:
                internal_text = _internal_tof_name(internal_status)
            lines.append(f"ToF 内部状态: {internal_text}")
        if raw_status is not None:
            raw_text = ",".join(
                f"CH{index}={raw_status[index]}" for index in (1, 2, 4, 5)
                if isinstance(raw_status, (list, tuple)) and index < len(raw_status)
            )
            lines.append(f"ToF 原始状态: {raw_text or '?'}")
        if "last_stop_reason" in diagnostics:
            lines.append(f"最近本地停车原因: {diagnostics.get('last_stop_reason')}")
    else:
        lines.append(
            "DIAGNOSTICS: 未收到"
            + (f"（{snapshot.get('diagnostics_reason')}）" if snapshot.get("diagnostics_reason") else "")
        )
    root_causes = _diagnostic_root_causes(state, diagnostics)
    if root_causes:
        lines.append("建议排查: " + " | ".join(root_causes))
    parser_issue_counts = mcu.get("parser_issue_counts", {})
    if isinstance(parser_issue_counts, dict) and parser_issue_counts:
        lines.append(f"串口解析问题统计: {parser_issue_counts}")
    if snapshot.get("last_parse_error"):
        lines.append(f"串口解析/读取异常: {snapshot['last_parse_error']}")

    lines.extend(["", "[串口 / USB]"])
    ports = serial.get("ports", [])
    if isinstance(ports, list):
        for port in ports:
            if not isinstance(port, dict):
                continue
            device = port.get("device", "?")
            description = port.get("description") or "?"
            lines.append(
                f"{device}: exists={port.get('exists', False)} desc={description} "
                f"by-id={port.get('by_id', [])} openers={port.get('openers', [])}"
            )
    else:
        lines.append("未获取到 USB-TTY 枚举信息。")
    if serial.get("enumeration_error"):
        lines.append(f"串口枚举异常: {serial['enumeration_error']}")

    lines.extend([
        "",
        "请同时附上 PB10/PB11 的逻辑分析仪截图：应看到 0x70、0x68、以及选通后的 0x29。",
        "复查命令: python3 main.py diagnose --duration 20（车辆静止；仅 HELLO/遥测）。",
    ])
    return "\n".join(lines)


def write_diagnostic_report(
    report: dict[str, object], log_dir: Path | None = None
) -> tuple[Path, Path]:
    """写入 JSON 和可读文本；文件名只依赖单调运行编号，避免 2000 年时钟覆盖。"""
    log_dir = Path(log_dir) if log_dir is not None else config.LOG_DIR
    log_dir.mkdir(parents=True, exist_ok=True)
    run_id = str(report.get("run_id") or f"mono{time.monotonic_ns()}")
    json_path = log_dir / f"rdk_diagnose_{run_id}.json"
    text_path = log_dir / f"rdk_diagnose_{run_id}.txt"
    # 先完整渲染两种内容；任一格式的渲染失败时不留下另一种“半份诊断”。
    json_content = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    text_content = _diagnostic_text(report) + "\n"
    json_path.write_text(json_content, encoding="utf-8")
    text_path.write_text(text_content, encoding="utf-8")
    return text_path, json_path


def print_diagnostic_summary(report: dict[str, object]) -> None:
    print(_diagnostic_text(report))

