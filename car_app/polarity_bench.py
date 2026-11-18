#!/usr/bin/env python3
"""STM32 极性台架测试。

运行前：
- 电机/编码器测试时，先架空车轮；
- 陀螺仪测试时，把车放在安全、空旷、平整的地面；
- 本脚本只在 STATE 的 ODOM_VALID、未忽略 fault_bits、两侧测距健康时允许运动；
- 电机极性必须由人观察左右轮物理转向，脚本不会根据里程“猜”电机极性。
"""

from __future__ import annotations

import time

import config
import mcu as mcu_mod
import runlog
from drivers.protocol_v2 import (
    REQUIRED_CAPABILITIES,
    StateFlag,
    StopReason,
    WireRangeStatus,
)

# 固件仅接受完整的 v2.3/v9、0xEDB HELLO 能力集合，包含轮速追踪。
BENCH_REQUIRED_CAPABILITIES = int(REQUIRED_CAPABILITIES)

# 当前 board_config.h 中的值。若已经改过极性，必须同步更新这里；不能重复按 +1 推断。
CURRENT_ENCODER_LEFT_POLARITY = 1
CURRENT_ENCODER_RIGHT_POLARITY = 1
CURRENT_GYRO_Z_POLARITY = 1

COMMAND_PERIOD_S = 0.10  # 小于 500 ms 运动租约
READY_TIMEOUT_S = 2.0
CONTROL_CONFIRM_TIMEOUT_S = 0.25
MIN_ENCODER_DELTA_MM = 1.0
MIN_YAW_DELTA_DEG = 5.0


def _active_fault_bits(m: mcu_mod.McuLink, state) -> int:
    checker = getattr(m, "_active_fault_bits", None)
    if callable(checker):
        return int(checker())
    return int(getattr(state, "fault_bits", 0)) & ~int(
        config.MCU_IGNORED_RUNTIME_FAULT_BITS
    )


def _telemetry_ok(m: mcu_mod.McuLink, *, require_ranges: bool) -> str | None:
    checker = m.telemetry_ok
    try:
        return checker(require_ranges=require_ranges)
    except TypeError:
        # 测试替身和冻结的旧适配器没有可选参数时仍保留原有检查。
        return checker()


def _fresh_state(
    m: mcu_mod.McuLink,
    timeout_s: float = READY_TIMEOUT_S,
    *,
    require_ranges: bool = True,
):
    """等待新鲜且有效的 ODOM STATE；不把缺失状态伪装成 0。"""
    deadline = time.monotonic() + timeout_s
    last_odom_status = "unknown"
    last_fault_bits = 0
    while time.monotonic() < deadline:
        m.pump()
        state = m.state
        if state is not None:
            last_odom_status = f"0x{state.odom_status:02X}"
            last_fault_bits = state.fault_bits
            # 上电校准阶段 ODOM 可能暂时无效；继续等到 deadline，
            # 但一旦 ODOM 有效，任何 fault_bits 都必须拒绝运动。
            if state.state_flags & StateFlag.ODOM_VALID:
                if _active_fault_bits(m, state) != 0:
                    raise RuntimeError(f"固件 fault_bits=0x{state.fault_bits:08X}")
                fault = _telemetry_ok(m, require_ranges=require_ranges)
                if fault is not None:
                    raise RuntimeError(f"遥测故障：{fault}")
                return state
        time.sleep(0.02)
    raise RuntimeError(
        f"等待 STATE/ODOM_VALID 超时 odom_status={last_odom_status} "
        f"fault_bits=0x{last_fault_bits:08X}"
    )


def _require_motion_range(m: mcu_mod.McuLink) -> None:
    """等待 ODOM 与首次四向测距，避免把启动瞬间的空缓存当成故障。"""
    deadline = time.monotonic() + READY_TIMEOUT_S
    last_state = "STATE 缺失"
    while time.monotonic() < deadline:
        m.pump()
        state = m.state
        if state is not None:
            last_state = (
                f"odom_status=0x{state.odom_status:02X} "
                f"state_flags=0x{int(state.state_flags):02X}"
            )
            if _active_fault_bits(m, state) != 0:
                raise RuntimeError(f"固件 fault_bits=0x{state.fault_bits:08X}")
            if state.state_flags & StateFlag.ODOM_VALID and m.directional is not None:
                _check_motion_safety(m)
                return
        time.sleep(0.02)
    raise RuntimeError(f"等待 ODOM_VALID + DIRECTIONAL_RANGE4 超时：{last_state}")


def _emergency_stop(m: mcu_mod.McuLink) -> None:
    for stop in (lambda: m.send_motion(0, 0), lambda: m.send_stop(StopReason.MISSION)):
        try:
            stop()
        except Exception:
            pass


def _check_motion_safety(m: mcu_mod.McuLink, *, require_ranges: bool = True) -> None:
    """即时检查当前样本，不等待传感器恢复后继续运动。"""
    state = m.state
    if state is None or not state.state_flags & StateFlag.ODOM_VALID or m.odom_mm is None:
        raise RuntimeError("运动缺少新鲜有效里程")
    if _active_fault_bits(m, state):
        raise RuntimeError(f"运动期间 fault_bits=0x{state.fault_bits:08X}")
    fault = _telemetry_ok(m, require_ranges=require_ranges)
    if fault is not None:
        raise RuntimeError(f"运动期间遥测故障：{fault}")
    if not require_ranges:
        return
    for name, status in (
        ("left", m.left_status),
        ("right", m.right_status),
    ):
        allowed = (WireRangeStatus.VALID, WireRangeStatus.NO_TARGET,
                   WireRangeStatus.OUT_OF_RANGE)
        if status not in allowed:
            raise RuntimeError(f"{name} 测距无效：status={status}")


def _service_for(m: mcu_mod.McuLink, duration_s: float, *, require_ranges: bool) -> None:
    """运动期间持续收遥测并检查故障。"""
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        m.pump()
        _check_motion_safety(m, require_ranges=require_ranges)
        time.sleep(0.01)


def _stop_motion(m: mcu_mod.McuLink, *, require_ranges: bool = True) -> None:
    """零速回读后，以连续新鲜编码器样本确认机械停稳。"""
    sequence = m.send_motion(0, 0)
    m.wait_for_control_applied(sequence, timeout_s=CONTROL_CONFIRM_TIMEOUT_S)
    previous, previous_token = m.odom_mm, m.state_sample_token
    if previous is None or previous_token is None:
        raise RuntimeError("停车确认缺少新鲜里程")
    deadline = time.monotonic() + config.STOP_STABLE_TIMEOUT_S
    stable = 0
    while time.monotonic() < deadline:
        m.pump()
        fault = _telemetry_ok(m, require_ranges=require_ranges)
        current, token = m.odom_mm, m.state_sample_token
        if fault is not None or current is None or token is None:
            raise RuntimeError(f"停车期间遥测故障：{fault or '里程不可用'}")
        if token != previous_token:
            if max(abs(a - b) for a, b in zip(current, previous)) <= config.STOP_STABLE_DELTA_MM:
                stable += 1
                if stable >= config.STOP_STABLE_SAMPLES:
                    return
            else:
                stable = 0
            previous, previous_token = current, token
        time.sleep(0.01)
    raise RuntimeError("台架停车后编码器未在期限内确认停稳")


def _segment(
    m: mcu_mod.McuLink,
    linear_mmps: int,
    angular_mradps: int,
    seconds: float,
):
    """执行一段带心跳的运动，返回段前/段后 STATE。"""
    try:
        pure_rotation = linear_mmps == 0 and angular_mradps != 0
        require_ranges = not pure_rotation
        before = _fresh_state(m, require_ranges=require_ranges)
        end = time.monotonic() + seconds
        next_send = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if now < next_send:
                _service_for(m, min(next_send - now, end - now), require_ranges=require_ranges)
                continue
            m.pump()
            _check_motion_safety(m, require_ranges=require_ranges)
            if pure_rotation and callable(getattr(m, "send_rotation", None)):
                m.send_rotation(angular_mradps)
            else:
                m.send_motion(linear_mmps, angular_mradps)
            next_send += COMMAND_PERIOD_S
            if next_send <= now:
                next_send = now + COMMAND_PERIOD_S
        _stop_motion(m, require_ranges=require_ranges)
        after = _fresh_state(m, require_ranges=require_ranges)
        return before, after
    except BaseException:
        _emergency_stop(m)
        raise


def _signed_polarity(observed: float, current: int, label: str, threshold: float) -> int:
    if abs(observed) < threshold:
        raise RuntimeError(f"{label} 变化量过小：{observed}; 不输出极性建议")
    if current not in (-1, 1):
        raise RuntimeError(f"{label} 当前配置必须是 +1 或 -1")
    # 固件上报的是 raw_delta * 当前极性；目标极性 = 观测符号 * 当前极性。
    return current if observed > 0 else -current


def _wrapped_delta_deg(after: float, before: float) -> float:
    """计算 [-180, 180) 航向读数的最短角差，避免跨边界误判。"""
    delta = after - before
    while delta >= 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def main() -> int:
    m = mcu_mod.McuLink()
    try:
        m.open()
        response = m.hello()
        print("build:", response.build_id)
        _require_motion_range(m)
        print(f"初始 fault_bits=0x{m.state.fault_bits:08X}")

        # 前进：请观察左右轮各自是否物理向前。此段只自动计算编码器极性。
        before, after = _segment(m, 100, 0, 1.0)
        left_delta = after.left_mm - before.left_mm
        right_delta = after.right_mm - before.right_mm
        print(f"[前进] Δleft_mm={left_delta} Δright_mm={right_delta}")
        print(m.wheel_trace_summary())
        enc_left = _signed_polarity(
            left_delta, CURRENT_ENCODER_LEFT_POLARITY, "左编码器", MIN_ENCODER_DELTA_MM
        )
        enc_right = _signed_polarity(
            right_delta, CURRENT_ENCODER_RIGHT_POLARITY, "右编码器", MIN_ENCODER_DELTA_MM
        )

        # 正 angular 表示左转；要求车身确实完成了左转。
        yaw0 = after.yaw_tenth_deg / 10.0
        _, turn_after = _segment(m, 0, 500, 1.0)
        yaw1 = turn_after.yaw_tenth_deg / 10.0
        dyaw = _wrapped_delta_deg(yaw1, yaw0)
        print(f"[左转] yaw {yaw0:.1f} -> {yaw1:.1f} (Δ={dyaw:.1f}°)")
        print(m.wheel_trace_summary())
        gyro = _signed_polarity(
            dyaw, CURRENT_GYRO_Z_POLARITY, "陀螺仪 Z", MIN_YAW_DELTA_DEG
        )

        print("=" * 52)
        print("自动计算出的编码器/陀螺仪极性：")
        print(f"FW_ENCODER_LEFT_POLARITY  = {enc_left}")
        print(f"FW_ENCODER_RIGHT_POLARITY = {enc_right}")
        print(f"FW_GYRO_Z_POLARITY        = {gyro}")
        print("电机极性不能由里程自动推导，请按物理观察分别填写：")
        print("FW_MOTOR_LEFT_POLARITY    = 1  # 左轮正命令物理向前；反之 -1")
        print("FW_MOTOR_RIGHT_POLARITY   = 1  # 右轮正命令物理向前；反之 -1")
        print("=" * 52)
        return 0
    finally:
        # 无论中途哪一步异常，都先走有界 STOP/断串口。
        m.shutdown()


if __name__ == "__main__":
    with runlog.session("polarity_bench", periodic_status=False):
        raise SystemExit(main())
