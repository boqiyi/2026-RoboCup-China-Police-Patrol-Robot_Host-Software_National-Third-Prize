"""v11：停稳采样后全程xx前进、车轴差速纠偏与固定参考±5°居中核验。"""

from __future__ import annotations

import argparse
from collections import deque
import math
import os
import queue
import random
import re
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from enum import IntEnum

import config
import mcu
import runlog
from motion_speed_input import TerminalSpeedInput
from drivers.protocol_v2 import (
    AckStatus, DIAGNOSTICS_PAYLOAD_LENGTH, HelloRequest, MessageType,
    MotionActionCode, MotionActionFlag, ParserIssue, PROTOCOL_MAJOR, PROTOCOL_MINOR,
    SequenceDisposition, StateFlag, StopReason, WireGyroStatus, WireRangeStatus,
    WireSafetyStopReason, classify_sequence, decode_diagnostics, decode_directional_range4,
    decode_hello_response, encode_hello_request,
)


AVERAGE_INTERVAL_S = 0.5
COMMAND_INTERVAL_S = 0.1
CONTROL_INTERVAL_S = 0.02
LASER_MAX_AGE_MS = 500
LASER_MEASUREMENT_MAX_AGE_MS = 250
LASER_FRESH_AGE_MS = LASER_MEASUREMENT_MAX_AGE_MS
LASER_CACHE_MAX_AGE_MS = 2000
DEPARTURE_SIDE_TIMEOUT_S = 2.0
LINEAR_SPEED_MMPS = 97
TURN_SPEED_MMPS = 97
MIN_WHEEL_SPEED_MMPS = 61
MAX_WHEEL_SPEED_MMPS = 310
WHEEL_BASE_MM = 125
START_SAMPLE_S = 1.0
STOP_OBSERVATION_S = 0.5
JUNCTION_WAIT_S = 3.0
POST_TURN_WAIT_S = 1.0
POST_STOP_PAIRS = 2
EXPECTED_CAPABILITIES = 0xEDB
EXPECTED_BUILD_ID = "car12-single-wheel-v12"
SIDE_DEADBAND_MM = 5.0
HEALTHY_RANGE_STATUSES = frozenset((WireRangeStatus.VALID,
                                  WireRangeStatus.NO_TARGET,
                                  WireRangeStatus.OUT_OF_RANGE))

# 首轮实测初值；下层激光定位置，下位机陀螺仪闭环继续负责航向执行。
CENTER_FILTER_TAU_S = 0.15
CENTER_LOOKAHEAD_MM = 150.0
NORMAL_HEADING_TOLERANCE_DEG = 5.0
POSITION_INTEGRAL_LIMIT_DEG = 10.0
CENTER_FALLBACK_ANGULAR_MRADPS = 768
CENTER_TRUSTED_ANGULAR_MRADPS = 768
CENTER_ANGULAR_QUANTUM_MRADPS = 16
# Ordinary forward commands keep BOTH nominal wheel targets >=61.
CENTER_MIN_WHEEL_SPEED_MMPS = 61
CENTER_SENSOR_FORWARD_MM = 39.5
CENTER_ACCEL_MMPS2 = 100.0
CENTER_YAW_RATE_DAMPING = 0.2
CENTER_FORWARD_SHIFT_YAW_GAIN_PER_S = 4.0
CENTER_GYRO_MAX_AGE_MS = 250
CENTER_HISTORY_S = 4.0


def _remaining(deadline: float, message: str = "局部确认超时") -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise mcu.McuError(message)
    return remaining


class _UserQuit(Exception):
    pass


class _SideRecoveryRequired(mcu.McuError):
    """发送前的年龄边界变化需要返回停车恢复阶段。"""


class _CenteringProposalExpired(Exception):
    """发送证据在写入前失效，下一控制周期重新计算。"""


def _check_input_exit(input_source) -> None:
    if input_source is not None:
        line = input_source.poll()
        if isinstance(line, str) and line.strip().lower() == "q":
            raise _UserQuit("收到 q，尝试停车")


def _parse_speed(line: str) -> int:
    match = re.fullmatch(r"([0-9]+)", line.strip())
    maximum, minimum = MAX_WHEEL_SPEED_MMPS, CENTER_MIN_WHEEL_SPEED_MMPS
    if match is None or len(match.group(1)) > 3:
        raise ValueError(f"请输入 {minimum}～{maximum} 的整数速度，例如 80")
    speed = int(match.group(1))
    if not minimum <= speed <= maximum:
        raise ValueError(f"速度超出范围：允许 {minimum}～{maximum} mm/s")
    return speed


def _wait_speed(link: mcu.McuLink, display: MotionDisplay, input_source) -> int:
    """仅在起步前输入一次；等待期间持续接收通信、监测故障。"""
    prompt = "车辆已停车。请输入 xx（61～310 mm/s 整数），回车确认；q 退出："
    display.action = "停车等待xx"
    display.set_stopped()
    display.confirmation = "等待唯一一次速度输入；激光与陀螺仪持续监测"
    display.maybe_print(force=True)
    try:
        input_source.begin_request(prompt)
        while True:
            display.tick()
            display.consume_samples()
            # 等待期间保持静默，避免遥测输出干扰输入；同时及时清空显示缓存。
            display._ready_batches.clear()
            line = input_source.poll()
            if line is not None:
                if line.strip().lower() == "q":
                    raise _UserQuit("收到 q，尝试停车")
                try:
                    speed = _parse_speed(line)
                except ValueError as exc:
                    display.emitter(f"{exc}；车辆继续停车。")
                    input_source.end_request()
                    input_source.begin_request(prompt)
                else:
                    display.input_speed_mmps = speed
                    display.emitter(f"已填写 xx={speed} mm/s；核验停稳后继续停车采样1秒。")
                    return speed
            time.sleep(CONTROL_INTERVAL_S)
    finally:
        input_source.end_request()


def _gyro_health_reason(link: mcu.McuLink) -> str | None:
    diagnostics = link.diagnostics
    if diagnostics is None:
        return "缺少陀螺仪诊断遥测"
    status = diagnostics.gyro_status
    if status != WireGyroStatus.READY:
        return f"陀螺仪未就绪：状态={getattr(status, 'name', status)}"
    age = diagnostics.gyro_sample_age_ms
    if isinstance(age, bool) or not isinstance(age, int) or age < 0:
        return "陀螺仪采样年龄无效"
    sample_time = getattr(link, "_sample_time", None)
    if callable(sample_time):
        age += max(0.0, (time.monotonic() - sample_time("diagnostics")) * 1000.0)
    if age > LASER_MAX_AGE_MS:
        return f"陀螺仪采样超龄：年龄={age:.1f} ms"
    return None


@dataclass(frozen=True)
class _SideSample:
    seq: int
    left_mm: int | None
    left_status: WireRangeStatus
    right_mm: int | None
    right_status: WireRangeStatus
    received_at: float
    source_times: tuple[float, float]
    ages_ms: tuple[int, int]
    source_keys: tuple[int, int] | None = None
    raw_ages_ms: tuple[int, int] | None = None
    receive_delay_ms: float = 0.0
    candidate_sides: frozenset[str] | None = None
    snapshot_uptime_ms: int | None = None
    source_identity: tuple[int | None, int | None] | None = None

    def diagnostics(self, now: float) -> dict:
        elapsed_ms = max(0.0, (now - self.received_at) * 1000.0)
        return dict(cycle_seq=self.seq, snapshot_uptime_ms=self.snapshot_uptime_ms,
                    raw_statuses=tuple(getattr(status, "name", str(status))
                                       for status in (self.left_status, self.right_status)),
                    distances_mm=(self.left_mm, self.right_mm),
                    raw_ages_ms=self.raw_ages_ms,
                    ages_ms=tuple(age + elapsed_ms for age in self.ages_ms),
                    receive_delay_ms=self.receive_delay_ms)

    @property
    def openings(self) -> frozenset[str]:
        return frozenset(name for name, status in (
            ("left", self.left_status), ("right", self.right_status),
        ) if status in (WireRangeStatus.NO_TARGET, WireRangeStatus.OUT_OF_RANGE))

    @property
    def dual_wall(self) -> bool:
        return self.left_status == self.right_status == WireRangeStatus.VALID

    @property
    def candidates(self) -> frozenset[str]:
        # Injected bench samples predate the receive-time eligibility metadata.
        return self.openings if self.candidate_sides is None else self.candidate_sides


def _side_snapshot_key(sample: _SideSample) -> tuple[int, int | None]:
    return sample.seq, sample.snapshot_uptime_ms


snapshot_key = _side_snapshot_key


def _source_key_newer(key, previous) -> bool:
    if previous is None:
        return True
    if isinstance(key, int) and isinstance(previous, int):
        return 0 < ((key - previous) & 0xFFFFFFFF) < 0x80000000
    return key > previous + 1e-9


def _snapshot_newer(sample: _SideSample, previous: _SideSample | None) -> bool:
    if previous is None:
        return True
    disposition = classify_sequence(sample.seq, previous.seq)
    if sample.seq != previous.seq and disposition is not SequenceDisposition.NEWER:
        return False
    if sample.snapshot_uptime_ms is not None and previous.snapshot_uptime_ms is not None:
        return _source_key_newer(sample.snapshot_uptime_ms, previous.snapshot_uptime_ms)
    return disposition is SequenceDisposition.NEWER


def _side_sample(link: mcu.McuLink) -> _SideSample:
    directional, ages = link.directional, link.directional_age_ms
    if (directional is None or ages is None or len(ages) != 4
            or len(directional.status) != 4 or len(directional.distance_mm) != 4):
        raise mcu.McuError("缺少下层激光周期、状态或样本年龄")
    now = time.monotonic()
    # 实际安装：CH1 右下、CH2 左下；内部样本始终按左、右排列。
    side_indices = (2, 1)
    side_ages = tuple(ages[index] for index in side_indices)
    if any(isinstance(age, bool) or not isinstance(age, int) or age < 0
           for age in side_ages):
        raise mcu.McuError("下层激光样本年龄无效")
    uptime = directional.uptime_ms
    keys = tuple((uptime - directional.age_ms[index]) & 0xFFFFFFFF
                 for index in side_indices)
    sample_time = getattr(link, "_sample_time", None)
    receive_delay_ms = (max(0.0, (link._last_directional_recv - sample_time("directional")) * 1000.0)
                        if callable(sample_time) else 0.0)
    source_times = tuple(now - age / 1000.0 for age in side_ages)
    if callable(sample_time):
        # 保留实际源时间；重复回传和取整后的年龄不能把旧 VALID 重新变新。
        generated_at = sample_time("directional")
        source_times = tuple(generated_at - directional.age_ms[index] / 1000.0
                             for index in side_indices)
    return _SideSample(
        directional.directional_cycle_seq,
        directional.distance_mm[2], directional.status[2],
        directional.distance_mm[1], directional.status[1], now,
        source_times, side_ages, keys,
        tuple(directional.age_ms[index] for index in side_indices),
        receive_delay_ms, snapshot_uptime_ms=uptime,
        source_identity=(getattr(link, "session_nonce", None), getattr(link, "boot_id", None)))


def _check_sides(sample: _SideSample, now: float, *, check_age: bool = True,
                 allow_stale: bool = False, allow_timeout: bool = False) -> None:
    elapsed_ms = max(0.0, (now - sample.received_at) * 1000.0)
    for name, distance, status, age in (
        ("左下CH2", sample.left_mm, sample.left_status, sample.ages_ms[0]),
        ("右下CH1", sample.right_mm, sample.right_status, sample.ages_ms[1]),
    ):
        age += elapsed_ms
        if status not in HEALTHY_RANGE_STATUSES and not (
                allow_stale and status == WireRangeStatus.STALE) and not (
                allow_timeout and status == WireRangeStatus.TIMEOUT):
            raise mcu.McuError(f"{name}激光故障：状态={getattr(status, 'name', status)}"
                               f"({status})，年龄={age:.1f} ms")
        if status == WireRangeStatus.VALID and (
            isinstance(distance, bool) or not isinstance(distance, (int, float))
            or not math.isfinite(distance) or not 0 <= distance < 65535
        ):
            raise mcu.McuError(f"{name}有效状态对应的距离数值非法")
        if check_age and age > LASER_MEASUREMENT_MAX_AGE_MS + 1e-6:
            raise mcu.McuError(f"{name}测量超龄：年龄={age:.1f} ms，上限={LASER_MEASUREMENT_MAX_AGE_MS} ms")


def _cached_side_feedback(link, sample: _SideSample, now: float, *,
                          require_stale: bool = True) -> bool:
    """4/6 或旧 VALID 仅可复用本段双墙缓存，不成为新的测量证据。"""
    if (not getattr(link, "side_age_required", True)
            or not getattr(link, "side_motion_active", False)
            or not getattr(link, "side_candidates_enabled", False)
            or not (getattr(link, "_active_motion_sent_at", None) is not None
                    or getattr(link, "moving", False))
            or getattr(link, "side_recovery_active", False)
            or getattr(link, "side_departure_active", False)
            or getattr(link, "_side_brake_pending", False)
            or getattr(link, "_stable_barrier", None) is not None):
        return False
    statuses = sample.left_status, sample.right_status
    missing = (any(status in (WireRangeStatus.STALE, WireRangeStatus.TIMEOUT)
                   for status in statuses)
               or any(now - source > LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                      for source in sample.source_times))
    if (require_stale and not missing
            or any(status not in (WireRangeStatus.VALID, WireRangeStatus.STALE,
                                  WireRangeStatus.TIMEOUT)
                   for status in statuses)):
        return False
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    barrier = getattr(link, "_side_source_barrier", None)
    cache = getattr(link, "_valid_side_cache", (None, None))
    return all(entry is not None and entry[:2] == identity
               and 0.0 <= now - entry[3] <= LASER_CACHE_MAX_AGE_MS / 1000.0 + 1e-9
               and (barrier is None or entry[3] >= barrier - 1e-9)
               for entry in cache)


def _side_recovery_needed(link, sample: _SideSample, now: float) -> bool:
    """同一门禁用于接收、健康检查与消费；真实错误始终优先。"""
    _check_sides(sample, now, check_age=False, allow_stale=True,
                 allow_timeout=True)
    if _pending_junction_openings(link):
        # 已取得真实开口：后续缺测或超龄不能推翻停车转弯决定。
        # 真实传感器错误仍由上面的逐帧检查中止。
        _latch_junction(link, sample)
        link.side_cached_active = False
        return False
    latest = getattr(link, "_latest_side_sample", None)
    if latest is not None and _side_snapshot_key(latest) != _side_snapshot_key(sample):
        return False  # 历史快照的软缺测不能覆盖最新状态；真实错误已在上面检查。
    normal = getattr(link, "side_age_required", True)
    moving = (getattr(link, "_active_motion_sent_at", None) is not None
              or getattr(link, "moving", False)
              or getattr(link, "_sending_nonzero", False))
    if not normal or not moving:
        link.side_cached_active = False
        return False
    if _ready_feedback_valid(link, now):
        # Fresh stop samples keep their original source time while authorizing the send.
        # They never create new control evidence while bridging a short miss.
        link.side_cached_active = True
        return False
    if getattr(link, "side_departure_active", False):
        link.side_cached_active = False
        identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
        barrier = getattr(link, "_departure_barrier", None)
        if barrier is None or getattr(link, "_departure_identity", None) != identity:
            return True
        observations = getattr(link, "_departure_side_sources", (None, None))
        return any(now - (entry[1] if entry is not None else barrier)
                   >= DEPARTURE_SIDE_TIMEOUT_S - 1e-9 for entry in observations)
    cached = _cached_side_feedback(link, sample, now)
    link.side_cached_active = cached
    return not cached and (
        any(status in (WireRangeStatus.STALE, WireRangeStatus.TIMEOUT)
            for status in (sample.left_status, sample.right_status))
        or any(now - source > LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
               for source in sample.source_times))


def _annotate_side_sample(link, sample: _SideSample) -> _SideSample:
    """在源事件到达时固定候选资格；消费时不能拿更新后的缓存重算资格。"""
    recent = getattr(link, "_recent_valid_sides", None)
    if recent is None:
        recent = link._recent_valid_sides = [None, None]
    valid_cache = getattr(link, "_valid_side_cache", None)
    if valid_cache is None:
        valid_cache = link._valid_side_cache = [None, None]
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    observed = getattr(link, "_observed_side_sources", None)
    if observed is None:
        observed = link._observed_side_sources = [None, None]
    candidates = set()
    for index, (status, age) in enumerate(zip(
            (sample.left_status, sample.right_status), sample.ages_ms)):
        key = sample.source_keys[index] if sample.source_keys is not None else sample.source_times[index]
        previous = recent[index]
        last_source = observed[index]
        fresh_source = (last_source is None or last_source[:2] != identity
                        or _source_key_newer(key, last_source[2]))
        name = "left" if index == 0 else "right"
        if (status == WireRangeStatus.NO_TARGET and fresh_source
                and age <= LASER_MEASUREMENT_MAX_AGE_MS):
            candidates.add(name)
        elif (status == WireRangeStatus.OUT_OF_RANGE and fresh_source
              and age <= LASER_MEASUREMENT_MAX_AGE_MS):
            candidates.add(name)
        if status in HEALTHY_RANGE_STATUSES and fresh_source:
            observed[index] = (*identity, key)
        # 延迟收到的 VALID 仍是成功测量，可作为2s缓存；不能算250ms当期证据。
        raw_age = sample.raw_ages_ms[index] if sample.raw_ages_ms is not None else age
        if (status == WireRangeStatus.VALID and age <= LASER_CACHE_MAX_AGE_MS
                and raw_age <= LASER_MEASUREMENT_MAX_AGE_MS):
            if previous is None or previous[:2] != identity or _source_key_newer(key, previous[2]):
                recent[index] = (*identity, key)
            distance = (sample.left_mm, sample.right_mm)[index]
            cached = valid_cache[index]
            newer = (cached is None or cached[:2] != identity
                     or _source_key_newer(key, cached[2]))
            if (newer and not isinstance(distance, bool)
                    and isinstance(distance, (int, float)) and math.isfinite(distance)
                    and 0 <= distance < 65535):
                valid_cache[index] = (*identity, key, sample.source_times[index], distance)
        if (getattr(link, "side_departure_active", False)
                and getattr(link, "_departure_identity", None) == identity
                and status in HEALTHY_RANGE_STATUSES
                and age <= LASER_MEASUREMENT_MAX_AGE_MS
                and sample.source_times[index] > link._departure_barrier + 1e-9):
            entry = link._departure_side_sources[index]
            if entry is None or _source_key_newer(key, entry[0]):
                link._departure_side_sources[index] = (key, sample.source_times[index])
    sample = replace(sample, candidate_sides=frozenset(candidates))
    return sample


def _pending_junction_openings(link) -> frozenset[str]:
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    if (getattr(link, "_junction_identity", None) != identity
            or getattr(link, "_hard_fault_reason", None) is not None):
        return frozenset()
    return getattr(link, "_junction_openings", frozenset())


def _clear_junction(link) -> None:
    link._junction_identity = None
    link._junction_openings = frozenset()
    link._junction_side_sources = [None, None]
    link._side_brake_pending = False


def _junction_wait_remaining(link, now: float | None = None) -> float:
    """The wait belongs to a successfully sent BRAKE in this MCU session."""
    now = time.monotonic() if now is None else now
    sent_at = getattr(link, "_last_brake_sent_at", None)
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    if sent_at is None or getattr(link, "_last_brake_identity", None) != identity:
        return math.inf
    return max(0.0, sent_at + JUNCTION_WAIT_S - now)


def _latch_junction(link, sample: _SideSample, *, initial: bool = False) -> None:
    """开口决定与距离解耦；停车期间只合并真正更新的1/2源事件。"""
    now = time.monotonic()
    _check_sides(sample, now, check_age=False, allow_stale=True, allow_timeout=True)
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    openings = set(_pending_junction_openings(link))
    if not openings:
        if not initial:
            return
        for index, (name, status) in enumerate((("left", sample.left_status),
                                               ("right", sample.right_status))):
            if (name in sample.candidates
                    and status in (WireRangeStatus.NO_TARGET, WireRangeStatus.OUT_OF_RANGE)
                    and 0 <= now - sample.source_times[index]
                    <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9):
                openings.add(name)
        if not openings:
            raise mcu.McuError("路口候选缺少250ms内合法新鲜1/2开口")
        link._junction_identity = identity
        observed = getattr(link, "_observed_side_sources", (None, None))
        keys = sample.source_keys or sample.source_times
        link._junction_side_sources = [
            keys[index] if status in HEALTHY_RANGE_STATUSES else (
                observed[index][2] if observed[index] is not None
                and observed[index][:2] == identity else None)
            for index, status in enumerate((sample.left_status, sample.right_status))]
    else:
        for index, (name, status) in enumerate((("left", sample.left_status),
                                               ("right", sample.right_status))):
            key = (sample.source_keys or sample.source_times)[index]
            previous = link._junction_side_sources[index]
            if (status in HEALTHY_RANGE_STATUSES
                    and 0 <= now - sample.source_times[index]
                    <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                    and _source_key_newer(key, previous)):
                link._junction_side_sources[index] = key
                if status in (WireRangeStatus.NO_TARGET, WireRangeStatus.OUT_OF_RANGE):
                    openings.add(name)
    link._junction_openings = frozenset(openings)
    _clear_side_recovery(link)
    link._side_brake_pending = True


def _decode_diagnostics(payload):
    return decode_diagnostics(payload)


class WheelSpeedEstimator:
    """用新STATE的MCU时间及轮程估算轮速；至少100ms，重复帧不刷新。"""

    def __init__(self) -> None:
        self._samples = deque(maxlen=256)
        self._raw_uptime = None
        self._uptime_ms = 0
        self._sample_at = None
        self._speeds = None

    def observe(self, state, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        raw = getattr(state, "uptime_ms", None)
        wheels = (state.left_mm, state.right_mm)
        if (isinstance(raw, bool) or not isinstance(raw, int)
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in wheels)):
            return
        if self._raw_uptime is not None:
            elapsed = (raw - self._raw_uptime) & 0xFFFFFFFF
            if not 0 < elapsed < 0x80000000:
                return
            if elapsed > config.TELEMETRY_STALE_MS:
                self._samples.clear()
                self._speeds = None
            self._uptime_ms += elapsed
        self._raw_uptime, self._sample_at = raw, now
        self._samples.append((self._uptime_ms, *wheels))
        while len(self._samples) > 2 and self._uptime_ms - self._samples[1][0] >= 100:
            self._samples.popleft()
        first = self._samples[0]
        elapsed = self._uptime_ms - first[0]
        if elapsed >= 100:
            self._speeds = tuple((wheels[i] - first[i + 1]) * 1000.0 / elapsed
                                 for i in (0, 1))

    def speeds(self, now: float | None = None) -> tuple[float, float] | None:
        now = time.monotonic() if now is None else now
        if self._sample_at is None or not 0 <= now - self._sample_at <= config.TELEMETRY_STALE_MS / 1000.0:
            return None
        return self._speeds


class _TurnDirectionGuard:
    """Only accepted/applied, new source STATE can prove reverse turn motion."""

    def __init__(self, sign, start_yaw, identity):
        self.sign = 1 if sign > 0 else -1
        self.start_yaw = self.yaw = float(start_yaw)
        self.identity = tuple(identity)
        self.uptime = self.source_at = None

    def observe(self, state, generated_at, now, *, applied=False):
        identity = getattr(state, "session_nonce", None), getattr(state, "boot_id", None)
        uptime, raw = getattr(state, "uptime_ms", None), getattr(state, "yaw_tenth_deg", None)
        flags = getattr(state, "state_flags", 0)
        if (not applied or identity != self.identity
                or not flags & StateFlag.APPLIED_CONTROL_PRESENT
                or not flags & (StateFlag.TURN_ACTIVE | StateFlag.TURN_COMPLETED)
                or isinstance(uptime, bool) or not isinstance(uptime, int)
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in (raw, generated_at, now))
                or not 0 <= now - generated_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9):
            return None
        if self.uptime is not None and (not 0 < ((uptime - self.uptime) & 0xFFFFFFFF) < 0x80000000
                                        or generated_at <= self.source_at):
            return None
        yaw = raw / 10.0
        self.yaw += (yaw - self.yaw + 180.0) % 360.0 - 180.0
        self.uptime, self.source_at = uptime, generated_at
        progress = self.sign * (self.yaw - self.start_yaw)
        if progress <= -1.0 + 1e-9:
            return (f"固定{'左' if self.sign > 0 else '右'}转已应用后反向航向累计"
                    f"{-progress:.2f}°达到1°，立即制动；禁止自动反号重试")
        return None


class RawTerminalStream:
    """日志与异步终端共用的原始写入器，避免退出时占有stdout缓冲锁。"""

    def __init__(self, stream=None) -> None:
        self._stream = sys.stdout if stream is None else stream
        self.encoding = getattr(self._stream, "encoding", None) or "utf-8"
        try:
            self._descriptor = self._stream.fileno()
            if not isinstance(self._descriptor, int) or self._descriptor < 0:
                raise TypeError("stdout has no file descriptor")
        except (AttributeError, OSError, ValueError, TypeError):
            self._descriptor = None

    def write(self, text: str) -> int:
        if self._descriptor is None:
            return self._stream.write(text)
        data = memoryview(text.encode(self.encoding, errors="backslashreplace"))
        while data:
            try:
                count = os.write(self._descriptor, data)
            except InterruptedError:
                continue
            if count <= 0:
                raise OSError("终端写入未取得进展")
            data = data[count:]
        return len(text)

    def flush(self) -> None:
        if self._descriptor is None:
            self._stream.flush()


class AsyncTerminal:
    """有界终端队列；慢输出不占用串口续租和STOP的控制线程。"""

    def __init__(self, write=None) -> None:
        self.queue = queue.Queue(maxsize=4)
        self._write = write if write is not None else self._default_writer()
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, name="motion-terminal", daemon=True)
        self._thread.start()

    @staticmethod
    def _default_writer():
        stream = RawTerminalStream()

        def write(line):
            stream.write(line + "\n")
            stream.flush()

        return write

    def __call__(self, text: str) -> None:
        try:
            self.queue.put_nowait(text)
        except queue.Full:
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(text)
            except queue.Full:
                pass

    def _run(self) -> None:
        while not self._stopping.is_set() or not self.queue.empty():
            try:
                text = self.queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                self._write(text)
            except Exception:
                pass
            finally:
                self.queue.task_done()

    def close(self) -> None:
        self._stopping.set()
        self._thread.join(timeout=0.3)


def _ignored_runtime_fault_bits(link):
    return (config.MCU_IGNORED_RUNTIME_FAULT_BITS
            | getattr(link, "_recoverable_link_diagnostic_mask", 0))


def _active_runtime_fault_bits(link, state=None):
    state = link.state if state is None else state
    return int(getattr(state, "fault_bits", 0)) & ~_ignored_runtime_fault_bits(link)


def _recoverable_turn_abort(link, state=None):
    # v17 finishes a stopped but inaccurate turn without Safety_Trip: the
    # original action remains applied, targets are zero, and TURN_ABORTED has
    # no safety fault bit. DIAG.last_stop_reason retains unrelated history.
    # Settling timeouts do trip safety (bit20), so remain outside this path.
    evidence = getattr(link, "_turn_abort_evidence", None)
    state = (evidence["state"] if state is None and evidence is not None else
             link.state if state is None else state)
    diagnostics = link.diagnostics
    record = (evidence["record"] if evidence is not None and evidence["state"] is state else
              getattr(link, "_command_ledger", {}).get(
        (MessageType.MOTION_ACTION, state.applied_control_sequence)) if state is not None else None)
    history = None if record is None else record[10].get("turn_diagnostic_history")
    return bool(getattr(link, "_recoverable_link_diagnostic_mask", 0) and state is not None
                and state.state_flags & StateFlag.TURN_ABORTED
                and (state.session_nonce, state.boot_id) == (link.session_nonce, link.boot_id)
                and not _active_runtime_fault_bits(link, state)
                and not _active_runtime_fault_bits(link)
                and state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                and not state.state_flags & (StateFlag.TURN_ACTIVE | StateFlag.TURN_COMPLETED)
                and not state.target_linear_mms and not state.target_angular_mrads
                and state.odom_status == 0 and state.state_flags & StateFlag.ODOM_VALID
                and diagnostics is not None
                and (diagnostics.session_nonce, diagnostics.boot_id) == (link.session_nonce, link.boot_id)
                and record is not None and tuple(record[6:8]) == (link.session_nonce, link.boot_id)
                and history is not None and tuple(history["identity"]) == tuple(record[6:8])
                and history["source_at"] <= record[0] + 1e-9
                and diagnostics.last_stop_reason in (history["reason"],
                    WireSafetyStopReason.NONE, WireSafetyStopReason.BOOT, WireSafetyStopReason.HOST)
                and record[5][1] in (
                    MotionActionCode.TURN_LEFT, MotionActionCode.TURN_RIGHT, MotionActionCode.UTURN_LEFT)
                and record[8] is not None and record[8].status is AckStatus.ACCEPTED
                and record[9] is not None and record[0] - 1e-9 <= record[9]
                <= record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S + 1e-9
                and (record[1] is None or 0 < ((state.uptime_ms - record[1]) & 0xffffffff) < 0x80000000)
                and link._sample_is_fresh("diagnostics") and _gyro_health_reason(link) is None)


class MotionSequenceLink(mcu.McuLink):
    """脚本内v12兼容、250ms新证据/2s缓存门禁与逐帧动作确认；共享模块不变。"""

    class _ReceiveClock:
        """Keep the shared UART pump, timestamping complete reads before parsing."""

        def __init__(self, serial_port, owner):
            self.serial_port, self.owner = serial_port, owner

        def __getattr__(self, name):
            return getattr(self.serial_port, name)

        def read(self, size):
            data = self.serial_port.read(size)
            if data:
                self.owner._frame_received_at = time.monotonic()
            return data

    def __init__(self, *args, **kwargs) -> None:
        self._frame_received_at = None
        self._side_events: deque[_SideSample] = deque()
        self._state_events = deque()
        self._diagnostic_events = deque()
        self._trace_events = deque(maxlen=512)
        self._trace_history = deque(maxlen=32)
        self._recent_valid_sides = [None, None]
        self._observed_side_sources = [None, None]
        self._valid_side_cache = [None, None]
        self._latest_side_sample = None
        self._ready_side_pairs = []
        self._ready_side_identity = self._ready_side_barrier = None
        self._side_source_barrier = None
        self.side_cached_active = False
        self.side_recovery_active = False
        self.side_candidates_enabled = False
        self.side_motion_active = False
        self.side_departure_active = False
        self._departure_barrier = None
        self._departure_identity = None
        self._departure_side_sources = [None, None]
        self.side_recovery_reason = None
        self._side_brake_pending = False
        self._timeout_candidate_sides = frozenset()
        _clear_junction(self)
        self._last_brake_sent_at = None
        self._last_brake_identity = None
        self._last_brake_sequence = None
        self._last_brake_origin = None
        self._pending_commands = {}
        self._confirmed_commands = {}
        self._confirmed_send_times = {}
        self._confirmed_evidence_times = {}
        self._superseded_commands = {}
        self._command_ledger = {}
        self._goal_epoch = 0
        self._current_target = None
        self._turn_events = None
        self._turn_abort_evidence = None
        self.side_age_required = True
        self.motion_checks_enabled = False
        self.speed = WheelSpeedEstimator()
        kwargs["baud"] = 115200
        super().__init__(*args, **kwargs)

    def hello(self, timeout_s: float = 3.0):
        _clear_junction(self)
        if self._ser is None:
            raise mcu.McuError("串口未打开，先调用open()")
        if (PROTOCOL_MAJOR, PROTOCOL_MINOR) != (2, 3):
            raise mcu.McuError("本测试需要协议2.3编解码器")
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
            self._send(MessageType.HELLO_REQ,
                       encode_hello_request(HelloRequest(2, 3, nonce, EXPECTED_CAPABILITIES)))
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
                time.sleep(CONTROL_INTERVAL_S)
            raise mcu.McuError("v12 HELLO握手超时")
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
            raise mcu.McuError(f"固件身份/能力不匹配：{response.build_id!r}/0x{response.capabilities:X}，"
                               f"需要{EXPECTED_BUILD_ID}/0xEDB")
        if response.boot_id == 0:
            raise mcu.McuError("固件boot_id=0")
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

    def _start_capture(self) -> None:
        super()._start_capture()
        self._capture_call("update_metadata", expected_build_id=EXPECTED_BUILD_ID,
                           required_capabilities=EXPECTED_CAPABILITIES,
                           serial_baud=115200, serial_format="8N1", protocol="2.3",
                           motion_sequence=dict(turn_speed_mmps=TURN_SPEED_MMPS,
                                                laser_max_delta_mmps=CENTER_TRUSTED_ANGULAR_MRADPS // 8,
                                                center_filter_tau_s=CENTER_FILTER_TAU_S,
                                                center_lookahead_mm=CENTER_LOOKAHEAD_MM,
                                                normal_heading_tolerance_deg=NORMAL_HEADING_TOLERANCE_DEG,
                                                centering_heading_limit_deg=NORMAL_HEADING_TOLERANCE_DEG,
                                                correction_heading_limit_deg=None,
                                                position_integral_limit_deg=POSITION_INTEGRAL_LIMIT_DEG,
                                                normal_max_wheel_difference_mmps=96,
                                                center_fallback_angular_mradps=CENTER_FALLBACK_ANGULAR_MRADPS,
                                                center_trusted_angular_mradps=CENTER_TRUSTED_ANGULAR_MRADPS,
                                                center_angular_quantum_mradps=CENTER_ANGULAR_QUANTUM_MRADPS,
                                                recovery_strategy="forward_only_direct_xx_axle_feedback",
                                                correction_min_wheel_speed_mmps=CENTER_MIN_WHEEL_SPEED_MMPS,
                                                junction_wait_s=JUNCTION_WAIT_S,
                                                post_turn_wait_s=POST_TURN_WAIT_S,
                                                correction_region_mm=None,
                                                input_speed_min_mmps=CENTER_MIN_WHEEL_SPEED_MMPS,
                                                continuous_correction=True,
                                                stop_observation_s=STOP_OBSERVATION_S,
                                                post_stop_pairs=POST_STOP_PAIRS,
                                                centered_rule="two_disjoint_half_second_axle_mean_windows_D_le_5mm_and_fixed_heading_le_5deg_display_only",
                                                normal_speed_policy="first_and_resumed_target_xx",
                                                normal_angular_slew_limit=None,
                                                low_speed_mean_target_mmps=None,
                                                actual_speed_window_s=CENTER_SPEED_WINDOW_S,
                                                actual_cruise_speed_tolerance_fraction=.05,
                                                startup_heading_mean_window_s=START_SAMPLE_S,
                                                forward_only=True,
                                                recovery_failure_policy="wait_for_health_or_stop_then_retry_without_global_timeout",
                                                laser_measurement_max_age_ms=LASER_MEASUREMENT_MAX_AGE_MS,
                                                laser_cache_max_age_ms=LASER_CACHE_MAX_AGE_MS,
                                                departure_side_timeout_s=DEPARTURE_SIDE_TIMEOUT_S,
                                                post_turn_laser_wait=False,
                                                average_interval_s=AVERAGE_INTERVAL_S,
                                                command_interval_s=COMMAND_INTERVAL_S,
                                                start_sample_s=START_SAMPLE_S,
                                                end_condition="second_confirmed_junction"))

    def _handle(self, frame) -> None:
        if frame.message_type is MessageType.DIRECTIONAL_RANGE4:
            try:
                self._accept_directional(decode_directional_range4(frame.payload))
            except Exception as exc:
                self._retain_parse_issues((ParserIssue("PAYLOAD_ERROR", str(exc)),))
                # The shared decoder rejects impossible VALID readings before
                # they reach the per-sample gate. Only an identifiable frame
                # from this session may become a local sensor fault.
                payload = frame.payload
                if (len(payload) >= 12 and self._session_matches(
                        int.from_bytes(payload[:8], "little"),
                        int.from_bytes(payload[8:12], "little"))):
                    self._latch_hard_fault(f"当前会话下层激光帧无效：{exc}")
            return
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
        self._side_events.clear()
        self._state_events.clear()
        self._diagnostic_events.clear()
        self._trace_events.clear()
        self._trace_history.clear()
        self._recent_valid_sides = [None, None]
        self._observed_side_sources = [None, None]
        self._valid_side_cache = [None, None]
        self._latest_side_sample = None
        self._ready_side_pairs = []
        self._ready_side_identity = self._ready_side_barrier = None
        self._side_source_barrier = None
        self.side_cached_active = False
        self.side_recovery_active = False
        self.side_candidates_enabled = False
        self.side_motion_active = False
        self.side_departure_active = False
        self._departure_barrier = None
        self._departure_identity = None
        self._departure_side_sources = [None, None]
        self.side_recovery_reason = None
        self._side_brake_pending = False
        self._timeout_candidate_sides = frozenset()
        _clear_junction(self)
        self._last_brake_sent_at = None
        self._last_brake_identity = None
        self._last_brake_sequence = None
        self._last_brake_origin = None
        self._brake_evidence_barrier = None
        self._stable_barrier = None
        self._stop_tracker = None
        self._pending_commands.clear()
        self._confirmed_commands.clear()
        self._confirmed_send_times.clear()
        self._confirmed_evidence_times.clear()
        self._superseded_commands.clear()
        self._command_ledger.clear()
        self._goal_epoch = 0
        self._current_target = None
        self._turn_events = None
        self._turn_abort_evidence = None
        self.side_age_required = True
        self.motion_checks_enabled = False
        self.speed = WheelSpeedEstimator()

    def _accept_state(self, state) -> None:
        previous = self.state
        super()._accept_state(state)
        if self.state is not state or previous is state:
            return
        if len(self._state_events) >= 256:
            self._latch_hard_fault("STATE队列溢出，动作完成证据不完整")
            return
        generated_at = self._sample_time("state")
        self._state_events.append((state, generated_at))
        if self._turn_events is not None:
            if len(self._turn_events) >= 256:
                self._latch_hard_fault("转向 STATE 队列溢出，动作证据不完整")
            else:
                self._turn_events.append((state, generated_at,
                                          self._family_uptime_ms.get("state")))
        self.speed.observe(state, generated_at)
        received_at = (self._last_state_recv if self._frame_received_at is None
                       else self._frame_received_at)
        for (_kind, seq), record in self._pending_commands.items():
            sent_at, baseline, _applied = record[:3]
            if (state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                    and state.applied_control_sequence == seq and received_at >= sent_at
                    and (baseline is None or 0 < ((state.uptime_ms - baseline) & 0xFFFFFFFF) < 0x80000000)
                    and (self.session_nonce, self.boot_id) == tuple(record[6:8])
                    # 确认资格检查完整帧读到时的源龄；处理排队不能抹掉
                    # 及时证据。实时运动健康仍按实际当前时间检查源龄。
                    and self._sample_is_fresh("state", received_at)):
                if record[2] is None:
                    record[2:4] = state, received_at
                    record[10]["state_handled_at"] = time.monotonic()
                    record[10]["state_source_at"] = generated_at
        reason = None
        if state.state_flags & StateFlag.TURN_ABORTED:
            record = self._command_ledger.get((MessageType.MOTION_ACTION, state.applied_control_sequence))
            if (self._turn_abort_evidence is None and record is not None
                    and record[5][1] in (MotionActionCode.TURN_LEFT,
                    MotionActionCode.TURN_RIGHT, MotionActionCode.UTURN_LEFT)
                    and record[5] == self._current_target
                    and tuple(record[6:8]) == (self.session_nonce, self.boot_id)):
                self._turn_abort_evidence = dict(state=state, record=record, source_at=generated_at)
        diagnostic_bits = state.fault_bits & getattr(self, "_recoverable_link_diagnostic_mask", 0)
        if diagnostic_bits and diagnostic_bits != getattr(self, "_last_link_diagnostic_bits", None):
            runlog.emit("motion_link_recoverable_diagnostic", "v17通信诊断位保留记录，按实时确认与租约检查运动健康",
                        terminal=False, raw_fault_bits=state.fault_bits, diagnostic_bits=diagnostic_bits,
                        session_nonce=state.session_nonce, boot_id=state.boot_id)
        self._last_link_diagnostic_bits = diagnostic_bits
        if getattr(self, "_recoverable_link_diagnostic_mask", 0) and self._active_fault_bits():
            reason = ("下位机本地停机锁存" if state.fault_bits & (1 << 20) else "MCU固件故障")
        elif state.state_flags & StateFlag.TURN_ABORTED:
            reason = "转向被下位机中止"
        elif state.fault_bits & (1 << 20):
            reason = "下位机本地停机锁存"
        elif self.motion_checks_enabled:
            if self._active_fault_bits():
                reason = "MCU固件故障"
            elif state.odom_status != 0 or not state.state_flags & StateFlag.ODOM_VALID:
                reason = "ODOM无效"
        if reason is not None:
            self._latch_hard_fault(reason)

    def _accept_diagnostics(self, sample) -> None:
        previous = self.diagnostics
        super()._accept_diagnostics(sample)
        if self.diagnostics is not sample or previous is sample:
            return
        if len(self._diagnostic_events) >= 256:
            self._latch_hard_fault("DIAG队列溢出，静止标定证据不完整")
            return
        self._diagnostic_events.append((sample, self._sample_time("diagnostics")))
        if not self.motion_checks_enabled:
            return
        pure_turn_failure = _recoverable_turn_abort(self)
        if sample.last_stop_reason not in (WireSafetyStopReason.NONE, WireSafetyStopReason.BOOT,
                                          WireSafetyStopReason.HOST) and not pure_turn_failure:
            detail = "（转向停稳失败）" if int(sample.last_stop_reason) == 12 else ""
            self._latch_hard_fault(f"下位机故障停车：{sample.last_stop_reason.name}"
                                   f"({int(sample.last_stop_reason)}){detail}")
        elif sample.gyro_status != WireGyroStatus.READY:
            self._latch_hard_fault(f"陀螺仪未就绪：{sample.gyro_status.name}")
        else:
            reason = _gyro_health_reason(self)
            if reason is not None:
                self._latch_hard_fault(reason)

    def drain_state_samples(self) -> list:
        samples = list(self._state_events)
        self._state_events.clear()
        return samples

    def drain_diagnostics_samples(self) -> list:
        samples = list(self._diagnostic_events)
        self._diagnostic_events.clear()
        return samples

    def _accept_wheel_trace(self, sample, frame_sequence=None) -> None:
        previous = self.wheel_trace
        super()._accept_wheel_trace(sample, frame_sequence)
        if self.wheel_trace is not sample or previous is sample:
            return
        generated_at = self._last_trace_recv
        if self._clock_offset_s is not None and self._uptime_reference_raw is not None:
            delta = ((sample.uptime_ms - self._uptime_reference_raw + 0x80000000)
                     & 0xFFFFFFFF) - 0x80000000
            generated_at = min(generated_at, self._clock_offset_s
                               + (self._uptime_reference_ms + delta) / 1000.0)
        self._trace_events.append((sample, generated_at))
        self._trace_history.append((sample, generated_at))
        runlog.emit("motion_sequence_wheel_trace", "接受轮迹观测",
                    sample_sequence=sample.sample_sequence, source_time=generated_at,
                    left_encoder_delta=sample.left_encoder_delta,
                    right_encoder_delta=sample.right_encoder_delta,
                    applied_control_sequence=sample.applied_control_sequence,
                    elapsed_ms=sample.elapsed_ms, flags=int(sample.flags))

    def drain_wheel_trace_samples(self) -> list:
        samples = list(self._trace_events)
        self._trace_events.clear()
        return samples

    @property
    def motion_target_pending(self) -> bool:
        """当前逻辑目标无及时ACK+STATE证据时续发原目标；旧续约独立监督。"""
        if self._current_target is None:
            return False
        identity = self.session_nonce, self.boot_id
        return not any(key in self._confirmed_commands
                       and record[4] == self._goal_epoch
                       and record[5] == self._current_target
                       and tuple(record[6:8]) == identity
                       for key, record in self._command_ledger.items())

    def _track_command(self, kind, sequence, sent_at, baseline, target) -> None:
        if (target != self._current_target
                and (kind is MessageType.MOTION_ACTION and target[1] is not MotionActionCode.BRAKE
                     or kind is MessageType.CONTROL_TARGET and any(target[1:3]))):
            self._turn_abort_evidence = None
        # Same-target renewals retain the action's original diagnostic history.
        # A later real stop reason cannot become harmless merely by renewing.
        turn_history = None
        if (getattr(self, "_recoverable_link_diagnostic_mask", 0)
                and kind is MessageType.MOTION_ACTION and target[1] in (
                    MotionActionCode.TURN_LEFT, MotionActionCode.TURN_RIGHT, MotionActionCode.UTURN_LEFT)):
            if target == self._current_target:
                for old_record in reversed(self._command_ledger.values()):
                    if (old_record[5] == target
                            and tuple(old_record[6:8]) == (self.session_nonce, self.boot_id)):
                        turn_history = old_record[10].get("turn_diagnostic_history")
                        break
            else:
                diagnostic = self.diagnostics
                if (diagnostic is not None and (diagnostic.session_nonce, diagnostic.boot_id)
                        == (self.session_nonce, self.boot_id)
                        and self._sample_is_fresh("diagnostics") and _gyro_health_reason(self) is None):
                    turn_history = dict(identity=(self.session_nonce, self.boot_id),
                                        reason=diagnostic.last_stop_reason,
                                        source_at=self._sample_time("diagnostics"))
        if target != self._current_target:
            self._goal_epoch += 1
            self._current_target = target
        key = kind, sequence
        # Serial numbers may wrap. A fresh ledger entry never inherits old proof.
        record = [sent_at, baseline, None, None, self._goal_epoch, target,
                  self.session_nonce, self.boot_id, None, None, {}]
        if turn_history is not None:
            record[10]["turn_diagnostic_history"] = turn_history
        self._command_ledger.pop(key, None)
        self._pending_commands[key] = self._command_ledger[key] = record
        self._confirmed_commands.pop(key, None)
        self._confirmed_send_times.pop(key, None)
        self._confirmed_evidence_times.pop(key, None)
        self._superseded_commands.pop(key, None)
        self._ack_cache.pop(key, None)
        while len(self._command_ledger) > 256:
            oldest = next(iter(self._command_ledger))
            if oldest in self._pending_commands:
                self._latch_hard_fault("命令证据队列溢出，不能跳过未确认目标")
                break
            self._command_ledger.pop(oldest)
            self._confirmed_commands.pop(oldest, None)
            self._confirmed_send_times.pop(oldest, None)
            self._confirmed_evidence_times.pop(oldest, None)
            self._superseded_commands.pop(oldest, None)

    def _accept_ack(self, ack) -> None:
        if not self._session_matches(ack.session_nonce, ack.boot_id):
            super()._accept_ack(ack)
            return
        handled_at = time.monotonic()
        received_at = handled_at if self._frame_received_at is None else self._frame_received_at
        if ack.status is not AckStatus.ACCEPTED:
            # Preserve the detailed first rejection before the shared link
            # latches its generic status-only message.
            self._record_ack_rejection(ack, received_at)
        super()._accept_ack(ack)
        record = self._command_ledger.get((ack.acknowledged_type, ack.acknowledged_sequence))
        if (record is not None and received_at >= record[0]
                and (ack.session_nonce, ack.boot_id) == tuple(record[6:8])):
            if record[8] is None or ack.status is not AckStatus.ACCEPTED:
                record[8:10] = ack, received_at
                record[10]["ack_handled_at"] = handled_at

    def _record_ack_rejection(self, ack, received_at: float) -> None:
        """记录拒绝时的原始原因及独立遥测年龄，不用旧 DIAG 代替新 STATE。"""
        reason_names = {
            0: "NONE", 1: "PROTOCOL", 2: "SESSION", 3: "SEQUENCE",
            4: "PROFILE_TBD", 5: "RANGE_GUARD", 6: "GYRO_NOT_READY",
            7: "CONFLICT", 8: "TOF_NOT_READY", 9: "ENCODER_NOT_READY",
            10: "SAFETY_LATCHED",
        }
        key = ack.acknowledged_type, ack.acknowledged_sequence
        record = self._command_ledger.get(key)
        if record is not None and tuple(record[6:8]) != (ack.session_nonce, ack.boot_id):
            record = None
        sent_at = record[0] if record is not None else None
        parameters = {}
        if record is not None and key[0] is MessageType.CONTROL_TARGET:
            parameters = dict(linear_mms=record[5][1], angular_mrads=record[5][2])
        elif key[0] is MessageType.MOTION_ACTION:
            action_record = self._action_sends.get(key[1])
            if action_record is not None and tuple(action_record[3:5]) == (ack.session_nonce, ack.boot_id):
                command = action_record[0]
                sent_at = action_record[1] if sent_at is None else sent_at
                parameters = dict(action=command.action.name, action_code=int(command.action),
                                  flags=int(command.flags), speed_mms=command.speed_mms,
                                  turn_angle_tenth_deg=command.turn_angle_tenth_deg)
        state, diagnostics = self.state, self.diagnostics
        state_snapshot = None if state is None else {
            field: getattr(state, field, None) for field in (
                "session_nonce", "boot_id", "uptime_ms", "state_flags", "odom_status",
                "left_mm", "right_mm", "yaw_tenth_deg", "applied_control_sequence",
                "target_linear_mms", "target_angular_mrads", "fault_bits")
        }
        diagnostic_snapshot = None if diagnostics is None else {
            field: getattr(diagnostics, field, None) for field in (
                "session_nonce", "boot_id", "uptime_ms", "last_stop_reason",
                "gyro_status", "gyro_sample_age_ms")
        }
        if diagnostic_snapshot is not None:
            diagnostic_snapshot["last_stop_reason_name"] = getattr(
                diagnostics.last_stop_reason, "name", "UNKNOWN")
            diagnostic_snapshot["gyro_status_name"] = getattr(diagnostics.gyro_status, "name", "UNKNOWN")
        reason_name = reason_names.get(int(ack.reason), "UNKNOWN")
        elapsed_ms = None if sent_at is None else (received_at - sent_at) * 1000.0
        details = dict(message_type=key[0].name, sequence=key[1],
                       session_nonce=ack.session_nonce, boot_id=ack.boot_id,
                       status=int(ack.status), status_name=ack.status.name,
                       reason=int(ack.reason), reason_name=reason_name,
                       command_parameters=parameters, sent_at_s=sent_at,
                       received_at_s=received_at, elapsed_ms=elapsed_ms,
                       confirm_deadline_s=None if sent_at is None else sent_at + config.MCU_COMMAND_CONFIRM_TIMEOUT_S,
                       state=state_snapshot,
                       state_age_ms=None if state is None else max(0.0, (received_at - self._sample_time("state")) * 1000.0),
                       diagnostics=diagnostic_snapshot,
                       diagnostics_age_ms=None if diagnostics is None else max(0.0, (received_at - self._sample_time("diagnostics")) * 1000.0))
        sent_text = "unknown" if sent_at is None else f"{sent_at:.6f}s"
        elapsed_text = "unknown" if elapsed_ms is None else f"{elapsed_ms:.1f} ms"
        message = (f"命令被拒: {key[0].name} seq={key[1]} "
                   f"status={ack.status.name}({int(ack.status)}) "
                   f"reason={reason_name}({int(ack.reason)}) command={parameters} "
                   f"sent_at={sent_text} received_at={received_at:.6f}s elapsed={elapsed_text}；"
                   f"{self.motion_health_snapshot()}")
        runlog.emit("motion_sequence_ack_rejected", message, **details)
        self._latch_hard_fault(message)

    def _clear_command_evidence(self, *, preserve_pending=False) -> None:
        if preserve_pending:
            # BRAKE may interrupt an unsafe pulse immediately; it does not
            # forgive a missing ACK/applied STATE or move the old200ms deadline.
            # Retain witnesses, but stop using pre-BRAKE proof to time drive.
            for record in self._command_ledger.values():
                record[10]["drive_invalidated_by_brake"] = True
        else:
            self._pending_commands.clear()
            self._confirmed_commands.clear()
            self._confirmed_send_times.clear()
            self._confirmed_evidence_times.clear()
            self._superseded_commands.clear()
            self._command_ledger.clear()
        self._current_target = None
        self._goal_epoch += 1

    def send_motion(self, linear_mmps, angular_mradps, **kwargs) -> int:
        sent_at, baseline = time.monotonic(), getattr(self.state, "uptime_ms", None)
        self._sending_nonzero = bool(linear_mmps or angular_mradps)
        try:
            if linear_mmps or angular_mradps:
                reason = self._laser_stale_reason(require_upper=False)
                if reason is not None:
                    raise mcu.McuError(reason)
            if (linear_mmps or angular_mradps) and (self._side_brake_pending
                                                    or self.side_recovery_active):
                raise _SideRecoveryRequired("侧激光待刹车或停车恢复期间禁止非零目标")
            sequence = super().send_motion(linear_mmps, angular_mradps, **kwargs)
        finally:
            self._sending_nonzero = False
        if linear_mmps or angular_mradps:
            self._stable_barrier = self._stop_tracker = self._brake_evidence_barrier = None
            self._ready_side_pairs = []
            target = (MessageType.CONTROL_TARGET, int(round(linear_mmps)), int(round(angular_mradps)))
            self._track_command(MessageType.CONTROL_TARGET, sequence, sent_at, baseline, target)
        else:
            self._clear_command_evidence()
        return sequence

    def send_motion_action(self, action, speed_mms=0, **kwargs) -> int:
        if (getattr(self, "_turn_waiting", False) and self._current_target is not None
                and not (action is MotionActionCode.BRAKE
                         and getattr(self, "_turn_direction_fault", False))):
            current = self._current_target
            flags = int(MotionActionFlag.BRAKE_AFTER_TURN) if kwargs.get("brake_after_turn", True) else 0
            if current[:4] != (MessageType.MOTION_ACTION, action, speed_mms, flags):
                raise mcu.McuError("固定转向执行中只允许同参数 MOTION_ACTION 续约；请先 STOP")
        sent_at, baseline = time.monotonic(), getattr(self.state, "uptime_ms", None)
        junction_turn = bool(_pending_junction_openings(self)) and action in (
            MotionActionCode.TURN_LEFT, MotionActionCode.TURN_RIGHT)
        if junction_turn:
            tracker = getattr(self, "_stop_tracker", None)
            if (tracker is None or getattr(self, "_stable_barrier", None) is None
                    or not tracker.update(self, [])):
                raise mcu.McuError("路口开口已保存，但尚未证明BRAKE后双轮连续500ms停稳，禁止转弯")
            if _junction_wait_remaining(self, sent_at) > 1e-9:
                runlog.emit("motion_sequence_turn_blocked", "路口BRAKE成功发送尚未满3秒，禁止转弯",
                            action=action.name, remaining_s=_junction_wait_remaining(self, sent_at),
                            brake_sent_at=getattr(self, "_last_brake_sent_at", None))
                raise mcu.McuError("路口BRAKE成功发送尚未满3秒，禁止转弯")
        self._sending_nonzero = action is not MotionActionCode.BRAKE
        try:
            if action is not MotionActionCode.BRAKE:
                reason = self._laser_stale_reason(require_upper=False)
                if reason is not None:
                    raise mcu.McuError(reason)
            if action is not MotionActionCode.BRAKE and (self._side_brake_pending
                                                           or self.side_recovery_active) and not junction_turn:
                raise _SideRecoveryRequired("侧激光待刹车或停车恢复期间禁止非零动作")
            sequence = super().send_motion_action(action, speed_mms, **kwargs)
        finally:
            self._sending_nonzero = False
        if action is MotionActionCode.BRAKE:
            self._last_brake_sent_at = time.monotonic()
            self._last_brake_identity = self.session_nonce, self.boot_id
            self._last_brake_sequence = sequence
            self._last_brake_origin = self.odom_mm
            self.side_cached_active = False
            self._clear_command_evidence(preserve_pending=True)
        else:
            self._stable_barrier = self._stop_tracker = self._brake_evidence_barrier = None
            self._ready_side_pairs = []
            if junction_turn:
                _clear_junction(self)
                _clear_side_recovery(self)
        command = self._action_sends[sequence][0]
        target = (MessageType.MOTION_ACTION, command.action, command.speed_mms,
                  int(command.flags), command.turn_angle_tenth_deg)
        self._track_command(MessageType.MOTION_ACTION, sequence, sent_at, baseline, target)
        return sequence

    def send_stop(self, reason=StopReason.MISSION) -> int:
        sequence = super().send_stop(reason)
        self.side_cached_active = False
        self._ready_side_pairs = []
        self._clear_command_evidence()
        _clear_junction(self)
        return sequence

    def pump(self):
        serial_port = self._ser
        self._frame_received_at = None
        if serial_port is not None:
            self._ser = self._ReceiveClock(serial_port, self)
        try:
            frames = super().pump()
        finally:
            self._ser = serial_port
            self._frame_received_at = None
        if self.last_parse_error and self.last_parse_error.startswith("READ_ERROR:"):
            self._latch_hard_fault(self.last_parse_error)
        now = time.monotonic()
        if self.motion_checks_enabled and self.directional is not None:
            reason = self._laser_stale_reason(require_upper=False)
            if reason is not None and self._hard_fault_reason is None:
                runlog.emit("motion_sequence_laser_fault", "最新下层激光不可用",
                            reason=reason, **_side_sample(self).diagnostics(now))
                self._latch_hard_fault(reason)
        # Evidence has its own receive clock: finishing a UART drain late cannot
        # turn timely ACK/STATE into a timeout. Check every rejection first.
        for key, record in list(self._pending_commands.items()):
            ack = record[8]
            if (ack is not None and ack.status is not AckStatus.ACCEPTED
                    and self._hard_fault_reason is None):
                self._record_ack_rejection(ack, record[9] if record[9] is not None else now)
        proven = [(key, record) for key, record in self._command_ledger.items()
                  if record[2] is not None and record[8] is not None
                  and record[8].status is AckStatus.ACCEPTED
                  and tuple(record[6:8]) == (self.session_nonce, self.boot_id)]
        for key, record in list(self._pending_commands.items()):
            sent_at, expiry = record[0], record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S
            proof = next(((candidate_key, candidate) for candidate_key, candidate in proven
                            if candidate[4:8] == record[4:8]
                            and candidate[0] >= sent_at
                            and max(candidate[3], candidate[9]) <= expiry + 1e-9), None)
            if proof is not None and self._hard_fault_reason is None:
                witness_key, witness = proof
                self._confirmed_commands[key] = witness[2]
                self._confirmed_send_times[key] = sent_at
                self._confirmed_evidence_times[key] = max(witness[3], witness[9])
                if witness_key != key:
                    self._superseded_commands[key] = witness_key
                    runlog.emit("motion_sequence_command_superseded", "相同目标续约的明确证据覆盖旧序号",
                                message_type=key[0].name, pending_sequence=key[1],
                                witness_sequence=witness_key[1], goal_epoch=record[4],
                                oldest_deadline=expiry,
                                evidence_received_at=self._confirmed_evidence_times[key])
                del self._pending_commands[key]
            elif now >= expiry:
                self._record_confirmation_timeout(key, record, expiry, now)
                self._latch_hard_fault(f"{key[0].name} seq={key[1]}未在确认窗口内应用")
        return frames

    def _record_confirmation_timeout(self, key, record, deadline, now) -> None:
        """One actionable timeout record; retain the original confirmation window."""
        if self._hard_fault_reason is not None or record is None:
            return
        ack, state = record[8], record[2]
        failures = []
        for name, evidence, received_at in (("ack", ack, record[9]),
                                            ("state", state, record[3])):
            if evidence is None:
                failures.append(f"missing_{name}")
            elif received_at > deadline + 1e-9:
                failures.append(f"late_{name}")
        witnesses = [dict(sequence=candidate_key[1], sent_at_s=candidate[0],
                          ack_received_at_s=candidate[9], state_received_at_s=candidate[3])
                     for candidate_key, candidate in self._command_ledger.items()
                     if candidate_key != key and candidate[4:8] == record[4:8]
                     and candidate[0] >= record[0]]
        runlog.emit("motion_sequence_confirmation_timeout", "命令确认超时证据",
                    message_type=key[0].name, sequence=key[1], target=record[5],
                    goal_epoch=record[4], session_nonce=record[6], boot_id=record[7],
                    sent_at_s=record[0], confirm_deadline_s=deadline, checked_at_s=now,
                    missing_or_late=failures,
                    ack_status=None if ack is None else ack.status.name,
                    ack_received_at_s=record[9], state_received_at_s=record[3],
                    ack_handled_at_s=record[10].get("ack_handled_at"),
                    state_handled_at_s=record[10].get("state_handled_at"),
                    state_uptime_ms=None if state is None else state.uptime_ms,
                    applied_control_sequence=None if state is None else state.applied_control_sequence,
                    latest_state_uptime_ms=getattr(self.state, "uptime_ms", None),
                    latest_applied_control_sequence=getattr(self.state, "applied_control_sequence", None),
                    baseline_state_uptime_ms=record[1], same_target_witnesses=witnesses)

    def _wait_applied(self, kind, sequence, timeout_s=None):
        key = kind, sequence
        pending = self._pending_commands.get(key)
        sent_at = pending[0] if pending is not None else self._confirmed_send_times.get(key)
        if sent_at is None:
            raise mcu.McuError(f"未知 {kind.name} seq={sequence}")
        deadline = sent_at + self._confirm_timeout(timeout_s)
        identity = self.session_nonce, self.boot_id
        while True:
            now = time.monotonic()
            if (self.session_nonce, self.boot_id) != identity or not self.connected:
                raise mcu.McuError("命令确认期间 MCU 会话变化")
            self._check_motion_lease(now)
            if self._hard_fault_reason is not None:
                raise mcu.McuError(self._hard_fault_reason)
            applied = self._confirmed_commands.get((kind, sequence))
            evidence_at = self._confirmed_evidence_times.get(key)
            if applied is not None and evidence_at is not None and evidence_at <= deadline + 1e-9:
                return applied
            if now >= deadline:
                self._record_confirmation_timeout(key, self._command_ledger.get(key), deadline, now)
                self._stop_failed_confirmation(f"{kind.name} seq={sequence}未在确认窗口内应用")
                raise mcu.McuError(self._hard_fault_reason)
            self.pump()
            if (kind, sequence) not in self._confirmed_commands:
                time.sleep(0.005)

    def wait_for_control_applied(self, sequence, timeout_s=None):
        return self._wait_applied(MessageType.CONTROL_TARGET, sequence, timeout_s)

    def wait_for_motion_action_applied(self, sequence, timeout_s=None):
        return self._wait_applied(MessageType.MOTION_ACTION, sequence, timeout_s)

    def wait_for_motion_action_complete(self, sequence, *, timeout_s=13.0,
                                        require_ranges=True, require_upper=True,
                                        on_poll=None):
        record = self._action_sends.get(sequence)
        if record is None:
            raise mcu.McuError(f"未知 MOTION_ACTION seq={sequence}")
        command, sent_at, baseline, nonce, boot = record
        if command.action not in (MotionActionCode.TURN_LEFT, MotionActionCode.TURN_RIGHT,
                                  MotionActionCode.UTURN_LEFT):
            raise mcu.McuError("只有固定转向动作可等待完成")
        if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
                or not math.isfinite(timeout_s) or timeout_s <= 0):
            raise mcu.McuError("动作超时必须是有限正数")
        deadline, next_renew = sent_at + timeout_s, sent_at + COMMAND_INTERVAL_S
        issued, active_uptime = {sequence}, None
        self._turn_events = deque()
        self._turn_waiting = True
        previous_side_age_required = self.side_age_required
        self.side_age_required = False
        try:
            while True:
                self.pump()
                now = time.monotonic()
                if (self.session_nonce, self.boot_id) != (nonce, boot) or not self.connected:
                    raise mcu.McuError("转向期间 MCU 会话变化")
                self._check_motion_lease(now)
                reason = self.motion_health_reason()
                if reason is not None:
                    raise mcu.McuError(reason)
                if on_poll is not None:
                    on_poll()
                # A later fault in this receive pass always wins over earlier completion.
                reason = self.motion_health_reason()
                if reason is None and require_ranges:
                    reason = self._laser_stale_reason(require_upper=require_upper)
                if reason is not None:
                    raise mcu.McuError(reason)
                if now >= deadline:
                    raise mcu.McuError("固件转向超时")
                completed = None
                while self._turn_events:
                    state, generated_at, uptime = self._turn_events[0]
                    eligible = (
                        uptime is not None and (baseline is None or uptime > baseline)
                        and 0 <= now - generated_at <= config.TELEMETRY_STALE_MS / 1000.0
                        and self._session_matches(state.session_nonce, state.boot_id)
                        and state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                        and state.applied_control_sequence in issued)
                    if not eligible:
                        self._turn_events.popleft()
                        continue
                    ack = self._cached_ack(MessageType.MOTION_ACTION,
                                           state.applied_control_sequence)
                    if ack is None:
                        break  # Retain early ACTIVE/COMPLETED until its bounded ACK arrives.
                    if ack.status is not AckStatus.ACCEPTED:
                        raise mcu.McuError(f"转向动作被拒：{ack.status.name}")
                    self._turn_events.popleft()
                    if state.state_flags & StateFlag.TURN_ABORTED:
                        raise mcu.McuError("固件转向已中止")
                    guard = getattr(self, "_turn_direction_guard", None)
                    reverse_reason = (guard.observe(state, generated_at, now, applied=True)
                                      if guard is not None else None)
                    if reverse_reason is not None:
                        self._turn_direction_fault = True
                        runlog.emit("motion_sequence_turn_direction_fault", reverse_reason,
                                    sequence=state.applied_control_sequence,
                                    source_at=generated_at, start_yaw_deg=guard.start_yaw,
                                    actual_unwrapped_yaw_deg=guard.yaw,
                                    intended_direction="left" if guard.sign > 0 else "right")
                        try:
                            self.send_motion_action(MotionActionCode.BRAKE, 0,
                                                    require_ranges=False, require_upper=False)
                        except Exception as exc:
                            runlog.exception("motion_turn_brake_failed", "错向后BRAKE发送失败", exc)
                        raise mcu.McuError(reverse_reason)
                    if state.state_flags & StateFlag.TURN_ACTIVE:
                        active_uptime = uptime
                        completed = None
                    elif (state.state_flags & StateFlag.TURN_COMPLETED
                          and active_uptime is not None and uptime > active_uptime):
                        completed = state
                if completed is not None:
                    self._active_motion_sent_at = None
                    return completed
                if now >= next_renew:
                    renewal = self.send_motion_action(
                        command.action, command.speed_mms,
                        brake_after_turn=bool(command.flags & MotionActionFlag.BRAKE_AFTER_TURN),
                        require_ranges=require_ranges, require_upper=require_upper)
                    issued.add(renewal)
                    next_renew = time.monotonic() + COMMAND_INTERVAL_S
                time.sleep(min(CONTROL_INTERVAL_S, max(0.001, next_renew - time.monotonic())))
        except BaseException:
            try:
                self.send_stop(StopReason.FAULT)
            except Exception as exc:
                runlog.exception("motion_turn_stop_failed", "转向失败后 STOP 发送失败", exc)
            raise
        finally:
            self._turn_events = None
            self._turn_waiting = False
            self._turn_direction_fault = False
            self.side_age_required = previous_side_age_required

    def _accept_directional(self, directional) -> None:
        if not self._session_matches(directional.session_nonce, directional.boot_id):
            if self.connected and directional.boot_id != self.boot_id:
                self._latch_hard_fault("MCU boot 变化（复位）")
            return
        if (self._last_directional_seq is not None
                and directional.directional_cycle_seq != self._last_directional_seq
                and classify_sequence(directional.directional_cycle_seq,
                                      self._last_directional_seq) is not SequenceDisposition.NEWER):
            return
        # 50ms快照并不等于新测量。同cycle仍可推进时间、状态和年龄。
        if not self._accept_uptime("directional", directional.uptime_ms):
            return
        self._last_directional_seq = directional.directional_cycle_seq
        self.directional = directional
        sample = _annotate_side_sample(self, _side_sample(self))
        self._latest_side_sample = sample
        try:
            _check_sides(sample, sample.received_at, check_age=False,
                         allow_stale=True, allow_timeout=True)
            if _pending_junction_openings(self):
                _latch_junction(self, sample)
            elif (self.motion_checks_enabled and self.side_age_required
                    and not self.side_departure_active
                    and self.side_candidates_enabled and sample.candidates):
                _latch_junction(self, sample, initial=True)
        except mcu.McuError as exc:
            if self.motion_checks_enabled:
                runlog.emit("motion_sequence_laser_fault", "下层激光状态或距离故障",
                            reason=str(exc), **sample.diagnostics(sample.received_at))
                self._latch_hard_fault(str(exc))
        if len(self._side_events) >= 256:
            self._latch_hard_fault("下层激光快照队列溢出，测量证据不完整")
            return
        self._side_events.append(sample)

    def begin_departure(self, stable_barrier: float) -> None:
        """停稳即离口；每侧真正新健康测量独立续期，快照和4/6不续期。"""
        self.side_departure_active = True
        _clear_junction(self)
        self._departure_barrier = stable_barrier
        self._departure_identity = self.session_nonce, self.boot_id
        self._departure_side_sources = [None, None]
        self.side_cached_active = False
        sample = getattr(self, "_latest_side_sample", None)
        if sample is not None:
            _annotate_side_sample(self, sample)

    def finish_departure(self) -> None:
        self.side_departure_active = False
        self._departure_barrier = self._departure_identity = None
        self._departure_side_sources = [None, None]

    def drain_side_samples(self) -> list[_SideSample]:
        samples = list(self._side_events)
        self._side_events.clear()
        return samples

    def cached_side_feedback(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        sample = self._latest_side_sample
        self.side_cached_active = (sample is not None
                                   and _cached_side_feedback(self, sample, now))
        return (sample is not None
                and _cached_side_feedback(self, sample, now, require_stale=False))

    def motion_health_reason(self) -> str | None:
        reason = super().motion_health_reason() or _gyro_health_reason(self)
        if reason is None and self.state.odom_status != 0:
            return "ODOM无效"
        return reason

    def _active_fault_bits(self):
        return _active_runtime_fault_bits(self)


    @property
    def directional_effective_statuses(self) -> tuple[WireRangeStatus, ...]:
        directional, ages = self.directional, self.directional_age_ms
        if directional is None or ages is None or len(directional.status) != 4:
            return ()
        return tuple(self._effective_range_status(status, ages[index], LASER_MEASUREMENT_MAX_AGE_MS)
                     for index, status in enumerate(directional.status))

    def _laser_stale_reason(self, *, require_upper: bool = True) -> str | None:
        try:
            sample = _side_sample(self)
            if (self._latest_side_sample is not None
                    and _side_snapshot_key(self._latest_side_sample) == _side_snapshot_key(sample)):
                sample = replace(sample, candidate_sides=self._latest_side_sample.candidate_sides)
            if _side_recovery_needed(self, sample, time.monotonic()) and self.motion_checks_enabled:
                self.side_recovery_active = True
                self.side_recovery_reason = ("离口单侧2秒没有健康新测量，停车等待恢复"
                                             if self.side_departure_active else
                                             "下层激光4/6或旧测量的VALID缓存超过2秒，停车等待恢复")
                if getattr(self, "_sending_nonzero", False):
                    raise _SideRecoveryRequired(self.side_recovery_reason)
        except _SideRecoveryRequired:
            raise
        except mcu.McuError as exc:
            return str(exc)
        return None


@dataclass(frozen=True)
class _MeanBatch:
    ended_at: float
    left_mm: float | None
    right_mm: float | None
    left_count: int
    right_count: int
    started_at: float | None = None
    identity: tuple[int, int] | None = None
    source_keys: tuple | None = None
    source_times: tuple[float, float] | None = None
    heading_error_deg: float | None = None

    @property
    def complete(self) -> bool:
        return self.left_mm is not None and self.right_mm is not None


class _HalfSecondAverages:
    def __init__(self, started_at: float) -> None:
        self._last_seq: int | None = None
        self._last_snapshot: _SideSample | None = None
        self._last_source_keys: list[int | None] = [None, None]
        self._identity = None
        self.reset(started_at)

    def reset(self, started_at: float, *, source_barrier: float | None = None) -> None:
        self.ends_at = started_at + AVERAGE_INTERVAL_S
        self.source_barrier = source_barrier
        self.latest: _MeanBatch | None = None
        self._values: tuple[list[float], list[float]] = ([], [])
        self._window_keys = [None, None]
        self._window_sources = [None, None]

    def advance(self, now: float) -> list[_MeanBatch]:
        batches = []
        while now + 1e-9 >= self.ends_at:
            left, right = self._values
            self.latest = _MeanBatch(
                self.ends_at, math.fsum(left) / len(left) if left else None,
                math.fsum(right) / len(right) if right else None, len(left), len(right),
                started_at=self.ends_at - AVERAGE_INTERVAL_S, identity=self._identity,
                source_keys=tuple(self._window_keys), source_times=tuple(self._window_sources))
            batches.append(self.latest)
            self._values = ([], [])
            self._window_keys = [None, None]
            self._window_sources = [None, None]
            self.ends_at += AVERAGE_INTERVAL_S
        return batches

    def add(self, sample: _SideSample) -> None:
        if sample.source_identity != self._identity:
            self._identity = sample.source_identity
            self._last_snapshot = None
            self._last_source_keys = [None, None]
            self.reset(sample.received_at, source_barrier=self.source_barrier)
        if not _snapshot_newer(sample, self._last_snapshot):
            return
        self._last_snapshot = sample
        self._last_seq = sample.seq
        for index, (distance, status) in enumerate((
            (sample.left_mm, sample.left_status), (sample.right_mm, sample.right_status),
        )):
            key = (sample.source_keys[index] if sample.source_keys is not None
                   else sample.source_times[index])
            if not _source_key_newer(key, self._last_source_keys[index]):
                continue
            if (self.source_barrier is not None
                    and sample.source_times[index] <= self.source_barrier + 1e-9):
                continue
            if (status == WireRangeStatus.VALID
                    and isinstance(distance, (int, float)) and not isinstance(distance, bool)
                    and math.isfinite(distance) and 0 <= distance < 65535
                    and sample.ages_ms[index] <= LASER_MEASUREMENT_MAX_AGE_MS):
                self._last_source_keys[index] = key
                self._values[index].append(float(distance))
                self._window_keys[index] = key
                self._window_sources[index] = sample.source_times[index]


def _wheel_targets(xx: int, delta: int) -> tuple[int, int, int]:
    half = math.trunc(delta / 2)
    average = max(MIN_WHEEL_SPEED_MMPS + abs(half), min(int(xx), MAX_WHEEL_SPEED_MMPS - abs(half)))
    return average, average - half, average + half


def _motion_wheels(linear_mmps: int, angular_mradps: int) -> tuple[int, int]:
    # Firmware C integer division truncates toward zero for negative w too.
    half = math.trunc(angular_mradps * WHEEL_BASE_MM / 2000)
    return linear_mmps - half, linear_mmps + half


CENTER_SPEED_WINDOW_S = 0.5
CENTER_SPEED_UPDATE_S = 0.2
CENTER_MAX_COMMAND_ANGULAR_MRADPS = 768


@dataclass(frozen=True)
class CenteringTarget:
    linear_mmps: int
    angular_mradps: int
    mode: str
    error_mm: float | None
    heading_error_deg: float | None
    yaw_rate_degps: float | None
    axis_trusted: bool
    centered: bool
    _proposal: dict | None = field(default=None, repr=False, compare=False)
    limited_reason: str | None = None
    estimated_speed_mmps: float | None = None
    speed_qualified: bool = False
    normal_motion_allowed: bool = False

    @property
    def wheel_targets(self) -> tuple[int, int]:
        return _motion_wheels(self.linear_mmps, self.angular_mradps)


@dataclass(frozen=True)
class _CenterPose:
    at: float
    x: float
    y: float
    yaw: float
    travel_mm: float = 0.0

    @property
    def x_mm(self) -> float:
        return self.x

    @property
    def y_mm(self) -> float:
        return self.y


@dataclass(frozen=True)
class _GyroCalibration:
    """Latest stopped-window mean and frozen bias; telemetry stays intact."""

    identity: tuple
    reference_at: float
    reference_deg: float
    bias_degps: float
    state_count: int
    gyro_count: int

    def yaw_correction(self, at):
        return self.bias_degps * (at - self.reference_at)


class _StartupGyroCalibration:
    """Fit stopped bias cumulatively; anchor direction to the latest1s mean."""

    def __init__(self, barrier):
        self.barrier = barrier
        self.identity = None
        self.states, self.rates = [], []
        self._state_key = self._gyro_key = self._diag_key = None
        self._state_at = self._gyro_at = None
        self._raw_yaw = self._unwrapped_yaw = None
        self._wheels = self._firmware_bias = None
        self.requires_restop = False

    @staticmethod
    def _newer(current, previous):
        return previous is None or 0 < ((current - previous) & 0xFFFFFFFF) < 0x80000000

    def add_state(self, state, at, now):
        values = (getattr(state, "uptime_ms", None), getattr(state, "yaw_tenth_deg", None),
                  getattr(state, "left_mm", None), getattr(state, "right_mm", None), at, now)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            return
        raw, angle, left, right, at, now = values
        if (not isinstance(raw, int) or at <= self.barrier + 1e-9
                or not 0 <= now - at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                or getattr(state, "odom_status", 0)
                or not getattr(state, "state_flags", StateFlag.ODOM_VALID) & StateFlag.ODOM_VALID):
            return
        identity = (getattr(state, "session_nonce", None), getattr(state, "boot_id", None))
        if None in identity:
            return
        if self.identity is None:
            self.identity = identity
        if identity != self.identity:
            self.requires_restop = True
            return
        if not self._newer(raw, self._state_key) or self._state_at is not None and at <= self._state_at:
            return
        if self._state_key is not None and (raw - self._state_key) & 0xFFFFFFFF > config.TELEMETRY_STALE_MS:
            self.requires_restop = True
            return
        wheels = left, right
        if self._wheels is not None and wheels != self._wheels:
            self.requires_restop = True
            return
        yaw = angle / 10.0
        self._unwrapped_yaw = (yaw if self._raw_yaw is None else
                               self._unwrapped_yaw + (yaw - self._raw_yaw + 180.0) % 360.0 - 180.0)
        self.states.append((at, self._unwrapped_yaw))
        self._state_key, self._state_at = raw, at
        self._raw_yaw, self._wheels = yaw, wheels

    def add_diagnostics(self, diagnostics, generated_at, now):
        if self.identity is None:
            return
        identity = (getattr(diagnostics, "session_nonce", None), getattr(diagnostics, "boot_id", None))
        values = (getattr(diagnostics, "uptime_ms", None), getattr(diagnostics, "gyro_sample_age_ms", None),
                  getattr(diagnostics, "gyro_raw_z", None), getattr(diagnostics, "gyro_bias_lsb", None),
                  generated_at, now)
        if (identity != self.identity or getattr(diagnostics, "gyro_status", None) != WireGyroStatus.READY
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values)):
            return
        raw, age, raw_z, bias, generated_at, now = values
        if not isinstance(raw, int) or age < 0 or not self._newer(raw, self._diag_key):
            return
        key, at = (raw - int(age)) & 0xFFFFFFFF, generated_at - age / 1000.0
        if (at <= self.barrier + 1e-9 or not 0 <= now - at <= CENTER_GYRO_MAX_AGE_MS / 1000.0 + 1e-9
                or not self._newer(key, self._gyro_key) or self._gyro_at is not None and at <= self._gyro_at):
            return
        if self._firmware_bias is not None and bias != self._firmware_bias:
            self.requires_restop = True
            return
        self.rates.append((at, (raw_z - bias) / 131.0))
        self._diag_key, self._gyro_key, self._gyro_at = raw, key, at
        self._firmware_bias = bias

    @staticmethod
    def _fit(states, rates, reference_at):
        """Huber joint fit: yaw=theta+b*dt; stopped raw gyro rate=b."""
        bias = statistics.median(rate for _, rate in rates)
        theta = statistics.median(yaw - bias * (at - reference_at) for at, yaw in states)
        rate_scale = max(.03, 1.4826 * statistics.median(abs(rate - bias) for _, rate in rates))
        yaw_scale = max(.05, 1.4826 * statistics.median(
            abs(yaw - theta - bias * (at - reference_at)) for at, yaw in states))
        for _ in range(12):
            aa = ab = bb = ay = by = 0.0
            for at, yaw in states:
                dt = at - reference_at
                residual = abs(yaw - theta - bias * dt)
                weight = min(1.0, 1.5 * yaw_scale / max(residual, 1e-12)) / yaw_scale ** 2
                aa += weight
                ab += weight * dt
                bb += weight * dt * dt
                ay += weight * yaw
                by += weight * dt * yaw
            for _, rate in rates:
                weight = min(1.0, 1.5 * rate_scale / max(abs(rate - bias), 1e-12)) / rate_scale ** 2
                bb += weight
                by += weight * rate
            determinant = aa * bb - ab * ab
            if determinant <= 0:
                return None
            fitted_theta, fitted_bias = (ay * bb - by * ab) / determinant, (by * aa - ay * ab) / determinant
            change = max(abs(fitted_theta - theta), abs(fitted_bias - bias))
            theta, bias = fitted_theta, fitted_bias
            if change <= 1e-9:
                break
        return theta, bias

    def evaluate(self, started_at, ended_at):
        states = [(at, yaw) for at, yaw in self.states if at <= ended_at + 1e-9]
        rates = [(at, rate) for at, rate in self.rates if at <= ended_at + 1e-9]
        recent_states = [(at, yaw) for at, yaw in states if at > started_at + 1e-9]
        recent_rates = [(at, rate) for at, rate in rates if at > started_at + 1e-9]
        details = dict(state_count=len(recent_states), gyro_count=len(recent_rates),
                       cumulative_state_count=len(states), cumulative_gyro_count=len(rates),
                       state_span_s=(recent_states[-1][0] - recent_states[0][0] if recent_states else 0.0),
                       gyro_span_s=(recent_rates[-1][0] - recent_rates[0][0] if recent_rates else 0.0),
                       yaw_span_deg=(max(y for _, y in recent_states) - min(y for _, y in recent_states)
                                     if recent_states else None))
        if self.requires_restop:
            details["reason"] = "静止段身份、逐轮位置或源连续性改变，重新核验停稳"
            return None, details
        if (len(recent_states) < 15 or len(recent_rates) < 8
                or details["state_span_s"] < .8 - 1e-9 or details["gyro_span_s"] < .7 - 1e-9):
            details["reason"] = "最近1秒真实新源数量或跨度不足，继续停车采样1秒"
            return None, details
        if details["yaw_span_deg"] > .3 + 1e-9:
            details["reason"] = "最近1秒原始航向跨度超过0.3°，继续停车采样1秒"
            return None, details
        # Keep the starting direction tied to the actual latest stopped window.
        # The cumulative fit estimates bias only; older yaw cannot move this mean.
        reference_at = math.fsum(at for at, _ in recent_states) / len(recent_states)
        reference_deg = math.fsum(yaw for _, yaw in recent_states) / len(recent_states)
        fit = self._fit(states, rates, reference_at)
        if fit is None or any(not math.isfinite(v) for v in fit):
            details["reason"] = "累计静止拟合不可用，继续停车采样1秒"
            return None, details
        theta, bias = fit
        residuals = sorted(abs(rate - bias) for _, rate in recent_rates)
        percentile = residuals[max(0, math.ceil(.95 * len(residuals)) - 1)]
        details.update(bias_degps=bias, fitted_reference_deg=theta,
                       window_mean_reference_deg=reference_deg, reference_at_s=reference_at,
                       raw_debiased_rate_p95_degps=percentile)
        if percentile > .2 + 1e-9:
            details["reason"] = "最近1秒95%去偏原始角速未在±0.2°/s内，继续停车采样1秒"
            return None, details
        details["reason"] = "最近1秒稳定，真实STATE航向均值作初始固定参考，累计拟合仅估计残余零偏"
        return _GyroCalibration(self.identity, reference_at, reference_deg, bias, len(states), len(rates)), details


class CenteringController:
    """Normal forward control in an immutable, manually aligned road frame.

    Real source clocks authorize feedback and mean-window evidence. Targets are
    proposals; only a successful write commits speed, hold-heading and integral
    changes. Centered windows describe position, independently of permission to
    drive at the requested speed; normal control only sends positive wheel targets.
    """

    def __init__(self, now: float | None = None) -> None:
        self.reset(time.monotonic() if now is None else now)

    @staticmethod
    def _angle(value: float) -> float:
        return (value + 180.0) % 360.0 - 180.0

    def reset(self, now: float, source_barrier: float | None = None) -> None:
        self._gyro_calibration = getattr(self, "_gyro_calibration", None)
        self.source_barrier = source_barrier
        self._road_reference = self._hold_heading = None
        self._initial_reference_fixed = False
        self._state_session = self._state_uptime = self._state_at = None
        self._state_source_s = self._wheels = None
        self._poses = deque(maxlen=256)
        self._yaw_rate = None
        self._rate_now = now
        self._gyro_uptime = self._gyro_source_uptime = None
        self._gyro_source_at = self._gyro_yaw_rate = None
        self._gyro_control_raw_rate = self._gyro_filtered_rate = None
        self._gyro_filter_samples = deque(maxlen=3)
        self._gyro_control_history = deque(maxlen=64)
        self._side_history = (deque(maxlen=64), deque(maxlen=64))
        self._source_keys = [None, None]
        self._paired_keys = [None, None]
        self._control_cache = [None, None]
        self._control_source_keys = [None, None]
        self._last_seq = self._last_snapshot_uptime = None
        self._pending_pair_queue = deque(maxlen=64)
        self._pending_pair_sample = None
        self._pair_at = self._latest_source_times = self._last_pair_pose = None
        self._last_raw_difference = None
        self._compensated_sources = deque(maxlen=256)
        self._error = self._raw_error = self._observed_error = None
        self._error_rate = 0.0
        self._usable_pair = False
        self.observation_token = 0
        self._last_sent_angular = self._last_angular = 0
        self._angular_residual_mradps = 0.0
        self._angular_residual_request_sign = 0
        self._last_sent_wheels = (0, 0)
        self._last_sent_mode = "WAIT"
        self._last_target_at = now
        self._last_sent_at = None
        self._stopped = True
        self._applied_linear = self._applied_angular = 0
        self._applied_at = self._applied_source_at = None
        self._forward_applied_barrier = None
        self._drive_started_at = None
        self._speed_samples = deque(maxlen=128)
        self._motion_epoch = 0
        self._clear_alignment(now)

    def _clear_alignment(self, now):
        self._motion_epoch += 1
        self._normal_motion_qualified = False
        self._hold_locked = False
        self._good_windows = 0
        self._mean_last_end = None
        self._mean_source_keys = (None, None)
        self._pending_means = deque(maxlen=4)
        self._compensated_sources.clear()
        self._last_mean_difference = None
        self._last_mean_heading = None
        self._mean_reason = "等待两个独立0.5秒新鲜双侧均值窗"
        self._position_integral_deg = 0.0
        self._position_integral_key = None
        self._position_integral_source_at = None
        self._angular_residual_mradps = 0.0
        self._angular_residual_request_sign = 0
        self._lateral_sources = deque(maxlen=32)
        self._speed_trim = 0.0
        self._speed_last_update_at = self._speed_last_integral_at = now
        self._speed_last_state_key = None
        self._speed_qualified = False
        self._speed_integral_frozen = True
        self._speed_limit_reason = None
        self._ramp_speed = self._ramp_complete_at = self._ramp_requested = None
        self._control_feedback = None

    def begin_alignment(self, now, *, source_barrier=None):
        """First post-turn walls restart normal control, retaining real sources."""
        self._clear_alignment(now)
        if source_barrier is not None:
            self.source_barrier = source_barrier

    @property
    def error_mm(self):
        return self._error

    @property
    def raw_error_mm(self):
        return self._raw_error

    @property
    def observed_error_mm(self):
        return self._observed_error

    @property
    def full_difference_mm(self):
        return None if self._raw_error is None else 2.0 * abs(self._raw_error)

    @property
    def signed_difference_mm(self):
        return None if self._raw_error is None else 2.0 * self._raw_error

    @property
    def error_rate_mmps(self):
        return self._error_rate

    @property
    def latest_source_time(self):
        return self._pair_at

    @property
    def latest_source_times(self):
        return self._latest_source_times

    @property
    def latest_source_keys(self):
        return tuple(self._paired_keys)

    @property
    def yaw_deg(self):
        return self._angle(self._poses[-1].yaw) if self._poses else None

    @property
    def odom_mm(self):
        return self._wheels

    @property
    def world_pose(self):
        pose = self._poses[-1] if self._poses else None
        return (pose.x, pose.y, pose.yaw) if pose is not None else None

    @property
    def heading_reference_deg(self):
        return self._road_reference

    @property
    def formal_reference_deg(self):
        return self._road_reference

    @property
    def provisional_reference_deg(self):
        return self._road_reference

    @property
    def axis_yaw_deg(self):
        return self._road_reference

    @property
    def axis_trusted(self):
        # Retained target/display field means an initialized fixed reference.
        return self._road_reference is not None

    @property
    def hold_heading_deg(self):
        return self._hold_heading

    @property
    def normal_motion_qualified(self):
        return self._normal_motion_qualified

    @property
    def speed_stage_qualified(self):
        return self._normal_motion_qualified

    @property
    def correction_complete(self):
        return self.centered

    @property
    def centered(self):
        return bool(self._good_windows >= 2 and self._usable_pair
                    and self._raw_error is not None
                    and 2.0 * abs(self._raw_error) <= SIDE_DEADBAND_MM + 1e-9
                    and self.heading_ready(tolerance_deg=NORMAL_HEADING_TOLERANCE_DEG)
                    and all(item is not None
                            and -1e-9 <= self._rate_now - item[0] <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                            for item in self._control_cache))

    @property
    def yaw_rate_degps(self):
        return self._effective_yaw_rate()

    @property
    def yaw_rate_source(self):
        if (self._gyro_source_at is not None and self._gyro_yaw_rate is not None
                and 0 <= self._rate_now - self._gyro_source_at <= CENTER_GYRO_MAX_AGE_MS / 1000.0 + 1e-9):
            return "DIAG"
        return "STATE" if self._effective_yaw_rate() is not None else None

    @property
    def sent_angular_mradps(self):
        return self._last_sent_angular

    @property
    def actual_speed_mmps(self):
        return self._estimate_speed(self._rate_now)

    @property
    def speed_qualified(self):
        return self._speed_qualified

    @property
    def speed_stable(self):
        return self._speed_qualified

    @property
    def speed_limit_reason(self):
        return self._speed_limit_reason


    def set_initial_reference(self, yaw):
        if (isinstance(yaw, bool) or not isinstance(yaw, (int, float))
                or not math.isfinite(yaw)):
            return False
        reference = self._angle(float(yaw))
        if self._initial_reference_fixed:
            return abs(self._angle(reference - self._road_reference)) <= 1e-9
        self._road_reference = self._hold_heading = reference
        self._initial_reference_fixed = True
        return True

    def _heading_error(self):
        return (self._angle(self.yaw_deg - self._road_reference)
                if self._road_reference is not None and self.yaw_deg is not None else None)

    def heading_ready(self, reference=None, *, tolerance_deg=NORMAL_HEADING_TOLERANCE_DEG, rate_limit_degps=None):
        reference = self._road_reference if reference is None else reference
        if (reference is None or self.yaw_deg is None or self._state_at is None
                or not -1e-9 <= self._rate_now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9):
            return False
        if abs(self._angle(self.yaw_deg - reference)) > tolerance_deg + 1e-9:
            return False
        rate = self._effective_yaw_rate()
        if (self._gyro_control_raw_rate is not None and self._gyro_source_at is not None
                and 0 <= self._rate_now - self._gyro_source_at <= CENTER_GYRO_MAX_AGE_MS / 1000.0 + 1e-9):
            rate = max((rate, self._gyro_control_raw_rate), key=abs) if rate is not None else self._gyro_control_raw_rate
        return rate_limit_degps is None or rate is not None and abs(rate) <= rate_limit_degps + 1e-9

    def refresh_yaw_rate(self, now):
        self._rate_now = max(self._rate_now, now)

    def install_gyro_calibration(self, calibration):
        if not isinstance(calibration, _GyroCalibration) or calibration.identity != self._state_session:
            return False
        previous = self._gyro_calibration
        self._gyro_calibration = calibration

        def corrected(pose):
            if pose is None:
                return None
            raw_yaw = pose.yaw + (previous.yaw_correction(pose.at) if previous is not None else 0.0)
            return replace(pose, yaw=raw_yaw - calibration.yaw_correction(pose.at))

        self._poses = deque((corrected(pose) for pose in self._poses), maxlen=256)
        self._last_pair_pose = corrected(self._last_pair_pose)
        self._gyro_filter_samples.clear()
        self._gyro_control_history.clear()
        self._gyro_filtered_rate = None
        if self._gyro_yaw_rate is not None and self._gyro_source_at is not None:
            self._gyro_control_raw_rate = self._gyro_yaw_rate - calibration.bias_degps
            self._filter_gyro_rate(self._gyro_control_raw_rate, self._gyro_source_at)
        self._update_state_yaw_rate()
        return True

    def _filter_gyro_rate(self, rate, source_at):
        previous = self._gyro_filter_samples[-1][0] if self._gyro_filter_samples else None
        if previous is not None and source_at - previous > CENTER_GYRO_MAX_AGE_MS / 1000.0 + 1e-9:
            self._gyro_filter_samples.clear()
            self._gyro_filtered_rate = None
            previous = None
        self._gyro_filter_samples.append((source_at, rate))
        median = statistics.median(value for _, value in self._gyro_filter_samples)
        if self._gyro_filtered_rate is None or previous is None:
            self._gyro_filtered_rate = median
        else:
            alpha = -math.expm1(-(source_at - previous) / .05)
            self._gyro_filtered_rate += alpha * (median - self._gyro_filtered_rate)
        self._gyro_control_history.append((source_at, rate, self._gyro_filtered_rate))

    def observe_diagnostics(self, diagnostics, generated_at):
        session = (getattr(diagnostics, "session_nonce", None), getattr(diagnostics, "boot_id", None))
        raw, age = getattr(diagnostics, "uptime_ms", None), getattr(diagnostics, "gyro_sample_age_ms", None)
        raw_z, bias = getattr(diagnostics, "gyro_raw_z", None), getattr(diagnostics, "gyro_bias_lsb", None)
        if (self._state_session is None or session != self._state_session or isinstance(raw, bool)
                or not isinstance(raw, int) or any(isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) for value in (generated_at, age, raw_z, bias)) or age < 0):
            return
        self.refresh_yaw_rate(generated_at)
        if self._gyro_uptime is not None and not _source_key_newer(raw, self._gyro_uptime):
            return
        self._gyro_uptime = raw
        if getattr(diagnostics, "gyro_status", None) != WireGyroStatus.READY:
            self._gyro_yaw_rate = self._gyro_source_at = None
            self._gyro_control_raw_rate = self._gyro_filtered_rate = None
            self._gyro_filter_samples.clear()
            self._gyro_control_history.clear()
            return
        source_uptime = (raw - int(age)) & 0xFFFFFFFF
        source_at = generated_at - age / 1000.0
        if (not 0 <= self._rate_now - source_at <= CENTER_GYRO_MAX_AGE_MS / 1000.0 + 1e-9
                or not _source_key_newer(source_uptime, self._gyro_source_uptime)
                or self._gyro_source_at is not None and source_at <= self._gyro_source_at):
            return
        self._gyro_source_uptime, self._gyro_source_at = source_uptime, source_at
        self._gyro_yaw_rate = (raw_z - bias) / 131.0
        correction = self._gyro_calibration.bias_degps if self._gyro_calibration is not None else 0.0
        self._gyro_control_raw_rate = self._gyro_yaw_rate - correction
        self._filter_gyro_rate(self._gyro_control_raw_rate, source_at)

    def gyro_calibration_snapshot(self, now=None):
        now = self._rate_now if now is None else now
        calibration = self._gyro_calibration
        pose = self._poses[-1] if self._poses else None
        correction = calibration.yaw_correction(pose.at) if calibration is not None and pose is not None else 0.0
        return dict(calibrated=calibration is not None,
                    bias_degps=calibration.bias_degps if calibration is not None else None,
                    reference_at_s=calibration.reference_at if calibration is not None else None,
                    initial_reference_deg=calibration.reference_deg if calibration is not None else None,
                    cumulative_state_count=calibration.state_count if calibration is not None else 0,
                    cumulative_gyro_count=calibration.gyro_count if calibration is not None else 0,
                    raw_yaw_deg=self._angle(pose.yaw + correction) if pose is not None else None,
                    control_yaw_deg=self.yaw_deg, yaw_correction_deg=correction,
                    raw_rate_degps=self._gyro_yaw_rate,
                    debiased_raw_rate_degps=self._gyro_control_raw_rate,
                    filtered_rate_degps=self._gyro_filtered_rate,
                    source_age_ms=(now - self._gyro_source_at) * 1000.0 if self._gyro_source_at is not None else None,
                    frozen=True, basis="静止窗平均航向与冻结残余零偏；运动中不学习")

    def _effective_yaw_rate(self, now=None):
        now = self._rate_now if now is None else now
        if (self._gyro_source_at is not None and self._gyro_yaw_rate is not None
                and 0 <= now - self._gyro_source_at <= CENTER_GYRO_MAX_AGE_MS / 1000.0 + 1e-9):
            return self._gyro_filtered_rate if self._gyro_filtered_rate is not None else self._gyro_control_raw_rate
        if self._state_at is not None and 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9:
            return self._yaw_rate
        return None

    def _source_yaw_rate(self, at):
        for source_at, raw, filtered in reversed(self._gyro_control_history):
            if 0 <= at - source_at <= CENTER_GYRO_MAX_AGE_MS / 1000.0 + 1e-9:
                return max((raw, filtered), key=abs)
        pose = self._pose_at(at)
        candidates = [p for p in self._poses if 0 <= at - p.at <= .18 + 1e-9]
        if pose is None or not candidates or at - candidates[0].at < .08 - 1e-9:
            return None
        return (pose.yaw - candidates[0].yaw) / (at - candidates[0].at)

    def _update_state_yaw_rate(self):
        self._yaw_rate = None
        if len(self._poses) < 2:
            return
        latest = self._poses[-1]
        recent = [pose for pose in self._poses if 0 <= latest.at - pose.at <= .18 + 1e-9]
        if recent and latest.at - recent[0].at >= .08 - 1e-9:
            self._yaw_rate = (latest.yaw - recent[0].yaw) / (latest.at - recent[0].at)

    def observe_state(self, state, generated_at):
        raw = getattr(state, "uptime_ms", None)
        values = (getattr(state, "left_mm", None), getattr(state, "right_mm", None),
                  getattr(state, "yaw_tenth_deg", None), generated_at)
        if (isinstance(raw, bool) or not isinstance(raw, int) or any(isinstance(v, bool)
                or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values)
                or getattr(state, "odom_status", 0)
                or not getattr(state, "state_flags", StateFlag.ODOM_VALID) & StateFlag.ODOM_VALID):
            return
        identity = (getattr(state, "session_nonce", None), getattr(state, "boot_id", None))
        if (self._state_session is not None and identity != self._state_session
                or self._gyro_calibration is not None and identity != self._gyro_calibration.identity):
            self._gyro_calibration = None
            self.reset(generated_at, source_barrier=generated_at)
        self._state_session = identity
        elapsed_ms = 0
        if self._state_uptime is not None:
            elapsed_ms = (raw - self._state_uptime) & 0xFFFFFFFF
            if not 0 < elapsed_ms < 0x80000000 or generated_at <= self._state_at:
                return
            if elapsed_ms > config.TELEMETRY_STALE_MS:
                self._speed_samples.clear()
                self._clear_alignment(generated_at)
                self._side_history = (deque(maxlen=64), deque(maxlen=64))
                self._source_keys = [None, None]
                self._paired_keys = [None, None]
                self._control_cache = [None, None]
                self._control_source_keys = [None, None]
                self._pending_pair_queue.clear()
                self._pair_at = self._latest_source_times = self._last_pair_pose = None
                self._usable_pair = False
                self._poses.clear()
                self.source_barrier = max(self.source_barrier or -math.inf, generated_at)
                self._drive_started_at = None
        wheels = float(values[0]), float(values[1])
        yaw = values[2] / 10.0
        if self._gyro_calibration is not None:
            yaw -= self._gyro_calibration.yaw_correction(generated_at)
        if self._poses:
            previous = self._poses[-1]
            yaw = previous.yaw + self._angle(yaw - previous.yaw)
            travel = sum(new - old for new, old in zip(wheels, self._wheels)) / 2.0
            middle = math.radians((yaw + previous.yaw) / 2.0)
            pose = _CenterPose(generated_at, previous.x + travel * math.cos(middle),
                               previous.y + travel * math.sin(middle), yaw, previous.travel_mm + travel)
            if wheels != self._wheels:
                self._stopped = False
        else:
            pose = _CenterPose(generated_at, 0.0, 0.0, yaw)
        self._poses.append(pose)
        self._state_source_s = (raw / 1000.0 if self._state_source_s is None
                                else self._state_source_s + elapsed_ms / 1000.0)
        self._state_uptime, self._state_at, self._wheels = raw, generated_at, wheels
        self._speed_samples.append((self._state_source_s, sum(wheels) / 2.0, generated_at, *wheels))
        while len(self._speed_samples) > 2 and self._state_source_s - self._speed_samples[0][0] > 2.0:
            self._speed_samples.popleft()
        while len(self._poses) > 2 and generated_at - self._poses[0].at > CENTER_HISTORY_S:
            self._poses.popleft()
        self.refresh_yaw_rate(generated_at)
        self._update_state_yaw_rate()
        self.retry_pending_pair(now=generated_at)
        self._consume_pending_means(generated_at)

    def _pose_at(self, at):
        if not self._poses or at < self._poses[0].at - 1e-9 or at > self._poses[-1].at + 1e-9:
            return None
        previous = self._poses[0]
        for current in self._poses:
            if current.at >= at - 1e-9:
                if current.at == previous.at:
                    return current
                fraction = max(0.0, min(1.0, (at - previous.at) / (current.at - previous.at)))
                return _CenterPose(at, *(getattr(previous, name) + fraction * (getattr(current, name) - getattr(previous, name))
                                        for name in ("x", "y", "yaw", "travel_mm")))
            previous = current
        return previous

    def pose_at(self, at):
        return self._pose_at(at)

    @staticmethod
    def _side_at(history, at):
        if not history or at < history[0][0] - 1e-9:
            return None
        previous = history[0]
        for current in history:
            if current[0] >= at:
                if current[0] == previous[0]:
                    return current[1:]
                fraction = (at - previous[0]) / (current[0] - previous[0])
                return tuple(previous[i] + fraction * (current[i] - previous[i]) for i in (1, 2))
            previous = current
        return previous[1:]

    def observe_side(self, sample):
        if sample.source_identity is not None and sample.source_identity != self._state_session:
            return False
        self.refresh_yaw_rate(sample.received_at)
        uptime = sample.snapshot_uptime_ms
        if self._last_seq is not None:
            newer = classify_sequence(sample.seq, self._last_seq) is SequenceDisposition.NEWER
            same_update = (sample.seq == self._last_seq and uptime is not None
                           and self._last_snapshot_uptime is not None
                           and _source_key_newer(uptime, self._last_snapshot_uptime))
            if not newer and not same_update:
                return False
        self._last_seq, self._last_snapshot_uptime = sample.seq, uptime
        for index, (distance, status) in enumerate(((sample.left_mm, sample.left_status),
                                                   (sample.right_mm, sample.right_status))):
            at, age = sample.source_times[index], sample.ages_ms[index]
            raw_age = sample.raw_ages_ms[index] if sample.raw_ages_ms is not None else age
            if (status != WireRangeStatus.VALID or isinstance(distance, bool)
                    or not isinstance(distance, (int, float)) or not math.isfinite(distance)
                    or not 0 <= distance < 65535 or not 0 <= raw_age <= LASER_MEASUREMENT_MAX_AGE_MS
                    or not 0 <= sample.received_at - at <= LASER_CACHE_MAX_AGE_MS / 1000.0 + 1e-9
                    or self.source_barrier is not None and at <= self.source_barrier + 1e-9):
                continue
            key = sample.source_keys[index] if sample.source_keys is not None else at
            cache = self._control_cache[index]
            if _source_key_newer(key, self._control_source_keys[index]) and (cache is None or at > cache[0]):
                self._control_cache[index] = (at, float(distance), key)
                self._control_source_keys[index] = key
            if (not 0 <= age <= LASER_MEASUREMENT_MAX_AGE_MS
                    or not 0 <= sample.received_at - at <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                    or not _source_key_newer(key, self._source_keys[index])):
                continue
            history = self._side_history[index]
            if history and at <= history[-1][0]:
                continue
            filtered = float(distance)
            if history:
                alpha = -math.expm1(-(at - history[-1][0]) / CENTER_FILTER_TAU_S)
                filtered = history[-1][2] + alpha * (distance - history[-1][2])
            history.append((at, float(distance), filtered))
            self._source_keys[index] = key
        if (sample.dual_wall and all(0 <= age <= LASER_MEASUREMENT_MAX_AGE_MS for age in sample.ages_ms)
                and all(self._side_history)):
            previous_keys = self._pending_pair_queue[-1][2] if self._pending_pair_queue else self._paired_keys
            if all(_source_key_newer(key, old) for key, old in zip(self._source_keys, previous_keys)):
                self._pending_pair_queue.append((sample, tuple(tuple(h) for h in self._side_history),
                                                 tuple(self._source_keys)))
        else:
            self._usable_pair = False
        self._pending_pair_sample = self._pending_pair_queue[0][0] if self._pending_pair_queue else None
        return self.retry_pending_pair()

    def retry_pending_pair(self, sample=None, now=None):
        if now is not None:
            self.refresh_yaw_rate(now)
        if sample is not None:
            return self.observe_side(sample)
        consumed = False
        while self._pending_pair_queue:
            candidate, histories, keys = self._pending_pair_queue[0]
            sources = tuple(history[-1][0] for history in histories)
            at = min(sources)
            if (candidate.source_identity is not None and candidate.source_identity != self._state_session
                    or self.source_barrier is not None and any(s <= self.source_barrier + 1e-9 for s in sources)
                    or any(not 0 <= self._rate_now - s <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9 for s in sources)
                    or self._pair_at is not None and at <= self._pair_at + 1e-9
                    or not all(_source_key_newer(key, old) for key, old in zip(keys, self._paired_keys))):
                self._pending_pair_queue.popleft()
                continue
            if not self._poses or at > self._poses[-1].at + 1e-9:
                break
            self._pending_pair_queue.popleft()
            pose = self._pose_at(at)
            aligned = [self._side_at(history, at) for history in histories]
            if pose is None or any(item is None for item in aligned):
                continue
            raw_difference = aligned[0][0] - aligned[1][0]
            psi = math.radians(self._angle(pose.yaw - self._road_reference)) if self._road_reference is not None else 0.0
            # Positive means axle right of the corridor centre. Each real
            # source pair is corrected before either filtering or averaging.
            observed = raw_difference * math.cos(psi) / 2.0 + CENTER_SENSOR_FORWARD_MM * math.sin(psi)
            error = observed
            if self._pair_at is not None and at > self._pair_at and self._error is not None:
                dt = at - self._pair_at
                alpha = -math.expm1(-dt / CENTER_FILTER_TAU_S)
                previous = self._error
                if self._last_pair_pose is not None and self._road_reference is not None:
                    axis = math.radians(self._road_reference)
                    lateral = (-(pose.x - self._last_pair_pose.x) * math.sin(axis)
                               + (pose.y - self._last_pair_pose.y) * math.cos(axis))
                    predicted = previous - lateral
                else:
                    predicted = previous
                error = predicted + alpha * (observed - predicted)
                self._error_rate += alpha * ((error - previous) / dt - self._error_rate)
            self._pair_at, self._latest_source_times = at, sources
            self._last_pair_pose = pose
            self._raw_error = self._observed_error = observed
            self._last_raw_difference = raw_difference
            self._error, self._usable_pair = error, True
            self._paired_keys = list(keys)
            self.observation_token += 1
            self._lateral_sources.append((at, pose.travel_mm, observed))
            self._compensated_sources.append((at, sources, tuple(keys), observed, pose.yaw))
            consumed = True
        self._pending_pair_sample = self._pending_pair_queue[0][0] if self._pending_pair_queue else None
        return consumed

    def control_available(self, now):
        return (self._error is not None and all(item is not None
                and 0 <= now - item[0] <= LASER_CACHE_MAX_AGE_MS / 1000.0 + 1e-9
                and (self.source_barrier is None or item[0] > self.source_barrier + 1e-9)
                for item in self._control_cache))

    def predicted_error_mm(self, now):
        if self._error is None or self._last_pair_pose is None or not self._poses or self._road_reference is None:
            return self._error
        pose, sensed = self._poses[-1], self._last_pair_pose
        axis = math.radians(self._road_reference)
        lateral = -(pose.x - sensed.x) * math.sin(axis) + (pose.y - sensed.y) * math.cos(axis)
        return self._error - lateral


    def observe_mean_batch(self, batch, now=None):
        now = self._rate_now if now is None else now
        if self._pending_means and batch.ended_at <= self._pending_means[-1].ended_at + 1e-9:
            return False
        if self._mean_last_end is not None and batch.ended_at <= self._mean_last_end + 1e-9:
            return False
        if len(self._pending_means) == self._pending_means.maxlen:
            self._pending_means.popleft()
            self._good_windows = 0
            self._mean_reason = "均值窗等待STATE覆盖溢出，重建连续证据"
        self._pending_means.append(batch)
        return self._consume_pending_means(now)

    def _consume_pending_means(self, now):
        consumed = False
        while self._pending_means:
            batch = self._pending_means[0]
            keys = getattr(batch, "source_keys", None)
            sources = getattr(batch, "source_times", None)
            identity = getattr(batch, "identity", None)
            started = getattr(batch, "started_at", None)
            valid = (batch.complete and batch.left_count > 0 and batch.right_count > 0
                     and identity == self._state_session and keys is not None and sources is not None
                     and isinstance(started, (int, float)) and not isinstance(started, bool)
                     and math.isfinite(started)
                     and abs(batch.ended_at - started - AVERAGE_INTERVAL_S) <= 1e-6
                     and (self._mean_last_end is None or abs(started - self._mean_last_end) <= 1e-6)
                     and len(keys) == 2 and len(sources) == 2
                     and all(isinstance(key, (int, float)) and not isinstance(key, bool)
                             and math.isfinite(key) for key in keys)
                     and -1e-9 <= now - batch.ended_at <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                     and all(_source_key_newer(key, old) for key, old in zip(keys, self._mean_source_keys))
                     and all(isinstance(s, (int, float)) and not isinstance(s, bool) and math.isfinite(s)
                             and 0 <= batch.ended_at - s <= AVERAGE_INTERVAL_S + LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                             for s in sources)
                     and (self.source_barrier is None or all(s > self.source_barrier + 1e-9 for s in sources))
                     and self._forward_applied_barrier is not None
                     and started >= self._forward_applied_barrier - 1e-9
                     and all(s > self._forward_applied_barrier + 1e-9 for s in sources)
                     and self._road_reference is not None)
            pose = self._pose_at(batch.ended_at)
            if valid and pose is None and self._poses and batch.ended_at > self._poses[-1].at + 1e-9:
                break
            self._pending_means.popleft()
            self._mean_last_end = batch.ended_at
            samples = ([item for item in self._compensated_sources
                        if all(started + 1e-9 < source <= batch.ended_at + 1e-9 for source in item[1])]
                       if isinstance(started, (int, float)) else [])
            # Averaging L/R first and compensating once would lose sensor
            # forward-offset changes while the car turns inside this window.
            self._last_mean_difference = (abs(2.0 * math.fsum(item[3] for item in samples) / len(samples))
                                          if samples else None)
            self._last_mean_heading = (self._angle(pose.yaw - self._road_reference)
                                      if pose is not None and self._road_reference is not None else None)
            if valid:
                self._mean_source_keys = tuple(keys)
            acceptable = (valid and pose is not None
                          and bool(samples)
                          and self._last_mean_difference <= SIDE_DEADBAND_MM + 1e-9
                          and abs(self._last_mean_heading) <= NORMAL_HEADING_TOLERANCE_DEG + 1e-9)
            self._good_windows = min(2, self._good_windows + 1) if acceptable else 0
            self._mean_reason = ("连续均值窗与固定参考偏角达标" if acceptable else
                                 "空窗、旧源、超龄、车轴补偿均值差或偏角未达标")
            consumed = True
        return consumed

    def stage_snapshot(self):
        return dict(stage="NORMAL" if self._normal_motion_qualified else "READY" if self.axis_trusted else "WAIT",
                    fixed_reference_deg=self._road_reference, hold_heading_deg=self._hold_heading,
                    normal_motion_allowed=self._normal_motion_qualified, centered=self.centered,
                    mean_good_windows=self._good_windows, mean_count=self._good_windows,
                    mean_difference_mm=self._last_mean_difference,
                    mean_heading_error_deg=self._last_mean_heading, mean_reason=self._mean_reason,
                    heading_tolerance_deg=NORMAL_HEADING_TOLERANCE_DEG,
                    correction_heading_limit_deg=None,
                    position_integral_deg=self._position_integral_deg,
                    position_integral_limit_deg=POSITION_INTEGRAL_LIMIT_DEG)


    def notify_stopped(self, now, *, stable=True):
        self.refresh_yaw_rate(now)
        self._clear_alignment(now)
        self._last_sent_angular = self._last_angular = 0
        self._last_sent_wheels = (0, 0)
        self._last_sent_mode = "WAIT"
        self._last_sent_at = None
        self._last_target_at = now
        self._stopped = bool(stable)
        self._applied_linear = self._applied_angular = 0
        self._applied_at = self._applied_source_at = self._forward_applied_barrier = None
        self._drive_started_at = None
        self._speed_samples.clear()
        self._gyro_filter_samples.clear()
        self._gyro_control_history.clear()
        self._gyro_filtered_rate = None
        self._pending_pair_queue.clear()
        self._pending_pair_sample = None
        self._usable_pair = False
        if stable:
            self.source_barrier = max(self.source_barrier or -math.inf, now)

    def revoke_normal_motion(self):
        self._normal_motion_qualified = False
        self._good_windows = 0

    def notify_sent(self, angular_mradps, *, sent_at=None):
        self._last_sent_angular = self._last_angular = int(angular_mradps)
        at = self._rate_now if sent_at is None else sent_at
        self._last_sent_at = at
        self._stopped = False

    def target_send_ready(self, target, now):
        """Revalidate source and motion identity immediately before a write."""
        self.refresh_yaw_rate(now)
        proposal = target._proposal or {}
        if (proposal.get("epoch") != self._motion_epoch
                or proposal.get("identity") != self._state_session
                or self._road_reference is None or self._state_at is None
                or not -1e-9 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9):
            return False
        if proposal.get("departure"):
            return True
        cached = proposal.get("cached", False)
        if cached and not self._normal_motion_qualified:
            return False
        maximum_age = LASER_CACHE_MAX_AGE_MS if cached else LASER_MEASUREMENT_MAX_AGE_MS
        return bool((cached or self._usable_pair) and proposal.get("token") == self.observation_token
                    and all(item is not None
                            and -1e-9 <= now - item[0] <= maximum_age / 1000.0 + 1e-9
                            and (self.source_barrier is None or item[0] > self.source_barrier + 1e-9)
                            for item in self._control_cache))

    def confirm_target_sent(self, target):
        if target.linear_mmps <= 0 or any(wheel < CENTER_MIN_WHEEL_SPEED_MMPS for wheel in target.wheel_targets):
            raise ValueError("前进居中控制器禁止倒车、零速或低于逐轮61mm/s下限")
        proposal = target._proposal or {}
        if not self.target_send_ready(target, self._rate_now):
            return False
        for name, value in proposal.get("state", {}).items():
            setattr(self, name, value)
        if not proposal.get("departure"):
            self._normal_motion_qualified = True
        if proposal.get("capture_hold") and target.angular_mradps == 0 and self.heading_ready():
            self._hold_heading = self.yaw_deg
            self._position_integral_deg = 0.0
            self._hold_locked = True
        if target.angular_mradps != 0:
            self._hold_locked = False
        self._last_sent_wheels = target.wheel_targets
        self._last_sent_mode = target.mode
        self._speed_limit_reason = target.limited_reason
        self._last_target_at = proposal.get("at", self._rate_now)
        self.notify_sent(target.angular_mradps, sent_at=max(self._last_sent_at or -math.inf, self._last_target_at))
        return True

    def notify_target_applied(self, linear_mmps, angular_mradps, applied_at=None, *, source_at=None):
        if linear_mmps < 0 or linear_mmps == 0 and angular_mradps != 0:
            raise ValueError("普通居中应用证据只允许前进或制动零目标")
        was_driving = self._applied_linear > 0
        self._applied_linear, self._applied_angular = int(linear_mmps), int(angular_mradps)
        self._applied_at = self._rate_now if applied_at is None else applied_at
        self._applied_source_at = source_at
        if linear_mmps > 0 and (not was_driving or self._forward_applied_barrier is None):
            self._forward_applied_barrier = source_at
            self._drive_started_at = source_at
        elif linear_mmps == 0:
            self._forward_applied_barrier = self._drive_started_at = None

    def _estimate_speed(self, now):
        if (len(self._speed_samples) < 2 or self._state_at is None or self._drive_started_at is None
                or not 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                or self._state_at - self._drive_started_at < CENTER_SPEED_WINDOW_S - 1e-9):
            return None
        latest = self._speed_samples[-1]
        desired = latest[0] - CENTER_SPEED_WINDOW_S
        previous = self._speed_samples[0]
        if desired < previous[0] - 1e-9:
            return None
        for current in self._speed_samples:
            if current[0] >= desired - 1e-9:
                span = current[0] - previous[0]
                baseline = current[1] if span <= 1e-9 else previous[1] + (desired - previous[0]) / span * (current[1] - previous[1])
                return (latest[1] - baseline) / CENTER_SPEED_WINDOW_S
            previous = current
        return None

    def speed_snapshot(self, now=None):
        now = self._rate_now if now is None else now
        measured = self._estimate_speed(now)
        goal = self._ramp_requested
        saturated = bool(self._speed_limit_reason and "上限" in self._speed_limit_reason)
        return dict(actual_speed_mmps=measured,
                    requested_speed_mmps=self._ramp_requested, ramp_speed_mmps=self._ramp_speed,
                    ramp_complete_at=self._ramp_complete_at, trim_mmps=self._speed_trim,
                    qualified=self._speed_qualified, limit_reason=self._speed_limit_reason,
                    target_mean_mmps=goal, measured_mean_mmps=measured,
                    measured_wheels_mmps=self._estimate_wheel_speeds(now), window_s=CENTER_SPEED_WINDOW_S,
                    integral_frozen=self._speed_integral_frozen,
                    saturated=saturated, limited_reason=self._speed_limit_reason,
                    status="SATURATED" if saturated else "QUALIFIED" if self._speed_qualified else
                           "NO_WINDOW" if measured is None else "TRANSITION_OR_CORRECTING",
                    speed_stage_qualified=self._normal_motion_qualified,
                    basis="真实连续STATE的500ms双轮平均净轮程；启动、加减速与缺窗期间不虚报达标")

    def _estimate_wheel_speeds(self, now):
        if self._estimate_speed(now) is None:
            return None
        latest, previous = self._speed_samples[-1], self._speed_samples[0]
        desired = latest[0] - CENTER_SPEED_WINDOW_S
        for current in self._speed_samples:
            if current[0] >= desired - 1e-9:
                span = current[0] - previous[0]
                baseline = tuple(current[index] if span <= 1e-9 else previous[index]
                                 + (desired - previous[0]) / span * (current[index] - previous[index])
                                 for index in (3, 4))
                return tuple((latest[index] - baseline[index - 3]) / CENTER_SPEED_WINDOW_S
                             for index in (3, 4))
            previous = current
        return None

    @staticmethod
    def _quantized_angular(requested):
        magnitude = min(CENTER_MAX_COMMAND_ANGULAR_MRADPS,
                        math.floor(abs(requested) / CENTER_ANGULAR_QUANTUM_MRADPS + .5) * CENTER_ANGULAR_QUANTUM_MRADPS)
        return int(math.copysign(magnitude, requested)) if requested else 0


    def _fine_angular_proposal(self, requested, now, *, enabled):
        """Accumulate a sub-quantum request only through successful writes.

        A persistent small heading correction must not become an unlimited
        equal-wheel hold.  Nearest rounding discards requests below8mrad/s;
        bounded error diffusion occasionally selects the existing16mrad/s
        step without changing the heading goal or angular limits.
        """
        active = (enabled and 1e-9 < abs(requested) < CENTER_ANGULAR_QUANTUM_MRADPS / 2.0)
        residual = self._angular_residual_mradps if active else 0.0
        request_sign = (1 if requested > 0 else -1) if active else 0
        # A negative residual after a positive16mrad/s pulse is the credit
        # for that pulse, not a reversal request. Only a real request reversal
        # clears old directional credit.
        if active and self._angular_residual_request_sign not in (0, request_sign):
            residual = 0.0
        proposed = requested + residual if active else requested
        angular = self._quantized_angular(proposed)
        if active and angular == self._quantized_angular(proposed):
            residual = max(-CENTER_ANGULAR_QUANTUM_MRADPS / 2.0,
                           min(CENTER_ANGULAR_QUANTUM_MRADPS / 2.0,
                               requested + residual - angular))
        return angular, residual, request_sign

    @staticmethod
    def _positive_action(wanted, angular, floor, direction=1):
        if direction != 1:
            raise ValueError("普通居中仅允许前进")
        angular = max(-CENTER_MAX_COMMAND_ANGULAR_MRADPS, min(CENTER_MAX_COMMAND_ANGULAR_MRADPS, int(angular)))
        half = abs(math.trunc(angular * WHEEL_BASE_MM / 2000))
        low, high = floor + half, MAX_WHEEL_SPEED_MMPS - half
        feasible = max(low, min(high, round(wanted)))
        reason = ("纠偏优先：逐轮61mm/s下限提高公共目标" if wanted < low - .5 else
                  "纠偏优先：逐轮310mm/s上限降低公共目标" if wanted > high + .5 else None)
        return feasible, angular, reason

    def _position_proposal(self, now, error, *, frozen):
        trim, key, source_at = self._position_integral_deg, self._position_integral_key, self._position_integral_source_at
        keys = tuple(self._paired_keys)
        confirmed = (self._applied_linear > 0
                     and _motion_wheels(self._applied_linear, self._applied_angular) == self._last_sent_wheels)
        new_pair = key is None or all(_source_key_newer(new, old) for new, old in zip(keys, key))
        points = list(self._lateral_sources)
        progress = None
        # Use the most recent complete progress interval. An older successful
        # shift must not hide a new quantized plateau for many later seconds.
        for count in range(3, len(points) + 1):
            start = points[-count]
            if points[-1][1] - start[1] >= 20.0 - 1e-9:
                progress = abs(start[2]) - abs(points[-1][2])
                break
        ready = (not frozen and confirmed and new_pair and progress is not None
                 and self._latest_source_times is not None
                 and self._forward_applied_barrier is not None
                 and all(s > self._forward_applied_barrier + 1e-9 for s in self._latest_source_times))
        if error is not None and trim * error < 0:
            trim = 0.0
        if ready:
            span = min(1.2, max(0.0, self._pair_at - source_at)) if source_at is not None else COMMAND_INTERVAL_S
            if abs(error) > SIDE_DEADBAND_MM / 2 and progress < 1.0 - 1e-9:
                requested_step = math.copysign(min(span, .1), error)
                # Bound only the learned trim; the geometric correction
                # remains free to request a heading outside this interval.
                trim = max(-POSITION_INTEGRAL_LIMIT_DEG,
                           min(POSITION_INTEGRAL_LIMIT_DEG, trim + requested_step))
            key, source_at = keys, self._pair_at
        state = dict(_position_integral_deg=trim, _position_integral_key=key,
                     _position_integral_source_at=source_at)
        feedback = dict(trim_deg=trim, source_keys=keys, source_at=source_at,
                        integral_frozen=not ready, confirmed_driving=confirmed,
                        inward_progress_mm=progress, basis="固定道路参考内的有界位置积分，绝不估计道路方向")
        return trim, state, feedback


    def _speed_proposal(self, goal, now, angular, *, cruise, frozen, departure=False):
        elapsed = min(COMMAND_INTERVAL_S, max(0.0, now - self._last_target_at))
        ramp = float(goal) if self._ramp_speed is None else self._ramp_speed
        if self._ramp_speed is not None:
            delta = goal - ramp
            ramp += math.copysign(min(abs(delta), CENTER_ACCEL_MMPS2 * elapsed), delta) if delta else 0.0
        complete_at = self._ramp_complete_at
        if self._ramp_requested != goal:
            complete_at = None
        if abs(ramp - goal) <= .5 and complete_at is None:
            complete_at = now
        estimate = self._estimate_speed(now)
        trim, updated, integral_at, key = self._speed_trim, self._speed_last_update_at, self._speed_last_integral_at, self._speed_last_state_key
        settled = (complete_at is not None and self._state_at is not None
                   and self._state_at - complete_at >= CENTER_SPEED_WINDOW_S - 1e-9)
        half = abs(math.trunc(angular * WHEEL_BASE_MM / 2000))
        low, high = CENTER_MIN_WHEEL_SPEED_MMPS + half, MAX_WHEEL_SPEED_MMPS - half
        confirmed = _motion_wheels(self._applied_linear, self._applied_angular) == self._last_sent_wheels and self._applied_linear > 0
        integral_ready = not frozen and confirmed and settled and estimate is not None
        if (integral_ready
                and self._state_uptime != key and now - updated >= CENTER_SPEED_UPDATE_S - 1e-9):
            tolerance = .05 * goal if cruise else 1.0
            residual = math.copysign(max(0.0, abs(goal - estimate) - tolerance), goal - estimate)
            dt = min(CENTER_SPEED_UPDATE_S, max(0.0, now - integral_at))
            wanted = ramp + trim
            if not (wanted >= high and residual > 0 or wanted <= low and residual < 0):
                trim = max(-MAX_WHEEL_SPEED_MMPS, min(MAX_WHEEL_SPEED_MMPS, trim + .2 * residual * dt))
            updated, integral_at, key = now, now, self._state_uptime
        elif not integral_ready:
            integral_at = now
        wanted = ramp + trim
        if not departure and self._ramp_speed is not None and all(wheel > 0 for wheel in self._last_sent_wheels):
            previous_common = sum(self._last_sent_wheels) / 2.0
            allowance = CENTER_ACCEL_MMPS2 * elapsed
            wanted = max(previous_common - allowance, min(previous_common + allowance, wanted))
        linear, angular, reason = self._positive_action(wanted, angular, CENTER_MIN_WHEEL_SPEED_MMPS)
        qualified = bool(settled and estimate is not None
                         and (abs(estimate - goal) <= .05 * goal + 1e-9 if cruise else estimate >= 61.0 - 1e-9))
        if cruise and not departure and abs(linear - goal) > .05 * goal + 1e-9:
            reason = reason or "实速反馈补偿或纠偏优先使公共命令暂超xx的5%；按500ms实测均速核验"
        if settled and estimate is not None and not qualified:
            reason = reason or ("500ms实测均速超出xx的5%，反馈补偿中" if cruise else
                                "500ms实测均速低于61，反馈补偿中")
        if ramp + trim >= high - .5 and estimate is not None and estimate < (goal * .95 if cruise else 61.0):
            reason = "逐轮速度已达可行上限，实速尚未达标；显示饱和，保留真实故障监督"
        state = dict(_ramp_speed=ramp, _ramp_complete_at=complete_at, _ramp_requested=goal,
                     _speed_trim=trim, _speed_last_update_at=updated, _speed_last_integral_at=integral_at,
                     _speed_last_state_key=key, _speed_qualified=qualified,
                     _speed_integral_frozen=not integral_ready)
        return linear, angular, reason, state, estimate

    def target(self, xx, now, *, cached_feedback=False, departure_only=False):
        self.refresh_yaw_rate(now)
        self._consume_pending_means(now)
        pose = self._poses[-1] if self._poses else None
        if (pose is None or self._road_reference is None
                or self._state_at is None or not 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9):
            return CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(), self._effective_yaw_rate(),
                                   self.axis_trusted, False)
        if not departure_only and not self.control_available(now):
            return CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(), self._effective_yaw_rate(),
                                   self.axis_trusted, False)
        controls_fresh = all(item is not None and 0 <= now - item[0] <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                             for item in self._control_cache)
        cached = not departure_only and (cached_feedback or not controls_fresh)
        if cached and not self._normal_motion_qualified:
            return CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(), self._effective_yaw_rate(),
                                   self.axis_trusted, False)
        psi = self._angle(pose.yaw - self._road_reference)
        rate = self._effective_yaw_rate(now)
        normal_allowed = bool(not departure_only and (cached and self._normal_motion_qualified
                              or not cached and self._usable_pair))
        cruise = True
        goal = xx
        state = {}
        hold_offset = self._angle((self._hold_heading if self._hold_heading is not None else self._road_reference)
                                  - self._road_reference)
        error = self.predicted_error_mm(now)
        position = math.copysign(max(0.0, abs(error or 0.0) - SIDE_DEADBAND_MM / 2.0), error or 0.0)
        desired = hold_offset
        capture = False
        position_feedback = None
        if departure_only:
            requested = 0.0
        elif cached:
            # Missing measurements may only retain or unload existing angular
            # authority. They cannot advance position/speed integrals or qualify.
            desired = hold_offset
            requested = (4.0 * math.radians(self._angle(self._hold_heading - pose.yaw))
                         - .2 * math.radians(rate or 0.0)) * 1000.0
            old = self._last_sent_angular
            requested = math.copysign(min(abs(old), abs(requested)), old) if old * requested > 0 else 0.0
        elif abs(error or 0.0) <= SIDE_DEADBAND_MM / 2.0 and abs(psi) <= NORMAL_HEADING_TOLERANCE_DEG + 1e-9:
            # Accept current direction. Zero v/w is not sent: positive v with
            # w=0 delegates holding THIS heading to the v11 firmware.
            desired = psi
            requested = 0.0
            capture = True
        else:
            # An earlier accepted tilted direction must not cancel a later
            # inward position request. Position selects its own road-frame goal.
            desired_without_integral = math.degrees(math.atan2(position, CENTER_LOOKAHEAD_MM))
            trim, position_state, position_feedback = self._position_proposal(
                now, error, frozen=not controls_fresh)
            state.update(position_state)
            desired = desired_without_integral + trim
            if position == 0 and abs(psi) > NORMAL_HEADING_TOLERANCE_DEG:
                # Position is already inside the corridor deadband. Return to
                # the fixed reference rather than keeping an excessive tilt.
                desired = 0.0
            requested = (CENTER_FORWARD_SHIFT_YAW_GAIN_PER_S * math.radians(desired - psi)
                         - CENTER_YAW_RATE_DAMPING * math.radians(rate or 0.0)) * 1000.0
        confirmed_driving = (self._applied_linear > 0
                             and _motion_wheels(self._applied_linear, self._applied_angular) == self._last_sent_wheels)
        angular, residual, residual_sign = self._fine_angular_proposal(requested, now,
            enabled=(not departure_only and not cached and controls_fresh and confirmed_driving
                     and not capture))
        state['_angular_residual_mradps'] = residual
        state['_angular_residual_request_sign'] = residual_sign
        if cached:
            old = self._last_sent_angular
            angular = int(math.copysign(min(abs(old), abs(angular)), old)) if old * angular > 0 else 0
        linear, angular, reason, speed_state, estimate = self._speed_proposal(
            goal, now, angular, cruise=cruise, frozen=cached, departure=departure_only)
        state.update(speed_state)
        mode = "DEPARTURE" if departure_only else "CACHE" if cached else "CRUISE_CORRECT"
        feedback = dict(raw_difference_mm=self._last_raw_difference,
                        axle_equivalent_difference_mm=self.signed_difference_mm, filtered_half_difference_mm=error,
                        fixed_reference_deg=self._road_reference, hold_heading_deg=self._hold_heading,
                        desired_heading_deg=desired, actual_heading_offset_deg=psi,
                        yaw_rate_degps=rate, selected_linear_mmps=linear, selected_angular_mradps=angular,
                        position_control=position_feedback, cached_feedback=cached,
                        heading_tolerance_deg=NORMAL_HEADING_TOLERANCE_DEG,
                        correction_heading_limit_deg=None, position_integral_limit_deg=POSITION_INTEGRAL_LIMIT_DEG,
                        basis="逐源激光前置补偿后的车轴横偏、固定参考及角速；全程前进直接差速")
        state["_control_feedback"] = feedback
        proposal = dict(epoch=self._motion_epoch, token=self.observation_token, at=now, state=state,
                        identity=self._state_session, departure=departure_only, cached=cached,
                        normal=normal_allowed, qualifies=normal_allowed,
                        capture_hold=capture and angular == 0 and not self._hold_locked,
                        adjustment=angular != 0)
        return CenteringTarget(int(linear), int(angular), mode, error, psi, rate, self.axis_trusted,
                               self.centered and not cached and not departure_only, proposal, reason, estimate,
                               speed_state["_speed_qualified"], normal_allowed)


def _signed_turn_rate_text(rate: float) -> str:
    """Display precision must not turn negative zero into a right-turn claim."""
    rounded = round(float(rate), 2)
    if rounded == 0:
        return "0.00°/s（显示精度内为零）"
    return f"{rounded:+.2f}°/s（{'左转' if rounded > 0 else '右转'}）"


class MotionDisplay:
    def __init__(self, link: mcu.McuLink, emitter=None) -> None:
        self.link = link
        self._terminal = AsyncTerminal() if emitter is None else None
        self.emitter = emitter if emitter is not None else self._terminal
        candidate = getattr(link, "speed", None)
        self.speed = candidate if isinstance(candidate, WheelSpeedEstimator) else WheelSpeedEstimator()
        self.state_samples = []
        self.diagnostics_samples = []
        self.input_speed_mmps = None
        self.wheel_targets = (0, 0)
        self.command_kind = None
        self.next_output = time.monotonic() + AVERAGE_INTERVAL_S
        self.action = "准备"
        self.origin: tuple[int, int] | None = None
        self.segment_distance_mm = 0.0
        self.after_turn = False
        self.directional_seq: int | None = None
        self.turn_result = "未转弯"
        self.command = (0, 0)
        self.raw_delta = 0.0
        self.wheel_delta = 0
        self.control_mode = "刹车"
        self.confirmation = "等待首次双侧有效激光与运动遥测"
        self.averages = _HalfSecondAverages(time.monotonic())
        self.controller = CenteringController(time.monotonic())
        self._controller_applied_records = set()
        self._controller_applied_source_at = None
        self._controller_applied_identity = None
        self.centering_target = None
        self.arc_phase = "普通前进矫正"
        self.arc_distance_mm = 0.0
        self.braking_tail_mm = 0.0
        self._samples: list[_SideSample] = []
        self._distance_observer = None
        self._ready_batches: list[_MeanBatch] = []
        self._cached_seq: tuple[int, int | None] | None = None
        self._snapshot_text: str | None = None
        self._previous_valid_sides = [None, None]
        self.side_changes = (None, None)
        self._response_identity = None
        self._response_source_key = None
        self._response_goal = None
        self._response_goal_at = None
        self._response_counts = dict(same_direction=0, opposite_direction=0,
                                     display_zero=0, transient=0, unconfirmed=0)
        self._response_latest = None
        self._response_unassociated = None

    def tick(self, *, pump: bool = True, check_health: bool = True,
             emit: bool = False) -> None:
        if pump:
            self.link.pump()
        # STATE arrival may release queued laser pairs. Check their age at
        # this host tick, before any replay can create fresh road evidence.
        self.controller.refresh_yaw_rate(time.monotonic())
        drain_states = getattr(self.link, "drain_state_samples", None)
        if callable(drain_states):
            self.state_samples = drain_states()
            for state, generated_at in self.state_samples:
                self.controller.observe_state(state, generated_at)
                if self._distance_observer is not None:
                    self._distance_observer(state)
            if self.speed is not getattr(self.link, "speed", None):
                for state, generated_at in self.state_samples:
                    self.speed.observe(state, generated_at)
        else:
            self.state_samples = []
            if self.link.state is not None:
                self.speed.observe(self.link.state)
                sample_time = getattr(self.link, "_sample_time", None)
                generated_at = (sample_time("state") if callable(sample_time)
                                else time.monotonic())
                self.controller.observe_state(self.link.state, generated_at)
                if self._distance_observer is not None:
                    self._distance_observer(self.link.state)
        _sync_controller_applied(self.link, self)
        drain_diagnostics = getattr(self.link, "drain_diagnostics_samples", None)
        self.diagnostics_samples = drain_diagnostics() if callable(drain_diagnostics) else []
        diagnostics = getattr(self.link, "diagnostics", None)
        if not callable(drain_diagnostics) and diagnostics is not None:
            sample_time = getattr(self.link, "_sample_time", None)
            generated_at = (sample_time("diagnostics") if callable(sample_time)
                            else getattr(self.link, "_last_diagnostics_recv", time.monotonic()))
            self.diagnostics_samples = [(diagnostics, generated_at)]
        for diagnostics, generated_at in self.diagnostics_samples:
            self.controller.observe_diagnostics(diagnostics, generated_at)
        self.controller.refresh_yaw_rate(time.monotonic())
        drain_traces = getattr(self.link, "drain_wheel_trace_samples", None)
        self.trace_samples = drain_traces() if callable(drain_traces) else []
        self._observe_turn_response(time.monotonic())
        check_lease = getattr(self.link, "_check_motion_lease", None)
        if check_health and callable(check_lease):
            check_lease()
        if check_health:
            fault = self.link.motion_health_reason() or _gyro_health_reason(self.link)
            if fault is not None:
                raise mcu.McuError(fault)
        odometry, yaw = self.link.odom_mm, self.link.yaw_deg
        if odometry is None or self.link.state_sample_token is None or yaw is None:
            raise mcu.McuError("编码器或陀螺仪遥测无效")
        if self.origin is not None:
            self.segment_distance_mm = ((odometry[0] - self.origin[0])
                                        + (odometry[1] - self.origin[1])) / 2.0
        drain = getattr(self.link, "drain_side_samples", None)
        if callable(drain):
            self._samples.extend(drain())
        elif self.link.directional is not None:
            sample = _side_sample(self.link)
            key = _side_snapshot_key(sample)
            if key != self._cached_seq:
                sample = _annotate_side_sample(self.link, sample)
                self.link._latest_side_sample = sample
                self._samples.append(sample)
                self._cached_seq = key
        if self.link.directional is not None:
            self.directional_seq = self.link.directional.directional_cycle_seq
        if emit:
            self.maybe_print()

    def consume_samples(self, *, require_healthy: bool = True,
                        collect_means: bool = True,
                        allow_stale: bool | None = None, pair_observer=None,
                        update_controller: bool | None = None) -> list[_SideSample]:
        samples, self._samples = self._samples, []
        now = time.monotonic()
        if update_controller is None:
            update_controller = collect_means
        if allow_stale is None:
            allow_stale = not getattr(self.link, "side_age_required", True)
        def publish_means(batches):
            # Control owns its evidence before console output can clear a batch.
            for batch in batches:
                if collect_means and update_controller:
                    self.controller.observe_mean_batch(batch, now=now)
                self._ready_batches.append(batch)
        if require_healthy:
            current = _side_sample(self.link)
            cached = getattr(self.link, "_latest_side_sample", None)
            if cached is not None and _side_snapshot_key(cached) == _side_snapshot_key(current):
                current = replace(current, candidate_sides=cached.candidate_sides)
            if _side_recovery_needed(self.link, current, now):
                self.link.side_recovery_active = True
                self.link.side_recovery_reason = ("离口单侧2秒没有健康新测量，停车等待恢复"
                                                 if getattr(self.link, "side_departure_active", False)
                                                 else "下层激光4/6或旧测量的VALID缓存超过2秒，停车等待恢复")
            for sample in samples:
                # 历史周期仍检查真实故障，但软缺测只由最新状态决定。
                # 否则恢复后的健康周期会被同批旧 STALE 再次停车。
                _side_recovery_needed(self.link, sample, now)
        for sample in samples:
            if getattr(self.link, "side_departure_active", False):
                _annotate_side_sample(self.link, sample)
            observation = (f"开口:{','.join(sorted(sample.openings))}"
                           if sample.openings else "双墙")
            if any(status in (WireRangeStatus.STALE, WireRangeStatus.TIMEOUT)
                   for status in (sample.left_status, sample.right_status)):
                latest = getattr(self.link, "_latest_side_sample", None)
                if latest is not None and _side_snapshot_key(sample) != _side_snapshot_key(latest):
                    observation = "缺测历史快照，门禁以最新反馈为准"
                elif getattr(self.link, "side_departure_active", False):
                    observation = "离口缺测，独立2秒门禁"
                else:
                    observation = ("4/6，短时VALID缓存过渡"
                                   if getattr(self.link, "side_cached_active", False)
                                   else "4/6，等待健康新测量")
            try:
                _side_recovery_needed(self.link, sample, now)
            except mcu.McuError:
                observation = "故障"
            changes = [None, None]
            for index, (distance, status) in enumerate((
                (sample.left_mm, sample.left_status), (sample.right_mm, sample.right_status),
            )):
                previous = self._previous_valid_sides[index]
                key = sample.source_keys[index] if sample.source_keys is not None else sample.seq
                if (observation != "故障" and status == WireRangeStatus.VALID
                        and sample.ages_ms[index] + max(0.0, (now - sample.received_at) * 1000.0)
                        <= LASER_MEASUREMENT_MAX_AGE_MS + 1e-6
                        and (previous is None or _source_key_newer(key, previous[0]))):
                    if previous is not None:
                        changes[index] = distance - previous[1]
                    self._previous_valid_sides[index] = (key, distance)
            self.side_changes = tuple(changes)
            if self.action != "停车等待xx":
                runlog.emit("motion_sequence_laser", "下层激光观测",
                            changes_mm=self.side_changes,
                            observation=observation,
                            candidate_sides=sorted(sample.candidates),
                            saved_openings=sorted(_pending_junction_openings(self.link)),
                            candidate_rule="同会话源键推进且真实源龄<=250ms的1/2开口",
                            **sample.diagnostics(now))
            publish_means(self.averages.advance(sample.received_at))
            if collect_means or update_controller:
                # UART快照曾经新鲜不代表消费时仍新鲜；保留原日志/源身份，仅证据用当前年龄。
                elapsed_ms = max(0.0, (now - sample.received_at) * 1000.0)
                evidence = replace(sample, received_at=now,
                                   ages_ms=tuple(age + elapsed_ms for age in sample.ages_ms))
                if collect_means:
                    self.averages.add(evidence)
            if update_controller:
                old_token = self.controller.observation_token
                self.controller.observe_side(evidence)
                if pair_observer is not None and self.controller.observation_token != old_token:
                    pair_observer(sample)
        publish_means(self.averages.advance(now))
        return samples

    def reset_averages(self, *, fresh_only: bool = True,
                       source_barrier: float | None = None,
                       reset_controller: bool = True) -> None:
        now = time.monotonic()
        barrier = (now if source_barrier is None else source_barrier) if fresh_only else None
        self.link._side_source_barrier = barrier
        self.averages.reset(now, source_barrier=barrier)
        if reset_controller:
            self.controller.reset(now, source_barrier=barrier)
            self._controller_applied_records.clear()
            self._controller_applied_source_at = None
        else:
            self.controller.source_barrier = barrier
        self.centering_target = None
        self._samples.clear()
        self._ready_batches.clear()
        drain = getattr(self.link, "drain_side_samples", None)
        if callable(drain):
            drain()
        if self.link.directional is not None:
            self._cached_seq = _side_snapshot_key(_side_sample(self.link))

    def set_stopped(self) -> None:
        self.link.side_cached_active = False
        self.controller.notify_stopped(time.monotonic(), stable=False)
        self.centering_target = None
        self.command = (0, 0)
        self.wheel_targets = (0, 0)
        self.command_kind = MessageType.MOTION_ACTION
        self.raw_delta = 0.0
        self.wheel_delta = 0
        self.control_mode = "刹车"

    def maybe_print(self, *, force: bool = False) -> None:
        self._ready_batches.clear()
        now = time.monotonic()
        if force or now + 1e-9 >= self.next_output:
            if now >= self.next_output:
                skipped = math.floor((now - self.next_output) / AVERAGE_INTERVAL_S)
                self.next_output += (skipped + 1) * AVERAGE_INTERVAL_S
            batch = self.averages.latest
            if batch is not None and now - batch.ended_at > LASER_MAX_AGE_MS / 1000.0:
                batch = None
            self._snapshot_text = self._format_snapshot(batch)
            self.print_snapshot()
            runlog.emit("motion_sequence_snapshot", "运动阶段快照", phase=self.action,
                        xx=self.input_speed_mmps, mean_speed_mmps=self.command[0],
                        target_wheels_mmps=self.wheel_targets,
                        encoder_estimated_wheels_mmps=self.speed.speeds(),
                        target_delta_mmps=self.wheel_delta, mean_batch=batch,
                        confirmation=self.confirmation, changes_mm=self.side_changes,
                        centering=self.centering_target,
                        command_angular_mradps=self.command[1], actual_yaw_deg=self.controller.yaw_deg,
                        raw_yaw_deg=self.link.yaw_deg,
                        gyro_calibration=self.controller.gyro_calibration_snapshot(now),
                        low_speed_control=self.low_speed_snapshot(now),
                        yaw_rate_degps=self.controller.yaw_rate_degps,
                        yaw_rate_source=self.controller.yaw_rate_source,
                        heading_reference_deg=self.controller.heading_reference_deg,
                        hold_heading_deg=self.controller.hold_heading_deg,
                        actual_speed_mmps=getattr(self.controller, "actual_speed_mmps", None),
                        speed_qualified=getattr(self.controller, "speed_qualified", False),
                        arc_phase=self.arc_phase, segment_distance_mm=self.arc_distance_mm,
                        braking_tail_mm=self.braking_tail_mm,
                        junction=self.junction_snapshot(now),
                        road_position_mm=self.road_position_mm(),
                        region=self.region_snapshot(),
                        gyro=self.gyro_snapshot(now),
                        turn_response=self.turn_response_snapshot(now),
                        control_basis="控制字段为上次计算；源时间新证据用于资格判定",
                        laser=(_side_sample(self.link).diagnostics(now)
                               if self.link.directional is not None else None))

    def gyro_snapshot(self, now: float | None = None) -> dict:
        """Read actual DIAG acquisition evidence, never relabel STATE as gyro."""
        now = time.monotonic() if now is None else now
        identity = getattr(self.link, "session_nonce", None), getattr(self.link, "boot_id", None)
        same_session = (None not in identity and getattr(self.controller, "_state_session", None) == identity)
        source_at = getattr(self.controller, "_gyro_source_at", None) if same_session else None
        rate = getattr(self.controller, "_gyro_yaw_rate", None) if same_session else None
        age_ms = None if source_at is None else (now - source_at) * 1000.0
        status = "waiting"
        if source_at is not None and rate is not None:
            status = ("valid" if 0 <= age_ms <= CENTER_GYRO_MAX_AGE_MS + 1e-6 else "expired")
        elif same_session:
            diagnostics = getattr(self.link, "diagnostics", None)
            diagnostic_identity = (getattr(diagnostics, "session_nonce", None),
                                   getattr(diagnostics, "boot_id", None))
            if diagnostics is not None and diagnostic_identity == identity:
                if getattr(diagnostics, "gyro_status", None) != WireGyroStatus.READY:
                    status = "unavailable"
                elif getattr(diagnostics, "gyro_sample_age_ms", 0) > CENTER_GYRO_MAX_AGE_MS:
                    status = "expired"
        fallback = None
        if same_session and status != "valid":
            effective_rate = getattr(self.controller, "_effective_yaw_rate", None)
            fallback = effective_rate(now) if callable(effective_rate) else None
        return dict(status=status, yaw_rate_degps=rate if status == "valid" else None,
                    source_at_s=source_at, source_age_ms=age_ms,
                    source_uptime_ms=(getattr(self.controller, "_gyro_source_uptime", None)
                                      if same_session else None),
                    session_nonce=identity[0], boot_id=identity[1],
                    state_estimated_rate_degps=fallback)

    def _format_gyro(self, now: float | None = None) -> str:
        gyro = self.gyro_snapshot(now)
        if gyro["status"] == "valid":
            text = f"陀螺仪角速：{_signed_turn_rate_text(gyro['yaw_rate_degps'])}"
        else:
            text = {"waiting": "陀螺仪角速：等待陀螺仪数据",
                    "expired": "陀螺仪角速：陀螺仪数据过期",
                    "unavailable": "陀螺仪角速：陀螺仪数据不可用"}[gyro["status"]]
        fallback = gyro["state_estimated_rate_degps"]
        if fallback is not None:
            text += f"\n控制角速：{_signed_turn_rate_text(fallback)}（STATE估算）"
        return text

    def _response_command(self, now: float) -> dict | None:
        """Bind diagnostics to the actual applied sequence and timely evidence."""
        state = getattr(self.link, "state", None)
        identity = getattr(self.link, "session_nonce", None), getattr(self.link, "boot_id", None)
        if (state is None or None in identity
                or (getattr(state, "session_nonce", None), getattr(state, "boot_id", None)) != identity
                or not getattr(state, "state_flags", 0) & StateFlag.APPLIED_CONTROL_PRESENT):
            return None
        sample_time = getattr(self.link, "_sample_time", None)
        state_source_at = (sample_time("state") if callable(sample_time)
                           else getattr(self.controller, "_state_at", None))
        if (state_source_at is None
                or not 0 <= now - state_source_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9):
            return None
        ledger = getattr(self.link, "_command_ledger", {})
        applied_seq = getattr(state, "applied_control_sequence", None)
        candidates = [(key, record) for key, record in ledger.items()
                      if key[1] == applied_seq and len(record) > 10 and tuple(record[6:8]) == identity]
        if not candidates:
            return None
        key, record = max(candidates, key=lambda item: item[1][0])
        target = record[5]
        if target[0] is not MessageType.CONTROL_TARGET:
            return None
        ack, applied = record[8], record[2]
        deadline = record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S
        timely = bool(ack is not None and getattr(ack, "status", None) is AckStatus.ACCEPTED
                      and applied is not None and record[3] is not None and record[9] is not None
                      and (getattr(ack, "session_nonce", None), getattr(ack, "boot_id", None)) == identity
                      and (getattr(applied, "session_nonce", None), getattr(applied, "boot_id", None)) == identity
                      and getattr(ack, "acknowledged_sequence", None) == key[1]
                      and getattr(ack, "acknowledged_type", None) == key[0]
                      and getattr(applied, "applied_control_sequence", None) == key[1]
                      and max(record[3], record[9]) <= deadline + 1e-9)
        return dict(sequence=key[1], message_type=key[0].name, goal_epoch=record[4],
                    target_linear_mmps=target[1], target_angular_mradps=target[2],
                    target_wheels_mmps=_motion_wheels(target[1], target[2]),
                    ack_status=None if ack is None else getattr(ack.status, "name", str(ack.status)),
                    ack_received_at_s=record[9], state_received_at_s=record[3],
                    latest_state_source_at_s=state_source_at,
                    applied_source_at_s=record[10].get("state_source_at"),
                    timely_accepted_and_applied=timely,
                    session_nonce=identity[0], boot_id=identity[1])

    def _response_association(self, command: dict, gyro: dict) -> dict | None:
        """A fresh rate alone cannot prove which pending goal drove its sample."""
        source_at, applied_at = gyro["source_at_s"], command["applied_source_at_s"]
        if applied_at is None or source_at < applied_at - 1e-9:
            return dict(reason="gyro_before_application" if applied_at is not None
                        else "missing_application_source", gyro=gyro,
                        state_sequence=command["sequence"], applied_source_at_s=applied_at)
        if source_at <= command["latest_state_source_at_s"] + 1e-9:
            return None
        identity = command["session_nonce"], command["boot_id"]
        for key, record in getattr(self.link, "_command_ledger", {}).items():
            if (len(record) <= 10 or tuple(record[6:8]) != identity
                    or record[4] == command["goal_epoch"]
                    or not applied_at < record[0] <= source_at + 1e-9):
                continue
            ack = record[8]
            if ack is not None and getattr(ack, "status", None) is not AckStatus.ACCEPTED:
                continue
            deadline = record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S
            proven = bool(record[2] is not None and ack is not None
                          and record[3] is not None and record[9] is not None
                          and max(record[3], record[9]) <= deadline + 1e-9)
            if not proven:
                return dict(reason="different_goal_pending", gyro=gyro,
                            state_sequence=command["sequence"], pending_sequence=key[1],
                            pending_message_type=key[0].name,
                            pending_target=record[5], pending_sent_at_s=record[0],
                            latest_state_source_at_s=command["latest_state_source_at_s"])
        return None

    def _observe_turn_response(self, now: float) -> None:
        """Diagnostic counters consume each true gyro source once; no safety action."""
        identity = getattr(self.link, "session_nonce", None), getattr(self.link, "boot_id", None)
        if identity != self._response_identity:
            self._response_identity, self._response_source_key = identity, None
            self._response_goal = self._response_goal_at = self._response_latest = None
            self._response_unassociated = None
            self._response_counts = {key: 0 for key in self._response_counts}
        gyro = self.gyro_snapshot(now)
        if gyro["status"] != "valid":
            return
        source_key = gyro["source_uptime_ms"], gyro["source_at_s"]
        if source_key == self._response_source_key:
            return
        self._response_source_key = source_key
        command = self._response_command(now)
        if command is None:
            return
        self._response_unassociated = self._response_association(command, gyro)
        if self._response_unassociated is not None:
            return
        angular = command["target_angular_mradps"]
        if angular == 0:
            return
        goal = command["goal_epoch"], command["target_linear_mmps"], angular
        if goal != self._response_goal:
            self._response_goal = goal
            self._response_goal_at = command["applied_source_at_s"]
        rate = gyro["yaw_rate_degps"]
        elapsed = (None if self._response_goal_at is None
                   else gyro["source_at_s"] - self._response_goal_at)
        if not command["timely_accepted_and_applied"]:
            result = "unconfirmed"
        elif elapsed is None or elapsed < config.MCU_COMMAND_CONFIRM_TIMEOUT_S - 1e-9:
            result = "transient"
        elif round(rate, 2) == 0:
            result = "display_zero"
        else:
            result = "same_direction" if angular * rate > 0 else "opposite_direction"
        self._response_counts[result] += 1
        actual_wheels = self.speed.speeds(now)
        target_wheels = command["target_wheels_mmps"]
        self._response_latest = dict(command=command, gyro=gyro, response=result,
                                     requested_direction="left" if angular > 0 else "right",
                                     observed_direction=("display_zero" if round(rate, 2) == 0
                                                         else "left" if rate > 0 else "right"),
                                     target_delta_mmps=target_wheels[1] - target_wheels[0],
                                     actual_wheels_mmps=actual_wheels,
                                     actual_delta_mmps=(None if actual_wheels is None
                                                       else actual_wheels[1] - actual_wheels[0]),
                                     actual_yaw_deg=self.controller.yaw_deg,
                                     response_since_application_s=elapsed)

    def turn_response_snapshot(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        self._observe_turn_response(now)
        current = False
        source_age_ms = None
        if self._response_latest is not None:
            latest = self._response_latest
            source_age_ms = (now - latest["gyro"]["source_at_s"]) * 1000.0
            command, gyro = self._response_command(now), self.gyro_snapshot(now)
            current = bool(command is not None and gyro["status"] == "valid"
                           and self._response_association(command, gyro) is None
                           and command["goal_epoch"] == latest["command"]["goal_epoch"]
                           and (command["target_linear_mmps"], command["target_angular_mradps"])
                           == (latest["command"]["target_linear_mmps"],
                               latest["command"]["target_angular_mradps"])
                           and gyro["source_at_s"] == latest["gyro"]["source_at_s"])
        return dict(latest=self._response_latest, counts=dict(self._response_counts),
                    unassociated=self._response_unassociated,
                    latest_evidence_current=current, latest_source_age_ms=source_age_ms,
                    rule="每个真实陀螺仪源样本仅统计一次；应用后200ms内为响应过渡；仅诊断")

    def _format_turn_response(self, now: float | None = None) -> str:
        response = self.turn_response_snapshot(now)
        unassociated = response["unassociated"]
        if unassociated is not None:
            reason = {"gyro_before_application": "陀螺仪源样本早于当前STATE确认的命令应用",
                      "missing_application_source": "缺少命令应用源时刻",
                      "different_goal_pending": "陀螺仪测量晚于最新STATE，期间不同目标待确认"}[unassociated["reason"]]
            source_age = max(0.0, ((time.monotonic() if now is None else now)
                                 - unassociated["gyro"]["source_at_s"]) * 1000)
            return (f"转向响应诊断：无法关联；{reason}；源龄={source_age:.1f}ms；"
                    f"统计={response['counts']}（仅诊断，不计入同向／反向统计）")
        latest = response["latest"]
        if latest is None:
            return "转向响应诊断：等待当前会话已应用差速命令和真实陀螺仪"
        labels = {"unconfirmed": "命令接受／应用证据待确认", "transient": "应用后的响应过渡",
                  "display_zero": "实际角速在显示精度内为零", "same_direction": "实际转向与请求同向",
                  "opposite_direction": "实际暂与请求反向，待实测核对"}
        command = latest["command"]
        evidence = ("当前证据" if response["latest_evidence_current"]
                    else f"上次记录，当前证据不足／已过期（源龄{response['latest_source_age_ms']:.1f}ms）")
        return (f"转向响应诊断：{evidence}；seq={command['sequence']}；请求"
                f"{'左转' if latest['requested_direction'] == 'left' else '右转'}；"
                f"{labels[latest['response']]}；统计={response['counts']}（仅诊断）")

    def region_snapshot(self) -> dict:
        return self.controller.stage_snapshot()


    def low_speed_snapshot(self, now: float | None = None) -> dict:
        return self.controller.speed_snapshot(time.monotonic() if now is None else now)

    def _format_calibrated_control(self) -> str:
        calibration = self.controller.gyro_calibration_snapshot(time.monotonic())
        if not calibration['calibrated']:
            return "起始1秒均值航向：等待合格静止采样"
        return (f"起始1秒平均航向={calibration['initial_reference_deg']:.3f}°；"
                f"平均源时刻={calibration['reference_at_s']:.6f}s；"
                f"冻结残余零偏={calibration['bias_degps']:+.4f}°/s；"
                f"原始航向={calibration['raw_yaw_deg']}°；控制航向={calibration['control_yaw_deg']}°；"
                f"去偏角速={calibration['debiased_raw_rate_degps']}°/s；"
                f"中值低通角速={calibration['filtered_rate_degps']}°/s")

    def junction_snapshot(self, now: float | None = None) -> dict | None:
        openings = _pending_junction_openings(self.link)
        if not openings:
            return None
        now = time.monotonic() if now is None else now
        remaining = _junction_wait_remaining(self.link, now)
        tracker = getattr(self.link, "_stop_tracker", None)
        origin = getattr(self.link, "_last_brake_origin", None)
        odometry = self.link.odom_mm
        net_tail = (tuple(current - initial for current, initial in zip(odometry, origin))
                    if odometry is not None and origin is not None else None)
        return dict(openings=sorted(openings), brake_sent_at=getattr(self.link, "_last_brake_sent_at", None),
                    wait_remaining_s=None if math.isinf(remaining) else remaining,
                    brake_sequence=getattr(self.link, "_last_brake_sequence", None),
                    wheel_tail_mm=None if tracker is None else (tracker.left_sum, tracker.right_sum),
                    brake_net_tail_mm=net_tail,
                    quiet_observed_ms=None if tracker is None else tracker.observed_ms,
                    quiet_reset_reason=None if tracker is None else tracker.reset_reason,
                    stable_barrier=getattr(self.link, "_stable_barrier", None))

    def road_position_mm(self) -> float | None:
        progress = getattr(self, "road_progress", None)
        return getattr(progress, "position_mm", None)


    def _format_snapshot(self, batch: _MeanBatch | None) -> str:
        left, right = (batch.left_mm, batch.right_mm) if batch else (None, None)
        counts = (batch.left_count, batch.right_count) if batch else (0, 0)
        difference = None if left is None or right is None else abs(left - right)
        mean = lambda value: "缺失" if value is None else f"{value:.3f} mm"
        stage = self.region_snapshot()
        speed = self.low_speed_snapshot()
        target = self.centering_target
        laser = (_side_sample(self.link).diagnostics(time.monotonic())
                 if self.link.directional is not None else None)
        laser_text = (f"激光左下CH2/右下CH1状态={laser['raw_statuses']}；距离={laser['distances_mm']} mm；"
                      f"真实源龄={laser['ages_ms']} ms；周期={laser['cycle_seq']}"
                      if laser else "激光：等待当前会话新鲜测量")
        measured = speed.get('measured_mean_mmps')
        speed_text = (f"500ms编码器平均实速={measured:.2f} mm/s" if measured is not None
                      else "500ms平均实速：缺少完整连续驱动窗口")
        speed_text += (f"；目标均速={speed.get('target_mean_mmps')} mm/s；"
                       f"状态={speed.get('status')}；达速={speed.get('qualified', False)}；"
                       f"速度补偿={speed.get('trim_mmps', 0.0):+.2f} mm/s；"
                       f"实测轮速={speed.get('measured_wheels_mmps')} mm/s")
        reason = speed.get('limited_reason') or getattr(self.controller, 'speed_limit_reason', None)
        if reason:
            speed_text += f"；限制／速度例外：{reason}"
        junction = self.junction_snapshot()
        junction_text = (f"路口保存开口={junction['openings']}；BRAKE序号={junction['brake_sequence']}；"
                         f"转前3秒剩余={junction['wait_remaining_s']}s；"
                         f"严格停稳窗口={junction['quiet_observed_ms']}ms；"
                         f"制动净尾程={junction['brake_net_tail_mm']}mm"
                         if junction else "路口：等待新鲜合法开口")
        return "\n".join((
            "=" * 64,
            f"阶段：{self.action}；控制：{self.control_mode}；{self.confirmation}",
            f"独立0.5s原始均值：左下CH2={mean(left)}（{counts[0]}个）；"
            f"右下CH1={mean(right)}（{counts[1]}个）；绝对差={mean(difference)}",
            laser_text,
            f"固定道路参考={self.controller.heading_reference_deg}°；"
            f"已接受锁头方向={getattr(self.controller, 'hold_heading_deg', None)}°；"
            f"当前控制航向={self.controller.yaw_deg}°；"
            f"当前偏角={None if target is None else target.heading_error_deg}°；"
            f"居中／锁头容差=±{NORMAL_HEADING_TOLERANCE_DEG:g}°；"
            f"居中均值窗={stage.get('mean_count', 0)}/2；"
            f"正常速度授权={self.controller.normal_motion_qualified}；实测居中={self.controller.centered}",
            f"前进纠偏反馈：{stage}；纠偏目标不设固定偏角上限；"
            f"位置积分上限=±{POSITION_INTEGRAL_LIMIT_DEG:g}°",
            self._format_gyro(),
            self._format_calibrated_control(),
            self._format_turn_response(),
            speed_text,
            junction_text,
            f"道路净轮程={self.road_position_mm()} mm；制动尾段={self.braking_tail_mm:.2f} mm",
            f"输入xx={self.input_speed_mmps} mm/s；下发平均目标={self.command[0]} mm/s；"
            f"轮目标左={self.wheel_targets[0]:+d}、右={self.wheel_targets[1]:+d} mm/s；"
            f"轮速差={self.wheel_delta:+d} mm/s；角速目标={self.command[1]:+d} mrad/s；{self.turn_result}",
            "=" * 64,
        ))

    def print_snapshot(self) -> None:
        if self._snapshot_text is not None:
            self.emitter(self._snapshot_text)

    def close(self) -> None:
        if self._terminal is not None:
            self._terminal.close()


class _StopTracker:
    """BRAKE证据后的新STATE：逐轮连续500ms零里程，轮迹运动可否决。"""

    def __init__(self, link: mcu.McuLink, *, source_barrier: float | None = None) -> None:
        self.previous, self.token = link.odom_mm, link.state_sample_token
        if self.previous is None or self.token is None:
            raise mcu.McuError("停车确认缺少编码器")
        self.token = link.state.uptime_ms
        self.count = 0
        self.left_sum = self.right_sum = 0
        self.moved = False
        self.observed_ms = 0
        self.source_barrier = (time.monotonic() if source_barrier is None else source_barrier)
        self.quiet_since = None
        self.last_source = None
        self.last_state_source = None
        self.last_trace_source = None
        self.last_trace_token = None
        self.identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
        self.reset_reason = "等待BRAKE应用后的新鲜STATE建立静止基线"

    def update(self, link: mcu.McuLink, samples=None, trace_samples=None) -> bool:
        if link.odom_mm is None or link.state_sample_token is None:
            raise mcu.McuError("停车确认期间编码器失效")
        if samples is None:
            sample_time = getattr(link, "_sample_time", None)
            generated_at = sample_time("state") if callable(sample_time) else time.monotonic()
            samples = [(link.state, generated_at)]
        if (getattr(link, "session_nonce", None), getattr(link, "boot_id", None)) != self.identity:
            raise mcu.McuError("停车确认期间 MCU 会话变化")
        if trace_samples is None:
            drain = getattr(link, "drain_wheel_trace_samples", None)
            trace_samples = drain() if callable(drain) else []
        events = [(at, 1, state) for state, at in samples]
        events.extend((at, 0, trace) for trace, at in trace_samples)
        for generated_at, kind, state in sorted(events, key=lambda event: event[:2]):
            if generated_at <= self.source_barrier + 1e-9:
                continue
            if kind == 0:
                if generated_at > time.monotonic() + 1e-9:
                    continue
                token = state.sample_sequence
                if (self.last_trace_token is not None and classify_sequence(token, self.last_trace_token)
                        is not SequenceDisposition.NEWER):
                    continue
                if self.last_trace_source is not None and generated_at <= self.last_trace_source:
                    continue
                self.last_trace_token, self.last_trace_source = token, generated_at
                if state.left_encoder_delta or state.right_encoder_delta:
                    self.reset_reason = "停后轮迹仍有编码器增量"
                    self.quiet_since = None
                    self.count = self.observed_ms = 0
                    self.moved = True
                    self.last_source = generated_at
                    runlog.emit("motion_sequence_stop_reset", "轮迹仍有运动，重建停稳窗口",
                                reason=self.reset_reason,
                                source_time=generated_at,
                                left_encoder_delta=state.left_encoder_delta,
                                right_encoder_delta=state.right_encoder_delta)
                continue
            if not 0 <= time.monotonic() - generated_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9:
                continue
            token = state.uptime_ms
            elapsed = (token - self.token) & 0xFFFFFFFF
            if not 0 < elapsed < 0x80000000:
                continue
            if self.last_source is not None and generated_at < self.last_source:
                continue
            current = state.left_mm, state.right_mm
            delta = tuple(abs(a - b) for a, b in zip(current, self.previous))
            self.left_sum += delta[0]
            self.right_sum += delta[1]
            gap = (self.last_state_source is not None and
                   generated_at - self.last_state_source > config.TELEMETRY_STALE_MS / 1000.0 + 1e-9)
            if any(delta):
                self.moved = True
                self.reset_reason = "单轮整数轮程仍在变化（计入制动尾程）"
                runlog.emit("motion_sequence_stop_reset", "单轮里程变化，重建停稳窗口",
                            reason=self.reset_reason, source_time=generated_at, wheel_delta_mm=delta,
                            wheel_tail_mm=(self.left_sum, self.right_sum))
            elif gap:
                self.reset_reason = "STATE断档，旧静止窗口不能续接"
                runlog.emit("motion_sequence_stop_reset", "STATE断档，重建停稳窗口",
                            reason=self.reset_reason, source_time=generated_at,
                            previous_state_source=self.last_state_source)
            elif self.quiet_since is None:
                self.reset_reason = "停后新鲜STATE建立静止基线"
            if any(delta) or gap or self.quiet_since is None:
                self.quiet_since, self.count = generated_at, 1
            else:
                self.count += 1
            self.observed_ms = round((generated_at - self.quiet_since) * 1000.0)
            self.previous, self.token = current, token
            self.last_source = self.last_state_source = generated_at
        now = time.monotonic()
        if self.last_state_source is not None and now - self.last_state_source > config.TELEMETRY_STALE_MS / 1000.0 + 1e-9:
            self.reset_reason = "最新STATE已超龄，等待新的静止证据"
            self.quiet_since = None
        return (self.quiet_since is not None and self.last_state_source is not None
                and self.count >= 2
                and self.last_state_source - self.quiet_since + 1e-9 >= STOP_OBSERVATION_S)


def _wait_stable(link: mcu.McuLink, display: MotionDisplay, deadline: float = math.inf,
                 *, input_source=None) -> float:
    """Wait for mechanical stop without a timeout; real health checks stay active."""
    tracker = getattr(link, "_stop_tracker", None)
    if tracker is None:
        barrier = getattr(link, "_brake_evidence_barrier", None)
        tracker = _StopTracker(link, source_barrier=time.monotonic() if barrier is None else barrier)
    link._stop_tracker = tracker
    while True:
        _check_input_exit(input_source)
        display.tick()
        _remaining(deadline)
        display.consume_samples(collect_means=False)
        if tracker.update(link, display.state_samples if display.state_samples else None,
                          getattr(display, "trace_samples", [])):
            link._stable_barrier = time.monotonic()
            link._ready_side_pairs = []
            display.controller.notify_stopped(link._stable_barrier)
            runlog.emit("motion_sequence_stable", "双轮连续500ms整数里程不变，已停稳",
                        brake_evidence_barrier=tracker.source_barrier,
                        quiet_since=tracker.quiet_since, state_source=tracker.last_state_source,
                        stable_barrier=link._stable_barrier, wheel_positions_mm=link.odom_mm,
                        observed_ms=tracker.observed_ms, state_count=tracker.count)
            return link._stable_barrier
        link._stable_barrier = None
        remaining = _junction_wait_remaining(link)
        display.confirmation = (f"等待双轮连续500ms静止；左右制动尾程="
                                f"{tracker.left_sum}/{tracker.right_sum} mm；"
                                f"静止窗口={tracker.observed_ms} ms；{tracker.reset_reason}"
                                + (f"；路口3秒剩余={remaining:.2f}s" if math.isfinite(remaining)
                                   and _pending_junction_openings(link) else ""))
        display.maybe_print()
        time.sleep(CONTROL_INTERVAL_S)


def _ensure_stable(link, display, deadline: float = math.inf, *, input_source=None) -> float:
    """Recheck stop evidence; renewed movement waits again without sending BRAKE."""
    if _assert_still_stable(link, display):
        return link._stable_barrier
    return _wait_stable(link, display, deadline, input_source=input_source)


def _apply_brake(link: mcu.McuLink, display: MotionDisplay, deadline: float) -> None:
    _remaining(deadline)
    sequence = link.send_motion_action(MotionActionCode.BRAKE, 0,
                                      require_ranges=False, require_upper=False)
    # Desktop injected links need the same successful-send timestamp as the UART link.
    if getattr(link, "_last_brake_sequence", None) != sequence:
        link._last_brake_sent_at = time.monotonic()
        link._last_brake_identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
        link._last_brake_sequence = sequence
        link._last_brake_origin = link.odom_mm
    display.set_stopped()
    runlog.emit("motion_sequence_brake_sent", "已发送BRAKE，等待接受与应用",
                sequence=sequence, sent_at=link._last_brake_sent_at,
                wheel_positions_mm=link._last_brake_origin,
                saved_openings=sorted(_pending_junction_openings(link)),
                stop_reason=display.action)
    link.wait_for_motion_action_applied(
        sequence, timeout_s=min(config.MCU_COMMAND_CONFIRM_TIMEOUT_S, _remaining(deadline)))
    key = MessageType.MOTION_ACTION, sequence
    link._brake_evidence_barrier = getattr(link, "_confirmed_evidence_times", {}).get(key, time.monotonic())
    link._stable_barrier = None
    link._stop_tracker = None
    runlog.emit("motion_sequence_brake_applied", "BRAKE已在原200ms窗口内接受并应用",
                sequence=sequence, evidence_received_at=link._brake_evidence_barrier)
    _remaining(deadline)


def _brake(link: mcu.McuLink, display: MotionDisplay, deadline: float = math.inf,
           *, input_source=None) -> None:
    display.action = "刹车与停稳确认"
    _apply_brake(link, display, deadline)
    _wait_stable(link, display, deadline, input_source=input_source)


def _wait_junction_turn_ready(link, display, deadline: float = math.inf, *, input_source=None) -> None:
    """Wait at least three seconds from successful BRAKE, then recheck strict stop."""
    display.action = "路口停车：制动停稳与3秒等待"
    while True:
        _check_input_exit(input_source)
        _remaining(deadline)
        display.tick()
        for sample in display.consume_samples(collect_means=False):
            if _pending_junction_openings(link):
                _latch_junction(link, sample)
        remaining = _junction_wait_remaining(link)
        if math.isinf(remaining):
            raise mcu.McuError("路口转弯缺少本会话成功BRAKE发送时刻")
        stable = _assert_still_stable(link, display)
        tracker = getattr(link, "_stop_tracker", None)
        display.confirmation = (f"BRAKE已接受并应用；3秒剩余={remaining:.2f}s；"
                                f"双轮连续500ms静止={'已确认' if stable else '等待'}；"
                                f"左右制动尾程={tracker.left_sum if tracker else 0}/"
                                f"{tracker.right_sum if tracker else 0} mm；"
                                f"{tracker.reset_reason if tracker else '等待静止基线'}")
        display.maybe_print()
        if remaining <= 1e-9 and stable:
            runlog.emit("motion_sequence_junction_turn_ready", "BRAKE发送满3秒且双轮连续500ms静止，允许转弯",
                        brake_sent_at=link._last_brake_sent_at,
                        elapsed_s=time.monotonic() - link._last_brake_sent_at,
                        stable_barrier=link._stable_barrier,
                        wheel_tail_mm=(tracker.left_sum, tracker.right_sum),
                        openings=sorted(_pending_junction_openings(link)))
            return
        if not stable:
            _wait_stable(link, display, deadline, input_source=input_source)
        else:
            time.sleep(CONTROL_INTERVAL_S)


def _prepare(link: mcu.McuLink, display: MotionDisplay, deadline: float,
             *, input_source=None) -> None:
    # Selfcheck has a deadline; soft laser availability has no deadline.
    telemetry_ready = False
    while True:
        _check_input_exit(input_source)
        link.pump()
        reason = link.motion_health_reason() or _gyro_health_reason(link)
        if reason is not None:
            if telemetry_ready or time.monotonic() >= deadline:
                raise mcu.McuError(f"运动遥测自检失败：{reason}")
        else:
            telemetry_ready = True
            display.tick(pump=False)
            if link.directional is not None:
                display.consume_samples(collect_means=False)
                sample = _side_sample(link)
                _check_sides(sample, time.monotonic(), check_age=False,
                             allow_stale=True, allow_timeout=True)
                if (sample.dual_wall and all(0 <= time.monotonic() - source
                        <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                        for source in sample.source_times)):
                    display.reset_averages(fresh_only=False)
                    display.confirmation = "首次双墙有效；保持停车，等待填写直行速度"
                    return
            display.confirmation = "遥测自检完成，保持停车等待首次健康双墙"
            display.maybe_print()
        time.sleep(CONTROL_INTERVAL_S)


def _set_ready_feedback(link, pairs, barrier):
    link._ready_side_pairs = list(pairs)
    link._ready_side_identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    link._ready_side_barrier = barrier


def _ready_feedback_valid(link, now):
    pairs = getattr(link, "_ready_side_pairs", [])
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    barrier = getattr(link, "_stable_barrier", None)
    if (not pairs or getattr(link, "_ready_side_identity", None) != identity
            or barrier is None or barrier != getattr(link, "_ready_side_barrier", None)):
        return False
    sample = pairs[-1]
    return all(source > barrier + 1e-9 and 0 <= now - source
               <= LASER_CACHE_MAX_AGE_MS / 1000.0 + 1e-9 for source in sample.source_times)


def _sample_start(link: mcu.McuLink, display: MotionDisplay, *, input_source=None) -> None:
    """Retry stopped1s windows; use fresh yaw mean and cumulative bias only."""
    display.action = "停车采样1秒"
    barrier = _ensure_stable(link, display, input_source=input_source)
    started_at = time.monotonic()
    display.reset_averages(source_barrier=barrier)
    previous, pairs = None, []
    calibration = _StartupGyroCalibration(barrier)
    attempt = 1
    while True:
        _check_input_exit(input_source)
        display.tick()
        samples = display.consume_samples()
        if not _assert_still_stable(link, display):
            barrier = _wait_stable(link, display, input_source=input_source)
            started_at = time.monotonic()
            previous, pairs = None, []
            calibration = _StartupGyroCalibration(barrier)
            display.reset_averages(source_barrier=barrier)
            continue
        now = time.monotonic()
        for state, generated_at in getattr(display, "state_samples", []):
            calibration.add_state(state, generated_at, now)
        diagnostics_batch = getattr(display, "diagnostics_samples", None)
        if diagnostics_batch is not None:
            for diagnostics, generated_at in diagnostics_batch:
                calibration.add_diagnostics(diagnostics, generated_at, now)
        else:
            diagnostics = getattr(link, "diagnostics", None)
            if diagnostics is not None:
                sample_time = getattr(link, "_sample_time", None)
                generated_at = (sample_time("diagnostics") if callable(sample_time)
                                else getattr(link, "_last_diagnostics_recv", now))
                calibration.add_diagnostics(diagnostics, generated_at, now)
        if calibration.requires_restop:
            # Even when the current tracker is quiet, an accepted intervening
            # wheel change/STATE gap must not join two different stopped segments.
            barrier = _wait_stable(link, display, input_source=input_source)
            started_at = time.monotonic()
            calibration = _StartupGyroCalibration(barrier)
            previous, pairs = None, []
            display.reset_averages(source_barrier=barrier)
            continue
        for sample in samples:
            if sample.dual_wall and _fresh_post_stop_pair(sample, now, barrier, previous):
                previous = sample
                pairs.append(sample)
                pairs = pairs[-POST_STOP_PAIRS:]
        display.confirmation = (f"停车采样第{attempt}个1秒窗；停后新鲜双墙 {len(pairs)}/{POST_STOP_PAIRS}；"
                                f"累计STATE={len(calibration.states)}、陀螺仪新源={len(calibration.rates)}")
        display.maybe_print()
        if now - started_at + 1e-9 >= START_SAMPLE_S:
            # A delayed pump must not certify an older quiet window while
            # excluding the latest visible body rotation. Accumulation is
            # continuous, but release proof always covers the latest second.
            window_start = now - START_SAMPLE_S
            qualified, details = calibration.evaluate(window_start, now)
            walls_ready = (len(pairs) >= POST_STOP_PAIRS
                           and all(0 <= now - source <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                                   for source in pairs[-1].source_times))
            if qualified is not None and not walls_ready:
                details["reason"] = "最近1秒陀螺仪稳定，仍等待停后两组真正新鲜双墙"
            runlog.emit("motion_sequence_startup_calibration", "停车1秒窗核验",
                        attempt=attempt, window_start_s=window_start, window_end_s=now,
                        stable=qualified is not None, walls_ready=walls_ready, **details)
            if qualified is not None and walls_ready:
                if not display.controller.install_gyro_calibration(qualified):
                    raise mcu.McuError("初始静止标定身份已失效，禁止启动")
                initial = display.controller._angle(qualified.reference_deg)
                display.controller.set_initial_reference(initial)
                runlog.emit("motion_sequence_initial_reference", "最近1秒真实STATE平均作为初始固定航向",
                            yaw_deg=initial, state_samples=qualified.state_count,
                            gyro_samples=qualified.gyro_count, residual_bias_degps=qualified.bias_degps,
                            window_mean_reference_deg=qualified.reference_deg,
                            reference_at_s=qualified.reference_at, window_start_s=window_start,
                            window_end_s=now, window_state_samples=details["state_count"])
                _set_ready_feedback(link, pairs, barrier)
                return
            display.confirmation = details["reason"]
            display.maybe_print(force=True)
            started_at = now
            attempt += 1
        time.sleep(CONTROL_INTERVAL_S)


def _fresh_post_stop_pair(sample: _SideSample, now: float, barrier: float,
                          previous: _SideSample | None) -> bool:
    if (sample.left_status not in HEALTHY_RANGE_STATUSES
            or sample.right_status not in HEALTHY_RANGE_STATUSES
            or any(source <= barrier + 1e-9 for source in sample.source_times)
            or any(age + max(0.0, (now - sample.received_at) * 1000.0) > LASER_MEASUREMENT_MAX_AGE_MS + 1e-6
                   for age in sample.ages_ms)):
        return False
    if previous is not None:
        if sample.source_keys is not None and previous.source_keys is not None:
            if not all(0 < ((a - b) & 0xFFFFFFFF) < 0x80000000
                       for a, b in zip(sample.source_keys, previous.source_keys)):
                return False
        elif not all(a > b + 1e-9 for a, b in zip(sample.source_times, previous.source_times)):
            return False
    return True


def _clear_side_recovery(link) -> None:
    link.side_recovery_active = False
    link.side_recovery_reason = None
    link._side_brake_pending = False
    link._timeout_candidate_sides = frozenset()


def _assert_still_stable(link, display) -> bool:
    tracker = getattr(link, "_stop_tracker", None)
    stable = tracker is not None and tracker.update(
        link, display.state_samples if display.state_samples else None,
        getattr(display, "trace_samples", []))
    if not stable:
        link._stable_barrier = None
        link._ready_side_pairs = []
        display.controller.notify_stopped(time.monotonic(), stable=False)
    return bool(stable and getattr(link, "_stable_barrier", None) is not None)


def _confirm_junction(link: mcu.McuLink, display: MotionDisplay,
                      first: _SideSample, deadline: float = math.inf,
                      *, input_source=None) -> frozenset[str]:
    _latch_junction(link, first, initial=True)
    tracker = getattr(link, "_stop_tracker", None)
    reusable = (tracker is not None and getattr(link, "_stable_barrier", None) is not None
                and not display.command[0] and not display.command[1])
    if reusable:
        display.tick()
        _ensure_stable(link, display, deadline, input_source=input_source)
    else:
        _brake(link, display, deadline, input_source=input_source)
    _remaining(deadline)
    # 注入的桌面链路由display收集源事件；实际链路在接收时已经合并。
    for sample in display.consume_samples(collect_means=False):
        _latch_junction(link, sample)
    _ensure_stable(link, display, deadline, input_source=input_source)
    openings = _pending_junction_openings(link)
    if not openings:
        raise mcu.McuError("路口停车期间会话变化，已保存开口失效")
    _clear_side_recovery(link)
    display.action = "路口已停稳：首路口等满3秒再转弯，第二路口直接结束"
    display.confirmation = (f"已保存开口 {','.join(sorted(openings))}；BRAKE已确认且双轮连续500ms停稳；"
                            f"仅首路口转弯前须等满3秒，剩余={_junction_wait_remaining(link):.2f}s")
    runlog.emit("motion_sequence_junction_confirmed", "合法开口已保存，严格停稳；首路口还须等满3秒才转弯",
                openings=sorted(openings), stable_barrier=link._stable_barrier,
                source_keys=link._junction_side_sources,
                brake_sent_at=getattr(link, "_last_brake_sent_at", None),
                wait_remaining_s=_junction_wait_remaining(link))
    display.maybe_print()
    return openings


def _recover_sides(link, display, *, source_barrier: float, deadline: float = math.inf,
                   input_source=None) -> _SideSample:
    previous, verified = None, []
    display.action = "侧激光停车恢复"
    while True:
        _check_input_exit(input_source)
        _remaining(deadline)
        display.tick()
        samples = display.consume_samples(collect_means=False, update_controller=True)
        if not _assert_still_stable(link, display):
            source_barrier = _wait_stable(link, display, deadline, input_source=input_source)
            display.controller.source_barrier = source_barrier
            previous, verified = None, []
            continue
        now = time.monotonic()
        if getattr(link, "side_candidates_enabled", False):
            for sample in samples:
                if sample.candidates:
                    _latch_junction(link, sample, initial=True)
        if _pending_junction_openings(link):
            _clear_side_recovery(link)
            return samples[-1] if samples else _side_sample(link)
        for sample in samples:
            if _fresh_post_stop_pair(sample, now, source_barrier, previous):
                previous = sample
                verified.append(sample)
                verified = verified[-POST_STOP_PAIRS:]
        latest = samples[-1] if samples else getattr(link, "_latest_side_sample", None)
        display.confirmation = (f"停车后新鲜双侧反馈 {len(verified)}/{POST_STOP_PAIRS}；继续等待；"
                                f"原始开口={sorted(latest.openings) if latest else []}；"
                                f"新鲜候选={sorted(latest.candidates) if latest else []}")
        covered = True
        if len(verified) >= POST_STOP_PAIRS:
            controller = display.controller
            retry = getattr(controller, "retry_pending_pair", None)
            if callable(retry):
                retry(now=now)
                poses = getattr(controller, "_poses", ())
                latest = controller.latest_source_times
                # Laser may precede STATE. Reuse these exact two source pairs;
                # action choice waits for actual stopped pose coverage and
                # their ordered consumption, without another laser cycle.
                covered = bool(poses and all(
                    poses[0].at - 1e-9 <= source <= poses[-1].at + 1e-9
                    and (not pair.dual_wall or latest is not None and latest[index] >= source - 1e-9)
                    for pair in verified for index, source in enumerate(pair.source_times)))
            if not covered:
                display.confirmation = "两组停后激光已保存，等待实际STATE覆盖后直接复用"
        if len(verified) >= POST_STOP_PAIRS and covered:
            link._verified_side_pairs = verified
            _set_ready_feedback(link, verified, source_barrier)
            _clear_side_recovery(link)
            runlog.emit("motion_sequence_laser_recovered", "停稳后两个独立双侧新样本恢复",
                        source_barrier=source_barrier, source_keys=previous.source_keys)
            return previous
        display.maybe_print()
        time.sleep(CONTROL_INTERVAL_S)


def _sync_controller_applied(link, display) -> None:
    """Only matched timely ACK/STATE evidence may advance applied trajectory state."""
    notify = getattr(display.controller, "notify_target_applied", None)
    if not callable(notify):
        return
    notified = getattr(display, "_controller_applied_records", None)
    if notified is None:
        notified = display._controller_applied_records = set()
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    if getattr(display, "_controller_applied_identity", None) != identity:
        notified.clear()
        display._controller_applied_source_at = None
        display._controller_applied_identity = identity
    ledger = getattr(link, "_command_ledger", {})
    confirmed = getattr(link, "_confirmed_commands", {})
    live = set()
    for key, record in ledger.items():
        token = (*identity, key, record[0])
        live.add(token)
        if (key not in confirmed or token in notified or tuple(record[6:8]) != identity
                or record[5][0] is not MessageType.CONTROL_TARGET
                or len(record) > 10 and record[10].get("drive_invalidated_by_brake")):
            continue
        state = confirmed[key]
        witness_key = getattr(link, "_superseded_commands", {}).get(key, key)
        witness = ledger.get(witness_key, record)
        source_at = witness[10].get("state_source_at") if len(witness) > 10 else None
        if source_at is None:
            # Map the exact confirmed MCU uptime into the same host-source clock
            # used by laser alignment. Pump completion is never an application time.
            reference_raw = getattr(link, "_uptime_reference_raw", None)
            offset = getattr(link, "_clock_offset_s", None)
            if reference_raw is not None and offset is not None:
                delta = ((state.uptime_ms - reference_raw + 0x80000000)
                         & 0xFFFFFFFF) - 0x80000000
                source_at = offset + (getattr(link, "_uptime_reference_ms", reference_raw) + delta) / 1000.0
            elif getattr(link, "state", None) is not None:
                sample_time = getattr(link, "_sample_time", None)
                if callable(sample_time):
                    delta = ((link.state.uptime_ms - state.uptime_ms) & 0xFFFFFFFF) / 1000.0
                    source_at = sample_time("state") - delta
        if source_at is not None:
            latest_source = getattr(display, "_controller_applied_source_at", None)
            if latest_source is None or source_at >= latest_source - 1e-9:
                notify(record[5][1], record[5][2], applied_at=source_at, source_at=source_at)
                display._controller_applied_source_at = source_at
            notified.add(token)
    notified.intersection_update(live)


def _send_feedback_target(link: mcu.McuLink, display: MotionDisplay,
                          linear: int, angular: int,
                          deadline: float = math.inf, *,
                          target: CenteringTarget | None = None) -> int:
    """Exact v11 integer wheel mapping for normal forward correction."""
    if (isinstance(linear, bool) or isinstance(angular, bool)
            or not isinstance(linear, int) or not isinstance(angular, int)
            or angular % CENTER_ANGULAR_QUANTUM_MRADPS or abs(angular) > CENTER_TRUSTED_ANGULAR_MRADPS
            or not 0 < linear <= MAX_WHEEL_SPEED_MMPS):
        raise mcu.McuError("非法居中目标：需要非零合法v/w和16 mrad/s整数角速度")
    half = math.trunc(angular * WHEEL_BASE_MM / 2000)
    left, right = linear - half, linear + half
    minimum = CENTER_MIN_WHEEL_SPEED_MMPS
    if left * right <= 0 or not all(minimum <= abs(wheel) <= MAX_WHEEL_SPEED_MMPS for wheel in (left, right)):
        raise mcu.McuError(f"居中目标轮速越界：{left}/{right}，非零下限{minimum}")
    _remaining(deadline)
    if target is not None and not display.controller.target_send_ready(target, time.monotonic()):
        raise _CenteringProposalExpired("发令前真实测量、身份或航向证据已改变，重新计算xx前进目标")
    sequence = link.send_motion(linear, angular, require_ranges=True, require_upper=False)
    link._stop_tracker = None
    link._stable_barrier = None
    link._ready_side_pairs = []
    link._brake_evidence_barrier = None
    display.command = linear, angular
    display.controller.notify_sent(angular, sent_at=time.monotonic())
    display.wheel_targets = left, right
    display.wheel_delta = right - left
    display.raw_delta = float(right - left)
    display.command_kind = MessageType.CONTROL_TARGET
    runlog.emit("motion_sequence_command", "正常xx前进差速目标发送", stage="NORMAL",
                sequence=sequence, linear_mmps=linear, angular_mradps=angular,
                wheels_mmps=(left, right))
    return sequence


class _WheelResponseTracker:
    """仅累计已确认驱动时间；暂停不能清除真实单轮无响应的累计证据。"""

    def __init__(self, link, now: float) -> None:
        self.at = now
        self.previous = link.odom_mm
        self.token = link.state.uptime_ms
        self.drive_s = [0.0, 0.0]
        self.travel_mm = [0.0, 0.0]
        self.opposite_mm = [0.0, 0.0]
        self.active_since = None
        self.applied_token = None
        self.applied_position = None

    def pause(self, link, now: float) -> None:
        self.at = now
        self.previous = link.odom_mm
        self.token = link.state.uptime_ms
        self.active_since = None
        self.applied_token = None
        self.applied_position = None

    def update(self, link, wheels, now: float, confirmed_since: float | None,
               samples=None, *, applied_state=None) -> None:
        begin = max(self.at, confirmed_since) if confirmed_since is not None else now
        elapsed = max(0.0, now - begin)
        self.at = now
        if confirmed_since is not None and self.active_since is None:
            self.active_since = confirmed_since
            if applied_state is not None:
                self.applied_token = applied_state.uptime_ms
                self.applied_position = applied_state.left_mm, applied_state.right_mm
        if samples is None:
            sample_time = getattr(link, "_sample_time", None)
            generated_at = sample_time("state") if callable(sample_time) else now
            samples = [(link.state, generated_at)]
        for state, generated_at in samples:
            step = (state.uptime_ms - self.token) & 0xFFFFFFFF
            if not 0 < step < 0x80000000:
                continue
            current = state.left_mm, state.right_mm
            # STATE可能比主循环滞后100ms。以本次已确认驱动段的屏障检查，
            # 不能要求50ms遥测的源时刻晚于上一个20ms主循环。
            if self.active_since is not None and self.applied_token is not None:
                progress_step = (state.uptime_ms - self.applied_token) & 0xFFFFFFFF
                usable = 0 < progress_step < 0x80000000
                previous = self.applied_position
            else:
                usable = bool(elapsed and self.active_since is not None
                              and generated_at >= self.active_since)
                previous = self.previous
            if usable:
                for index, wheel in enumerate(wheels):
                    sign = 1 if wheel > 0 else -1 if wheel < 0 else 0
                    self.travel_mm[index] += max(0.0, sign * (current[index] - previous[index]))
                    self.opposite_mm[index] += max(0.0, -sign * (current[index] - previous[index]))
                if self.applied_token is not None:
                    self.applied_token, self.applied_position = state.uptime_ms, current
            self.previous, self.token = current, state.uptime_ms
        for index, wheel in enumerate(wheels):
            if not wheel or not elapsed:
                continue
            self.drive_s[index] += elapsed
            if self.travel_mm[index] >= config.ROAD_PROGRESS_MIN_MM:
                self.drive_s[index] = self.travel_mm[index] = 0.0
                self.opposite_mm[index] = 0.0
            elif self.drive_s[index] + 1e-9 >= config.ROAD_NO_PROGRESS_TIMEOUT_S:
                side = "左" if index == 0 else "右"
                reason = "反向或无进展" if self.opposite_mm[index] >= config.ROAD_PROGRESS_MIN_MM else "无进展"
                raise mcu.McuError(f"直行编码器{reason}：{side}轮已确认驱动累计3秒，命令方向进展不足5 mm")


def _record_wheels(target):
    if target[0] is MessageType.CONTROL_TARGET:
        half = math.trunc(target[2] * WHEEL_BASE_MM / 2000)
        return target[1] - half, target[1] + half
    if target[0] is MessageType.MOTION_ACTION and target[1] is MotionActionCode.STRAIGHT:
        return target[2], target[2]
    return 0, 0


def _wheel_signs(wheels):
    return tuple(1 if value > 0 else -1 if value < 0 else 0 for value in wheels)


def _confirmed_motion_records(link):
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    return [(key, record) for key, record in getattr(link, "_command_ledger", {}).items()
            if key in getattr(link, "_confirmed_commands", {})
            and tuple(record[6:8]) == identity and any(_record_wheels(record[5]))
            and not (len(record) > 10 and record[10].get("drive_invalidated_by_brake"))]


def _motion_confirmed_since(link, sent_at: float | None, linear: int = 0, *, wheels=None) -> float | None:
    """Response supervision only; does not waive any command's confirmation."""
    if sent_at is None:
        return None
    if getattr(link, "_command_ledger", None) is not None:
        signs = _wheel_signs(wheels if wheels is not None else (linear, linear))
        matching = [link._confirmed_evidence_times[key] for key, record in _confirmed_motion_records(link)
                    if _wheel_signs(_record_wheels(record[5])) == signs]
        return min(matching) if matching else None
    pending = getattr(link, "motion_target_pending", False)
    return None if (pending() if callable(pending) else pending) else sent_at


def _motion_applied_state(link, linear: int = 0, *, wheels=None):
    signs = _wheel_signs(wheels if wheels is not None else (linear, linear))
    matching = [(link._confirmed_evidence_times[key], key) for key, record in _confirmed_motion_records(link)
                if _wheel_signs(_record_wheels(record[5])) == signs]
    return link._confirmed_commands[min(matching)[1]] if matching else None


def _confirmed_wheel_response(link, sent_at, wheels):
    """Supervise the actually confirmed old goal while a new goal is pending."""
    if getattr(link, "_command_ledger", None) is None:
        return wheels, _motion_confirmed_since(link, sent_at, wheels=wheels), None
    records = _confirmed_motion_records(link)
    if not records:
        return wheels, None, None
    _, latest = max(records, key=lambda pair: pair[1][0])
    executing = _record_wheels(latest[5])
    return (executing, _motion_confirmed_since(link, sent_at, wheels=executing),
            _motion_applied_state(link, wheels=executing))


def _wheel_transition_requires_stop(previous, proposed) -> bool:
    """每个已驱动轮的归零或换向都须经过整车实际停稳。"""
    return any(old and (not new or old * new < 0)
               for old, new in zip(previous, proposed))


class _RoadProgress:
    """Fixed road origin in net wheel travel; only real STATE covers an origin."""

    def __init__(self, origin_mm=None):
        self.origin_mm = tuple(origin_mm) if origin_mm is not None else None
        self.origin_source_at = None
        self.last_positions = self.origin_mm
        self._history = deque(maxlen=512)
        self._token = None

    def update(self, state, source_at):
        token = state.uptime_ms
        if self._token is not None and not 0 < ((token - self._token) & 0xFFFFFFFF) < 0x80000000:
            return False
        if self._history and source_at <= self._history[-1][0]:
            return False
        self._token = token
        self.last_positions = (state.left_mm, state.right_mm)
        self._history.append((source_at, self.last_positions))
        while len(self._history) > 2 and source_at - self._history[0][0] > CENTER_HISTORY_S:
            self._history.popleft()
        return True

    def establish(self, source_at):
        if self.origin_mm is not None:
            return True
        if not self._history or not self._history[0][0] - 1e-9 <= source_at <= self._history[-1][0] + 1e-9:
            return False
        previous = self._history[0]
        for current in self._history:
            if current[0] + 1e-9 >= source_at:
                fraction = (source_at - previous[0]) / (current[0] - previous[0]) if current[0] > previous[0] else 0.0
                self.origin_mm = tuple(a + fraction * (b - a) for a, b in zip(previous[1], current[1]))
                self.origin_source_at = source_at
                return True
            previous = current
        return False

    @property
    def position_mm(self):
        if self.origin_mm is None or self.last_positions is None:
            return None
        return sum(current - origin for current, origin in zip(self.last_positions, self.origin_mm)) / 2.0


def _drive_segment(link: mcu.McuLink, display: MotionDisplay,
                   deadline: float = math.inf, *, after_turn: bool,
                   linear_speed_mmps: int = LINEAR_SPEED_MMPS,
                   input_source=None) -> frozenset[str] | None:
    """Forward-only xx correction with unchanged sensor and motion supervision."""
    controller = display.controller
    source_barrier = display.averages.source_barrier
    if after_turn:
        source_barrier = getattr(link, "post_turn_barrier", source_barrier)
    armed = not after_turn
    link.side_motion_active = True
    link.side_candidates_enabled = armed
    link.side_age_required = True
    pending_first_wall = None
    first_wall_source_at = None
    now = time.monotonic()
    next_send, sent_at = now, None
    response = _WheelResponseTracker(link, now)
    progress = _RoadProgress(None if after_turn else link.odom_mm)
    display.road_progress = progress
    display.origin = progress.origin_mm
    previous_distance_observer = display._distance_observer

    def observe_distance(state):
        at = getattr(controller, "_state_at", None)
        progress.update(state, time.monotonic() if at is None else at)
        if previous_distance_observer is not None:
            previous_distance_observer(state)


    display._distance_observer = observe_distance
    progress.update(link.state, getattr(controller, "_state_at", None) or now)

    def refresh_after_stop(reason):
        nonlocal sent_at, next_send, source_barrier, pending_first_wall
        response.pause(link, time.monotonic())
        before = sum(link.odom_mm) / 2.0
        if any(display.command) or getattr(link, "_stop_tracker", None) is None:
            _brake(link, display, deadline, input_source=input_source)
        else:
            _ensure_stable(link, display, deadline, input_source=input_source)
        display.braking_tail_mm = abs(sum(link.odom_mm) / 2.0 - before)
        source_barrier = getattr(link, "_stable_barrier", None) or time.monotonic()
        pending_first_wall = None
        display.reset_averages(source_barrier=source_barrier, reset_controller=False)
        display.confirmation = reason + "；停稳后等待两组真正新健康反馈"
        runlog.emit("motion_sequence_recovery_pause", "严格停稳复核后重新计算前进纠偏",
                    reason=reason, direction=1, road_position_mm=progress.position_mm,
                    braking_tail_mm=display.braking_tail_mm,
                    full_difference_mm=controller.full_difference_mm,
                    fixed_reference_deg=controller.heading_reference_deg)
        _recover_sides(link, display, source_barrier=source_barrier,
                       deadline=deadline, input_source=input_source)
        source_barrier = getattr(link, "_stable_barrier", None) or source_barrier
        controller.begin_alignment(time.monotonic(), source_barrier=source_barrier)
        if not armed:
            begin = getattr(link, "begin_departure", None)
            if callable(begin):
                begin(source_barrier)
        sent_at, next_send = None, time.monotonic()
        response.pause(link, next_send)


    try:
        while True:
            _check_input_exit(input_source)
            display.tick()
            _remaining(deadline)
            samples = display.consume_samples(collect_means=armed)
            now = time.monotonic()
            if progress.position_mm is not None:
                display.segment_distance_mm = progress.position_mm
            was_armed = armed
            if not armed:
                for sample in samples:
                    if (not sample.dual_wall or source_barrier is not None
                            and any(source <= source_barrier + 1e-9 for source in sample.source_times)
                            or any(not 0 <= now - source <= LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                                   for source in sample.source_times)):
                        continue
                    if pending_first_wall is None:
                        pending_first_wall = min(sample.source_times), sample
                    controller.observe_side(sample)
                if pending_first_wall is not None:
                    source_at, first_wall = pending_first_wall
                    controller.retry_pending_pair(now=now)
                    poses = getattr(controller, "_poses", ())
                    if (poses and poses[0].at - 1e-9 <= source_at <= poses[-1].at + 1e-9
                            and controller.latest_source_time is not None
                            and controller.latest_source_time >= source_at - 1e-9
                            and progress.establish(source_at)):
                        armed = True
                        first_wall_source_at = source_at
                        display.origin = progress.origin_mm
                        display.segment_distance_mm = progress.position_mm
                        finish = getattr(link, "finish_departure", None)
                        if callable(finish):
                            finish()
                        link.side_candidates_enabled = True
                        controller.begin_alignment(now, source_barrier=source_barrier)
                        display.averages.reset(now, source_barrier=source_barrier)
                        # The departure sample arms control, not either complete
                        # alignment window. Only subsequent arrivals form means.
                        display._ready_batches.clear()
                        display.confirmation = "首组停后新双墙已对齐STATE；按xx启用激光纠偏，均值窗只核验居中"
                        runlog.emit("motion_sequence_centering_armed", "离口结束，xx前进纠偏",
                                    first_wall_source_keys=first_wall.source_keys,
                                    first_wall_source_at=source_at,
                                    road_origin_mm=progress.origin_mm,
                                    fixed_reference_deg=controller.heading_reference_deg)
                        pending_first_wall = None
            candidate = next((sample for sample in samples if armed and sample.candidates
                and (was_armed or first_wall_source_at is None or any(
                    source > first_wall_source_at + 1e-9 for index, source in enumerate(sample.source_times)
                    if ("left" if index == 0 else "right") in sample.candidates))), None)
            if armed and (candidate is not None or _pending_junction_openings(link)):
                response.pause(link, now)
                candidate = candidate or _side_sample(link)
                runlog.emit("motion_sequence_junction_candidate", "新鲜开口立即抢占前进居中或巡航",
                            openings=sorted(candidate.candidates | _pending_junction_openings(link)),
                            **candidate.diagnostics(now))
                return _confirm_junction(link, display, candidate, deadline, input_source=input_source)
            if getattr(link, "side_recovery_active", False):
                refresh_after_stop(getattr(link, "side_recovery_reason", None) or "激光缓存已过期")
                continue
            executing_wheels, confirmed_at, applied = _confirmed_wheel_response(
                link, sent_at, display.wheel_targets)
            response.update(link, executing_wheels, now, confirmed_at,
                            display.state_samples if display.state_samples else None,
                            applied_state=applied)
            if now + 1e-9 < next_send:
                display.maybe_print()
                time.sleep(CONTROL_INTERVAL_S)
                continue
            cached = bool(getattr(link, "side_cached_active", False))
            pending = getattr(link, "motion_target_pending", False)
            pending = pending() if callable(pending) else pending
            if pending and any(display.wheel_targets) and not cached:
                target = None
                linear, angular = display.command
            else:
                target = controller.target(linear_speed_mmps, now,
                                           cached_feedback=cached, departure_only=not armed)
                if target is None or not any(target.wheel_targets):
                    if any(display.wheel_targets):
                        refresh_after_stop("缺少可控制的新鲜双侧反馈")
                    else:
                        display.action = "停稳等待健康新反馈"
                        display.maybe_print()
                        time.sleep(CONTROL_INTERVAL_S)
                    continue
                linear, angular = target.linear_mmps, target.angular_mradps
                if _wheel_transition_requires_stop(display.wheel_targets, target.wheel_targets):
                    raise mcu.McuError("前进控制器产生非法换向目标")
                display.centering_target = target
                display.action = "转后xx离口，等待停后新双墙" if not armed else target.mode
                display.control_mode = target.mode
                display.arc_phase = display.action
            try:
                _send_feedback_target(link, display, linear, angular, deadline, target=target)
            except _CenteringProposalExpired as exc:
                display.confirmation = str(exc)
                next_send = time.monotonic()
                time.sleep(CONTROL_INTERVAL_S)
                continue
            except _SideRecoveryRequired as exc:
                refresh_after_stop(str(exc))
                continue
            sent_at = time.monotonic()
            if target is not None:
                controller.confirm_target_sent(target)
            # Start each lease interval at the actual successful write. Waiting
            # for the first STATE must not shorten or skip the next renewal.
            next_send = sent_at + COMMAND_INTERVAL_S
            display.arc_distance_mm = display.segment_distance_mm
            display.maybe_print()
            time.sleep(CONTROL_INTERVAL_S)
    finally:
        display._distance_observer = previous_distance_observer
        link.side_candidates_enabled = False
        link.side_motion_active = False
        finish = getattr(link, "finish_departure", None)
        if callable(finish):
            finish()


def _normalize_yaw(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _turn_at_junction(link: mcu.McuLink, display: MotionDisplay,
                      openings: frozenset[str], deadline: float = math.inf, *,
                      turn_speed_mmps: int = TURN_SPEED_MMPS,
                      input_source=None) -> None:
    _check_input_exit(input_source)
    if not openings or not openings <= {"left", "right"}:
        raise mcu.McuError("转弯缺少已确认的有效开口")
    latched = bool(_pending_junction_openings(link))
    _remaining(deadline)
    if turn_speed_mmps != TURN_SPEED_MMPS:
        raise mcu.McuError("v11 固定转弯轮速请求必须为97 mm/s")
    if getattr(link, "_stop_tracker", None) is None:
        _brake(link, display, deadline, input_source=input_source)
    else:
        display.tick()
        display.consume_samples(collect_means=False)
        _ensure_stable(link, display, deadline, input_source=input_source)
    _wait_junction_turn_ready(link, display, deadline, input_source=input_source)
    if latched:
        openings = _pending_junction_openings(link)
        if not openings:
            raise mcu.McuError("转弯前已保存开口失效，禁止使用旧会话方向")
    side = random.choice(("left", "right")) if len(openings) == 2 else next(iter(openings))
    name = "左90°" if side == "left" else "右90°"
    action = MotionActionCode.TURN_LEFT if side == "left" else MotionActionCode.TURN_RIGHT
    _clear_side_recovery(link)
    revoke = getattr(display.controller, "revoke_normal_motion", None)
    if callable(revoke):
        revoke()
    finish_departure = getattr(link, "finish_departure", None)
    if callable(finish_departure):
        finish_departure()
    start_yaw = link.yaw_deg
    if start_yaw is None:
        raise mcu.McuError("转弯前航向无效")
    previous_reference = getattr(display.controller, "formal_reference_deg", None)
    if previous_reference is None:
        previous_reference = getattr(display.controller, "provisional_reference_deg", None)
    if previous_reference is None:
        previous_reference = display.controller.heading_reference_deg
    # A direct turn fixture may not have completed startup sampling. The real
    # run establishes its road prior in _sample_start before any movement.
    if previous_reference is None:
        previous_reference = start_yaw
    link.side_age_required = False
    turn_angle = 90 if side == "left" else -90
    target_yaw = _normalize_yaw(start_yaw + turn_angle)
    next_reference = _normalize_yaw(previous_reference + turn_angle)
    display.origin = None
    display.reset_averages()
    display.action = name
    display.turn_target_yaw = target_yaw
    display.control_mode = f"固定90°转弯，轮速 {turn_speed_mmps} mm/s，居中纠偏关闭"
    display.emitter(f"{'双侧开口随机选择' if len(openings) == 2 else '单侧开口选择'}："
                    f"{name}，转弯轮速请求 {turn_speed_mmps} mm/s。")
    stable_barrier = getattr(link, "_stable_barrier", None)
    brake_sent_at = link._last_brake_sent_at
    try:
        sequence = link.send_motion_action(action, turn_speed_mmps,
                                          brake_after_turn=True,
                                          require_ranges=False, require_upper=False)
    except mcu.McuError as exc:
        runlog.emit("motion_sequence_turn_not_sent", "固定转弯发令门禁中止",
                    action=action.name, reason=str(exc), brake_sent_at=brake_sent_at,
                    elapsed_s=time.monotonic() - brake_sent_at, stable_barrier=stable_barrier)
        raise
    runlog.emit("motion_sequence_turn_sent", "BRAKE发送满3秒并停稳核验通过，已发送固定90度转弯",
                action=action.name, sequence=sequence, speed_mmps=turn_speed_mmps,
                brake_sent_at=brake_sent_at, elapsed_s=time.monotonic() - brake_sent_at,
                stable_barrier=stable_barrier, wheel_positions_mm=link.odom_mm,
                saved_openings=sorted(openings), start_yaw_deg=start_yaw,
                target_yaw_deg=target_yaw, previous_road_reference_deg=previous_reference,
                next_road_reference_deg=next_reference)
    link._active_turn_details = dict(action=action.name, sequence=sequence,
                                     origin_mm=link.odom_mm)
    display.turn_result = f"{name}执行中，动作序号={sequence}"
    sign = 1 if side == "left" else -1
    link._turn_direction_guard = _TurnDirectionGuard(
        sign, start_yaw, (getattr(link, "session_nonce", None), getattr(link, "boot_id", None)))
    display.command = (0, sign * 2 * turn_speed_mmps * 8)
    display.command_kind = MessageType.MOTION_ACTION
    display.wheel_targets = (-sign * turn_speed_mmps, sign * turn_speed_mmps)
    display.maybe_print(force=True)

    def poll() -> None:
        _check_input_exit(input_source)
        _remaining(deadline)
        display.tick(pump=False)
        display.consume_samples(collect_means=False, allow_stale=True)
        display.maybe_print()

    try:
        link.wait_for_motion_action_complete(
            sequence, timeout_s=min(config.TURN_TIMEOUT_S, _remaining(deadline)),
            require_ranges=False, require_upper=False, on_poll=poll)
        turn_completed_at = time.monotonic()
    except mcu.McuError as exc:
        runlog.emit("motion_sequence_turn_aborted", "固定90度转弯确认或执行中止",
                    action=action.name, sequence=sequence, reason=str(exc))
        raise
    finally:
        link._turn_direction_guard = None
    runlog.emit("motion_sequence_turn_completion_received", "已确认本次有效转弯完成反馈，开始1秒计时",
                action=action.name, sequence=sequence, turn_completed_at=turn_completed_at,
                post_turn_wait_s=POST_TURN_WAIT_S)
    _remaining(deadline)
    display.set_stopped()
    display.action = f"{name}完成，编码器停稳确认"
    _brake(link, display, deadline, input_source=input_source)
    display.action = f"{name}完成，等待满1秒并复检停稳"
    while True:
        _check_input_exit(input_source)
        _remaining(deadline)
        display.tick()
        display.consume_samples(collect_means=False, allow_stale=True)
        stable = _assert_still_stable(link, display)
        remaining = max(0.0, turn_completed_at + POST_TURN_WAIT_S - time.monotonic())
        display.confirmation = (f"转弯完成反馈后1秒剩余={remaining:.2f}s；"
                                f"双轮连续500ms静止={'已确认' if stable else '等待'}；"
                                "保持BRAKE，尚未允许直行")
        display.maybe_print()
        if remaining <= 1e-9 and stable:
            break
        if not stable:
            _wait_stable(link, display, deadline, input_source=input_source)
        else:
            time.sleep(min(CONTROL_INTERVAL_S, remaining))
    actual_yaw = link.yaw_deg
    if actual_yaw is None or abs(_normalize_yaw(actual_yaw - target_yaw)) > config.TURN_YAW_TOLERANCE_DEG:
        raise mcu.McuError(f"{name}航向未达到目标：目标 {target_yaw:.1f}°，实际 {actual_yaw}°")
    post_turn_barrier = getattr(link, "_stable_barrier", None)
    if post_turn_barrier is None:
        post_turn_barrier = time.monotonic()
    link.post_turn_barrier = post_turn_barrier
    display.reset_averages(source_barrier=post_turn_barrier)
    display.controller.set_initial_reference(next_reference)
    runlog.emit("motion_sequence_initial_reference", "上一道路固定参考加减90度建立新参考",
                yaw_deg=next_reference, previous_road_reference_deg=previous_reference,
                actual_yaw_deg=actual_yaw, after_turn=True)
    link.side_age_required = True
    begin_departure = getattr(link, "begin_departure", None)
    if callable(begin_departure):
        begin_departure(post_turn_barrier)
    display.turn_target_yaw = None
    display.origin = None
    display.after_turn = True
    display.segment_distance_mm = 0.0
    display.tick(pump=False)
    display.confirmation = ("转弯完成反馈后已满1秒且停稳核验通过，按xx离口；首组停后新双墙启用xx前进纠偏，"
                            "两个0.5秒补偿均值窗只确认居中，不限制速度")
    display.turn_result = f"{name}完成并停稳，目标 {target_yaw:.1f}°，实际 {actual_yaw:.1f}°"
    runlog.emit("motion_sequence_turn_completed", "有效完成反馈后已满1秒，显式BRAKE并核验停稳和航向",
                action=action.name, sequence=sequence, target_yaw_deg=target_yaw,
                actual_yaw_deg=actual_yaw, post_turn_barrier=post_turn_barrier,
                turn_completed_at=turn_completed_at, post_turn_wait_s=POST_TURN_WAIT_S,
                elapsed_s=time.monotonic() - turn_completed_at)
    link._active_turn_details = None
    display.maybe_print(force=True)


def _report_failure(link: mcu.McuLink, display: MotionDisplay | None, reason: str) -> None:
    snapshot = link.motion_health_snapshot()
    delta = None
    if display is not None and display.origin is not None and link.state is not None:
        delta = (link.state.left_mm - display.origin[0], link.state.right_mm - display.origin[1])
    try:
        if link.connected:
            link.send_stop(StopReason.FAULT)
    except Exception:
        pass
    if display is not None:
        display.set_stopped()
        display.confirmation = f"失败：{reason}"
        try:
            display.tick(check_health=False)
            display.consume_samples(require_healthy=False, collect_means=False)
        except BaseException:
            pass
        display.maybe_print(force=True)
    action = display.action if display is not None else "准备"
    emitter = display.emitter if display is not None else None
    message = f"v11运动流程未完成：{reason}；当前动作：{action}\n失败状态：{snapshot}"
    if delta is not None:
        message += f"\n本段双轮里程增量：{delta} mm"
    traces = [dict(sample_sequence=trace.sample_sequence, source_time=at,
                   left_encoder_delta=trace.left_encoder_delta,
                   right_encoder_delta=trace.right_encoder_delta,
                   applied_control_sequence=trace.applied_control_sequence)
              for trace, at in getattr(link, "_trace_history", [])]
    active_turn = getattr(link, "_active_turn_details", None)
    if active_turn is not None:
        raw_state = getattr(link, "state", None)
        raw_positions = ((raw_state.left_mm, raw_state.right_mm) if raw_state is not None else None)
        turn_delta = (tuple(a - b for a, b in zip(raw_positions, active_turn["origin_mm"]))
                      if raw_positions is not None else None)
        runlog.emit("motion_sequence_turn_aborted", "固定转弯流程中止，保留停机保护",
                    action=active_turn["action"], sequence=active_turn["sequence"],
                    reason=reason, wheel_travel_mm=turn_delta, wheel_trace_evidence=traces)
        link._active_turn_details = None
    runlog.emit("motion_sequence_failed", message, reason=reason, phase=action,
                wheel_trace_evidence=traces, wheel_travel_mm=delta)
    if emitter is not None:
        emitter(message)


def run_motion_sequence(*, link: mcu.McuLink | None = None, input_source=None) -> bool:
    link = link if link is not None else MotionSequenceLink()
    display: MotionDisplay | None = None
    terminal = AsyncTerminal()
    try:
        if isinstance(link, mcu.McuLink) and not isinstance(link, MotionSequenceLink):
            raise mcu.McuError("本测试的实车链路必须使用 MotionSequenceLink 的局部门禁")
        if input_source is None:
            input_source = TerminalSpeedInput(write_line=terminal)
        link.set_motion_deadline(None)
        link.open()
        link.hello()
        link.synchronize_clock()
        link.send_stop(StopReason.MISSION)
        display = MotionDisplay(link, emitter=terminal)
        _prepare(link, display, time.monotonic() + config.MCU_SELFCHECK_READY_TIMEOUT_S,
                 input_source=input_source)
        link.motion_checks_enabled = True
        _brake(link, display, input_source=input_source)
        linear_speed = _wait_speed(link, display, input_source)
        _wait_stable(link, display, input_source=input_source)
        _sample_start(link, display, input_source=input_source)
        display.origin = link.odom_mm
        display.segment_distance_mm = 0.0
        openings = _drive_segment(link, display, after_turn=False,
                                  linear_speed_mmps=linear_speed, input_source=input_source)
        _turn_at_junction(link, display, openings, input_source=input_source)
        _drive_segment(link, display, after_turn=True,
                       linear_speed_mmps=linear_speed, input_source=input_source)
        display.action = "测试完成，车辆已停稳"
        display.confirmation = "第二处路口已确认并停稳；共转弯1次"
        display.maybe_print(force=True)
        terminal(f"v11运动流程完成：第二路口已停稳；输入xx={linear_speed} mm/s；"
                 f"共转弯1次；最终双轮里程 {link.odom_mm} mm。")
        return True
    except KeyboardInterrupt:
        _report_failure(link, display, "收到 Ctrl+C，尝试停车")
        return False
    except EOFError:
        _report_failure(link, display, "终端输入已关闭，尝试停车")
        return False
    except Exception as exc:
        _report_failure(link, display, str(exc))
        if display is None:
            terminal(f"v11运动流程未完成：{exc}")
        return False
    finally:
        try:
            if link.connected:
                link.send_stop(StopReason.CLOSE)
        except Exception:
            pass
        try:
            if input_source is not None:
                input_source.close()
        finally:
            try:
                link.shutdown()
            finally:
                terminal.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="v11：仅填写一次xx；停车采样1秒、激光与陀螺仪协同居中、首路口90°转弯、第二路口停稳结束。")
    parser.parse_args(argv)
    return 0 if run_motion_sequence() else 1


if __name__ == "__main__":
    with runlog.RunLog("motion_sequence", terminal=RawTerminalStream(), periodic_status=False):
        sys.exit(main())
