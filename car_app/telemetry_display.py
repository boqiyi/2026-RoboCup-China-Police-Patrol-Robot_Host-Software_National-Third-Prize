"""Format cached telemetry after both laser groups update, without doing I/O."""

from __future__ import annotations

import math
import time

import config
from drivers.protocol_v2 import WireGyroStatus, WireRangeStatus


INCOMPLETE_WAIT_S = 0.25
_RANGE_LABELS = {
    WireRangeStatus.VALID: "有效", WireRangeStatus.NO_TARGET: "无目标",
    WireRangeStatus.OUT_OF_RANGE: "超量程", WireRangeStatus.SIGNAL_ERROR: "信号错误",
    WireRangeStatus.TIMEOUT: "超时", WireRangeStatus.BUS_ERROR: "总线故障",
    WireRangeStatus.STALE: "过期",
}
_GYRO_LABELS = {
    WireGyroStatus.NOT_INITIALIZED: "未初始化",
    WireGyroStatus.WHO_AM_I_ERROR: "身份错误",
    WireGyroStatus.UNCALIBRATED: "未校准",
    WireGyroStatus.CALIBRATING: "校准中", WireGyroStatus.READY: "就绪",
    WireGyroStatus.BUS_ERROR: "总线故障", WireGyroStatus.SAMPLE_GAP: "采样间隔异常",
}


def _number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _at(values, index):
    return values[index] if isinstance(values, (tuple, list)) and len(values) > index else None


def _enum(value, enum):
    try:
        return enum(value) if value is not None and not isinstance(value, bool) else None
    except (TypeError, ValueError):
        return None


def _status(value, enum, labels) -> str:
    if value is None:
        return "未收到"
    status = _enum(value, enum)
    return f"{labels[status]} {status.name}({int(status)})" if status is not None else f"未知({value})"


def _value(value, suffix="", digits=None) -> str:
    if not _number(value):
        return "—"
    shown = f"{value:.{digits}f}" if digits is not None else str(value)
    return shown + suffix


class TelemetryDisplay:
    """Read-only display boundary; two independent group seqs need not be equal.

    An incomplete/forced output consumes the current update boundary too.
    Age changes alone do not emit logs; entering/leaving a missing or faulty
    state does. This is a combined update, not a physical four-sensor round.
    """

    def __init__(self, link) -> None:
        self.link = link
        self._seqs = [None, None]
        self._updated = [False, False]
        self._observed_at = [None, None]
        self._interval_ms = [None, None]
        self._pending_at = None
        self._last_validity = None
        self._printed = False
        self._output_at = None
        self._combination = 0
        self._last_encoder = None
        self._speed_mmps = None

    def _group(self, upper: bool):
        telemetry = getattr(self.link, "culvert_upper" if upper else "directional", None)
        seq = getattr(telemetry, "upper_cycle_seq" if upper else "directional_cycle_seq", None)
        if not isinstance(seq, int) or isinstance(seq, bool) or not 0 <= seq <= 65535:
            seq = None
        ages = getattr(self.link, "upper_age_ms" if upper else "directional_age_ms", None)
        limit = config.CULVERT_UPPER_LASER_MAX_AGE_MS if upper else config.DIRECTIONAL_LASER_MAX_AGE_MS
        descriptors = (("CH4 左上4", "upper_left", 0), ("CH5 右上5", "upper_right", 1)) if upper else (
            ("CH1 左下1", "left", 1), ("CH2 右下2", "right", 2))
        channels = []
        for label, name, index in descriptors:
            raw_distance = _at(getattr(telemetry, "distance_mm", None), index)
            raw_status = _at(getattr(telemetry, "status", None), index)
            effective_status = getattr(self.link, name + "_status", raw_status)
            status = _enum(effective_status, WireRangeStatus)
            if status is None and effective_status is not None:
                status = effective_status
            distance = getattr(self.link, name + "_mm", raw_distance)
            age = _at(ages, index)
            age_valid = isinstance(age, int) and not isinstance(age, bool) and age >= 0
            if age_valid and age > limit:
                status = WireRangeStatus.STALE
            if status is WireRangeStatus.VALID and (
                    not _number(distance) or not 0 <= distance < 65535):
                status = WireRangeStatus.SIGNAL_ERROR
            if status is not WireRangeStatus.VALID:
                distance = None
            healthy = (seq is not None and age_valid and age <= limit
                       and status in (WireRangeStatus.VALID, WireRangeStatus.NO_TARGET,
                                      WireRangeStatus.OUT_OF_RANGE))
            channels.append((label, distance, status, age if age_valid else None,
                             raw_distance, raw_status, healthy))
        return seq, channels

    @staticmethod
    def _validity(groups):
        """Keep failure categories visible without logging every age tick."""
        result = []
        for seq, channels in groups:
            states = []
            for _, _, status, age, _, _, healthy in channels:
                if healthy:
                    states.append("usable")
                elif status is None:
                    states.append("missing_status")
                elif status in (WireRangeStatus.VALID, WireRangeStatus.NO_TARGET,
                                WireRangeStatus.OUT_OF_RANGE):
                    states.append("missing_age" if age is None else "missing_group")
                else:
                    states.append(("fault", str(status)))
            result.append((seq is not None, tuple(states)))
        return tuple(result)

    def _encoders(self):
        odometry = getattr(self.link, "odom_mm", None)
        if not (isinstance(odometry, (tuple, list)) and len(odometry) == 2
                and all(_number(value) for value in odometry)):
            odometry = None
        state = getattr(self.link, "state", None)
        uptime = getattr(state, "uptime_ms", None)
        if odometry is not None and isinstance(uptime, int) and not isinstance(uptime, bool):
            current = (uptime, tuple(odometry))
            if self._last_encoder is not None:
                previous_uptime, previous = self._last_encoder
                elapsed = (uptime - previous_uptime) & 0xFFFFFFFF
                if 0 < elapsed < 0x80000000:
                    left = (odometry[0] - previous[0]) * 1000.0 / elapsed
                    right = (odometry[1] - previous[1]) * 1000.0 / elapsed
                    self._speed_mmps = (left, right, (left + right) / 2.0)
            if self._last_encoder is None or uptime != self._last_encoder[0]:
                self._last_encoder = current
        else:
            self._last_encoder = None
            self._speed_mmps = None
        return odometry

    def poll(self, *, phase="", segment_distance_mm=None, reference_yaw_deg=None,
             command=None, confirmation="", turn_result="", speed_mmps=None,
             force=False) -> str | None:
        now = time.monotonic()
        groups = [self._group(False), self._group(True)]
        for index, (seq, _) in enumerate(groups):
            if seq != self._seqs[index]:
                if seq is not None and self._observed_at[index] is not None:
                    self._interval_ms[index] = (now - self._observed_at[index]) * 1000.0
                self._observed_at[index] = now if seq is not None else None
                self._seqs[index] = seq
                self._updated[index] = True
        pending = any(self._updated) or not self._printed
        if pending and self._pending_at is None:
            self._pending_at = now
        healthy = all(channel[-1] for _, channels in groups for channel in channels)
        validity = self._validity(groups)
        validity_changed = self._last_validity is not None and validity != self._last_validity
        ready = all(self._updated) and all(seq is not None for seq, _ in groups)
        timed_out = pending and self._pending_at is not None and now - self._pending_at >= INCOMPLETE_WAIT_S
        odometry = self._encoders()
        if not (ready or validity_changed or pending and (force or timed_out)):
            return None

        complete = ready and healthy
        reasons = []
        for index, (seq, channels) in enumerate(groups):
            name = "上侧组" if index else "下侧组"
            if seq is None:
                reasons.append(name + "缺失")
            elif not self._updated[index]:
                reasons.append(name + "未更新")
            if seq is not None and not all(channel[-1] for channel in channels):
                reasons.append(name + "故障、超龄或数据不可用")
        self._combination += 1
        interval = None if self._output_at is None else (now - self._output_at) * 1000.0
        lines = ["=" * 64,
                 f"组合更新周期 #{self._combination}（{'完整' if complete else '不完整'}）"
                 + (f"；原因：{'；'.join(reasons)}" if reasons else ""),
                 f"侧激光周期：{_value(groups[0][0])}；上层激光周期：{_value(groups[1][0])}；"
                 f"观测间隔：下侧 {_value(self._interval_ms[0], ' ms', 1)}，"
                 f"上侧 {_value(self._interval_ms[1], ' ms', 1)}；距上次输出 {_value(interval, ' ms', 1)}"]
        for _, channels in groups:
            for label, distance, status, age, raw_distance, raw_status, _ in channels:
                line = (f"{label}：{_value(distance, ' mm')}；状态："
                        f"{_status(status, WireRangeStatus, _RANGE_LABELS)}；年龄：{_value(age, ' ms')}")
                if (raw_distance is not None and raw_distance != distance
                        or raw_status is not None and _enum(raw_status, WireRangeStatus) != status):
                    line += (f"；原始：距离 {_value(raw_distance, ' mm')}，"
                             f"状态 {_status(raw_status, WireRangeStatus, _RANGE_LABELS)}")
                lines.append(line)
        left, right = odometry if odometry is not None else (None, None)
        lines.append(f"双轮里程：左 {_value(left, ' mm')}，右 {_value(right, ' mm')}；"
                     f"本段前进距离：{_value(segment_distance_mm, ' mm', 1)} / "
                     f"{_value(segment_distance_mm / 10.0 if _number(segment_distance_mm) else None, ' cm', 1)}")
        speed = (speed_mmps if speed_mmps is not None else self._speed_mmps) if odometry is not None else None
        lines.append(f"编码器估算速度：左轮 {_value(_at(speed, 0), ' mm/s', 1)}，"
                     f"右轮 {_value(_at(speed, 1), ' mm/s', 1)}，车体 {_value(_at(speed, 2), ' mm/s', 1)}")
        summary = getattr(self.link, "wheel_trace_summary", None)
        lines.append(summary() if callable(summary) else "轮速追踪：等待数据")
        yaw = getattr(self.link, "yaw_deg", None)
        error = (yaw - reference_yaw_deg + 180.0) % 360.0 - 180.0 if (
            _number(yaw) and _number(reference_yaw_deg)) else None
        lines.append(f"航向：{_value(yaw, '°', 1)}；参考：{_value(reference_yaw_deg, '°', 1)}；"
                     f"误差：{_value(error, '°', 1)}")
        diagnostics = getattr(self.link, "diagnostics", None)
        sample_age = getattr(diagnostics, "gyro_sample_age_ms", None)
        age_text = "不可用（65535）" if sample_age == 65535 else _value(sample_age, " ms")
        lines.append("陀螺仪：" + _status(getattr(diagnostics, "gyro_status", None), WireGyroStatus, _GYRO_LABELS)
                     + f"；raw Z={_value(getattr(diagnostics, 'gyro_raw_z', None), ' LSB')}；"
                     f"零偏={_value(getattr(diagnostics, 'gyro_bias_lsb', None), ' LSB')}；"
                     f"校准样本数={_value(getattr(diagnostics, 'gyro_calibration_count', None))}；"
                     f"DIAG生成时采样年龄={age_text}；DIAG uptime="
                     f"{_value(getattr(diagnostics, 'uptime_ms', None), ' ms')}")
        lines.append(f"阶段：{phase or '—'}；确认进度：{confirmation or '—'}；"
                     f"纠偏命令：v={_value(_at(command, 0), ' mm/s')}，"
                     f"w={_value(_at(command, 1), ' mrad/s')}；转向结果：{turn_result or '—'}")
        lines.append("=" * 64)
        self._updated = [False, False]
        self._pending_at = None
        self._last_validity = validity
        self._printed = True
        self._output_at = now
        return "\n".join(lines)
