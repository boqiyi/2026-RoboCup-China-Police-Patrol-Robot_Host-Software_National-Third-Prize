"""Three continuous 10-second wheel-speed steps on the RDK.

Run: python3 -u test_wheel_speed.py [--port /dev/ttyUSB1]
This entrypoint sends only v=the requested speed, w=0. The MCU retains its
existing speed/heading control. No camera, laser or navigation gate is used.
"""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import ExitStack
import csv
from dataclasses import dataclass, field
import json
from pathlib import Path
import time

import config
import mcu
import runlog
from motion_speed_input import TerminalSpeedInput
from test_motion_sequence import MotionSequenceLink, EXPECTED_BUILD_ID
from drivers.protocol_v2 import (
    AckStatus, ControlFlag, ControlTarget, MessageType, StateFlag, StopReason,
    WireGimbalPose, WireSafetyStopReason, encode_control_target,
)

STAGE_SECONDS = 10.0
COMMAND_SECONDS = PRINT_SECONDS = 0.1
TRACE_WINDOW_SECONDS = 0.1
STATE_WINDOW_SECONDS = 0.5
# Exact conversion in the current v12 board_config.h / encoder.c.
MM_PER_COUNT = 48 * 3.1416 * 267 / (500 * 1470)
CSV_COLUMNS = (
    "host_monotonic_s", "stage", "stage_elapsed_s", "requested_mmps",
    "measurement_source", "window_s", "left_mmps", "right_mmps", "mean_mmps",
    "left_absolute_error_mmps", "right_absolute_error_mmps", "mean_absolute_error_mmps",
    "left_error_percent", "right_error_percent", "mean_error_percent",
    "measurement_source_end_monotonic_s", "measurement_source_age_s",
)


class TestCancelled(Exception):
    """Operator q/EOF; main performs the same bounded cleanup as Ctrl-C."""


class WheelSpeedTestLink(MotionSequenceLink):
    """Reuse v12 identity and command evidence, without motion-sequence gates."""

    def _start_capture(self):
        mcu.McuLink._start_capture(self)
        self._capture_call("update_metadata", expected_build_id=EXPECTED_BUILD_ID,
                           test="three_wheel_speed_steps", stage_seconds=STAGE_SECONDS,
                           mm_per_encoder_count=MM_PER_COUNT, angular_mrads=0)

    def _handle(self, frame):
        if frame.message_type in (MessageType.DIRECTIONAL_RANGE4, MessageType.CULVERT_UPPER2):
            return
        super()._handle(frame)

    def _accept_state(self, state):
        super()._accept_state(state)
        if self.state is not state or not state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT:
            return
        record = self._command_ledger.get((MessageType.CONTROL_TARGET, state.applied_control_sequence))
        if record is not None and record[2] is state:
            if (state.target_linear_mms, state.target_angular_mrads) != record[5][1:3]:
                self._latch_hard_fault("应用STATE的速度字段与本次速度命令不一致")

    def _accept_wheel_trace(self, sample, frame_sequence=None):
        previous = self.wheel_trace
        mcu.McuLink._accept_wheel_trace(self, sample, frame_sequence)
        if self.wheel_trace is not sample or previous is sample:
            return
        source_at = self._last_trace_recv
        if self._clock_offset_s is not None and self._uptime_reference_raw is not None:
            delta = ((sample.uptime_ms - self._uptime_reference_raw + 0x80000000)
                     & 0xFFFFFFFF) - 0x80000000
            source_at = min(source_at, self._clock_offset_s
                            + (self._uptime_reference_ms + delta) / 1000)
        self._trace_events.append((sample, source_at))

    def send_motion(self, linear_mmps, angular_mradps=0, **_kwargs):
        if type(linear_mmps) is not int or not 31 <= linear_mmps <= 310 or angular_mradps != 0:
            raise ValueError("速度测试只接受31～310 mm/s整数、角速度0")
        if not self.connected or self._hard_fault_reason is not None:
            raise mcu.McuError(self._hard_fault_reason or "STM32会话未建立")
        self._check_motion_lease()
        sent_at = time.monotonic()
        baseline = getattr(self.state, "uptime_ms", None)
        target = ControlTarget(self.session_nonce, ControlFlag.MOTION_VALID,
                               linear_mmps, 0, 0, WireGimbalPose.UNKNOWN, 0, 0)
        seq = self._send(MessageType.CONTROL_TARGET, encode_control_target(target))
        self._active_motion_sent_at = sent_at
        self._track_command(MessageType.CONTROL_TARGET, seq, sent_at, baseline,
                            (MessageType.CONTROL_TARGET, linear_mmps, 0))
        return seq


@dataclass
class Stage:
    number: int
    speed: int
    started_at: float
    commands: set = field(default_factory=set)
    trace_seconds: float = 0.0
    trace_distance: list = field(default_factory=lambda: [0.0, 0.0])
    state_seconds: float = 0.0
    state_distance: list = field(default_factory=lambda: [0.0, 0.0])
    ended_at: float | None = None
    intervals: dict = field(default_factory=lambda: {"trace": [], "state": []})

    @property
    def deadline(self):
        return self.started_at + STAGE_SECONDS

    def add(self, source, begin, end, distances, sequences, *, continuous=True):
        boundary = min(self.deadline, self.ended_at) if self.ended_at is not None else self.deadline
        if begin < self.started_at - 1e-9 or end > boundary + 1e-9:
            return
        if any(seq not in self.commands for seq in sequences):
            return
        duration = end - begin
        if duration <= 0:
            return
        name = "trace" if source == "WHEEL_TRACE" else "state"
        setattr(self, name + "_seconds", getattr(self, name + "_seconds") + duration)
        totals = getattr(self, name + "_distance")
        for index in (0, 1):
            totals[index] += distances[index]
        self.intervals[name].append((begin, end, tuple(distances), continuous))

    def _qualified_intervals(self, name, minimum_seconds):
        """Only runs supporting the source's complete speed window are usable."""
        qualified, group = [], []
        for interval in self.intervals[name]:
            if group and (not interval[3] or abs(interval[0] - group[-1][1]) > 1e-7):
                if group[-1][1] - group[0][0] >= minimum_seconds - 1e-7:
                    qualified.extend(group)
                group = []
            group.append(interval)
        if group and group[-1][1] - group[0][0] >= minimum_seconds - 1e-7:
            qualified.extend(group)
        return qualified

    def summary(self):
        trace = self._qualified_intervals("trace", TRACE_WINDOW_SECONDS)
        state = self._qualified_intervals("state", STATE_WINDOW_SECONDS)
        covered = [(begin, end) for begin, end, _distance, _continuous in trace]
        trace_seconds = sum(end - begin for begin, end in covered)
        state_seconds = 0.0
        distance = [sum(interval[2][i] for interval in trace) for i in (0, 1)]
        for begin, end, values, _continuous in state:
            pieces = [(begin, end)]
            for trace_begin, trace_end in covered:
                if trace_end <= begin or trace_begin >= end:
                    continue
                remaining = []
                for left, right in pieces:
                    if trace_end <= left or trace_begin >= right:
                        remaining.append((left, right))
                    else:
                        if left < trace_begin:
                            remaining.append((left, trace_begin))
                        if trace_end < right:
                            remaining.append((trace_end, right))
                pieces = remaining
            usable = sum(right - left for left, right in pieces)
            state_seconds += usable
            for i in (0, 1):
                # Same linear interpolation as the STATE500ms display fallback.
                distance[i] += values[i] * usable / (end - begin)
        duration = trace_seconds + state_seconds
        source = ("WHEEL_TRACE+STATE" if trace_seconds > 0 and state_seconds > 1e-7 else
                  "WHEEL_TRACE" if trace_seconds > 0 else "STATE" if state_seconds > 0 else "MISSING")
        speeds = [value / duration for value in distance] if duration > 0 else None
        mean = sum(speeds) / 2 if speeds is not None else None
        result = dict(stage=self.number, requested_mmps=self.speed, started_monotonic_s=self.started_at,
                    ended_monotonic_s=self.ended_at, measurement_source=source if duration else "MISSING",
                    valid_observed_seconds=duration, trace_observed_seconds=trace_seconds,
                    state_observed_seconds=state_seconds, trace_available_seconds=self.trace_seconds,
                    state_available_seconds=self.state_seconds, left_mmps=None if speeds is None else speeds[0],
                    right_mmps=None if speeds is None else speeds[1], mean_mmps=mean,
                    mean_absolute_error_mmps=None if mean is None else abs(mean - self.speed),
                    mean_error_percent=None if mean is None else (mean - self.speed) / self.speed * 100,
                    basis="连续轮迹满100ms优先；连续STATE满500ms补轮迹缺口，按源区间线性分配；"
                          "每个时间区间只累计一次，不包含切换边界或未知间隔，不判定通过")
        for index, name in enumerate(("left", "right")):
            value = None if speeds is None else speeds[index]
            result[name + "_absolute_error_mmps"] = None if value is None else abs(value - self.speed)
            result[name + "_error_percent"] = None if value is None else (value - self.speed) / self.speed * 100
        return result


class SpeedMeasurements:
    """Signed source-time measurements; missing samples never become zero."""

    def __init__(self):
        self.traces = deque()
        self.states = deque()
        self.last_trace = self.last_state = None
        self.stages = []
        self._published_keys = (None, None)
        self.last_measurement_at = None

    def observe_trace(self, sample, source_at):
        identity = (sample.session_nonce, sample.boot_id)
        previous = self.last_trace
        continuous = False
        if previous is not None:
            old, old_at = previous
            if identity == (old.session_nonce, old.boot_id):
                sequence_delta = (sample.sample_sequence - old.sample_sequence) & 0xFFFF
                uptime_delta = (sample.uptime_ms - old.uptime_ms) & 0xFFFFFFFF
                if not 0 < sequence_delta < 0x8000 or not 0 < uptime_delta < 0x80000000:
                    return
                continuous = sequence_delta == 1 and uptime_delta == sample.elapsed_ms
                if not continuous or source_at <= old_at:
                    self.traces.clear()
            else:
                self.traces.clear()
        elapsed = sample.elapsed_ms / 1000
        begin = source_at - elapsed
        distances = (sample.left_encoder_delta * MM_PER_COUNT,
                     sample.right_encoder_delta * MM_PER_COUNT)
        self.last_trace = sample, source_at
        self.traces.append((begin, source_at, *distances))
        while self.traces and source_at - self.traces[0][1] > TRACE_WINDOW_SECONDS + 1e-9:
            self.traces.popleft()
        for stage in self.stages:
            stage.add("WHEEL_TRACE", begin, source_at, distances, (sample.applied_control_sequence,),
                      continuous=continuous)

    def observe_state(self, sample, source_at):
        if not sample.state_flags & StateFlag.ODOM_VALID or sample.odom_status != 0:
            self.states.clear()
            self.last_state = None
            return
        previous = self.last_state
        if previous is not None:
            old, old_at = previous
            delta = (sample.uptime_ms - old.uptime_ms) & 0xFFFFFFFF
            same = (sample.session_nonce, sample.boot_id) == (old.session_nonce, old.boot_id)
            if same and not 0 < delta < 0x80000000:
                return
            if not same or delta > config.TELEMETRY_STALE_MS or source_at <= old_at:
                self.states.clear()
            else:
                distances = (sample.left_mm - old.left_mm, sample.right_mm - old.right_mm)
                for stage in self.stages:
                    stage.add("STATE", old_at, source_at, distances,
                              (old.applied_control_sequence, sample.applied_control_sequence))
        self.last_state = sample, source_at
        self.states.append((source_at, sample.left_mm, sample.right_mm))
        while len(self.states) > 2 and source_at - self.states[1][0] >= STATE_WINDOW_SECONDS - 1e-9:
            self.states.popleft()

    def snapshot(self, now):
        trace = self.last_trace[0] if self.last_trace is not None else None
        state = self.last_state[0] if self.last_state is not None else None
        trace_key = None if trace is None else (trace.session_nonce, trace.boot_id, trace.sample_sequence, trace.uptime_ms)
        state_key = None if state is None else (state.session_nonce, state.boot_id, state.uptime_ms)
        previous_keys, self._published_keys = self._published_keys, (trace_key, state_key)
        self.last_measurement_at = None
        if (trace_key != previous_keys[0] and self.traces and
                0 <= now - self.traces[-1][1] <= config.TELEMETRY_STALE_MS / 1000 + 1e-9):
            finish = self.traces[-1][1]
            begin = finish - TRACE_WINDOW_SECONDS
            if self.traces[0][0] <= begin + 1e-9:
                distances = [0.0, 0.0]
                coverage = 0.0
                for start, end, left, right in self.traces:
                    duration = max(0.0, end - max(start, begin))
                    fraction = duration / (end - start)
                    coverage += duration
                    distances[0] += left * fraction
                    distances[1] += right * fraction
                if abs(coverage - TRACE_WINDOW_SECONDS) <= 1e-7:
                    self.last_measurement_at = finish
                    return "WHEEL_TRACE", TRACE_WINDOW_SECONDS, tuple(v / coverage for v in distances)
        if (state_key != previous_keys[1] and len(self.states) >= 2 and
                0 <= now - self.states[-1][0] <= config.TELEMETRY_STALE_MS / 1000 + 1e-9):
            latest = self.states[-1]
            begin = latest[0] - STATE_WINDOW_SECONDS
            previous = self.states[0]
            if previous[0] <= begin + 1e-9:
                for current in list(self.states)[1:]:
                    if current[0] >= begin - 1e-9:
                        ratio = (begin - previous[0]) / (current[0] - previous[0])
                        baseline = [previous[i] + ratio * (current[i] - previous[i]) for i in (1, 2)]
                        self.last_measurement_at = latest[0]
                        return "STATE", STATE_WINDOW_SECONDS, tuple(
                            (latest[i] - baseline[i - 1]) / STATE_WINDOW_SECONDS for i in (1, 2))
                    previous = current
        return "MISSING", None, None


def speed_row(stage, measurement, now, *, source_end_at=None):
    source, window, speeds = measurement
    row = dict.fromkeys(CSV_COLUMNS)
    row.update(host_monotonic_s=now, stage=stage.number, stage_elapsed_s=max(0.0, now - stage.started_at),
               requested_mmps=stage.speed, measurement_source=source, window_s=window)
    if speeds is not None and source_end_at is not None:
        row.update(measurement_source_end_monotonic_s=source_end_at,
                   measurement_source_age_s=max(0.0, now - source_end_at))
    if speeds is not None:
        left, right = speeds
        for name, value in zip(("left", "right", "mean"), (left, right, (left + right) / 2)):
            row[name + "_mmps"] = value
            row[name + "_absolute_error_mmps"] = abs(value - stage.speed)
            row[name + "_error_percent"] = (value - stage.speed) / stage.speed * 100
    return row


def format_row(row):
    prefix = f"阶段{row['stage']}/3 {row['stage_elapsed_s']:.2f}/10s 目标L/R={row['requested_mmps']}/{row['requested_mmps']} mm/s"
    if row["measurement_source"] == "MISSING":
        return prefix + " 实测L/R/均速=缺测"
    values = "/".join(f"{row[name + '_mmps']:.1f}" for name in ("left", "right", "mean"))
    errors = "/".join(f"{row[name + '_absolute_error_mmps']:.1f}" for name in ("left", "right", "mean"))
    percents = "/".join(f"{row[name + '_error_percent']:+.1f}%" for name in ("left", "right", "mean"))
    origin = "轮迹100ms" if row["measurement_source"] == "WHEEL_TRACE" else "STATE500ms估算(备用)"
    return prefix + f" 实测L/R/均速={values} mm/s |绝对误差|={errors} mm/s 相对误差={percents} [{origin}]"


class WheelSpeedRun:
    def __init__(self, link, input_source, directory, *, write_line=print):
        self.link, self.input, self.write_line = link, input_source, write_line
        self.directory = Path(directory).absolute()
        self.csv_path = self.directory / "wheel-speed.csv"
        self.summary_path = self.directory / "summary.json"
        self.measurements = SpeedMeasurements()
        self.csv_writer = None
        self.next_print = float("inf")
        self.active_stage = None
        self._stop_sent = False

    def _cancel(self):
        try:
            line = self.input.poll()
        except EOFError as exc:
            raise TestCancelled("标准输入已关闭") from exc
        if line is not None and line.strip().lower() == "q":
            raise TestCancelled("操作员输入q")

    def _service(self):
        self._cancel()
        self.link.pump()
        for state, source_at in self.link.drain_state_samples():
            self.measurements.observe_state(state, source_at)
        for sample, source_at in self.link.drain_wheel_trace_samples():
            self.measurements.observe_trace(sample, source_at)
        self.link.drain_diagnostics_samples()
        self.link.drain_side_samples()
        if self.link._hard_fault_reason is not None:
            raise mcu.McuError(self.link._hard_fault_reason)
        if not self.link.connected:
            raise mcu.McuError("STM32连接丢失")
        if self.link.state is not None:
            if not self.link._sample_is_fresh("state"):
                raise mcu.McuError("STATE通信超龄")
            if self.link._active_fault_bits():
                raise mcu.McuError(self.link.motion_health_snapshot())
        diagnostics = self.link.diagnostics
        if diagnostics is not None and diagnostics.last_stop_reason not in (
                WireSafetyStopReason.NONE, WireSafetyStopReason.BOOT, WireSafetyStopReason.HOST):
            raise mcu.McuError(f"下位机故障停车：{diagnostics.last_stop_reason.name}")
        now = time.monotonic()
        if self.active_stage is not None and now >= self.next_print - 1e-9:
            row = speed_row(self.active_stage, self.measurements.snapshot(now), now,
                            source_end_at=self.measurements.last_measurement_at)
            self.csv_writer.writerow(row)
            self.write_line(format_row(row))
            self.next_print = now + PRINT_SECONDS

    def _send(self, speed, stage=None):
        self._cancel()
        seq = self.link.send_motion(speed, 0)
        if stage is not None:
            stage.commands.add(seq)
        return seq

    def _confirm(self, seq, commands=None):
        key = MessageType.CONTROL_TARGET, seq
        record = self.link._command_ledger[key]
        deadline = record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S
        next_send = record[0] + COMMAND_SECONDS
        commands = {seq} if commands is None else commands
        while True:
            # Same-target renewals keep the lease alive while the first command
            # awaits evidence. They do not restart its confirmation deadline or
            # substitute a later sequence for this stage's first application.
            if (key in self.link._confirmed_commands and record[2] is not None
                    and record[8] is not None and record[8].status is AckStatus.ACCEPTED
                    and max(record[3], record[9]) <= deadline + 1e-9):
                return record[2], record[3]
            self._service()
            if (key in self.link._confirmed_commands and record[2] is not None
                    and record[8] is not None and record[8].status is AckStatus.ACCEPTED
                    and max(record[3], record[9]) <= deadline + 1e-9):
                # Application arrival, rather than ACK arrival or loop wake-up,
                # starts the stage's ten seconds.
                return record[2], record[3]
            now = time.monotonic()
            if now >= deadline - 1e-9:
                reason = f"首条速度命令seq={seq}未在原确认窗口内取得自身ACK和应用STATE"
                self.link._latch_hard_fault(reason)
                raise mcu.McuError(reason)
            if now >= next_send - 1e-9:
                renewal = self._send(record[5][1])
                commands.add(renewal)
                next_send = self.link._command_ledger[(MessageType.CONTROL_TARGET, renewal)][0] + COMMAND_SECONDS
            time.sleep(min(0.005, max(0.0, deadline - time.monotonic())))

    def stop(self, reason=StopReason.MISSION):
        if self._stop_sent:
            return
        self._stop_sent = True
        try:
            self.link.send_stop(reason)
        except (Exception, KeyboardInterrupt) as exc:
            runlog.exception("wheel_speed_stop_failed", "STOP发送失败；停止续约并继续有界清理", exc)

    def run(self, speeds):
        if len(speeds) != 3 or any(type(v) is not int or not 31 <= v <= 310 for v in speeds):
            raise ValueError("必须给出三个31～310 mm/s整数")
        self.directory.mkdir(parents=True, exist_ok=True)
        complete = False
        outcome = "interrupted"
        self.write_line(f"本次日志目录：{self.directory}")
        try:
            with ExitStack() as cleanup:
                handle = cleanup.enter_context(self.csv_path.open("w", newline="", encoding="utf-8", buffering=32768))
                # STOP precedes the file flush/close on completion and exceptions.
                cleanup.callback(lambda: self.stop(StopReason.MISSION if complete else StopReason.FAULT))
                self.csv_writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
                self.csv_writer.writeheader()
                for index, speed in enumerate(speeds, 1):
                    seq = self._send(speed)
                    commands = {seq}
                    _state, applied_at = self._confirm(seq, commands)
                    stage = Stage(index, speed, applied_at, commands)
                    self.measurements.stages.append(stage)
                    self.active_stage = stage
                    self.next_print = time.monotonic()
                    next_send = max(self.link._command_ledger[(MessageType.CONTROL_TARGET, command)][0]
                                    for command in commands) + COMMAND_SECONDS
                    runlog.emit("wheel_speed_stage_start", f"阶段{index}已应用{speed} mm/s",
                                stage=index, requested_mmps=speed, applied_state_received_s=applied_at)
                    while time.monotonic() < stage.deadline - 1e-9:
                        self._service()
                        now = time.monotonic()
                        if now >= stage.deadline - 1e-9:
                            break
                        if now >= next_send - 1e-9:
                            renewal = self._send(speed, stage)
                            next_send = self.link._command_ledger[(MessageType.CONTROL_TARGET, renewal)][0] + COMMAND_SECONDS
                        time.sleep(min(0.005, max(0.0, stage.deadline - time.monotonic())))
                    stage.ended_at = stage.deadline
                    self.active_stage = None
                complete, outcome = True, "complete"
        except TestCancelled as exc:
            outcome = str(exc)
            raise
        except KeyboardInterrupt:
            outcome = "Ctrl-C"
            raise
        except Exception as exc:
            outcome = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            if self.active_stage is not None:
                self.active_stage.ended_at = min(time.monotonic(), self.active_stage.deadline)
            self.active_stage = None
            self.stop(StopReason.MISSION if complete else StopReason.FAULT)
            summaries = [stage.summary() for stage in self.measurements.stages]
            result = dict(complete=complete, outcome=outcome, speeds_mmps=list(speeds),
                          mm_per_encoder_count=MM_PER_COUNT, stages=summaries,
                          interpretation="编码器轮速均值不是独立地面测速；保留下位机既有航向及轮速控制")
            self.summary_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
            for item in summaries:
                values = "缺测" if item["mean_mmps"] is None else (
                    f"L={item['left_mmps']:.1f} R={item['right_mmps']:.1f} 均速={item['mean_mmps']:.1f} mm/s "
                    f"|绝对误差|L/R/均值={item['left_absolute_error_mmps']:.1f}/"
                    f"{item['right_absolute_error_mmps']:.1f}/{item['mean_absolute_error_mmps']:.1f} mm/s "
                    f"相对误差L/R/均值={item['left_error_percent']:+.1f}%/"
                    f"{item['right_error_percent']:+.1f}%/{item['mean_error_percent']:+.1f}%")
                self.write_line(f"阶段{item['stage']}汇总：目标={item['requested_mmps']} mm/s {values} "
                                f"有效源时间覆盖={item['valid_observed_seconds']:.3f}s [{item['measurement_source']}]")
            self.write_line(f"速度CSV：{self.csv_path}\n阶段汇总：{self.summary_path}")
        return result


def prompt_speeds(input_source, *, write_line=print):
    speeds = []
    while len(speeds) < 3:
        input_source.begin_request(f"请输入第{len(speeds) + 1}个速度（31～310 mm/s整数；q退出）：")
        try:
            while True:
                line = input_source.poll()
                if line is None:
                    time.sleep(0.02)
                    continue
                if line.strip().lower() == "q":
                    raise TestCancelled("操作员输入q")
                try:
                    value = int(line.strip())
                    if str(value) != line.strip() or not 31 <= value <= 310:
                        raise ValueError
                except ValueError:
                    write_line("请输入31～310的整数。")
                    break
                speeds.append(value)
                break
        finally:
            input_source.end_request()
    return speeds


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.MCU_PORT, help="STM32串口（默认配置中的MCU_PORT）")
    args = parser.parse_args(argv)
    link = input_source = runner = None
    with runlog.session("wheel_speed_test", periodic_status=False) as log:
        try:
            input_source = TerminalSpeedInput()
            speeds = prompt_speeds(input_source)
            link = WheelSpeedTestLink(port=args.port)
            link.open()
            link.hello()
            runner = WheelSpeedRun(link, input_source, log.directory)
            runner.run(speeds)
            return 0
        except (TestCancelled, EOFError, KeyboardInterrupt) as exc:
            print(f"测试已结束：{str(exc) or 'Ctrl-C'}", flush=True)
            return 0
        except Exception as exc:
            runlog.exception("wheel_speed_error", "轮速测试故障结束", exc)
            return 1
        finally:
            try:
                if link is not None:
                    try:
                        if runner is not None:
                            runner.stop(StopReason.FAULT)
                        else:
                            link.send_stop(StopReason.FAULT)
                        link.receive_shutdown_tail()
                    except (Exception, KeyboardInterrupt) as exc:
                        runlog.exception("wheel_speed_cleanup_error", "停车后有界接收中断，继续关闭", exc)
                    finally:
                        link.close()
            finally:
                if input_source is not None:
                    input_source.close()


if __name__ == "__main__":
    raise SystemExit(main())
