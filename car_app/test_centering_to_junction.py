"""实车居中调参：普通道路持续前进，首个有效路口 BRAKE 并严格停稳。

与固定路线测试共用 route_test_settings；本入口只创建底盘链路，
不创建 RFID、音频、云台、相机或识别模型。桌面回归位于 car_app_tests。
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import math
import time

import config
import coverage
import mcu
import mission_ledger
import runlog
import test_motion_sequence as motion
import test_stage1_patrol as stage1
from route_test_settings import load_settings


class _JunctionReached(Exception):
    """Only raised after the saved first junction has a verified stationary stop."""


def _test_road():
    """Local control metadata, never a claim of arrival on the competition map."""
    start, end, road = "centering_origin", "centering_junction", "centering_road"
    field_map = coverage.FieldMap(
        nodes={start: coverage.MapNode(start, coverage.KIND_INTERSECTION, None, 0, 0),
               end: coverage.MapNode(end, coverage.KIND_INTERSECTION, None, 0, 1)},
        roads={road: coverage.MapRoad(road, start, end, False, 1.0)},
        start_node_id=start,
        adjacency={start: [(road, end)], end: [(road, start)]})
    return field_map, [coverage.Step(0, start, end, road, False)]


class CenteringToJunctionRunner(stage1.PatrolRunner):
    """Reuse the full ordinary-road controller and source-bound junction detector."""

    control_interval_s = .005

    def __init__(self, link, display, *, settings, input_source=None, deadline=math.inf):
        field_map, steps = _test_road()
        ledger = mission_ledger.MissionLedger(field_map)
        super().__init__(link, display, field_map, steps, ledger,
                         input_source=input_source, deadline=deadline, settings=settings)
        self.phase = "CRUISE"
        self.goal = self.cruise_speed_mmps
        # A physical turn is unnecessary in this dedicated straight-road test.
        # These flags select the inherited ordinary-road control paths only.
        self.first_actual_turn_completed = True
        self.departure_normal_turn_completed = True
        self._initial_complete = True
        self._test_origin_mm = tuple(link.odom_mm)
        self._test_started_at = time.monotonic()
        self._next_diagnostic_at = self._test_started_at
        self._max_abs_difference_mm = None
        self._junction_origin_mm = None
        self._junction_captured_at = None
        self.test_result = None

    def _set_step(self, *, after_turn=False):
        super()._set_step(after_turn=after_turn)
        self.phase, self.goal = "CRUISE", self.cruise_speed_mmps
        self.cruise_transition_pending = True
        self.display.controller.set_patrol_road_policy("ORDINARY")
        self.detector.reset(600., 1000.,
                            (self.link.session_nonce, self.link.boot_id), require_window=True)
        self.post_turn_sampler.start(self.display.controller, time.monotonic())
        self.display.input_speed_mmps = self.goal
        self._checkpoint()

    def _capture_junction(self, candidate):
        # next_action is None for this single local road: the inherited capture
        # writes BRAKE immediately and never takes the departure-advance path.
        if self._junction_origin_mm is None:
            self._junction_origin_mm = tuple(self.link.odom_mm)
            self._junction_captured_at = time.monotonic()
        super()._capture_junction(candidate)

    def _junction(self, candidate):
        if self.junction_evidence is None:
            raise mcu.McuError("居中调参停车缺少有效路口证据")
        if self._junction_origin_mm is None:
            self._capture_junction(candidate)
        # A recovery may re-enter with a saved semantic decision and no live
        # sample. Ensure a current BRAKE before using the stationary tracker.
        if stage1._brake_evidence_at(self.link) is None or any(self.display.wheel_targets):
            motion._apply_brake(self.link, self.display, self.deadline)
        stage1._ensure_stable(self.link, self.display, self.deadline,
                              input_source=self.input_source)
        if not motion._assert_still_stable(self.link, self.display):
            raise mcu.McuError("居中调参终点严格停稳证据失效")
        self.test_result = self.summary(stopped=True)
        self.test_result["junction_evidence"] = asdict(self.junction_evidence)
        self.display.action = "首个有效路口已BRAKE并严格停稳，居中调参结束"
        runlog.emit("centering_to_junction_stopped", self.display.action,
                    terminal=False, **self.test_result)
        raise _JunctionReached

    def _observe_post_turn_sampling(self, kind, sample, now, healthy):
        super()._observe_post_turn_sampling(kind, sample, now, healthy)
        if kind != "tick" or now + 1e-9 < self._next_diagnostic_at:
            return
        self._next_diagnostic_at = now + .1
        controller = self.display.controller
        difference = getattr(controller, "_last_raw_difference", None)
        if isinstance(difference, (int, float)) and math.isfinite(difference):
            absolute = abs(difference)
            self._max_abs_difference_mm = (absolute if self._max_abs_difference_mm is None
                                           else max(self._max_abs_difference_mm, absolute))
        runlog.emit("centering_to_junction_feedback", "普通道路居中调参反馈",
                    terminal=False, healthy=healthy, phase=self.phase,
                    requested_speed_mmps=self.goal, laser_difference_mm=difference,
                    heading_error_deg=controller._heading_error(),
                    wheel_positions_mm=self.link.odom_mm,
                    wheel_targets_mmps=self.display.wheel_targets,
                    road_position_mm=self.road_position_mm,
                    control_feedback=getattr(controller, "_control_feedback", None))

    def summary(self, *, stopped=False):
        positions = self.link.odom_mm
        valid_positions = (isinstance(positions, (tuple, list)) and len(positions) == 2
                           and all(isinstance(value, (int, float)) and not isinstance(value, bool)
                                   and math.isfinite(value) for value in positions))
        wheels = (tuple(a - b for a, b in zip(positions, self._test_origin_mm))
                  if valid_positions else None)
        tail = (None if self._junction_origin_mm is None or not valid_positions else
                tuple(a - b for a, b in zip(positions, self._junction_origin_mm)))
        return dict(stopped=bool(stopped), elapsed_s=time.monotonic() - self._test_started_at,
                    requested_speed_mmps=self.cruise_speed_mmps,
                    small_error_gain_per_s=self.settings.small_error_gain_per_s,
                    large_error_gain_per_s=self.settings.large_error_gain_per_s,
                    wheel_travel_mm=wheels,
                    mean_travel_mm=sum(wheels) / 2. if wheels is not None else None,
                    brake_tail_mm=tail,
                    laser_difference_mm=getattr(self.display.controller, "_last_raw_difference", None),
                    max_abs_laser_difference_mm=self._max_abs_difference_mm,
                    junction_captured_at=self._junction_captured_at,
                    stable_barrier=getattr(self.link, "_stable_barrier", None))

    def run(self):
        try:
            super().run()
        except _JunctionReached:
            return self.test_result
        raise mcu.McuError("居中调参未取得路口严格停车结果")


def _prepare(link, display, input_source):
    """The existing chassis-only preparation, without any peripheral service."""
    for name, operation in (("OPEN", link.open), ("HELLO", link.hello),
                            ("CLOCK", link.synchronize_clock),
                            ("STOP", lambda: link.send_stop(motion.StopReason.MISSION))):
        while True:
            motion._check_input_exit(input_source)
            try:
                operation()
                break
            except (mcu.McuError, OSError) as exc:
                runlog.exception("centering_startup_retry", "底盘启动失败，保持停车等待重试",
                                 exc, action=name)
                until = time.monotonic() + 1.
                while time.monotonic() < until:
                    motion._check_input_exit(input_source)
                    time.sleep(motion.CONTROL_INTERVAL_S)
    for operation in (
            lambda: motion._prepare(link, display,
                time.monotonic() + config.MCU_SELFCHECK_READY_TIMEOUT_S, input_source=input_source),
            lambda: stage1._brake(link, display, input_source=input_source),
            lambda: motion._sample_start(link, display, input_source=input_source)):
        while True:
            try:
                operation()
                break
            except mcu.McuError as exc:
                link._latch_hard_fault(str(exc))
                link.recover_fault(display, input_source=input_source)


def run_centering(*, settings=None, link=None, input_source=None, emitter=None):
    """Run on the vehicle; injected components support external desktop checks."""
    settings = load_settings() if settings is None else settings
    terminal = motion.AsyncTerminal() if emitter is None else None
    output = terminal if terminal is not None else emitter
    owned_input = input_source is None
    display = runner = None
    completed, reason = False, None
    try:
        if input_source is None:
            input_source = motion.TerminalSpeedInput(write_line=output)
        if link is None:
            link = stage1.PatrolLink(settings=settings)
        link.set_motion_deadline(None)
        link._patrol_input = input_source
        display = stage1.PatrolDisplay(link, emitter=output, settings=settings)
        output(f"居中调参：普通道路{settings.cruise_speed_mmps} mm/s；"
               f"小误差增益{settings.small_error_gain_per_s:g}，"
               f"大误差增益{settings.large_error_gain_per_s:g}；首个有效路口停车；q/Ctrl-C退出。")
        runlog.emit("centering_to_junction_settings", "居中调参与固定路线共用本次冻结参数",
                    terminal=False, settings=asdict(settings))
        _prepare(link, display, input_source)
        link.motion_checks_enabled = True
        runner = CenteringToJunctionRunner(link, display, settings=settings,
                                           input_source=input_source)
        result = runner.run()
        completed = bool(result["stopped"])
        output(f"居中调参结束：首个有效路口已严格停稳；"
               f"平均轮程{result['mean_travel_mm']:.1f} mm；"
               f"结束左右差{result['laser_difference_mm']} mm。")
    except (KeyboardInterrupt, EOFError, motion._UserQuit) as exc:
        reason = "用户中断：" + (str(exc) or type(exc).__name__)
        output("居中调参中断，尝试停车：" + reason)
    except Exception as exc:
        reason = f"{type(exc).__name__}: {exc}"
        runlog.exception("centering_to_junction_failed", "居中调参未完成", exc)
        output("居中调参未完成：" + reason)
    finally:
        if not completed and link is not None:
            try:
                motion._report_failure(link, display, reason or "未取得路口停车结果")
            except Exception as exc:
                runlog.exception("centering_stop_failed", "调参退出停车未完成确认", exc)
        try:
            summary = runner.summary(stopped=completed) if runner is not None else dict(stopped=False)
            runlog.emit("centering_to_junction_summary", "居中调参结束结果",
                        terminal=False, reason=reason, **summary)
        except Exception as exc:
            runlog.exception("centering_summary_failed", "调参结束诊断保存失败", exc)
        finally:
            try:
                if link is not None:
                    link._patrol_input = None
                    link._patrol_display = None
                    link.shutdown()
            except Exception as exc:
                runlog.exception("centering_close_failed", "调参退出关闭底盘失败", exc)
                completed = False
            finally:
                if owned_input and input_source is not None:
                    input_source.close()
                if terminal is not None:
                    terminal.close()
    return completed


def main_cli(argv=None):
    parser = argparse.ArgumentParser(description="实车普通道路居中调参：首个有效路口刹车并严格停稳")
    parser.parse_args(argv)
    with runlog.RunLog("centering_to_junction", terminal=motion.RawTerminalStream(),
                       periodic_status=False):
        return 0 if run_centering() else 1


if __name__ == "__main__":
    raise SystemExit(main_cli())
