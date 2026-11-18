"""正式比赛协调器：一份路线、一份位置、一个串口写入门禁。

阶段入口保持独立。本模块复用阶段一导航操作、阶段二采集/障碍服务及
阶段三停车扫描操作，不运行它们的交互测试循环。
"""
from __future__ import annotations

from collections import deque
from contextlib import contextmanager, ExitStack
from dataclasses import asdict, dataclass, replace
import math
import multiprocessing
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import clues
import config
import coverage
import detector
import gimbal
import mcu
import mission_ledger
import runlog
import test_motion_sequence as motion
import test_stage1_patrol as stage1
import test_stage2_obstacle as stage2
from drivers.protocol_v2 import StateFlag, WireGimbalPose
from competition_link import CompetitionLink, CompetitionMotionBlocked
from competition_planner import PlanRequest, PlanResult, PlanningError, solve_plan
from competition_services import CompetitionAudio, CompetitionRfidReader, CompetitionServices
from competition_vision import (
    CompetitionScanner, CompetitionVision, RoadContext, UpperSelector,
    VisionUnavailable, FRONT, SCAN, TUNNEL,
    PAN_FORWARD_US, TILT_US, SCAN_SETTLE_S,
)


TUNNEL_ROADS = frozenset(("r07", "r14", "r21", "r28"))
STOP_TIMEOUT_S = 5.


class DisabledCompetitionVision:
    """Explicitly disabled images: no camera, detector, worker or recovery token."""

    approach_active = False

    def __init__(self):
        self._condition = threading.RLock()
        self.camera = SimpleNamespace(last_error=None)
        self.context = None
        self.mode = FRONT

    def start(self):
        pass

    def close(self):
        pass

    def set_mode(self, mode):
        self.mode = mode

    def set_context(self, context, *, preserve_approach=False):
        self.context = context

    def poll(self):
        return SimpleNamespace(ready=None, disabled=True)

    def approach_speed(self, requested_speed):
        return requested_speed

    def peek_obstacle(self):
        return None

    peek_obstacle_fact = peek_obstacle
    take_obstacle = peek_obstacle

    def motion_health_reason(self, *args, **kwargs):
        return None


def build_fixed_route(field_map, actions):
    """Resolve manual relative actions against the real map before any device opens."""
    actions = tuple(actions)
    if not actions:
        raise PlanningError("固定路线没有路口动作，禁止发车")
    start = field_map.start_node_id
    first = field_map.adjacency.get(start, ())
    if len(first) != 1:
        raise PlanningError("固定路线要求出发区恰有一条出路，禁止发车")
    road_id, node = first[0]
    steps = [coverage.Step(0, start, node, road_id, field_map.roads[road_id].dark_tunnel)]
    previous = start
    for number, action in enumerate(actions, 1):
        if action not in (coverage.TURN_LEFT, coverage.TURN_RIGHT, coverage.TURN_STRAIGHT):
            raise PlanningError(f"固定路线第{number}个动作非法：{action!r}，禁止发车")
        heading = coverage.direction_between(field_map, previous, node)
        choices = [(rid, other) for rid, other in field_map.adjacency[node]
                   if other != previous and coverage.turn_kind(heading,
                       coverage.direction_between(field_map, node, other)) == action]
        if len(choices) != 1:
            raise PlanningError(f"固定路线第{number}个动作{action}在{node}有{len(choices)}条匹配道路，禁止发车")
        rid, other = choices[0]
        steps.append(coverage.Step(number, node, other, rid, field_map.roads[rid].dark_tunnel))
        previous, node = node, other
        if node == start and number != len(actions):
            raise PlanningError(f"固定路线第{number}个动作提前回到出发区，后续动作不可执行，禁止发车")
    if node != start:
        raise PlanningError(f"固定路线第{len(actions)}个动作后位于{node}，未返回出发区，禁止发车")
    return tuple(steps)


class _TaskRequired(Exception):
    def __init__(self, kind):
        self.kind = kind
        super().__init__(kind)


class ObstacleLocationError(RuntimeError):
    """A parked business result, never an MCU recovery request."""


@dataclass(frozen=True)
class ObstacleLocationEvidence:
    at: float
    context: object
    step: object
    side_sample: object
    junction_evidence: object = None
    post_turn: bool = False
    returning: bool = False
    road_position_mm: float | None = None
    junction_position_mm: float | None = None
    axis_deg: float | None = None
    junction_odom_mm: tuple | None = None
    node_probe: bool = False


@dataclass(frozen=True)
class RoadPosition:
    road_id: str
    from_node: str
    to_node: str
    last_confirmed_node: str
    position_mm: float
    axis_deg: float | None
    identity: tuple[int, int]
    leg_id: int
    phase: str
    captured_at: float


@dataclass(frozen=True)
class RetreatPrefix:
    road_id: str
    entrance: str
    opposite: str
    stopped_position_mm: float
    heading: tuple[int, int]
    identity: tuple[int, int]
    leg_id: int
    blocked_road_id: str | None = None


@dataclass(frozen=True)
class RetreatArrival:
    prefix: RetreatPrefix
    identity: tuple[int, int]
    stable_barrier: float
    wheel_positions_mm: tuple[int, int]


@dataclass(frozen=True)
class CompletePlan:
    request: PlanRequest
    result: PlanResult
    retreat: RetreatPrefix | None = None

    @property
    def distance_mm(self):
        return self.result.distance_mm + (self.retreat.stopped_position_mm
                                          if self.retreat is not None else 0.)


@dataclass
class _ObstacleTask:
    event: object
    stage: str = "STOP"
    upper_event: object = None
    node_only: bool = False
    short_node_return: bool = False
    reference_deg: float | None = None


@dataclass(frozen=True)
class ObstacleDisposition:
    """Map ownership resolved from the confirmed frame's frozen evidence."""
    event: object
    kind: str
    node: str | None = None
    heading: tuple[int, int] | None = None
    blocked_step: object = None
    incoming_step: object = None
    junction_evidence: object = None
    road_origin_mm: float | None = None
    reason: str | None = None


@dataclass
class _NodeObstacleTask:
    disposition: ObstacleDisposition
    stage: str = "STOP"
    upper_event: object = None
    steps: tuple = ()
    index: int = 0
    action: str | None = None


@dataclass
class _RetreatInstallTask:
    prefix: RetreatPrefix
    steps: tuple
    index: int
    action: str
    stage: str = "TURN"


def _plan_child(connection, field_map, request):
    """Spawn target imports no entrypoint and never opens a device."""
    try:
        result = solve_plan(field_map, start_node=request.start_node,
                            completed_roads=request.completed_roads,
                            blocked_roads=request.blocked_roads,
                            heading=request.heading, home_node=request.home_node)
        connection.send((True, result))
    except BaseException as exc:
        connection.send((False, (type(exc).__name__, str(exc))))
    finally:
        connection.close()


def validate_complete_plan(field_map, complete):
    from competition_planner import validate_result
    validate_result(field_map, complete.request, complete.result)
    prefix = complete.retreat
    if prefix is not None:
        road = field_map.roads.get(prefix.road_id)
        if (road is None or {prefix.entrance, prefix.opposite} != {road.a, road.b}
                or (prefix.blocked_road_id or prefix.road_id) not in complete.request.blocked_roads
                or (prefix.blocked_road_id is not None and prefix.blocked_road_id != prefix.road_id
                    and prefix.road_id not in complete.request.completed_roads)
                or prefix.entrance != complete.request.start_node
                or prefix.heading != coverage.direction_between(field_map, prefix.opposite, prefix.entrance)
                or prefix.heading != complete.request.heading
                or not math.isfinite(prefix.stopped_position_mm) or prefix.stopped_position_mm < 0):
            raise PlanningError("封路撤回前缀不合法，禁止发车")
    return complete


class PlanningProcess:
    """Parent remains parked and responsive until exact proof and validation."""
    def __init__(self, context=None):
        self.context = context or multiprocessing.get_context("spawn")
        self.process = self.connection = None

    def solve(self, field_map, request, poll_hook):
        parent, child = self.context.Pipe(duplex=False)
        self.connection = parent
        self.process = self.context.Process(target=_plan_child, args=(child, field_map, request),
                                            name="competition-exact-planner", daemon=True)
        try:
            self.process.start()
            child.close()
            while True:
                poll_hook()
                if parent.poll():
                    ok, value = parent.recv()
                    if not ok:
                        raise PlanningError(f"精确规划失败：{value[0]}: {value[1]}")
                    if not isinstance(value, PlanResult):
                        raise PlanningError("规划进程没有返回完整结果")
                    return value
                if not self.process.is_alive():
                    raise PlanningError(f"规划进程异常退出：{self.process.exitcode}")
                time.sleep(motion.CONTROL_INTERVAL_S)
        finally:
            child.close()
            self.close()

    def close(self):
        process, self.process = self.process, None
        if process is not None and process.pid is not None:
            if process.is_alive():
                process.terminate()
            process.join(timeout=1.)
            if process.is_alive():
                process.kill()
                process.join(timeout=1.)
        if self.connection is not None:
            self.connection.close()
            self.connection = None


class CompetitionDisplay(stage1.PatrolDisplay):
    def __init__(self, link, emitter=None, *, settings=None):
        super().__init__(link, emitter=emitter, **({"settings": settings} if settings is not None else {}))
        self.coordinator = None

    def tick(self, **kwargs):
        if self.coordinator is None:
            return super().tick(**kwargs)
        # Detection commits stop facts independently. Only this control owner
        # binds their location after all admitted UART samples are processed.
        self.coordinator._processing_uart_batch = True
        firmware_fault = False
        try:
            super().tick(**kwargs)
        except mcu.McuError:
            firmware_fault = True
            raise
        finally:
            try:
                if (not firmware_fault and getattr(self.coordinator, "yolo_obstacles", False)
                        and self.coordinator._peek_obstacle_fact() is not None
                        and any(self.wheel_targets)):
                    motion._apply_brake(self.link, self, math.inf)
                samples = getattr(self, "_samples", ())
                for sample in samples:
                    self.coordinator.record_obstacle_location(sample)
                if not samples:
                    self.coordinator.record_obstacle_location()
            finally:
                self.coordinator._processing_uart_batch = False
            if not firmware_fault:
                self.coordinator.collect_events()
        self.coordinator.poll()


class CompetitionRunner(stage1.PatrolRunner):
    """Stage 1 continuous controller with explicit competition route transitions."""
    control_interval_s = .005

    def __init__(self, *args, coordinator, **kwargs):
        super().__init__(*args, **kwargs)
        self.coordinator = coordinator

    def _target_speed_goal(self):
        requested = super()._target_speed_goal()
        effective = self.coordinator.vision.approach_speed(requested)
        self.display.controller.obstacle_approach_speed_cap = (
            effective if getattr(self.coordinator.vision, "approach_active", False) else None)
        previous = getattr(self, "_competition_effective_goal", None)
        if previous is not None and effective > previous:
            self.display.controller.request_cruise_speed_transition(effective)
        self._competition_effective_goal = effective
        return effective

    def _use_start_heading_control(self, now):
        return super()._use_start_heading_control(now)

    def _capture_junction(self, candidate):
        # Freeze qualified source evidence before super writes BRAKE or a task
        # interrupts the runner and removes its sampling observer.
        c = self.coordinator
        # Save the qualified junction before obstacle priority interrupts the
        # action; its source evidence must survive the rest of this UART batch.
        self.pending_candidate = candidate
        if self.junction_evidence is not None:
            self.pending_junction_decision = dict(step=self.step.index, action=self.next_action,
                                                 evidence=asdict(self.junction_evidence))
        c.record_obstacle_location(candidate, junction_evidence=self.junction_evidence)
        fact = c._peek_obstacle_fact()
        if c.pending_obstacle is not None or fact is not None:
            # The stop fact has already been published. Neither its location
            # binding nor a busy visual observer may delay this first BRAKE.
            if any(self.display.wheel_targets):
                motion._apply_brake(self.link, self.display, self.deadline)
            raise _TaskRequired("obstacle")
        # Save the first-decision odometry, or write ordinary-junction BRAKE,
        # before trying any camera publication lock.
        super()._capture_junction(candidate)
        if not getattr(c, "_processing_uart_batch", False):
            c.collect_events()
        if c.pending_obstacle is not None or c._peek_obstacle() is not None:
            raise _TaskRequired("obstacle")
        if self.next_action in (coverage.TURN_LEFT, coverage.TURN_RIGHT, coverage.TURN_UTURN):
            c.begin_navigation_turn()
            c.mode = "JUNCTION_ADVANCE" if self._departure_turn_pending() else "NODE_TURN"

    @property
    def home_leg(self):
        return (self.index == len(self.steps) - 1
                and self.step.to_node == self.field_map.start_node_id
                and not self._midroad_returning)

    def _set_step(self, *, after_turn=False):
        if self.phase == "HOME" and not self.home_leg:
            self.phase, self.goal = "DEPARTURE", self.cruise_speed_mmps
        super()._set_step(after_turn=after_turn)
        self.coordinator.enter_road(self)
        if (after_turn and self.coordinator.distance_profile is not None
                and getattr(self.coordinator, "vision_enabled", True)):
            self.coordinator._post_turn_barrier = max(
                getattr(self.link, "post_turn_barrier", None) or self.link._stable_barrier,
                getattr(self.coordinator.vision, "_front_pose_barrier", 0.))

    def _confirm_road(self):
        if self.step.road_id in self.ledger.blocked_roads:
            raise mcu.McuError("封闭道路撤回不得计为整条道路完成")
        c = self.coordinator
        c.collect_events()
        with c._vision_control_read():
            if c.pending_obstacle is not None or c._peek_obstacle() is not None:
                raise _TaskRequired("obstacle")
            committed = self._commit_road_arrival()
            c._publish_context_locked(None)
        c.pending_upper = None
        if committed is not None:
            self._finish_road_arrival(committed)

    def _junction(self, candidate):
        if self._midroad_returning:
            self._finish_midroad_return()
            return
        c = self.coordinator
        # The stopped collector checks obstacle evidence once more before the
        # graph/node commit. Upper business events never delay a saved turn.
        c.mode = "NODE_TURN"
        c.collect_events()
        if c.pending_obstacle is not None:
            raise _TaskRequired("obstacle")
        c._post_turn_node_heading = coverage.direction_between(
            self.field_map, self.step.from_node, self.step.to_node)
        if candidate is not None and self.next_action != coverage.TURN_STRAIGHT and any(self.display.wheel_targets):
            self._capture_junction(candidate)
        # Parking may have added a fresh opening after the first route evidence.
        openings = motion._pending_junction_openings(self.link)
        if openings and self.junction_evidence.identity == (self.link.session_nonce, self.link.boot_id):
            self.junction_evidence = replace(self.junction_evidence,
                openings=tuple(sorted(set(self.junction_evidence.openings) | openings)))
        c._post_turn_node_evidence = self.junction_evidence
        self.sampler.cancel()
        self.post_turn_sampler.cancel()
        self._post_turn_pending_states.clear()
        self.post_turn_low_speed = self.cruise_transition_pending = False
        self.display.controller._patrol_pending_cruise_goal = None
        action = self.next_action
        departure_pivot = (self._departure_turn_pending()
                           and self.departure_strategy == "40mm_pivot")
        side = ("left" if action in (coverage.TURN_LEFT, coverage.TURN_UTURN)
                else "right" if action == coverage.TURN_RIGHT else None)
        self.link.store_checkpoint(junction_evidence=asdict(self.junction_evidence),
                                   saved_openings=self.junction_evidence.openings,
                                   planned_side=side,
                                   planned_angle_deg=180 if action == coverage.TURN_UTURN else 90 if side else 0)
        if action is None:
            c.park()
            self._confirm_road()
            self.ledger.route_plan_finished = True
            c.finished = True
            c.mode = "FINISHED"
            raise _TaskRequired("finished")
        next_home = (self.index + 1 == len(self.steps) - 1
                     and self.steps[self.index + 1].to_node == self.field_map.start_node_id)
        departure_speed = self.home_speed_mmps if next_home else self.cruise_speed_mmps
        c._check_node_action(self.step.to_node, c._post_turn_node_heading,
            self.steps[self.index + 1], action, self.junction_evidence.openings,
            incoming=self.step, check_opening=getattr(c, "settings", None) is None)
        if action == coverage.TURN_STRAIGHT:
            evidence = asdict(self.junction_evidence)
            inherited = self.display.controller.formal_reference_deg
            self.goal = self._straight_passage_speed_goal(departure_speed)
            self.alignment_goal = self.cruise_speed_mmps
            if getattr(self.link, "_stable_barrier", None) is not None and not any(self.display.command):
                self._begin_passage(evidence["confirmed_at"], stopped_opening=candidate,
                                    saved_passage=candidate is None)
            else:
                self._begin_passage(evidence["confirmed_at"], moving_opening=candidate,
                                    saved_passage=candidate is None)
            self._confirm_road()
            if inherited is not None:
                controller = self.display.controller
                controller._road_reference = controller._hold_heading = inherited
                controller._patrol_cruise_hold_goal = inherited
                controller._patrol_cruise_hold_sources = None
            self.index += 1
            self.phase = "HOME" if next_home else "DEPARTURE"
            self._set_step()
            runlog.emit("stage1_straight_passage", "直穿临时继承旧参考；新双墙后按本道路规则重新采样",
                        step=self.step.index, goal_mmps=self.goal,
                        source_barrier=self.passage_barrier,
                        fixed_reference_deg=self.display.controller.heading_reference_deg)
            c.mode = "DRIVE"
            return
        if side is None:
            raise mcu.McuError("比赛路线包含未授权节点动作")
        # Recovery can re-enter with a saved candidate instead of capturing it
        # again. Keep detection suppressed throughout the remaining action.
        c.begin_navigation_turn()
        if self._departure_turn_pending():
            c.mode = "JUNCTION_ADVANCE"
            self._advance_departure_junction()
        c.mode = "NODE_TURN"
        self.response.pause(self.link, time.monotonic())
        self.alignment_goal = (self.cruise_speed_mmps if getattr(self, "settings", None) is not None
                               else stage1.POST_TURN_SPEED_MMPS)
        openings = stage1._confirm_junction(self.link, self.display, candidate,
                                            input_source=self.input_source)
        if getattr(c, "settings", None) is not None:
            # Route-test openings can arrive during advance, braking or the extra wait.
            stage1._wait_junction_turn_ready(self.link, self.display, input_source=self.input_source)
            openings = motion._pending_junction_openings(self.link) or openings
            self.junction_evidence = replace(self.junction_evidence, openings=tuple(sorted(openings)))
            c._post_turn_node_evidence = self.junction_evidence
            self.link.store_checkpoint(junction_evidence=asdict(self.junction_evidence),
                                       saved_openings=self.junction_evidence.openings)
        c.collect_events()
        if c.pending_obstacle is not None:
            raise _TaskRequired("obstacle")
        c._check_node_action(self.step.to_node, c._post_turn_node_heading,
            self.steps[self.index + 1], action, openings, incoming=self.step)
        self._confirm_road()
        camera_turn = ((getattr(c, "settings", None) is not None
                        and action in (coverage.TURN_LEFT, coverage.TURN_RIGHT))
                       or c.yolo_obstacles and (action == coverage.TURN_UTURN or departure_pivot))
        camera_key = (self.step.index, self.step.road_id, self.step.to_node, action)
        camera_completed = camera_turn and getattr(self, "_camera_turn_completed", None) == camera_key
        if camera_turn and not camera_completed:
            stage1._ensure_stable(self.link, self.display, input_source=self.input_source)
            c.set_camera_pwm(2500, c.camera_tilt_us)
        self._coordinate_mode = "PIVOT" if action == coverage.TURN_UTURN else "TRANSIT"
        c.turn_authorized = True
        try:
            if not camera_completed:
                stage1._turn_planned(self.link, self.display, openings, side=side,
                                    turn_speed_mmps=self.turn_speed_mmps,
                                    angle_deg=180 if action == coverage.TURN_UTURN else 90,
                                    departure_speed_mmps=departure_speed,
                                    input_source=self.input_source,
                                    pivot=departure_pivot,
                                    goal_reference_deg=self._short_road_reference(self.steps[self.index + 1]))
        finally:
            c.turn_authorized = False
        if camera_turn:
            self._camera_turn_completed = camera_key
            stage1._ensure_stable(self.link, self.display, input_source=self.input_source)
            c.set_camera_pwm(PAN_FORWARD_US, c.camera_tilt_us)
        self.first_actual_turn_completed = True
        if action in (coverage.TURN_LEFT, coverage.TURN_RIGHT):
            self.departure_normal_turn_completed = True
        self.index += 1
        self.phase = "HOME" if next_home else "DEPARTURE"
        self.goal = departure_speed
        self._set_step(after_turn=True)
        c.finish_navigation_turn()
        self._begin_passage(self.link.post_turn_barrier, after_turn=True)
        self.sent_at, self.next_send = None, time.monotonic()
        self.response.pause(self.link, self.next_send)
        c.mode = "DRIVE"

    def _finish_midroad_return(self):
        if self.junction_evidence is not None:
            self.link.store_checkpoint(junction_evidence=asdict(self.junction_evidence),
                                       saved_openings=self.junction_evidence.openings)
        self.coordinator.park()
        self.ledger.confirm_rollback(self.step.from_node)
        self.coordinator.confirm_retreat_arrival()
        self._midroad_return_wait = True
        self.pending_candidate = None
        self.armed = False
        self.coordinator.leave_road()
        raise _TaskRequired("retreat_arrived")

    def _finish_home(self):
        c = self.coordinator
        self.sampler.cancel()
        self.response.pause(self.link, time.monotonic())
        before = self.link.odom_mm
        c.park()
        self._measure_brake(before, self.home_speed_mmps)
        c.collect_events()
        result = self._arrival("home")
        if not self.display.controller.heading_ready(tolerance_deg=5.):
            raise mcu.McuError("回出发区已停稳，但返回道路航向未达标")
        with c._vision_control_read():
            if c.pending_obstacle is not None or c._peek_obstacle() is not None:
                raise _TaskRequired("obstacle")
            if c.pending_upper is not None:
                raise _TaskRequired("culvert")
            self.ledger.confirm_home_arrival(self.step, result)
            self.home_result = result
            self.ledger.route_plan_finished = True
            self.ledger.stage1_home_stop_confirmed = True
            c._publish_context_locked(None)
        c.pending_upper = None
        c.finished = c.returned = True
        c.mode = "FINISHED"
        runlog.emit("competition_home_confirmed", "沿用阶段一返家里程、航向及严格停稳确认",
                    stopped_distance_mm=result.odom_distance_mm, target_mm=stage1.HOME_TRAVEL_MM,
                    brake_tail_mm=self.display.braking_tail_mm)

    def _refresh_sampling_identity(self):
        super()._refresh_sampling_identity()
        self.coordinator.refresh_identity()


class CompetitionCoordinator:
    yolo_obstacles = False
    camera_tilt_us = TILT_US

    def __init__(self, components, *, field_map=None, input_source=None, emitter=print,
                 planner=None, vision=None, distance_profile=None, stage3_enabled=True,
                 evidence_writer=None, departure_strategy="10mm_turn", yolo_obstacles=False,
                 settings=None, vision_enabled=True):
        self.c = components
        self.settings = settings
        self.vision_enabled = bool(vision_enabled) and settings is None
        self.distance_profile = distance_profile
        self.stage3_enabled = bool(stage3_enabled) and self.vision_enabled
        self.yolo_obstacles = bool(yolo_obstacles) and self.vision_enabled
        self.camera_tilt_us = 700 if self.yolo_obstacles or settings is not None else TILT_US
        if departure_strategy not in ("10mm_turn", "40mm_pivot"):
            raise ValueError("departure_strategy must be 10mm_turn or 40mm_pivot")
        self.departure_strategy = departure_strategy
        self._desired_pwm = (PAN_FORWARD_US, self.camera_tilt_us)
        self._confirmed_pwm_identity = None
        self._pose_barrier = 0.
        self._post_turn_barrier = None
        self._post_turn_node_probe_active = False
        self._node_departed_at = None
        self._road_entry_pending = False
        self._road_entry_context = None
        self._road_entry_pairs = []
        self._post_turn_node_evidence = None
        self._post_turn_node_heading = None
        self.obstacle_disposition = None
        self._node_obstacle_task = None
        self._route_conflict = None
        self.evidence_writer = evidence_writer
        self.field_map = field_map or coverage.load_map()
        self._fixed_steps = build_fixed_route(self.field_map, settings.actions) if settings is not None else None
        self.route_authorized = False
        self._authorized_fixed_plan = None
        self.link = components["mcu"]
        if isinstance(self.link, mcu.McuLink) and not isinstance(self.link, CompetitionLink):
            raise mcu.McuError("正式比赛必须使用CompetitionLink的v12及实际串口写入门禁")
        self.input_source, self.output = input_source, emitter
        self.deadline = None
        self.mode = "STARTUP"
        self.ledger = mission_ledger.MissionLedger(self.field_map)
        self.ledger.culvert_completed_roads = set()
        self.display = CompetitionDisplay(self.link, emitter=emitter, settings=settings)
        self.display.coordinator = self
        self.link._patrol_input = input_source
        self.services = components.get("services") or CompetitionServices(
            components, self.ledger, display=emitter, stage3_enabled=self.stage3_enabled)
        self.display.services = self.services
        self.planner = planner or PlanningProcess()
        self.vision = (vision or components.get("vision") or CompetitionVision(
            resolve_context=self.context_at, preview_enabled=False,
            distance_profile=distance_profile, evidence_writer=evidence_writer,
            yolo_obstacles=self.yolo_obstacles)) if self.vision_enabled else DisabledCompetitionVision()
        self.vision.resolve_context = self.context_at
        self.vision.resolve_obstacle_location = self.obstacle_location_at
        self.upper = UpperSelector(resolve_context=self.context_at)
        self.scanner = CompetitionScanner(self.vision, components.get("gimbal"), components.get("detector"),
                                          confidence=.8,
                                          audio_callback=self.announce_target,
                                          pose_callback=self.set_camera_pwm)
        self.runner = None
        self.plan = None
        self.retreat = None
        self.retreat_arrival = None
        self._obstacle_task = self._retreat_install_task = None
        self.pending_obstacle = self.pending_upper = None
        self.context = None
        self.epochs = deque(maxlen=2048)
        self._obstacle_location_history = deque(maxlen=512)
        startup_step = coverage.Step(-1, self.field_map.start_node_id,
                                     self.field_map.start_node_id, "startup", False)
        self.rfid_epochs = deque(((float("-inf"), None, startup_step),), maxlen=2048)
        self.leg_id = 0
        self.turn_authorized = False
        self.finished = self.returned = False
        self._stop_wait_since = None
        self._last_wait_message = None
        self._next_recovery = 0.
        self._next_yolo_prepare = 0.
        self._detector_ready = False
        self._vision_permission = None
        self.visual_gap_roads = set()
        self.services.rfid_context_at = self.rfid_context_at
        self.link.competition_motion_guard = self.motion_guard
        self.link.competition_motion_context = self.motion_write_context
        self.link.competition_before_motion_write = self.revoke_camera_stationary
        self.link.competition_motion_sent = self.motion_sent
        self.link.patrol_recovery_callback = self._restore_recovered_pwm
        self.link.set_motion_deadline(None)

    def _restore_recovered_pwm(self):
        self.set_camera_pwm(*getattr(self, "_desired_pwm", (PAN_FORWARD_US, TILT_US)), restore=True)

    def set_camera_pwm(self, pan, tilt, *, restore=False):
        # Save intended pose before UART work. A failed/recovered side scan may
        # never silently change to a different wall or front-facing position.
        if getattr(self, "yolo_obstacles", False) or getattr(self, "settings", None) is not None:
            tilt = self.camera_tilt_us
        self._desired_pwm = (pan, tilt)
        identity = self.link.session_nonce, self.link.boot_id
        state = self.link.state
        fresh = getattr(self.link, "_sample_is_fresh", None)
        still_enabled = (state is not None and callable(fresh) and fresh("state")
            and (state.session_nonce, state.boot_id) == identity
            and state.state_flags & StateFlag.GIMBAL_PRESENT
            and state.state_flags & StateFlag.GIMBAL_PWM_ENABLED
            and state.gimbal_pose == WireGimbalPose.LEVEL_CENTER)
        if (still_enabled and getattr(self, "_confirmed_pwm_identity", None) == (identity, pan, tilt)):
            return self._pose_barrier
        output = self.c.get("gimbal")
        if output is None:
            raise VisionUnavailable("云台组件尚未恢复")
        self.vision.set_mode(SCAN)
        output.set_pwm(pan, tilt)
        self._confirmed_pwm_identity = identity, pan, tilt
        self._pose_barrier = time.monotonic() + SCAN_SETTLE_S
        if (pan, tilt) == (PAN_FORWARD_US, getattr(self, "camera_tilt_us", TILT_US)):
            front_ready = getattr(self.vision, "front_pose_ready", None)
            if callable(front_ready):
                front_ready(self._pose_barrier)
            else:
                self.vision.set_mode(FRONT)
        return self._pose_barrier

    def position(self):
        r = self.runner
        if r is None or not r._initialized:
            return None
        return RoadPosition(r.step.road_id, r.step.from_node, r.step.to_node,
                            self.ledger.last_confirmed_node, r.road_position_mm, r.road_axis_deg,
                            (self.link.session_nonce, self.link.boot_id), self.leg_id,
                            r.phase, time.monotonic())

    def context_at(self, source_at):
        for start, end, context, _step in reversed(self.epochs):
            if start < source_at and (end is None or source_at <= end):
                return context
        return None

    def record_obstacle_location(self, sample=None, *, at=None, junction_evidence=None):
        """Keep immutable evidence before side queues and road origins reset."""
        if not getattr(self, "vision_enabled", True):
            return
        if self.runner is None or self.context is None:
            return
        r = self.runner
        if sample is None:
            sample = getattr(self.link, "_latest_side_sample", None)
            if sample is None:
                try:
                    sample = motion._side_sample(self.link)
                except (mcu.McuError, AttributeError, TypeError, ValueError, IndexError):
                    pass
        # Measurement time, not tick completion, decides whether a frame can
        # use this observation. A delayed UART batch must not become future
        # evidence merely because it was processed after camera capture.
        measured = max(sample.source_times) if sample is not None else time.monotonic()
        at = max(self.context.source_barrier, measured) if at is None else at
        if (not getattr(self, "_post_turn_node_probe_active", False)
                or getattr(self, "yolo_obstacles", False)):
            at = max(at, getattr(self, "_node_departed_at", None) or 0.)
        if junction_evidence is not None:
            at = max(at, junction_evidence.confirmed_at)
        saved = (junction_evidence is not None or getattr(r, "pending_candidate", None) is not None
                 or getattr(r, "pending_junction_decision", None) is not None)
        junction = (junction_evidence or getattr(r, "junction_evidence", None)) if saved else None
        if junction is not None and getattr(junction, "confirmed_at", math.inf) > at:
            junction = None
        returning = getattr(r, "_midroad_returning", False)
        entry_now = time.monotonic()
        if (self.yolo_obstacles and self._road_entry_pending and self._node_departed_at is not None
                and sample is not None and sample.dual_wall and sample.source_keys is not None
                and sample.source_identity == self.context.identity
                and all(source <= min(at, entry_now)+1e-9 for source in sample.source_times)):
            previous = self._road_entry_pairs[-1] if self._road_entry_pairs else None
            if previous is not None and previous.source_identity != self.context.identity:
                self._road_entry_pairs = []
                previous = None
            if motion._fresh_post_stop_pair(sample, entry_now, self._node_departed_at, previous):
                self._road_entry_pairs.append(sample)
            if len(self._road_entry_pairs) >= 2:
                self._road_entry_pending = self._post_turn_node_probe_active = False
        post_turn = not saved and not returning and (
            self._post_turn_barrier is not None
            or getattr(self, "_post_turn_node_probe_active", False) or (
                not getattr(r, "armed", True) and getattr(r, "phase", None) == "DEPARTURE"
                and self.ledger.last_confirmed_node == r.step.from_node))
        if self.yolo_obstacles:
            post_turn = not saved and not returning and self._road_entry_pending
        if post_turn:
            previous = self._post_turn_node_evidence
            before = self._post_turn_node_heading
            if previous is not None and before is not None:
                original_identity = tuple(previous.identity)
                current_identity = tuple(self.context.identity)
                if (original_identity != current_identity
                        and original_identity[1] == current_identity[1]
                        and original_identity in getattr(self.link, "_patrol_coordinate_identities", ())):
                    # Publish a migrated copy, leaving old source-time snapshots
                    # and the original confirmed node evidence immutable.
                    previous = replace(previous, identity=current_identity)
                heading = coverage.direction_between(self.field_map, r.step.from_node, r.step.to_node)
                openings = set()
                for _road, other in self.field_map.adjacency[r.step.from_node]:
                    direction = coverage.direction_between(self.field_map, r.step.from_node, other)
                    original = coverage.turn_kind(before, direction)
                    relative = coverage.turn_kind(heading, direction)
                    if (original in previous.openings or original == coverage.TURN_UTURN
                            or original == coverage.TURN_STRAIGHT):
                        if relative in (coverage.TURN_LEFT, coverage.TURN_RIGHT):
                            openings.add(relative)
                junction = replace(previous, openings=tuple(sorted(openings)))
        lookup = getattr(r, "_position_at", None)
        origin = (lookup(junction.confirmed_at) if junction is not None
                  and getattr(junction, "confirmed_at", None) is not None and callable(lookup)
                  else None)
        wheel_origin = None
        if junction is not None and getattr(junction, "confirmed_at", None) is not None:
            progress = motion._RoadProgress()
            progress._history = deque(getattr(getattr(r, "progress", None), "_history", ()), maxlen=512)
            if progress.establish(junction.confirmed_at):
                wheel_origin = progress.origin_mm
        evidence = ObstacleLocationEvidence(at, self.context, r.step, sample, junction,
            post_turn, returning,
            getattr(r, "road_position_mm", None), origin, getattr(r, "road_axis_deg", None), wheel_origin,
            node_probe=post_turn and (getattr(self, "_post_turn_node_probe_active", False)
                                     or getattr(self, "_post_turn_barrier", None) is not None
                                     or self.yolo_obstacles and self._road_entry_pending))
        self._obstacle_location_history.append(evidence)

    def obstacle_location_at(self, source_at):
        """Control-owned history lookup using the event's original source."""
        context = self.context_at(source_at)
        eligible = (evidence for evidence in reversed(tuple(self._obstacle_location_history))
                    if evidence.at <= source_at and evidence.context == context)
        return max(eligible, key=lambda item: item.at, default=None)

    def rfid_context_at(self, source_at):
        for start, end, step in reversed(self.rfid_epochs):
            if start < source_at and (end is None or source_at <= end):
                distance = self.runner._position_at(source_at) if self.runner and step == self.runner.step else None
                return step, distance
        return None, None

    def _publish_context(self, context, step=None, *, preserve_approach=False):
        with self._vision_control_read():
            self._publish_context_locked(context, step, preserve_approach=preserve_approach)
        self.record_obstacle_location()

    def _publish_context_locked(self, context, step=None, *, preserve_approach=False):
        # All fallible vision gates run before coordinator epoch mutation.
        if preserve_approach:
            self.vision.set_context(context, preserve_approach=True)
        else:
            self.vision.set_context(context)
        self.upper.set_context(context)
        now = time.monotonic()
        if self.epochs and self.epochs[-1][1] is None:
            start, _, old, old_step = self.epochs[-1]
            self.epochs[-1] = start, now, old, old_step
        self.context = context
        if context is not None:
            self.epochs.append((now, None, context, step))

    @contextmanager
    def _vision_control_read(self):
        if not self.vision._condition.acquire(timeout=.005):
            raise motion._CenteringProposalExpired("比赛视觉发布繁忙，继续接收UART后重新计算目标")
        try:
            yield
        finally:
            self.vision._condition.release()

    def enter_road(self, runner, *, reverse=False, preserve_approach=False):
        next_leg_id = self.leg_id + 1
        step = runner.step
        a, b = (step.to_node, step.from_node) if reverse else (step.from_node, step.to_node)
        context = RoadContext(step.road_id, a, b, (self.link.session_nonce, self.link.boot_id),
                              dark_tunnel=step.road_id in TUNNEL_ROADS,
                              leg_id=next_leg_id, source_barrier=time.monotonic())
        entry = (step.road_id, a, b)
        if self.yolo_obstacles and entry != self._road_entry_context:
            self._road_entry_context = entry
            self._road_entry_pending = bool(not reverse and self._post_turn_node_evidence is not None
                and self.ledger.last_confirmed_node == step.from_node)
            self._road_entry_pairs = []
            self._node_departed_at = None
        self._publish_context(context, step, preserve_approach=preserve_approach)
        self.leg_id = next_leg_id
        now = time.monotonic()
        if self.rfid_epochs and self.rfid_epochs[-1][1] is None:
            start, _, previous = self.rfid_epochs[-1]
            self.rfid_epochs[-1] = start, now, previous
        self.rfid_epochs.append((now, None, step))
        self.pending_upper = None
        runlog.emit("competition_road_context", "道路身份由确认节点和已完成转向推进",
                    context=asdict(context), position_mm=runner.road_position_mm,
                    closed=step.road_id in self.ledger.blocked_roads,
                    retreat_authorized=self.retreat is not None)

    def leave_road(self):
        self._publish_context(None)
        self.pending_upper = None

    def _check_node_action(self, node, heading, outgoing, action, openings, *, incoming=None, check_opening=True):
        if not self.yolo_obstacles and getattr(self, "settings", None) is None:
            return
        if self._route_conflict is not None:
            raise _TaskRequired("route_conflict")
        reason, map_action = None, None
        road = self.field_map.roads.get(outgoing.road_id)
        if (node not in self.field_map.nodes or outgoing.from_node != node
                or road is None or {outgoing.from_node, outgoing.to_node} != {road.a, road.b}
                or outgoing.road_id in self.ledger.blocked_roads):
            reason = "节点与下一道路的地图端点或通路不一致"
        elif heading not in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            reason = "节点缺少可信的有向来向"
        elif incoming is not None:
            before = self.field_map.roads.get(incoming.road_id)
            if (before is None or incoming.to_node != node
                    or {incoming.from_node, incoming.to_node} != {before.a, before.b}
                    or heading != coverage.direction_between(self.field_map, incoming.from_node, node)):
                reason = "来路与节点或冻结来向不一致"
        if reason is None:
            map_action = coverage.turn_kind(heading, coverage.direction_between(
                self.field_map, node, outgoing.to_node))
            if action != map_action:
                reason = "计划转向与当前有向地图转换不一致"
            elif check_opening and action in (coverage.TURN_LEFT, coverage.TURN_RIGHT) and action not in openings:
                reason = "计划90度转向侧没有本节点保存的开口证据"
        if reason is not None:
            self._route_conflict = dict(reason=reason, node=node, heading=heading,
                incoming=None if incoming is None else asdict(incoming), outgoing=asdict(outgoing),
                planned_action=action, map_action=map_action, observed_openings=sorted(openings),
                required_side=action if action in (coverage.TURN_LEFT, coverage.TURN_RIGHT) else None,
                identity=(self.link.session_nonce, self.link.boot_id))
            raise _TaskRequired("route_conflict")

    def _node_evidence_after_heading(self, evidence, node, before, heading):
        """Rotate a saved node decision without inventing a new measurement."""
        if evidence is None or before is None:
            raise ObstacleLocationError("原节点缺少已确认方向证据，保持停车")
        identity = self.link.session_nonce, self.link.boot_id
        if (evidence.identity != identity
                and evidence.identity not in getattr(self.link, "_patrol_coordinate_identities", ())):
            raise ObstacleLocationError("原节点证据身份改变，保持停车")
        openings = set()
        for road, other in self.field_map.adjacency[node]:
            if road in self.ledger.blocked_roads:
                continue
            direction = coverage.direction_between(self.field_map, node, other)
            old = coverage.turn_kind(before, direction)
            new = coverage.turn_kind(heading, direction)
            if (old in evidence.openings or old in (coverage.TURN_STRAIGHT, coverage.TURN_UTURN)):
                if new in (coverage.TURN_LEFT, coverage.TURN_RIGHT):
                    openings.add(new)
        return replace(evidence, openings=tuple(sorted(openings)))

    def _save_node_evidence(self, evidence):
        # Old left/right latches belong to the old body heading. A saved proof
        # retains its original identity/time; it is never re-labelled fresh.
        motion._clear_junction(self.link)
        self.runner.junction_evidence = evidence
        self.link.store_checkpoint(junction_evidence=asdict(evidence),
                                   saved_openings=evidence.openings)

    def begin_navigation_turn(self):
        """Freeze old front observations; committed obstacle facts stay queued."""
        self._post_turn_node_probe_active = False
        begin = getattr(self.vision, "begin_navigation_turn", None)
        if callable(begin):
            begin()
        else:
            self.vision.set_mode(SCAN)
        self.pending_upper = None

    def finish_navigation_turn(self):
        """The new road and corrected heading must exist before front detection."""
        if not getattr(self, "vision_enabled", True):
            self._post_turn_barrier = None
            self._post_turn_node_probe_active = False
            return
        barrier = max(getattr(self, "_post_turn_barrier", None) or 0.,
                      getattr(self.link, "_stable_barrier", None) or 0.,
                      getattr(self, "_pose_barrier", 0.), time.monotonic())
        self._post_turn_barrier = barrier
        self._post_turn_node_probe_active = (not self.yolo_obstacles or self._road_entry_pending)
        self._node_departed_at = None
        finish = getattr(self.vision, "finish_navigation_turn", None)
        if callable(finish):
            finish(barrier)
        else:
            ready = getattr(self.vision, "front_pose_ready", None)
            if callable(ready):
                ready(barrier)
            else:
                self.vision.set_mode(FRONT)
        self.record_obstacle_location(at=barrier)

    def refresh_identity(self):
        if self.retreat is not None:
            identity = self.link.session_nonce, self.link.boot_id
            if self.retreat.identity != identity:
                migrated = self.retreat.identity in getattr(self.link, "_patrol_coordinate_identities", ())
                if not migrated or self.retreat.identity[1] != identity[1]:
                    raise mcu.McuError("撤回会话改变但道路坐标未经核验")
                self.retreat = replace(self.retreat, identity=identity)
                if self.plan is not None and self.plan.retreat is not None:
                    self.plan = replace(self.plan, retreat=self.retreat)
                proof = getattr(self, "retreat_arrival", None)
                if proof is not None:
                    self.retreat_arrival = replace(proof, prefix=self.retreat, identity=identity,
                                                    stable_barrier=self.link._stable_barrier)
        if self.runner is None or self.context is None:
            return
        committed_upper = self.pending_upper
        self.enter_road(self.runner, reverse=self.runner._midroad_returning,
                        preserve_approach=True)
        if (committed_upper is not None and committed_upper.road_id == self.context.road_id
                and (committed_upper.context.from_node, committed_upper.context.to_node)
                == (self.context.from_node, self.context.to_node)
                and (committed_upper.identity == self.context.identity
                     or committed_upper.identity in getattr(self.link, "_patrol_coordinate_identities", ()))):
            self.pending_upper = committed_upper
        if self.mode == "SCAN" or getattr(self, "_desired_pwm", (PAN_FORWARD_US, TILT_US)) != (PAN_FORWARD_US, self.camera_tilt_us):
            self.vision.set_mode(SCAN)

    def _peek_obstacle(self):
        peek = getattr(self.vision, "peek_obstacle", None)
        return peek() if callable(peek) else getattr(self.vision, "_obstacle_event", None)

    def _peek_obstacle_fact(self):
        peek = getattr(self.vision, "peek_obstacle_fact", None)
        return peek() if callable(peek) else None

    def _has_route_authorization(self):
        if getattr(self, "settings", None) is not None:
            return bool(self.route_authorized and self.plan is self._authorized_fixed_plan
                        and self.plan is not None and self.plan.result.steps == self._fixed_steps)
        return self.plan is not None and self.plan.result.optimal

    def motion_guard(self):
        """Pure reads under the vision publication lock, at the UART boundary."""
        if getattr(self, "_route_conflict", None) is not None:
            return False
        if getattr(self, "settings", None) is not None and not self._has_route_authorization():
            return False
        if self.mode == "JUNCTION_ADVANCE":
            intent = getattr(self.link, "competition_write_intent", None)
            authorization = getattr(self.link, "_patrol_exclusive_motion", None) or {}
            r, context = self.runner, self.context
            if (intent is None or intent[0] is not motion.MessageType.CONTROL_TARGET
                    or authorization.get("kind") != "departure_advance"
                    or r is None or context is None
                    or context.identity != (self.link.session_nonce, self.link.boot_id)
                    or not self._has_route_authorization()
                    or self.pending_obstacle is not None or self._peek_obstacle() is not None
                    or self.services.fault_reason is not None):
                return False
            command = mcu.decode_control_target(intent[1])
            valid = getattr(self.link, "patrol_exclusive_motion_valid", None)
            return bool(callable(valid) and valid(command.linear_mms, command.angular_mrads))
        if self.mode in ("PIVOT", "NODE_TURN"):
            intent = getattr(self.link, "competition_write_intent", None)
            if not self.turn_authorized or self.context is not None or intent is None:
                return False
            kind, payload = intent
            if (self.mode == "NODE_TURN" and self.runner is not None
                    and getattr(self.runner, "_midroad_returning", False)
                    and self.retreat_arrival is None):
                return False
            if self.mode == "PIVOT" and kind is motion.MessageType.CONTROL_TARGET:
                if not self.yolo_obstacles:
                    return False
                turn = getattr(self.link, "patrol_checkpoint", {}).get("turn") or {}
                authorization = getattr(self.link, "_patrol_exclusive_motion", None) or {}
                command = mcu.decode_control_target(payload)
                valid = getattr(self.link, "patrol_exclusive_motion_valid", None)
                return bool(self.yolo_obstacles and turn.get("absolute_uturn")
                    and turn.get("uturn_action_completed") and turn.get("pivot")
                    and turn.get("correction") and turn.get("angle_deg") == 180
                    and authorization.get("kind") == "departure_pivot"
                    and authorization.get("checkpoint") is turn
                    and command.linear_mms == 0 and abs(command.angular_mrads) == 1456
                    and callable(valid) and valid(command.linear_mms, command.angular_mrads))
            if self.mode == "NODE_TURN" and kind is motion.MessageType.CONTROL_TARGET:
                authorization = getattr(self.link, "_patrol_exclusive_motion", None) or {}
                if authorization.get("kind") != "departure_pivot":
                    return False
                command = mcu.decode_control_target(payload)
                valid = getattr(self.link, "patrol_exclusive_motion_valid", None)
                return bool(callable(valid) and valid(command.linear_mms, command.angular_mrads))
            if kind is not motion.MessageType.MOTION_ACTION:
                return False
            action = mcu.decode_motion_action(payload)
            allowed = ((motion.MotionActionCode.UTURN_LEFT,) if self.mode == "PIVOT" else
                       (motion.MotionActionCode.TURN_LEFT, motion.MotionActionCode.TURN_RIGHT,
                        motion.MotionActionCode.UTURN_LEFT))
            expected_speed = getattr(self.link, "authorized_turn_speed_mmps", 91)
            return action.action in allowed and action.speed_mms == expected_speed
        if self.mode != "DRIVE" or not self._has_route_authorization():
            return False
        if (self.pending_obstacle is not None or self.pending_upper is not None
                or self.services.fault_reason is not None):
            return False
        if self.yolo_obstacles and getattr(self.vision, "yolo_recovery_needed", False):
            return False
        if self._peek_obstacle() is not None:
            return False
        r, context = self.runner, self.context
        if r is None or context is None or context.identity != (self.link.session_nonce, self.link.boot_id):
            return False
        if r.pending_candidate is not None and r.next_action != coverage.TURN_STRAIGHT:
            return False
        intent = getattr(self.link, "competition_write_intent", None)
        if (getattr(self.vision, "approach_active", False) and intent is not None
                and intent[0] is motion.MessageType.CONTROL_TARGET):
            command = mcu.decode_control_target(intent[1])
            cap = min(r.goal, 100 if getattr(self, "yolo_obstacles", False)
                      else stage2.STAGE2_OBSTACLE_APPROACH_SPEED_MMPS)
            if command.linear_mms > cap:
                raise motion._CenteringProposalExpired(f"障碍接近已限速{cap}，重算旧高速目标")
        if getattr(self, "_post_turn_barrier", None) is not None:
            return False
        if (getattr(self, "stage3_enabled", True) and not context.dark_tunnel
                and context.road_id not in self.scanner.completed_roads
                and self.upper.peek_event(self.link) is not None):
            return False
        if getattr(r, "_midroad_returning", False) or context.road_id in self.ledger.blocked_roads:
            prefix = self.retreat
            return bool(prefix and r._midroad_returning and prefix.road_id == context.road_id
                        and (not self.yolo_obstacles or (
                            prefix.road_id == r.step.road_id
                            and (prefix.opposite, prefix.entrance) == (context.from_node, context.to_node)
                            and (r.step.from_node, r.step.to_node) == (prefix.entrance, prefix.opposite)))
                        and prefix.entrance == context.to_node
                        and prefix.identity == context.identity)
        return True

    def motion_sent(self):
        if self.mode == "DRIVE":
            contract = getattr(self.runner, "_retreat_contract", None)
            if contract is not None and contract.get("motion_at") is None:
                contract["motion_at"] = time.monotonic()
            if (getattr(self, "yolo_obstacles", False) and self._road_entry_pending
                    and self._node_departed_at is None):
                self._node_departed_at = time.monotonic()
            elif (not getattr(self, "yolo_obstacles", False)
                    and getattr(self, "_post_turn_node_probe_active", False)):
                self._post_turn_node_probe_active = False
                self._node_departed_at = time.monotonic()
        arm = getattr(self.vision.camera, "arm_motion", None)
        if callable(arm):
            arm()
        self.revoke_camera_stationary()
        self._stop_wait_since = None

    def revoke_camera_stationary(self):
        grant = getattr(self.vision.camera, "authorize_stationary", None)
        if callable(grant):
            grant(False)

    @contextmanager
    def motion_write_context(self):
        if self.mode in ("PIVOT", "NODE_TURN"):
            # The committed obstacle owns this gyroscope turn. Camera recovery
            # and publication cannot delay its actual command or renewal.
            yield
            return
        with self._vision_control_read():
            # Firmware and obstacle permissions are still checked at write.
            # A camera owner or USB recovery may not veto chassis movement.
            yield

    def _vision_drive_reason(self):
        check = getattr(self.vision, "motion_health_reason", None)
        if callable(check):
            return check(getattr(self, "_vision_permission", None))
        return getattr(self.vision.camera, "last_error", None)

    def _prepare_yolo(self, poll_hook=None):
        prepare = getattr(self.vision, "prepare_yolo", None)
        if (not self.yolo_obstacles or not callable(prepare)
                or not getattr(self.vision, "yolo_recovery_needed", True)):
            return True
        if time.monotonic() < self._next_yolo_prepare:
            return False
        self._vision_permission = None
        try:
            ready = prepare(poll_hook=poll_hook)
        finally:
            self._next_yolo_prepare = time.monotonic() + 1.
        if not ready:
            self.wait_message(getattr(self.vision, "yolo_prepare_reason", None)
                              or "YOLO加载或预热尚未完成")
        return bool(ready)

    def _wait_camera_ready(self):
        if not getattr(self, "vision_enabled", True):
            return
        self.mode = "WAIT_PERIPHERAL"
        while True:
            try:
                self.parked_poll()
                self.collect_events()
                if self.pending_obstacle is not None:
                    return
                if not self._prepare_yolo(poll_hook=self.parked_poll):
                    time.sleep(motion.CONTROL_INTERVAL_S)
                    continue
                snapshot = self.vision.poll()
                barrier = self.link._stable_barrier
                if self.context is not None:
                    barrier = max(barrier, self.context.source_barrier)
                token = self.vision.recovery_token(barrier)
                if token is not None:
                    self._vision_permission = token
                    self.collect_events()
                    runlog.emit("competition_camera_recovered", "停后两张当前代次成功新帧，继续原任务",
                                source_barrier=barrier, token=token)
                    return
                self.wait_message(getattr(snapshot, "startup_reason", None)
                                  or "等待停后两张当前代次的成功新帧")
                time.sleep(motion.CONTROL_INTERVAL_S)
            except mcu.McuError as exc:
                self.recover_mcu(exc)
                self.mode = "WAIT_PERIPHERAL"

    def collect_events(self):
        if getattr(self, "_processing_uart_batch", False):
            return
        fact = self._peek_obstacle_fact()
        if fact is not None and getattr(self, "_bound_obstacle_fact", None) is not fact:
            self._accept_obstacle_event(fact)
            self._bound_obstacle_fact = fact
        try:
            snapshot = self.vision.poll()
        except motion._CenteringProposalExpired:
            if self.pending_obstacle is not None:
                return
            raise
        if (self.context is not None and not self.context.dark_tunnel
                and getattr(snapshot, "ready", None) is False):
            gaps = getattr(self, "visual_gap_roads", None)
            if gaps is None:
                self.visual_gap_roads = gaps = set()
            gaps.add(self.context.road_id)
        try:
            event = self.vision.take_obstacle()
        except motion._CenteringProposalExpired:
            if self.pending_obstacle is not None:
                return
            raise
        if event is not None:
            if getattr(self, "_bound_obstacle_fact", None) is not event:
                self._accept_obstacle_event(event)
        if not getattr(self, "stage3_enabled", True):
            self.pending_upper = None
            return
        self.upper.observe(self.link)
        event = self.upper.take_event()
        if event is not None and self.context is not None and event.context == self.context:
            self.pending_upper = self.pending_upper or event
            observe = getattr(self.scanner, "observe_road", None)
            if observe is not None:
                observe(event.road_id)

    def _accept_obstacle_event(self, event):
        if getattr(self, "yolo_obstacles", False) and any(self.display.wheel_targets):
            motion._apply_brake(self.link, self.display, math.inf)
        bind = getattr(self.vision, "bind_obstacle_location", None)
        if callable(bind):
            event = bind(event, self.obstacle_location_at)
        self.pending_obstacle = self.pending_obstacle or event
        if self.pending_obstacle is event and (
                getattr(self, "distance_profile", None) is not None
                or getattr(self, "_post_turn_barrier", None) is not None
                or isinstance(event.location_evidence, ObstacleLocationEvidence)):
            self.obstacle_disposition = self._freeze_obstacle_disposition(event)
        context = self.context
        same_road = (context is not None and event.road_id == context.road_id
                     and (event.context.from_node, event.context.to_node)
                     == (context.from_node, context.to_node))
        identity_verified = (context is not None and (event.context.identity == context.identity
                            or event.context.identity in getattr(self.link, "_patrol_coordinate_identities", ())))
        if not same_road or not identity_verified:
            if getattr(self, "distance_profile", None) is not None:
                self.park()
                raise ObstacleLocationError("已确认障碍的源道路或身份与当前上下文不一致，停车退出")
            self.link.patrol_manual_required = "已确认障碍的原道路与当前道路不一致，停车等待位置核验"
            raise mcu.McuError(self.link.patrol_manual_required)

    def _freeze_obstacle_disposition(self, event):
        evidence = getattr(event, "location_evidence", None)
        def unresolved(reason):
            runlog.emit("competition_obstacle_assignment_ambiguous", reason,
                        source_at=event.source_at, context=asdict(event.context))
            return ObstacleDisposition(event, "ambiguous", reason=reason)
        if (not isinstance(evidence, ObstacleLocationEvidence)
                or evidence.context != event.context or evidence.at > event.source_at
                or evidence.step is None):
            return unresolved("障碍确认帧缺少匹配的源时刻位置证据，停车退出")
        step, junction = evidence.step, evidence.junction_evidence
        saved = (not evidence.post_turn and junction is not None and junction.identity == event.context.identity
                 and junction.confirmed_at <= event.source_at)
        known_node = (evidence.post_turn and junction is not None
                      and junction.identity == event.context.identity and junction.confirmed_at <= event.source_at)
        yolo_crossing = False
        if self.yolo_obstacles and not evidence.returning:
            context = event.context
            if (not context.active or step != self.runner.step
                    or (step.road_id, step.from_node, step.to_node)
                    != (context.road_id, context.from_node, context.to_node)
                    or not all(math.isfinite(v) for v in (context.source_barrier, evidence.at, event.source_at))
                    or not context.source_barrier <= evidence.at <= event.source_at
                    or not (context.identity == (self.link.session_nonce, self.link.boot_id)
                            or context.identity in getattr(self.link, "_patrol_coordinate_identities", ()))):
                return unresolved("障碍确认帧的活动道路、源时刻或身份无法核验，停车退出")
            if junction is not None and (junction.identity != context.identity
                    or not math.isfinite(junction.confirmed_at) or junction.confirmed_at > event.source_at):
                return unresolved("障碍确认帧的路口身份或源时刻无法核验，停车退出")
            yolo_crossing = saved and self.runner.next_action == coverage.TURN_STRAIGHT
            if not yolo_crossing:
                # Bottom-band confirmation occurs inside the road; old node
                # probe flags cannot override the already committed route step.
                runlog.emit("competition_obstacle_assignment", "按确认帧冻结的当前道路上下文归属障碍",
                            source_at=event.source_at, source_context=asdict(context),
                            kind="road", blocked_road=step.road_id,
                            location_basis="frozen_road_context")
                return ObstacleDisposition(event, "road", blocked_step=step)
        dual = self._location_dual_valid(evidence, event.source_at)
        if self.yolo_obstacles and (saved or evidence.node_probe):
            dual = False
        # Old wall sources from before the opening are not newly entered walls.
        if dual and saved:
            dual = all(source > junction.confirmed_at + 1e-9
                       for source in evidence.side_sample.source_times)
        node_probe = bool(evidence.node_probe)
        if node_probe and (not known_node or evidence.returning):
            return unresolved("转后前向确认缺少匹配身份的已确认节点，停车退出")
        if dual and not node_probe and (not saved or evidence.returning):
            return ObstacleDisposition(event, "road", blocked_step=step)
        if evidence.returning:
            return unresolved("撤回途中障碍缺少可靠道路位置，不能认定已回入口节点，停车退出")
        if not saved and not known_node:
            context = event.context
            if (junction is None and not evidence.post_turn and context.active
                    and (step.road_id, step.from_node, step.to_node)
                    == (context.road_id, context.from_node, context.to_node)
                    and math.isfinite(evidence.at) and math.isfinite(event.source_at)
                    and context.source_barrier <= evidence.at <= event.source_at
                    and (context.identity == (self.link.session_nonce, self.link.boot_id)
                         or context.identity in getattr(self.link, "_patrol_coordinate_identities", ()))):
                runlog.emit("competition_obstacle_assignment", "按确认帧冻结的当前道路上下文归属障碍",
                            source_at=event.source_at, source_context=asdict(context),
                            kind="road", blocked_road=step.road_id,
                            location_basis="frozen_road_context")
                return ObstacleDisposition(event, "road", blocked_step=step)
            return unresolved("障碍确认时既无新鲜双侧有效激光，也无可靠路口证据，停车退出")
        incoming = step if saved else None
        node = step.to_node if saved else step.from_node
        heading = coverage.direction_between(self.field_map, step.from_node, step.to_node)
        choices = [(road_id, other) for road_id, other in self.field_map.adjacency[node]
                   if coverage.direction_between(self.field_map, node, other) == heading]
        if len(choices) != 1:
            return unresolved("障碍对应节点的车头前向道路无法唯一确认，停车退出")
        blocked = coverage.Step(step.index + 1, node, choices[0][1], choices[0][0], False)
        if (dual and not node_probe) or yolo_crossing:
            origin = evidence.junction_position_mm
            if (origin is None or evidence.road_position_mm is None or evidence.axis_deg is None
                    or evidence.junction_odom_mm is None
                    or not all(math.isfinite(v) for v in (origin, evidence.road_position_mm, evidence.axis_deg))
                    or (yolo_crossing and (len(evidence.junction_odom_mm) != 2
                        or not all(math.isfinite(v) for v in evidence.junction_odom_mm)))
                    or evidence.road_position_mm < origin - 1e-9):
                return unresolved("新道路缺少可验证的穿口里程原点，停车退出")
            frozen = ObstacleDisposition(event, "road", node, heading, blocked, incoming,
                                         junction, origin)
        else:
            frozen = ObstacleDisposition(event, "node", node, heading, blocked, incoming, junction)
        runlog.emit("competition_obstacle_assignment", "按确认帧冻结道路、节点与地图车头方向",
                    source_at=event.source_at, source_context=asdict(event.context),
                    kind=frozen.kind, node=node, map_heading=heading, blocked_road=blocked.road_id,
                    incoming_road=getattr(incoming, "road_id", None), road_origin_mm=frozen.road_origin_mm)
        return frozen

    @staticmethod
    def _location_dual_valid(evidence, source_at):
        sample = evidence.side_sample
        if (sample is None or sample.source_identity != evidence.context.identity
                or not sample.dual_wall or sample.source_keys is None):
            return False
        values = sample.left_mm, sample.right_mm
        return (all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(v) and 0 <= v < 65535 for v in values)
                and all(evidence.context.source_barrier <= source <= source_at
                        and source_at-source <= .250+1e-9 for source in sample.source_times))

    def _fresh_dual_valid(self):
        try:
            identity = self.link.session_nonce, self.link.boot_id
            directional = self.link.directional
            if directional is None or (directional.session_nonce, directional.boot_id) != identity:
                return False
            sample = motion._side_sample(self.link)
            now = time.monotonic()
            motion._check_sides(sample, now)
            return sample.dual_wall and all(0 <= now-source <= .250+1e-9
                                           for source in sample.source_times)
        except (mcu.McuError, AttributeError, TypeError, ValueError, IndexError):
            return False

    def _wait_front_clear(self):
        self.park()
        barrier = max(self._post_turn_barrier or 0., self.link._stable_barrier,
                      getattr(self, "_pose_barrier", 0.))
        self.vision.begin_front_clear(barrier)
        self.mode = "WAIT_PERIPHERAL"
        while True:
            try:
                self.parked_poll()
                self.collect_events()
                if self.pending_obstacle is not None:
                    return
                if not self._prepare_yolo(poll_hook=self.parked_poll):
                    time.sleep(motion.CONTROL_INTERVAL_S)
                    continue
                token = self.vision.front_clear_token(barrier)
                if token is not None:
                    self._vision_permission = token
                    self._post_turn_barrier = None
                    self.vision.finish_front_clear()
                    runlog.emit("competition_post_turn_clear", "前向姿态停后新帧核验完成，允许离口",
                                source_barrier=barrier, token=token)
                    return
                with self._vision_control_read():
                    candidate = getattr(self.vision, "_candidate", None)
                    current = getattr(self.vision, "_current", None)
                    camera_error = getattr(self.vision.camera, "last_error", None)
                    reason = ("转后前向相机尚未恢复：" + str(camera_error) if camera_error else
                              "转后前向画面仍有障碍命中或候选，保持停车"
                              if getattr(current, "detected", False) or getattr(candidate, "active", False)
                              else "转后等待前向成功新帧，无命中且无候选才离口")
                self.wait_message(reason)
                time.sleep(motion.CONTROL_INTERVAL_S)
            except mcu.McuError as exc:
                self.recover_mcu(exc)
                self.park()
                barrier = max(barrier, self.link._stable_barrier, self._pose_barrier)
                self.vision.begin_front_clear(barrier)

    def node_obstacle(self):
        if self.yolo_obstacles:
            return self.obstacle()
        r = self.runner
        task = self._node_obstacle_task
        if task is None:
            disposition = self.obstacle_disposition
            if disposition is None or disposition.kind != "node":
                raise mcu.McuError("节点障碍缺少已冻结的道路归属")
            self._node_obstacle_task = task = _NodeObstacleTask(disposition)
        d = task.disposition
        if task.stage == "STOP":
            self.park()
            if d.blocked_step is None or (d.incoming_step is not None and d.junction_evidence is None):
                raise ObstacleLocationError("障碍对应节点的前向出口无法唯一确认，停车退出")
            # Preserve and scan the original source's culvert before changing
            # route context. Neither scan completion nor recovery renames it.
            self.collect_events()
            if self.stage3_enabled and self.pending_upper is not None:
                task.upper_event = self.pending_upper
            task.stage = "SCAN"
        if task.stage == "SCAN":
            if task.upper_event is not None:
                self.scan(task.upper_event)
            task.stage = "COMMIT"
        if task.stage == "COMMIT":
            if d.incoming_step is not None:
                if r.step != d.incoming_step:
                    raise mcu.McuError("已冻结的入口道路改变，禁止按新路线改写障碍归属")
                # Stage1 owns both explicit opening and missing inference.
                # Calling that implementation preserves completed incoming
                # roads without executing its now-cancelled planned turn.
                r.junction_evidence = d.junction_evidence
                stage1.PatrolRunner._confirm_road(r)
            self.ledger.confirmed_nodes.add(d.node)
            self.ledger.last_confirmed_node = d.node
            self.ledger.position_confidence = mission_ledger.POSITION_CONFIRMED_NODE
            self.ledger.current_progress = None
            if d.blocked_step.road_id not in self.ledger.blocked_roads:
                result = mission_ledger.DriveResult("obstacle_at_node", 0., 0., True,
                                                     self.link.yaw_deg, True)
                self.ledger.record_blocked_road(d.blocked_step, result, d.event.result)
            checkpoint = getattr(self.link, "patrol_checkpoint", {})
            turn = checkpoint.get("turn")
            if turn and turn.get("issued") and turn.get("state") not in ("SETTLED", "COMPLETED"):
                raise mcu.McuError("已发送转弯尚未到达原绝对终点，禁止改写节点动作")
            if turn is not None and not turn.get("issued"):
                checkpoint.pop("turn", None)
            r.pending_candidate = r.pending_junction_decision = None
            self.leave_road()
            runlog.emit("competition_node_road_blocked", "保留已完成入口道路，永久封闭下一道路并原节点重规划",
                        node=d.node, blocked_road=d.blocked_step.road_id,
                        source_at=d.event.source_at, source_context=asdict(d.event.context))
            task.stage = "PLAN"
        if task.stage == "PLAN":
            complete = self.exact_plan(d.node, d.heading)
            old = list(r.steps[:r.index + (1 if d.incoming_step is not None else 0)])
            next_index = max((step.index for step in r.steps), default=-1) + 1
            following = tuple(replace(step, index=next_index+i)
                              for i, step in enumerate(complete.result.steps))
            if not following:
                self.finished = True
                self.returned = d.node == self.field_map.start_node_id
                self.ledger.route_plan_finished = True
                self.pending_obstacle = self.obstacle_disposition = self._node_obstacle_task = None
                self.mode = "FINISHED"
                return
            first = following[0]
            task.steps, task.index = tuple(old) + following, len(old)
            task.action = coverage.turn_kind(d.heading,
                coverage.direction_between(self.field_map, first.from_node, first.to_node))
            task.stage = "TURN"
        if task.stage == "TURN":
            openings = frozenset(d.junction_evidence.openings) if d.junction_evidence else motion._pending_junction_openings(self.link)
            self._check_node_action(d.node, d.heading, task.steps[task.index], task.action,
                                    openings, incoming=d.incoming_step)
            if task.action != coverage.TURN_STRAIGHT:
                if not openings:
                    raise ObstacleLocationError("原节点新路线转向缺少可靠节点方向证据，停车退出")
                self.mode, self.turn_authorized = "NODE_TURN", True
                self.begin_navigation_turn()
                if self.yolo_obstacles:
                    self.set_camera_pwm(2500, self.camera_tilt_us)
                self.link.patrol_route_context.update(node=d.node, planned_action=task.action)
                r._coordinate_mode = ("PIVOT" if self.yolo_obstacles
                                      or task.action == coverage.TURN_UTURN else "TRANSIT")
                self._post_turn_node_evidence = d.junction_evidence
                self._post_turn_node_heading = d.heading
                try:
                    stage1._turn_planned(self.link, self.display, openings,
                        turn_speed_mmps=getattr(self.link, "authorized_turn_speed_mmps", 91),
                        side="right" if task.action == coverage.TURN_RIGHT else "left",
                        angle_deg=180 if task.action == coverage.TURN_UTURN else 90,
                        input_source=self.input_source,
                        pivot=self.yolo_obstacles and task.action != coverage.TURN_UTURN,
                        goal_reference_deg=r._short_road_reference(task.steps[task.index]))
                finally:
                    self.turn_authorized = False
                r.first_actual_turn_completed = True
                if (not self.yolo_obstacles
                        and task.action in (coverage.TURN_LEFT, coverage.TURN_RIGHT)):
                    r.departure_normal_turn_completed = True
            task.stage = "PARK" if self.yolo_obstacles else "SETUP"
        if task.stage == "PARK":
            self.park()
            self.set_camera_pwm(PAN_FORWARD_US, self.camera_tilt_us)
            task.stage = "SETUP"
        if task.stage == "SETUP":
            r.steps, r.index = list(task.steps), task.index
            r.phase = "HOME" if r.home_leg else "DEPARTURE"
            r.goal = stage1.HOME_SPEED_MMPS if r.home_leg else stage1.CRUISE_SPEED_MMPS
            r._set_step(after_turn=task.action != coverage.TURN_STRAIGHT)
            self._post_turn_barrier = max(self.link._stable_barrier, self._pose_barrier)
            if task.action != coverage.TURN_STRAIGHT:
                self.finish_navigation_turn()
            task.stage = "PASSAGE"
        if task.stage == "PASSAGE":
            r._begin_passage(self.link._stable_barrier, after_turn=True)
            task.stage = "RESUME"
        if task.stage == "RESUME":
            self._resume_drive()
            self.pending_obstacle = self.obstacle_disposition = None
            self._node_obstacle_task = None

    def poll(self):
        motion._check_input_exit(self.input_source)
        now = time.monotonic()
        if self.runner is not None and self.runner._initialized:
            # Maintain encoder supervision through BRAKE tails and every pause.
            self.runner._observe_coordinate(self.link.state, self.display.controller._state_at or now)
        parked_modes = ("BRAKING", "PLANNING", "SCAN", "WAIT_PERIPHERAL", "FINISHED", "PIVOT", "NODE_TURN")
        waiting_stop = (self.mode in parked_modes and not any(self.display.wheel_targets)
                        and getattr(self.link, "_stop_tracker", None) is not None)
        if waiting_stop and not motion._assert_still_stable(self.link, self.display):
            self._stop_wait_since = self._stop_wait_since or now
            if now - self._stop_wait_since > STOP_TIMEOUT_S:
                raise mcu.McuError("本次严格停稳超过独立5秒窗口")
        else:
            self._stop_wait_since = None
        if self.mode != "DRIVE" or self.runner is None or not self.runner._initialized:
            # Camera capture/recovery runs independently, including in tunnels.
            return
        if getattr(self, "_route_conflict", None) is not None:
            raise _TaskRequired("route_conflict")
        self.collect_events()
        if getattr(self, "_post_turn_barrier", None) is not None:
            raise _TaskRequired("post_turn_front")
        if self.pending_obstacle is not None:
            raise _TaskRequired("obstacle")
        if self.runner.pending_candidate is not None:
            if self.pending_upper is not None:
                runlog.emit("competition_scan_deferred", "路口优先，本次涵洞事件跳过，保留以后扫描资格",
                            road=self.pending_upper.road_id)
                self.pending_upper = None
            return
        if self.yolo_obstacles and getattr(self.vision, "yolo_recovery_needed", False):
            raise _TaskRequired("vision")
        if self.pending_upper is not None:
            raise _TaskRequired("culvert")
        context = self.context
        if self.services.fault_reason is not None:
            raise _TaskRequired("services")
        prefix = self.retreat
        contract = getattr(self.runner, "_retreat_contract", None)
        if (prefix is not None and contract is not None and self.runner._midroad_returning
                and not contract.get("short_node_return")
                and prefix.entrance != self.field_map.start_node_id
                and self.runner.road_position_mm < 0.
                and not contract.get("entrance_overrun_warned", False)):
            contract["entrance_overrun_warned"] = True
            self.output("定位提示：返程里程已越过预计入口，继续等待激光路口证据")
            runlog.emit("competition_retreat_localization_warning", "返程里程越过预计入口，仅告警并继续等待激光",
                        road=prefix.road_id, entrance=prefix.entrance,
                        remaining_coordinate_mm=self.runner.road_position_mm,
                        expected_return_distance_mm=prefix.stopped_position_mm,
                        armed=self.runner.armed)
        if prefix is not None and contract is not None and contract.get("short_node_return"):
            remaining = self.runner.road_position_mm
            translated = prefix.stopped_position_mm - remaining
            if contract.get("motion_at") is not None and translated > 0.:
                reserve = self.runner.home_brake_reserve_mm(now)
                if remaining <= reserve:
                    contract["arrival_reserve_mm"] = reserve
                    raise _TaskRequired("retreat_node")
        if prefix is not None and prefix.entrance == self.field_map.start_node_id:
            reserve = self.runner.home_brake_reserve_mm(now)
            translated = prefix.stopped_position_mm - self.runner.road_position_mm
            if (self.runner.road_position_mm <= 0.
                    or translated > 0. and self.runner.road_position_mm <= reserve):
                raise _TaskRequired("retreat_home")

    def recover_mcu(self, exc):
        previous_mode = self.mode
        self.mode = "WAIT_PERIPHERAL"
        if self.runner is not None:
            self.runner._checkpoint()
        self.link._latch_hard_fault(str(exc))
        self.link.recover_fault(self.display, input_source=self.input_source)
        self.mode = previous_mode
        self.refresh_identity()
        self._stop_wait_since = None

    def park(self):
        self.mode = "BRAKING"
        self._stop_wait_since = None
        if self.runner is not None:
            self.display._distance_observer = self.runner._observe_distance
            self.runner.response.pause(self.link, time.monotonic())
        while True:
            try:
                if motion._assert_still_stable(self.link, self.display) and not any(self.display.command):
                    stage1._ensure_stable(self.link, self.display, time.monotonic() + STOP_TIMEOUT_S,
                                          input_source=self.input_source)
                else:
                    stage1._brake(self.link, self.display, time.monotonic() + STOP_TIMEOUT_S,
                                  input_source=self.input_source)
                if getattr(self, "distance_profile", None) is not None:
                    stage1._wait_junction_turn_ready(self.link, self.display,
                                                    input_source=self.input_source)
                return
            except mcu.McuError as exc:
                self.recover_mcu(exc)
                self.mode = "BRAKING"

    def parked_poll(self):
        self.display.tick()
        self.display.consume_samples(collect_means=False, update_controller=False, allow_stale=True)
        if not motion._assert_still_stable(self.link, self.display):
            stage1._wait_stable(self.link, self.display, time.monotonic() + STOP_TIMEOUT_S,
                               input_source=self.input_source)
        grant = getattr(self.vision.camera, "authorize_stationary", None)
        if callable(grant):
            grant(True, barrier=self.link._stable_barrier)
        self.display.maybe_print()

    def wait_message(self, reason):
        if reason != self._last_wait_message:
            self._last_wait_message = reason
            self.output("保持停车等待恢复：" + str(reason))
            runlog.emit("competition_peripheral_wait", "必要任务外设等待恢复", reason=str(reason))

    def wait_peripherals(self, *, upper=False):
        self.mode = "WAIT_PERIPHERAL"
        while True:
            try:
                self.parked_poll()
                if self.stage3_enabled:
                    self.upper.observe(self.link)
                event = self.upper.take_event() if self.stage3_enabled else None
                if event is not None and self.context is not None and event.context == self.context:
                    self.pending_upper = self.pending_upper or event
                self.vision.poll()
                if time.monotonic() >= self._next_recovery:
                    self.services.recover_pending(reuse_tick=True)
                    self._next_recovery = time.monotonic() + 1.
                reason = self.services.fault_reason or (self.upper.reason if upper and not self.upper.available else None)
                if reason is None:
                    return
                self.wait_message(reason)
                time.sleep(motion.CONTROL_INTERVAL_S)
            except mcu.McuError as exc:
                self.recover_mcu(exc)

    def announce_target(self, request_key, class_id):
        path = self.c["clues"].detection_audio_path(class_id)
        if path is None:
            raise VisionUnavailable(f"缺少目标{class_id}音频映射")
        self.services.queue_audio(request_key, path, class_id=class_id)
        while not self.services.audio_completed(request_key):
            self.parked_poll()
            if self.services.fault_reason is not None:
                self.wait_message(self.services.fault_reason)
                self.services.recover_pending(reuse_tick=True)
            time.sleep(motion.CONTROL_INTERVAL_S)
        return True

    def scan(self, event, *, skip_visual_failure=False):
        if not getattr(self, "stage3_enabled", True):
            return
        self.mode = "SCAN"
        context = event.context
        while True:
            try:
                self.scanner.scan(context, self.parked_poll)
                state = self.scanner.states.get(context.road_id)
                if state is not None and state.scan_status == "SKIPPED":
                    self.upper.skip_road(context.road_id, state.skip_reason)
                    self.pending_upper = None
                    if not state.forward_restored:
                        self.set_camera_pwm(PAN_FORWARD_US, TILT_US)
                        state.forward_restored = True
                    self.vision.set_mode(FRONT)
                    return
                self.upper.consume_road(context.road_id)
                self.pending_upper = None
                self.ledger.culvert_completed_roads.add(context.road_id)
                runlog.emit("competition_culvert_complete", "两侧识别尝试已结束，整条道路不再扫描",
                            road=context.road_id,
                            failed_sides={side: getattr(value, "reason", None)
                                          for side, value in self.scanner.states[context.road_id].sides.items()
                                          if getattr(value, "outcome", None) == "FAILED"},
                            sides={side: asdict(value) for side, value in self.scanner.states[context.road_id].sides.items()})
                return
            except VisionUnavailable as exc:
                if skip_visual_failure and exc.kind in ("camera", "inference"):
                    self.scanner.skip_road(context.road_id, str(exc))
                    state = self.scanner.states[context.road_id]
                    if state.completed:
                        self.upper.consume_road(context.road_id)
                        self.ledger.culvert_completed_roads.add(context.road_id)
                    else:
                        self.upper.skip_road(context.road_id, str(exc))
                    self.pending_upper = None
                    runlog.emit("competition_culvert_skipped", "已确认障碍，同路视觉任务放弃，本轮不再识别",
                                road=context.road_id, reason=str(exc), kind=exc.kind)
                    # Restore only the confirmed PWM, not a new camera frame.
                    self.set_camera_pwm(PAN_FORWARD_US, TILT_US)
                    state.forward_restored = True
                    self.vision.set_mode(FRONT)
                    return
                self.wait_message(str(exc))
                self.mode = "SCAN"
                self.parked_poll()
                self.services.recover_pending(reuse_tick=True)
                if getattr(self.c["detector"], "_disabled", False):
                    self._detector_ready = False
                    self.ensure_model()
                time.sleep(.1)
            except mcu.McuError as exc:
                self.recover_mcu(exc)
                self.mode = "SCAN"

    def exact_plan(self, start_node, heading=None, *, prefix=None):
        self.mode = "PLANNING"
        self.plan = None
        # Departure permission from a completed pivot cannot release movement.
        self.link.finish_departure()
        request = PlanRequest(start_node, frozenset(self.ledger.completed_roads),
                              frozenset(self.ledger.blocked_roads), heading,
                              self.field_map.start_node_id)
        while True:
            try:
                result = self.planner.solve(self.field_map, request, self.parked_poll)
                if prefix is not None:
                    prefix = self.retreat
                complete = validate_complete_plan(self.field_map, CompletePlan(request, result, prefix))
                self.plan = complete
                self.output(f"精确路线已校验：{complete.distance_mm / 1000:.3f}m，"
                            f"{len(result.steps)}段，转弯/掉头{result.turn_actions}次，终点{result.endpoint}"
                            + ("" if result.home_reachable else "；起点不可达，本轮不返航"))
                runlog.emit("competition_plan_validated", "最优性证明及完整路线校验通过，才允许撤回或发车",
                            request=asdict(request), result=asdict(result),
                            retreat=None if prefix is None else asdict(prefix),
                            complete_distance_mm=complete.distance_mm)
                return complete
            except (PlanningError, OSError, EOFError) as exc:
                self.wait_message(str(exc))
                self.parked_poll()
                time.sleep(.25)
            except mcu.McuError as exc:
                self.recover_mcu(exc)
                self.mode = "PLANNING"

    def _resume_drive(self):
        if getattr(self, "_route_conflict", None) is not None:
            raise _TaskRequired("route_conflict")
        r = self.runner
        self.display.patrol_deadline = math.inf
        r.sent_at, r.next_send = None, time.monotonic()
        self.display._distance_observer = r._observe_distance
        r.response.pause(self.link, r.next_send)
        self.display.controller.begin_alignment(r.next_send, source_barrier=self.link._stable_barrier)
        self.display.reset_averages(source_barrier=self.link._stable_barrier, reset_controller=False)
        if not r.armed:
            r._begin_passage(self.link._stable_barrier, after_turn=True)
        self.mode = "DRIVE"

    def _adopt_entered_obstacle_road(self, disposition):
        """Commit a crossed node without assigning its new walls to the incoming road."""
        r, d = self.runner, disposition
        if r.step == d.blocked_step:
            return
        if r.step != d.incoming_step or d.road_origin_mm is None:
            raise ObstacleLocationError("穿口障碍缺少匹配的来路或新道路原点，停车退出")
        turn = getattr(self.link, "patrol_checkpoint", {}).get("turn")
        if turn and turn.get("issued") and turn.get("state") not in ("SETTLED", "COMPLETED"):
            raise ObstacleLocationError("源道路切换时已有未完成转向，无法确认新道路，停车退出")
        frozen = d.event.location_evidence
        if frozen.junction_odom_mm is None:
            raise ObstacleLocationError("穿口时双轮里程无法建立新道路原点，停车退出")
        progress = motion._RoadProgress(frozen.junction_odom_mm)
        progress.origin_source_at = d.junction_evidence.confirmed_at
        progress._history = deque(r.progress._history, maxlen=512)
        progress.last_positions = r.progress.last_positions
        progress._token = r.progress._token
        position = r.road_position_mm - d.road_origin_mm
        if not math.isfinite(position) or position < -1e-9:
            raise ObstacleLocationError("新道路停车坐标无法验证，停车退出")
        r.junction_evidence = d.junction_evidence
        stage1.PatrolRunner._confirm_road(r)
        checkpoint = getattr(self.link, "patrol_checkpoint", {})
        if turn is not None and not turn.get("issued"):
            checkpoint.pop("turn", None)
        r.steps = list(r.steps[:r.index+1]) + [d.blocked_step]
        r.index += 1
        r.phase, r.goal = "DEPARTURE", stage1.CRUISE_SPEED_MMPS
        r._set_step()
        r.progress = progress
        r.road_position_mm = max(0., position)
        if r._coordinate_history:
            at, _ = r._coordinate_history[-1]
            r._coordinate_history[-1] = at, r.road_position_mm
        self.display.road_progress = progress
        self.display.origin = progress.origin_mm
        self.display.segment_distance_mm = progress.position_mm
        self.ledger.current_progress.entry_odom_mm = progress.origin_mm
        r._checkpoint()
        self.record_obstacle_location()
        runlog.emit("competition_obstacle_entered_road", "来路只提交一次，新双墙障碍归属实际前向新道路",
                    incoming_road=d.incoming_step.road_id, blocked_road=d.blocked_step.road_id,
                    node=d.node, road_origin_mm=d.road_origin_mm,
                    stopped_position_mm=r.road_position_mm)

    def obstacle(self):
        disposition = getattr(self, "obstacle_disposition", None)
        if disposition is not None and disposition.kind == "ambiguous":
            self.park()
            raise ObstacleLocationError(disposition.reason or "已确认障碍但道路或节点归属不足，停车退出")
        if disposition is not None and disposition.kind == "node" and not self.yolo_obstacles:
            return self.node_obstacle()
        r = self.runner
        task = getattr(self, "_obstacle_task", None)
        if task is None:
            event = self.pending_obstacle
            node = disposition is not None and disposition.kind == "node"
            entered = disposition is not None and disposition.incoming_step is not None and not node
            owned = disposition.blocked_step if disposition is not None else r.step
            if event is None or (not entered and not node and owned != r.step):
                raise mcu.McuError("障碍事件没有可信活动道路归属")
            if self.retreat is not None and r._midroad_returning:
                self.park()
                self.link.patrol_manual_required = (
                    "获准撤回片段再次确认障碍，无法保证返回原入口；保持停车等待位置及通路人工核验")
                raise mcu.McuError(self.link.patrol_manual_required)
            self._obstacle_task = task = _ObstacleTask(event,
                "NODE_STOP" if node else "ENTERED_ROAD" if entered else "STOP")
        event = task.event
        if task.stage == "NODE_STOP":
            self.park()
            if disposition.blocked_step is None or disposition.junction_evidence is None:
                raise ObstacleLocationError("节点避障缺少冻结出口及节点证据，保持停车")
            incoming = disposition.incoming_step
            if incoming is not None:
                if r.step != incoming:
                    raise ObstacleLocationError("节点避障来路已改变，保持停车")
                progress = r.progress
                r.junction_evidence = disposition.junction_evidence
                stage1.PatrolRunner._confirm_road(r)
                # Completion is semantic; the physical return retains the
                # original axis, origin, history and the entire BRAKE tail.
                r.progress = progress
            elif r.step.road_id != disposition.blocked_step.road_id:
                raise ObstacleLocationError("转后节点与前向道路不一致，保持停车")
            position = r.road_position_mm
            if not math.isfinite(position) or position < 0.:
                raise ObstacleLocationError("节点撤回缺少合法有符号道路坐标，保持停车")
            task.node_only = incoming is None and position <= 1e-9
            task.short_node_return = incoming is None and not task.node_only
            checkpoint = getattr(self.link, "patrol_checkpoint", {})
            old_turn = checkpoint.get("turn")
            if old_turn and old_turn.get("issued") and old_turn.get("state") not in ("SETTLED", "COMPLETED"):
                raise ObstacleLocationError("原转弯尚未完成，不得覆盖绝对终点")
            if old_turn is not None:
                checkpoint.pop("turn", None)
            r.pending_candidate = r.pending_junction_decision = None
            task.stage = "STOP"
        if task.stage == "ENTERED_ROAD":
            self.park()
            if (self.stage3_enabled and self.pending_upper is not None
                    and self.pending_upper.context == event.context):
                self.scan(self.pending_upper)
            self._adopt_entered_obstacle_road(disposition)
            task.stage = "STOP"
        owned_road = (disposition.blocked_step.road_id if disposition is not None else event.road_id)
        node = disposition is not None and disposition.kind == "node" and self.yolo_obstacles
        if owned_road != r.step.road_id and not node:
            raise mcu.McuError("未完成障碍任务的道路已改变，保持停车核验")
        if task.stage == "STOP":
            self.park()
            blocked_step = disposition.blocked_step if node else r.step
            first_closure = blocked_step.road_id not in self.ledger.blocked_roads
            if first_closure:
                self.ledger.record_blocked_road(blocked_step, r._arrival("obstacle"), event.result)
            if first_closure and len(self.ledger.blocked_roads) > 3:
                self.output("告警：出现第4条或更多封闭道路，仍执行封路和精确重规划")
            runlog.emit("competition_road_blocked", "严格停稳后永久封闭无向整条道路",
                        road=owned_road, source_at=event.source_at, context=asdict(event.context),
                        position=asdict(self.position()), blocked=sorted(self.ledger.blocked_roads))
            # Preserve a same-road trigger arriving in the brake tail as part
            # of this task, even if recovery subsequently refreshes the epoch.
            if getattr(self, "stage3_enabled", True):
                self.upper.observe(self.link)
            upper_event = self.upper.take_event() if getattr(self, "stage3_enabled", True) else None
            if upper_event is not None and upper_event.context == self.context:
                self.pending_upper = self.pending_upper or upper_event
            if self.pending_upper is not None and self.pending_upper.road_id == event.road_id:
                task.upper_event = self.pending_upper
            task.stage = "SCAN"
        if task.stage == "SCAN":
            if task.upper_event is not None:
                self.scan(task.upper_event, skip_visual_failure=getattr(self, "distance_profile", None) is None)
            reverse_heading = coverage.direction_between(self.field_map, r.step.to_node, r.step.from_node)
            if self.retreat is None:
                reference = getattr(self.display.controller, "formal_reference_deg", None)
                if reference is None:
                    reference = getattr(self.display.controller, "heading_reference_deg", None)
                if self.yolo_obstacles and (reference is None or not math.isfinite(reference)):
                    raise ObstacleLocationError("避障掉头缺少已采纳道路方向，保持停车")
                task.reference_deg = None if reference is None else motion._normalize_yaw(reference + 180.)
                self.retreat_arrival = None
                self.retreat = RetreatPrefix(r.step.road_id, r.step.from_node, r.step.to_node,
                                             r.road_position_mm, reverse_heading,
                                             (self.link.session_nonce, self.link.boot_id), self.leg_id,
                                             owned_road if self.yolo_obstacles else None)
            task.stage = "PIVOT"
        if task.stage == "PIVOT":
            authorization = dict(kind="midroad_uturn", road=r.step.road_id,
                                 identity=self.retreat.identity, reason="已确认障碍永久封路",
                                 position_mm=self.retreat.stopped_position_mm)
            self.link._patrol_midroad_authorization = authorization
            if self.yolo_obstacles:
                self.begin_navigation_turn()
                self.set_camera_pwm(2500, self.camera_tilt_us)
            self.leave_road()
            self.mode, self.turn_authorized = "PIVOT", True
            r._coordinate_mode = "PIVOT"
            try:
                options = (dict(absolute_uturn=True, goal_reference_deg=task.reference_deg)
                           if self.yolo_obstacles else {})
                stage1._turn_planned(self.link, self.display, frozenset(), side="left", angle_deg=180,
                                    turn_speed_mmps=getattr(self.link, "authorized_turn_speed_mmps", 91),
                                    input_source=self.input_source, midroad_authorization=authorization,
                                    **options)
            finally:
                self.turn_authorized = False
            r.first_actual_turn_completed = True
            task.stage = "PARK"
        if task.stage == "PARK":
            r._coordinate_mode = "ROAD"
            self.refresh_identity()
            self.park()
            if self.yolo_obstacles:
                self.set_camera_pwm(PAN_FORWARD_US, self.camera_tilt_us)
            task.stage = "PLAN"
        if task.stage == "PLAN":
            self.exact_plan(self.retreat.entrance, self.retreat.heading, prefix=self.retreat)
            task.stage = "NODE_INSTALL" if task.node_only else "SETUP"
        if task.stage == "NODE_INSTALL":
            evidence = self._node_evidence_after_heading(disposition.junction_evidence,
                disposition.node, disposition.heading, self.retreat.heading)
            self._save_node_evidence(evidence)
            self.ledger.confirm_rollback(self.retreat.entrance)
            self.confirm_retreat_arrival()
            # Installation may already have advanced onto the next route
            # step before a transient failure. Do not prove the old node or
            # erase its new current_progress again when resuming that task.
            task.stage = "NODE_INSTALL_PROVEN"
        if task.stage == "NODE_INSTALL_PROVEN":
            self._install_after_retreat()
            self.pending_obstacle = self.obstacle_disposition = self._obstacle_task = None
            return
        if task.stage == "SETUP":
            r._midroad_returning = True
            r.pending_candidate = r.pending_junction_decision = None
            r.junction_evidence = None
            r.phase = "RETURN_ROAD"
            r.goal = (stage1.HOME_SPEED_MMPS if self.retreat.entrance == self.field_map.start_node_id
                      else stage1.CRUISE_SPEED_MMPS)
            if r.goal == stage1.HOME_SPEED_MMPS:
                self.display.controller.set_patrol_road_policy("HOME")
            r.first_wall_source = None
            if self.yolo_obstacles:
                distance = self.retreat.stopped_position_mm
                r._retreat_contract = dict(distance_mm=distance,
                    short_node_return=task.short_node_return, motion_at=None,
                    wall_pairs=[], progress_seen=False,
                    node_evidence=disposition.junction_evidence if task.short_node_return else None,
                    node_heading=disposition.heading if task.short_node_return else None)
                r.detector.reset(distance * (.4 if r.step.road_id in ("r02", "r03") else .6),
                                 distance, self.retreat.identity, require_window=True)
                r._localization_samples.clear()
                r.reference_fixed, r.post_turn_low_speed = True, False
                r._n06_slow_active = False
                self.display.controller.patrol_road_speed_cap = None
                self.link._patrol_road_speed_cap = None
                r.post_turn_sampler.cancel()
                if r.goal != stage1.HOME_SPEED_MMPS:
                    self.display.controller.set_patrol_road_policy("ORDINARY")
                    r.post_turn_sampler.start(self.display.controller, time.monotonic())
                forward = mission_ledger.DriveResult("obstacle", distance, distance, False,
                    None if task.reference_deg is None else motion._normalize_yaw(task.reference_deg - 180.), True)
                self.ledger.begin_rollback(r.step, forward, self.link.odom_mm, self.link.yaw_deg)
            else:
                r.detector.reset(0., math.inf, self.retreat.identity, require_window=False)
            self.enter_road(r, reverse=True)
            if self.yolo_obstacles:
                self.finish_navigation_turn()
            if getattr(self, "distance_profile", None) is not None:
                self._post_turn_barrier = max(self.link._stable_barrier, self._pose_barrier)
            task.stage = "PASSAGE"
        if task.stage == "PASSAGE":
            r._begin_passage(self.link._stable_barrier, after_turn=True)
            task.stage = "RESUME"
        if task.stage == "RESUME":
            self._resume_drive()
            # The confirmed fact is cleared only after all retreat setup has
            # succeeded. A transient failure resumes its saved stage.
            self.pending_obstacle = None
            self.obstacle_disposition = None
            self._obstacle_task = None

    def _install_after_retreat(self):
        r, complete, prefix = self.runner, self.plan, self.retreat
        task = getattr(self, "_retreat_install_task", None)
        proof = getattr(self, "retreat_arrival", None)
        if task is None and (complete is None or prefix is None or self.ledger.last_confirmed_node != prefix.entrance
                or self.ledger.position_confidence != mission_ledger.POSITION_CONFIRMED_NODE
                or self.ledger.current_progress is not None
                or proof is None or proof.prefix != prefix
                or proof.identity != (self.link.session_nonce, self.link.boot_id)
                or not motion._assert_still_stable(self.link, self.display)):
            raise mcu.McuError("撤回入口未确认，不得执行后续节点动作")
        if not motion._assert_still_stable(self.link, self.display):
            raise mcu.McuError("撤回路线安装期间缺少严格停稳证据")
        if task is None:
            validate_complete_plan(self.field_map, complete)
            result = complete.result
            if not result.steps:
                self.retreat = None
                self.retreat_arrival = None
                r._midroad_returning = r._midroad_return_wait = False
                r._retreat_contract = None
                self.finished = True
                self.returned = prefix.entrance == self.field_map.start_node_id
                self.ledger.route_plan_finished = True
                self.mode = "FINISHED"
                return
            old = list(r.steps[:r.index])
            next_index = max((step.index for step in r.steps), default=-1) + 1
            following = [replace(step, index=next_index + i) for i, step in enumerate(result.steps)]
            # Keep this assignment fixed across retries, including START.
            if not old:
                old = [r.step]
            first = following[0]
            action = coverage.turn_kind(prefix.heading,
                                         coverage.direction_between(self.field_map, first.from_node, first.to_node))
            self._retreat_install_task = task = _RetreatInstallTask(
                prefix, tuple(old + following), len(old), action)
        first, action = task.steps[task.index], task.action
        prefix = task.prefix
        if task.stage == "TURN":
            openings = frozenset(r.junction_evidence.openings) if r.junction_evidence else motion._pending_junction_openings(self.link)
            self._check_node_action(prefix.entrance, prefix.heading, first, action, openings)
        if task.stage == "TURN" and action != coverage.TURN_STRAIGHT:
            if not openings:
                raise mcu.McuError("撤回入口转向缺少新鲜确认开口，保持停车")
            self.mode = "NODE_TURN"
            self.begin_navigation_turn()
            self._post_turn_node_evidence = r.junction_evidence
            self._post_turn_node_heading = prefix.heading
            self.link.patrol_route_context.update(node=prefix.entrance, planned_action=action)
            if self.yolo_obstacles and action == coverage.TURN_UTURN:
                self.set_camera_pwm(2500, self.camera_tilt_us)
            r._coordinate_mode = "PIVOT" if action == coverage.TURN_UTURN else "TRANSIT"
            self.turn_authorized = True
            try:
                stage1._turn_planned(self.link, self.display, openings,
                                    turn_speed_mmps=getattr(self.link, "authorized_turn_speed_mmps", 91),
                                    side="right" if action == coverage.TURN_RIGHT else "left",
                                    angle_deg=180 if action == coverage.TURN_UTURN else 90,
                                    input_source=self.input_source,
                                    goal_reference_deg=r._short_road_reference(first))
            finally:
                self.turn_authorized = False
            if action in (coverage.TURN_LEFT, coverage.TURN_RIGHT):
                r.departure_normal_turn_completed = True
        if task.stage == "TURN":
            task.stage = "FRONT" if self.yolo_obstacles and action == coverage.TURN_UTURN else "SETUP"
        if task.stage == "FRONT":
            stage1._ensure_stable(self.link, self.display, input_source=self.input_source)
            self.set_camera_pwm(PAN_FORWARD_US, self.camera_tilt_us)
            task.stage = "SETUP"
        if task.stage == "SETUP":
            r.steps, r.index = list(task.steps), task.index
            r._midroad_returning = r._midroad_return_wait = False
            r._retreat_contract = None
            self.pending_obstacle = self.pending_upper = None
            r.phase = "HOME" if r.home_leg else "DEPARTURE"
            r.goal = stage1.HOME_SPEED_MMPS if r.home_leg else stage1.CRUISE_SPEED_MMPS
            r._set_step(after_turn=action != coverage.TURN_STRAIGHT)
            if action != coverage.TURN_STRAIGHT:
                self.finish_navigation_turn()
            task.stage = "PASSAGE"
        if task.stage == "PASSAGE":
            r._begin_passage(self.link._stable_barrier, after_turn=True)
            task.stage = "RESUME"
        if task.stage == "RESUME":
            self._resume_drive()
            self.retreat = None
            self.retreat_arrival = None
            self._retreat_install_task = None

    def handle(self, kind):
        if getattr(self, "_route_conflict", None) is not None:
            self.park()
            self.mode = "ROUTE_CONFLICT"
            if not getattr(self, "_route_conflict_reported", False):
                self.output("转向与节点证据冲突：保持停车，保留任务等待人工核验；q/Ctrl-C退出")
                runlog.emit("competition_route_conflict", "节点动作冲突，保留原任务等待人工核验",
                            **self._route_conflict)
                self._route_conflict_reported = True
            while True:
                motion._check_input_exit(self.input_source)
                self.parked_poll()
                time.sleep(motion.CONTROL_INTERVAL_S)
        if kind == "finished":
            return
        if kind == "obstacle":
            self.obstacle()
            return
        if kind == "post_turn_front":
            self._wait_front_clear()
            if self.pending_obstacle is not None:
                self.obstacle()
            else:
                self._resume_drive()
            return
        if kind == "retreat_arrived":
            self._install_after_retreat()
            return
        if kind == "retreat_node":
            if getattr(self, "_retreat_install_task", None) is None:
                r = self.runner
                contract = getattr(r, "_retreat_contract", None) or {}
                self.park()
                reserve = contract.get("arrival_reserve_mm")
                position = r.road_position_mm
                if (not contract.get("short_node_return") or reserve is None
                        or contract.get("motion_at") is None
                        or not math.isfinite(position) or abs(position) > max(1., reserve)
                        or position >= self.retreat.stopped_position_mm
                        or not self.display.controller.heading_ready(tolerance_deg=5.)):
                    raise ObstacleLocationError("短撤回尚未证明回到原节点范围，保持真实坐标停车")
                evidence = self._node_evidence_after_heading(contract.get("node_evidence"),
                    self.retreat.entrance, contract.get("node_heading"), self.retreat.heading)
                self._save_node_evidence(evidence)
                self.ledger.confirm_rollback(self.retreat.entrance)
                self.confirm_retreat_arrival()
                self.leave_road()
                runlog.emit("competition_short_node_return", "有符号投影与严格停稳确认短撤回原节点",
                            remaining_coordinate_mm=position, arrival_reserve_mm=reserve,
                            frozen_distance_mm=self.retreat.stopped_position_mm)
            self._install_after_retreat()
            return
        if kind == "retreat_home":
            if getattr(self, "_retreat_install_task", None) is None:
                self.park()
                if not self.display.controller.heading_ready(tolerance_deg=5.):
                    raise mcu.McuError("撤回起点已停稳但返回航向未确认")
                self.ledger.confirm_rollback(self.retreat.entrance)
                self.confirm_retreat_arrival()
                self.leave_road()
            self._install_after_retreat()
            return
        committed_upper = self.pending_upper
        self.park()
        if kind == "vision":
            self._wait_camera_ready()
            if self.pending_obstacle is not None:
                self.obstacle()
                return
        elif kind == "culvert":
            if committed_upper is None:
                raise mcu.McuError("涵洞任务缺少已保存的源事件")
            self.scan(committed_upper)
            self.collect_events()
            if self.pending_obstacle is not None:
                self.obstacle()
                return
        else:
            self.wait_peripherals(upper=kind == "upper")
            if self.pending_upper is not None:
                self.scan(self.pending_upper)
        self._resume_drive()

    def confirm_retreat_arrival(self):
        if (self.retreat is None or not motion._assert_still_stable(self.link, self.display)
                or self.ledger.last_confirmed_node != self.retreat.entrance
                or self.ledger.current_progress is not None):
            raise mcu.McuError("本次撤回缺少节点和严格停稳证据")
        self.retreat_arrival = RetreatArrival(self.retreat,
                                              (self.link.session_nonce, self.link.boot_id),
                                              self.link._stable_barrier, tuple(self.link.odom_mm))
        runlog.emit("competition_retreat_arrived", "本次撤回原入口已确认，才允许后续节点动作",
                    proof=asdict(self.retreat_arrival),
                    remaining_coordinate_mm=self.runner.road_position_mm)

    def execute_task(self, kind):
        while True:
            try:
                self.handle(kind)
                return
            except _TaskRequired as task:
                if task.kind != "route_conflict":
                    raise
                kind = task.kind
            except mcu.McuError as exc:
                self.recover_mcu(exc)
                self.park()
            except CompetitionMotionBlocked as exc:
                if self._recover_blocked_turn(exc):
                    continue
                self.parked_poll()
                time.sleep(motion.CONTROL_INTERVAL_S)
            except motion._CenteringProposalExpired:
                # The saved parked task owns resumption; a busy visual update
                # cannot discard it or restore an earlier moving target.
                time.sleep(motion.CONTROL_INTERVAL_S)

    def _recover_blocked_turn(self, exc):
        """Preserve a blocked action for its owner; camera health is not a veto."""
        if self.mode not in ("NODE_TURN", "JUNCTION_ADVANCE"):
            return False
        turn = getattr(self.link, "patrol_checkpoint", {}).get("turn")
        active = bool(turn and turn.get("state") == "ACTIVE" and turn.get("issued"))
        self.park()
        if active:
            turn["state"] = "PARTIAL"
        runlog.emit("competition_turn_authorization_blocked", str(exc),
                    checkpoint_state=None if turn is None else turn.get("state"),
                    goal_world=None if turn is None else turn.get("goal_world"))
        return False

    def _initial_plan(self):
        if getattr(self, "_fixed_steps", None) is None:
            return self.exact_plan(self.field_map.start_node_id)
        self.mode = "PLANNING"
        self.route_authorized = False
        self.plan = None
        # Resolve and validate again at the authorization boundary.  This route
        # is user-specified and deliberately makes no global optimality claim.
        steps = build_fixed_route(self.field_map, self.settings.actions)
        if steps != self._fixed_steps:
            raise PlanningError("固定路线在启动校验期间改变，禁止发车")
        result = PlanResult(steps=steps,
            distance_mm=round(math.fsum(self.field_map.roads[s.road_id].length_m * 1000 for s in steps)),
            turn_actions=sum(a != coverage.TURN_STRAIGHT for a in self.settings.actions),
            endpoint=self.field_map.start_node_id, home_reachable=True,
            unreachable_roads=frozenset(), optimal=False,
            required_roads=frozenset(s.road_id for s in steps))
        complete = CompletePlan(PlanRequest(self.field_map.start_node_id,
            home_node=self.field_map.start_node_id), result)
        self.plan = self._authorized_fixed_plan = complete
        self.route_authorized = True
        self.output(f"固定路线已校验：{len(self.settings.actions)}个动作、{len(steps)}段；"
                    f"覆盖{len(result.required_roads)}/{len(self.field_map.roads)}条道路")
        runlog.emit("route_test_plan_validated", "指定动作路线校验完成，未宣称全图最优覆盖",
                    actions=self.settings.actions, steps=[asdict(step) for step in steps],
                    distance_mm=result.distance_mm, route_authorized=True, optimal=False)
        return complete

    def _early_test_camera_pose(self):
        """The first PWM follows communication STOP, before side-wall readiness."""
        if getattr(self, "settings", None) is None:
            return
        while True:
            motion._check_input_exit(self.input_source)
            try:
                if self.c.get("gimbal") is None:
                    factory = self.c.get("_factories", {}).get("gimbal")
                    if factory is None:
                        raise VisionUnavailable("路线测试缺少云台组件")
                    self.c["gimbal"] = factory()
                self.set_camera_pwm(PAN_FORWARD_US, self.camera_tilt_us)
                return
            except mcu.McuError as exc:
                self.recover_mcu(exc)
            except Exception as exc:
                self.wait_message(f"启动云台：{exc}")
                time.sleep(.1)

    def startup(self):
        if self.yolo_obstacles:
            while True:
                motion._check_input_exit(self.input_source)
                if self._prepare_yolo(
                        poll_hook=lambda: motion._check_input_exit(self.input_source)):
                    break
                time.sleep(motion.CONTROL_INTERVAL_S)
        try:
            self.vision.start()
        except Exception as exc:
            runlog.exception("competition_camera_start", "相机启动异常，保持停车并后台恢复", exc)
        for name, operation in (("OPEN", self.link.open), ("HELLO", self.link.hello),
                                ("CLOCK", self.link.synchronize_clock),
                                ("STOP", lambda: self.link.send_stop(motion.StopReason.MISSION))):
            while True:
                motion._check_input_exit(self.input_source)
                try:
                    operation()
                    break
                except (mcu.McuError, OSError) as exc:
                    self.wait_message(f"启动{name}：{exc}")
                    time.sleep(.1)
        self._early_test_camera_pose()
        self.services.active_step = coverage.Step(0, self.field_map.start_node_id,
                                                  self.field_map.start_node_id, "startup", False)
        for operation in (
                lambda: motion._prepare(self.link, self.display,
                                        time.monotonic() + config.MCU_SELFCHECK_READY_TIMEOUT_S,
                                        input_source=self.input_source),
                self.park,
                lambda: motion._sample_start(self.link, self.display, input_source=self.input_source)):
            while True:
                try:
                    operation()
                    break
                except mcu.McuError as exc:
                    self.recover_mcu(exc)
        self.mode = "WAIT_PERIPHERAL"
        while True:
            try:
                self.parked_poll()
                if not self.prepare_components():
                    time.sleep(.25)
                    continue
                self.services.recover_pending(reuse_tick=True)
                if not self.services.prepare():
                    self.wait_message(self.services.fault_reason or "RFID/音频自检未通过")
                    time.sleep(.25)
                    continue
                self.services.start_polling()
                self.set_camera_pwm(PAN_FORWARD_US, self.camera_tilt_us)
                self.output(f"启动云台PWM已确认：左右{PAN_FORWARD_US}，上下{self.camera_tilt_us}；实际视野待现场验收")
                break
            except mcu.McuError as exc:
                self.recover_mcu(exc)
        if self.stage3_enabled:
            self.ensure_model()
            self.upper.observe(self.link)
        self.wait_peripherals()
        complete = self._initial_plan()
        steps = list(complete.result.steps)
        if not steps:
            self.finished = self.returned = True
            self.ledger.route_plan_finished = True
            self.mode = "FINISHED"
            return
        self.runner = CompetitionRunner(self.link, self.display, self.field_map, steps,
                                        self.ledger, self.services, coordinator=self,
                                        departure_strategy=self.departure_strategy,
                                        **({"settings": self.settings} if getattr(self, "settings", None) is not None else {}),
                                        deadline=math.inf, input_source=self.input_source)
        self.link.motion_checks_enabled = True
        self.runner._set_step()
        self.runner._initialized = True
        self._wait_camera_ready()
        prepare_motion = getattr(self.vision.camera, "prepare_motion", None)
        while callable(prepare_motion) and not prepare_motion():
            self.parked_poll()
            time.sleep(motion.CONTROL_INTERVAL_S)
        self.mode = "DRIVE"
        self.output(("自动自检和固定路线校验完成，开始路线测试" if getattr(self, "settings", None) is not None
                     else "自动自检和精确规划完成，开始比赛") + "；无全场时限；q/Ctrl-C停车")

    def ensure_model(self):
        if not self.stage3_enabled:
            return
        # A timed-out native worker must actually exit before any replacement.
        # The outer loader thread keeps telemetry and quit handling responsive.
        import threading
        while not self._detector_ready:
            current = self.c["detector"]
            if getattr(current, "_disabled", False):
                current.close()
                worker = getattr(current, "_worker", None)
                if worker is not None and worker.is_alive():
                    self.parked_poll()
                    time.sleep(motion.CONTROL_INTERVAL_S)
                    continue
                factory = self.c.get("_factories", {}).get(
                    "detector", lambda: detector.Detector(conf_thresh=.8))
                self.c["detector"] = self.scanner.detector = current = factory()
            work = {"done": threading.Event()}
            def load_model():
                try:
                    current.load()
                except Exception as exc:
                    work["error"] = str(exc)
                finally:
                    work["done"].set()
            threading.Thread(target=load_model, name="competition-model-load", daemon=True).start()
            while not work["done"].is_set():
                try:
                    self.parked_poll()
                except mcu.McuError as exc:
                    self.recover_mcu(exc)
                time.sleep(motion.CONTROL_INTERVAL_S)
            if "error" in work:
                self.wait_message(work["error"])
                time.sleep(.25)
            else:
                self._detector_ready = True

    def run(self):
        self.startup()
        while not self.finished:
            try:
                if getattr(self, "_route_conflict", None) is not None:
                    self.execute_task("route_conflict")
                self.runner.run()
                self.finished = True
                self.returned = bool(getattr(self.ledger, "stage1_home_stop_confirmed", False))
            except CompetitionMotionBlocked as exc:
                if self._recover_blocked_turn(exc):
                    continue
                self.collect_events()
                kind = ("obstacle" if self.pending_obstacle else "culvert" if self.pending_upper
                        else "post_turn_front" if getattr(self, "_post_turn_barrier", None) is not None
                        else "services" if self.services.fault_reason
                        else "vision" if self.yolo_obstacles and getattr(
                            self.vision, "yolo_recovery_needed", False) else "upper")
                self.execute_task(kind)
            except _TaskRequired as task:
                self.execute_task(task.kind)
            except motion._CenteringProposalExpired:
                # No new command or lease is created while publication is
                # busy. The next tick receives UART and checks the old lease.
                self.runner.next_send = time.monotonic()
                time.sleep(motion.CONTROL_INTERVAL_S)
            except mcu.McuError as exc:
                self.recover_mcu(exc)
                self.park()
                if getattr(self, "_route_conflict", None) is not None:
                    self.execute_task("route_conflict")
                else:
                    self._resume_drive()
        self.park()
        self.mode = "FINISHED"
        while True:
            try:
                self.services.finish_after_stop(on_poll=self.parked_poll)
                break
            except mcu.McuError as exc:
                self.recover_mcu(exc)
                self.park()
        self.returned = self.returned or bool(getattr(self.ledger, "stage1_home_stop_confirmed", False))
        return self.summary()

    def summary(self):
        result = self.plan.result if self.plan is not None else None
        unreachable = set(result.unreachable_roads) if result is not None else set()
        uncovered = set(self.field_map.roads) - self.ledger.completed_roads - self.ledger.blocked_roads - unreachable
        missing_rfid = sorted(set(range(1, 13)) - self.ledger.confirmed_patrol_nos)
        sides = {road: {side: asdict(value) for side, value in state.sides.items()}
                 for road, state in self.scanner.states.items()}
        skipped = {road: state.skip_reason for road, state in self.scanner.states.items()
                   if state.scan_status == "SKIPPED"}
        untriggered = (self.ledger.completed_roads - TUNNEL_ROADS - set(self.scanner.states)
                       if getattr(self, "stage3_enabled", True) else set())
        report = dict(route_finished=self.finished, coverage_complete=self.finished and not uncovered,
                      stage3_enabled=getattr(self, "stage3_enabled", True),
                      departure_strategy=getattr(self, "departure_strategy", "10mm_turn"),
                      distance_profile=getattr(self, "distance_profile", None),
                      completed_roads=sorted(self.ledger.completed_roads),
                      blocked_roads=sorted(self.ledger.blocked_roads), unreachable_roads=sorted(unreachable),
                      uncovered_roads=sorted(uncovered), returned=self.returned,
                      stopped_node=self.ledger.last_confirmed_node,
                      culverts=sides, culvert_completed_roads=sorted(self.scanner.completed_roads),
                      culvert_failed_sides={road: {side: getattr(value, "reason", None)
                          for side, value in state.sides.items() if getattr(value, "outcome", None) == "FAILED"}
                          for road, state in self.scanner.states.items()
                          if any(getattr(value, "outcome", None) == "FAILED" for value in state.sides.values())},
                      culvert_observed_roads=sorted(self.scanner.states),
                      culvert_skipped_roads=sorted(skipped), culvert_skip_reasons=skipped,
                      culvert_unfinished_roads=sorted(set(self.scanner.states) - self.scanner.completed_roads - set(skipped)),
                      culvert_untriggered_roads=sorted(untriggered),
                      visual_gap_roads=sorted(getattr(self, "visual_gap_roads", ())),
                      missing_rfid=missing_rfid, rfid_errors=self.ledger.rfid_errors,
                      rfid_missing_intervals=getattr(self.services, "rfid_missing_intervals", []),
                      audio_outcomes=self.services.audio_request_outcomes,
                      terminal_reasons=self.ledger.terminal_reasons)
        if getattr(self, "settings", None) is not None:
            route_complete = bool(self._has_route_authorization() and self.finished and self.returned
                and getattr(self.ledger, "stage1_home_stop_confirmed", False)
                and self.runner is not None and self.runner.index == len(self._fixed_steps) - 1
                and tuple(self.runner.steps) == self._fixed_steps)
            report.update(route_test_complete=route_complete,
                          expected_actions=len(self.settings.actions),
                          completed_actions=self.runner.index if self.runner is not None else 0)
            self.output(f"指定路线动作：{report['completed_actions']}/{report['expected_actions']}；"
                        f"路线测试：{'完成' if route_complete else '未完成'}")
        self.output(f"道路覆盖：{len(self.ledger.completed_roads)}/{len(self.field_map.roads)}；"
                    f"封路：{report['blocked_roads']}；不可达：{report['unreachable_roads']}")
        self.output(f"实际停车节点：{report['stopped_node']}；返回n01并停稳：{'是' if self.returned else '否'}；"
                    f"涵洞完成道路：{report['culvert_completed_roads']}")
        self.output(f"涵洞各固定墙侧结果：{sides}；未完成：{report['culvert_unfinished_roads']}；"
                    f"已放弃：{skipped}")
        self.output(f"已走普通道路中未触发涵洞扫描：{report['culvert_untriggered_roads']}"
                    f"（未触发不证明没有涵洞）；视觉观测有空缺：{report['visual_gap_roads']}")
        runlog.emit("competition_summary", "道路、封路、返航及任务遗漏分别报告", terminal=False, **report)
        return report

    def close(self):
        self.mode = "FINISHED"
        self.link.competition_gimbal_monitor = None
        self.link.competition_gimbal_pending = None
        self.revoke_camera_stationary()
        operations = [self.planner.close]
        if self.link.connected:
            operations.append(lambda: self.link.send_stop(motion.StopReason.CLOSE))
        operations.extend((self.vision.close, self.services.close, self.display.close))
        if self.c.get("detector") is not None:
            operations.append(self.c["detector"].close)
        operations.append(self.link.shutdown)
        for close in operations:
            try:
                close()
            except Exception as exc:
                runlog.exception("competition_close_failed", "退出清理失败，继续关闭其余资源", exc)

    def prepare_components(self):
        ok = True
        for key, factory in self.c.get("_factories", {}).items():
            if self.c.get(key) is None:
                try:
                    self.c[key] = factory()
                except Exception as exc:
                    self.wait_message(f"{key}初始化：{exc}")
                    if key != "rfid":
                        ok = False
        self.scanner.gimbal = self.c.get("gimbal")
        self.scanner.detector = self.c.get("detector")
        return ok


def build_components(*, stage3_enabled=True, settings=None):
    link = CompetitionLink(**({"settings": settings} if settings is not None else {}))
    factories = dict(gimbal=lambda: gimbal.Gimbal(link), clues=clues.Clues,
                     audio=CompetitionAudio, rfid=CompetitionRfidReader)
    if stage3_enabled:
        factories["detector"] = lambda: detector.Detector(conf_thresh=.8)
    c = dict(mcu=link, _factories=factories)
    for key, factory in factories.items():
        try:
            c[key] = factory()
        except Exception as exc:
            c[key] = None
            runlog.exception("competition_component_init", "必要组件初始化失败，启动时停车重试", exc,
                             component=key)
    return c


def run_competition(*, components=None, input_source=None, emitter=None,
                    profile="combined", stage3_enabled=True, departure_strategy="10mm_turn",
                    yolo_obstacles=False, settings=None, vision_enabled=True):
    field_map = None
    if settings is not None:
        # Deliberately before terminals, logs/evidence writers and component
        # factories: malformed manual edits must never open a vehicle device.
        field_map = coverage.load_map()
        build_fixed_route(field_map, settings.actions)
        stage3_enabled, yolo_obstacles, vision_enabled = False, False, False
    terminal = motion.AsyncTerminal() if emitter is None else None
    output = terminal if terminal is not None else emitter
    owned_input = input_source is None
    if owned_input:
        input_source = motion.TerminalSpeedInput(write_line=output)
    coordinator = None
    try:
        writer = None
        if vision_enabled:
            from obstacle_evidence import DistanceEvidenceWriter
            directory = runlog.current_directory()
            if directory is None:
                directory = Path(config.LOG_DIR) / ("competition-" + str(time.time_ns()))
            writer = DistanceEvidenceWriter(directory / "obstacle-results")
        c = (build_components(stage3_enabled=stage3_enabled,
                             **({"settings": settings} if settings is not None else {}))
             if components is None else components)
        coordinator = CompetitionCoordinator(c, field_map=field_map, input_source=input_source, emitter=output,
                                             distance_profile=profile, stage3_enabled=stage3_enabled,
                                             departure_strategy=departure_strategy,
                                             yolo_obstacles=yolo_obstacles,
                                             settings=settings, vision_enabled=vision_enabled,
                                             evidence_writer=writer)
        result = coordinator.run()
        return 0 if result["route_test_complete" if settings is not None else "coverage_complete"] else 1
    except (KeyboardInterrupt, EOFError, motion._UserQuit) as exc:
        output("比赛中断，尝试安全停车：" + str(exc))
        if coordinator is not None:
            coordinator.ledger.note_terminal(str(exc))
            coordinator.summary()
        return 130
    except Exception as exc:
        runlog.exception("competition_failed", "比赛协调器异常，停止发车", exc)
        output("比赛未完成：" + str(exc))
        if coordinator is not None:
            coordinator.ledger.note_terminal(str(exc))
            coordinator.summary()
        return 2
    finally:
        if coordinator is not None:
            coordinator.close()
        if "writer" in locals() and writer is not None:
            evidence_summary = writer.close()
            output("障碍识别结果：" + str((directory / "obstacle-results").resolve()))
            if isinstance(evidence_summary, dict) and (evidence_summary.get("errors")
                    or evidence_summary.get("dropped_events") or evidence_summary.get("incomplete")):
                output("障碍结果保存未全部完成：" + str(evidence_summary))
        if owned_input:
            input_source.close()
        if terminal is not None:
            terminal.close()


def competition_entrypoint(profile, stage3_enabled, argv=None, *, departure_strategy="10mm_turn",
                          yolo_obstacles=False):
    """Fixed-profile CLI; argument parsing and --help never open devices."""
    import argparse
    parser = argparse.ArgumentParser(description=(
        f"阶段1+2{' +3' if stage3_enabled else ''}比赛入口，障碍{profile}模式；"
        f"出发区{departure_strategy}；无浏览器预览"))
    parser.add_argument("command", nargs="?", choices=("start",), default="start")
    parser.parse_args(argv)
    with runlog.RunLog(f"competition_stage{'123' if stage3_enabled else '12'}_{profile}_{departure_strategy}"):
        return run_competition(profile=profile, stage3_enabled=stage3_enabled,
                               departure_strategy=departure_strategy,
                               **({"yolo_obstacles": True} if yolo_obstacles else {}))
