"""Competition vision adapters; one camera, source-bound events and parked scans.

The coordinator owns motion and stopping. Camera/recognition unavailability is
reported here and never creates a motor command or clears an MCU fault.
"""
from __future__ import annotations

from collections import deque
from copy import copy
from dataclasses import dataclass, field, replace
import math
import threading
import time
from typing import Callable

import config
import obstacle
import runlog
import test_stage2_obstacle as stage2
from test_stage3_vision import _Stage3Streak, annotate


FRONT = "FRONT"
TUNNEL = "TUNNEL"
SCAN = "SCAN"
NAVIGATION_TURN = "NAVIGATION_TURN"
PAN_FORWARD_US = 1500
TILT_US = 550
SCAN_TILT_US = 800
SCAN_SETTLE_S = .6
SCAN_SIDE_TIMEOUT_S = 5.


class VisionUnavailable(RuntimeError):
    """A parked task must wait for a peripheral and resume its saved state."""

    def __init__(self, message, *, kind="peripheral"):
        super().__init__(message)
        self.kind = kind


@dataclass(frozen=True)
class RoadContext:
    road_id: str
    from_node: str
    to_node: str
    identity: tuple[int, int]
    dark_tunnel: bool = False
    active: bool = True
    leg_id: object = None
    source_barrier: float = 0.

    def canonical_side(self, vehicle_side: str) -> str:
        if vehicle_side not in ("left", "right"):
            raise ValueError("scan side must be left or right")
        if self.from_node < self.to_node:
            return vehicle_side
        return "right" if vehicle_side == "left" else "left"


def _context(value=None, **fields):
    if isinstance(value, RoadContext):
        return value
    if value is not None:
        if not isinstance(value, dict):
            raise TypeError("road context must be RoadContext or dict")
        fields = dict(value, **fields)
    if not fields:
        return None
    fields["identity"] = tuple(fields["identity"])
    fields.setdefault("source_barrier", time.monotonic())
    return RoadContext(**fields)


@dataclass(frozen=True)
class ObstacleEvent:
    context: RoadContext
    result: obstacle.ObstacleResult
    source_at: float
    source_generation: int | None
    frame_sequence: int
    target_kind: str | None
    location_evidence: object = None
    location_bound: bool = False

    @property
    def road_id(self):
        return self.context.road_id


@dataclass(frozen=True)
class UpperEvent:
    context: RoadContext
    channel: int
    source_key: int
    source_at: float
    cycle: int
    identity: tuple[int, int]

    @property
    def road_id(self):
        return self.context.road_id


class UpperSelector:
    """CH5 numeric observations only; missing numbers never gate movement."""

    def __init__(self, *, resolve_context: Callable | None = None):
        self.resolve_context = resolve_context
        self.context = None
        self.completed_roads = set()
        self.skipped_roads = {}
        self.selected_channel = None
        self.available = False
        self.reason = "waiting for a fresh upper measurement"
        self.pending = None
        self._identity = None
        self._keys = {4: None, 5: None}
        self._offered_legs = set()

    def set_context(self, context=None, **fields):
        updated = _context(context, **fields)
        if updated != self.context:
            self.pending = None
        self.context = updated

    def consume_road(self, road_id):
        self.completed_roads.add(road_id)
        if self.pending is not None and self.pending.road_id == road_id:
            self.pending = None

    def skip_road(self, road_id, reason):
        self.skipped_roads.setdefault(road_id, str(reason))
        if self.pending is not None and self.pending.road_id == road_id:
            self.pending = None

    def take_event(self):
        event, self.pending = self.pending, None
        return event

    def peek_event(self, link, *, now=None):
        """Project current receive evidence without consuming source or task state.

        The UART write gate calls this while the camera publication lock is
        held. It must not receive serial bytes, send commands or write logs.
        The real collector can subsequently commit the exact same source.
        """
        projected = copy(self)
        projected._keys = dict(self._keys)
        projected._offered_legs = set(self._offered_legs)
        projected.completed_roads = set(self.completed_roads)
        projected.skipped_roads = dict(self.skipped_roads)
        return projected.observe(link, now=now, _emit=False)

    def _at(self, source_at):
        context = self.context
        if self.resolve_context is not None:
            context = _context(self.resolve_context(source_at))
        if context is None or source_at <= context.source_barrier:
            return None
        return context

    def observe(self, link, *, now=None, _emit=True):
        now = time.monotonic() if now is None else now
        upper = getattr(link, "culvert_upper", None)
        identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
        if identity != self._identity:
            self._identity = identity
            self._keys = {4: None, 5: None}
            self.pending = None
            self.selected_channel = None
        self.available = False
        if upper is None or (upper.session_nonce, upper.boot_id) != identity:
            self.reason = "upper measurement is missing or belongs to another session"
            return self.pending
        try:
            generated = link._sample_time("upper")
            choices = {}
            for channel, index in ((5, 1),):
                status, age = None, upper.age_ms[index]
                value = upper.distance_mm[index]
                legal_age = (isinstance(age, int) and not isinstance(age, bool) and age >= 0)
                source_at = generated - age / 1000. if legal_age else float("-inf")
                fresh = legal_age and 0 <= now - source_at <= .250 + 1e-9
                numeric = (isinstance(value, (int, float))
                           and not isinstance(value, bool) and math.isfinite(value)
                           and 0 <= value < 65535)
                key = (upper.uptime_ms - age) & 0xFFFFFFFF if legal_age else None
                advancing = key is not None and stage2.sequence._source_key_newer(key, self._keys[channel])
                if advancing:
                    self._keys[channel] = key
                choices[channel] = status, source_at, key, advancing, fresh and numeric
        except (AttributeError, TypeError, ValueError, IndexError, OverflowError):
            self.reason = "upper measurement lacks valid status, age or source identity"
            return self.pending
        selected = 5 if choices[5][4] else None
        self.selected_channel = selected
        self.available = selected is not None
        self.reason = None if self.available else "CH5没有新鲜有效数值，当前无涵洞触发"
        if selected is None:
            return self.pending
        status, source_at, key, advancing, _healthy = choices[selected]
        context = self._at(source_at)
        if (not advancing or context is None or not context.active
                or context.dark_tunnel or context.identity != identity
                or context.road_id in self.completed_roads or context.road_id in self.skipped_roads):
            return self.pending
        leg = (context.road_id, context.from_node, context.to_node,
               context.leg_id if context.leg_id is not None else context.source_barrier, identity)
        if leg not in self._offered_legs and self.pending is None:
            self._offered_legs.add(leg)
            self.pending = UpperEvent(context, selected, key, source_at, upper.upper_cycle_seq, identity)
            if _emit:
                runlog.emit("competition_culvert_entry", "新鲜CH5有效数值触发本道路涵洞", road=context.road_id,
                            channel=selected, source_key=key, source_at=source_at, leg_id=context.leg_id)
        return self.pending


class CompetitionCameraSource(stage2._Stage2CameraSource):
    """Keep USB recovery at startup; motion permanently selects software reopen."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._competition_motion_started = False

    def arm_motion(self):
        self._competition_motion_started = True
        super().arm_motion()

    def prepare_motion(self):
        """Revoke startup USB permission and report when its active helper has exited."""
        if not self._lock.acquire(timeout=.005):
            return False
        try:
            self.arm_motion()
            return not self._usb_active
        finally:
            self._lock.release()

    def authorize_stationary(self, stationary, *, barrier=None):
        super().authorize_stationary(stationary and not self._competition_motion_started,
                                     barrier=barrier)

    def _stationary_authorized(self):
        return not self._competition_motion_started and super()._stationary_authorized()

    def _recover_usb(self):
        if not self._competition_motion_started:
            super()._recover_usb()


class CompetitionVision(stage2.Stage2Vision):
    """Stage2 recognition with a separate raw-camera health path and mode epoch."""

    def __init__(self, camera=None, *, host="0.0.0.0", port=8080, resolve_context=None,
                 preview_enabled=True, distance_profile=None, evidence_writer=None,
                 resolve_obstacle_location=None, yolo_obstacles=False):
        self._owns_camera = camera is None
        super().__init__(camera if camera is not None else CompetitionCameraSource(),
                         host=host, port=port, obstacle_approach=True,
                         preview_enabled=preview_enabled,
                         distance_profile=None if yolo_obstacles else distance_profile,
                         evidence_writer=evidence_writer)
        self.yolo_obstacles = bool(yolo_obstacles)
        self._yolo_detector = None
        self._yolo_loaded = False
        self._yolo_prepared = False
        self._yolo_prepare_failed = False
        self._yolo_lock = threading.Lock()
        self._yolo_diagnostic_generation = 0
        self.yolo_prepare_reason = None
        self._yolo_candidate_present = False
        self._yolo_stop_eligible = False
        if self.yolo_obstacles:
            self._yolo_detector = self._new_yolo_detector()
            self.obstacle_approach_enabled = False
        self.mode = FRONT
        self.context = None
        self.resolve_context = resolve_context
        # Coordinator-owned resolver: pure reads of source-time location only.
        self.resolve_obstacle_location = resolve_obstacle_location
        # Confirmation is a committed parking fact, separate from the camera's
        # current candidates. Recovery may discard observations but never this
        # queue. Each event permanently retains its measurement-time road.
        self._obstacle_events = deque()
        self._obstacle_offered_legs = set()
        self._preview_retry_at = 0.
        self._raw_successful_evidence = deque(maxlen=16)
        self._business_enabled = False
        self._business_enabled_since = None
        self._front_pose_barrier = 0.
        self._post_turn_probe = False
        self._navigation_turn_active = False

    def _new_yolo_detector(self):
        from detector import Detector
        model_dir = config.ASSETS_ROOT / "obstacle_yolo"
        return Detector(str(model_dir / "obstacle_bayese_640x640_nv12.bin"), conf_thresh=.6,
                        manifest_path=model_dir / "runtime-manifest.json", class_names=("0", "1", "2"),
                        profile_timing=True)

    @property
    def yolo_recovery_needed(self):
        return self.yolo_obstacles and (not self._yolo_prepared or self._yolo_prepare_failed
            or getattr(self._yolo_detector, "_disabled", False) is True)

    def _invalidate_yolo_evidence_locked(self):
        self._interrupt_yolo_observation_locked()
        self._generation += 1
        self._observation_generation += 1
        self._epoch_since = self._recovery_barrier = time.monotonic()
        self._ready = self._soft_ready = False
        self._current = self._captured_at = self._detect_pending = None
        self._yolo_stop_eligible = False
        self._successful_evidence.clear()
        self._raw_successful_evidence.clear()
        self._recovery_keys.clear()
        self._clear_unconfirmed_locked()

    def _interrupt_yolo_observation_locked(self):
        if not getattr(self, "yolo_obstacles", False):
            return
        self._yolo_diagnostic_generation += 1
        if self._evidence_writer is not None:
            try:
                self._evidence_writer.observe_yolo(None, (), {
                    "inference_status": "interrupted",
                    "observation_generation": self._yolo_diagnostic_generation})
            except Exception as exc:
                self._diagnostic_locked("competition_obstacle_evidence",
                                        "障碍诊断中断记录失败：" + str(exc), terminal=False)

    def _sync_source_generation_locked(self):
        previous = self._source_generation
        super()._sync_source_generation_locked()
        if previous != self._source_generation:
            self._yolo_stop_eligible = False
            self._interrupt_yolo_observation_locked()

    def _recover_startup(self, reason, *, retry_camera=False, kind="fault", event_key=None, **details):
        with self._condition:
            self._yolo_stop_eligible = False
            # Expiry of an older business result alone is not a camera gap.
            if (kind == "fault" or getattr(self.camera, "last_error", None)
                    or self._observed_at is None or not self._fresh(self._observed_at)):
                self._interrupt_yolo_observation_locked()
            return super()._recover_startup(reason, retry_camera=retry_camera, kind=kind,
                                            event_key=event_key, **details)

    def prepare_yolo(self, poll_hook=None):
        """Warm the private model without publishing synthetic or in-flight frames."""
        if not self.yolo_obstacles:
            return True
        if self._closed:
            self.yolo_prepare_reason = "视觉已关闭"
            return False
        if not self.yolo_recovery_needed:
            return True
        if not self._yolo_lock.acquire(blocking=False):
            return False
        work = {"done": threading.Event(), "ok": False}
        with self._condition:
            self._yolo_prepared = False
            self._invalidate_yolo_evidence_locked()
            self.yolo_prepare_reason = "YOLO障碍模型准备中"

        def prepare():
            try:
                import numpy as np
                if self._closed:
                    raise RuntimeError("视觉已关闭")
                current = self._yolo_detector
                if self._yolo_prepare_failed or getattr(current, "_disabled", False) is True:
                    current.close()
                    worker = getattr(current, "_worker", None)
                    if worker is not None and worker.is_alive():
                        self.yolo_prepare_reason = "旧YOLO推理线程尚未退出，保持停车等待"
                        return
                    if self._closed:
                        raise RuntimeError("视觉已关闭")
                    self._yolo_detector = current = self._new_yolo_detector()
                    self._yolo_loaded = False
                timeout = config.DETECTOR_LOAD_TIMEOUT_S
                if self._closed:
                    raise RuntimeError("视觉已关闭")
                if not self._yolo_loaded:
                    current.load(timeout_s=timeout)
                    self._yolo_loaded = True
                if self._closed:
                    raise RuntimeError("视觉已关闭")
                current.infer_bgr(np.zeros((config.YOLO_INPUT_H, config.YOLO_INPUT_W, 3),
                                          dtype=np.uint8), timeout_s=timeout)
                with self._condition:
                    if self._closed:
                        raise RuntimeError("视觉已关闭")
                    self._invalidate_yolo_evidence_locked()
                    self._yolo_prepared = True
                    self._yolo_prepare_failed = False
                    self.yolo_prepare_reason = None
                work["ok"] = True
            except Exception as exc:
                with self._condition:
                    self._yolo_prepared = False
                    self._yolo_prepare_failed = True
                    self.yolo_prepare_reason = "YOLO障碍模型准备失败：" + str(exc)
                    self._recover_startup(self.yolo_prepare_reason)
            finally:
                try:
                    if self._closed:
                        self._yolo_detector.close()
                finally:
                    self._yolo_lock.release()
                    work["done"].set()

        threading.Thread(target=prepare, name="competition-yolo-prepare", daemon=True).start()
        while not work["done"].wait(.01):
            if poll_hook is not None:
                poll_hook()
        return work["ok"]

    def close(self):
        super().close()
        self._yolo_prepared = False
        if self._yolo_detector is not None:
            self._yolo_detector.close()
        if self._owns_camera:
            self.camera.close()

    def _at(self, source_at):
        context = self.context
        if self.resolve_context is not None:
            context = _context(self.resolve_context(source_at))
        if (context is None or source_at <= context.source_barrier or not context.active
                or context.dark_tunnel and not self._post_turn_probe or context != self.context):
            return None
        return context

    def set_context(self, context=None, *, preserve_approach=False, **fields):
        updated = _context(context, **fields)
        with self.control_read():
            if updated == self.context:
                return
            previous = self.context
            same_road = (previous is not None and updated is not None and previous.active and updated.active
                         and (previous.road_id, previous.from_node, previous.to_node)
                         == (updated.road_id, updated.from_node, updated.to_node))
            self.context = updated
            self._change_mode_locked(TUNNEL if updated is not None and updated.dark_tunnel else FRONT,
                                     force=True, preserve_approach=preserve_approach and same_road)

    def _change_mode_locked(self, mode, *, force=False, preserve_approach=False):
        if mode not in (FRONT, TUNNEL, SCAN, NAVIGATION_TURN):
            raise ValueError("invalid competition vision mode")
        if self._navigation_turn_active:
            mode = NAVIGATION_TURN
        if self.mode == mode and not force:
            return
        self._interrupt_yolo_observation_locked()
        self.mode = mode
        if self.yolo_obstacles:
            self._yolo_candidate_present = False
            self._yolo_stop_eligible = False
        enabled = mode == FRONT and self.context is not None and self.context.active
        if enabled and not self._business_enabled:
            self._business_enabled_since = time.monotonic()
        elif not enabled:
            self._business_enabled_since = None
        self._business_enabled = enabled
        self._generation += 1
        self._observation_generation += 1
        self._epoch_since = time.monotonic()
        self._epoch = "outbound" if mode == FRONT and self.context is not None and self.context.active else None
        self._detect_pending = None
        if preserve_approach:
            self.suspend_approach_confirmation()
            self._pending_confirmation = None
        else:
            self._discard_decision_locked()
        self._current = self._captured_at = None
        self._ready = False
        self._successful_evidence.clear()
        self._condition.notify_all()

    def set_mode(self, mode):
        with self.control_read():
            keep = (self.context is not None and self.context.active
                    and self.mode in (FRONT, SCAN) and mode in (FRONT, SCAN))
            self._change_mode_locked(mode, preserve_approach=keep)

    def begin_navigation_turn(self):
        """Revoke candidates and in-flight results under the event publication lock."""
        with self.control_read():
            self._navigation_turn_active = True
            self._post_turn_probe = False
            self._change_mode_locked(NAVIGATION_TURN, force=True)

    def finish_navigation_turn(self, barrier):
        """Only the stopped final heading can open a new front-image epoch."""
        with self.control_read():
            self._navigation_turn_active = False
            self._front_pose_barrier = max(self._front_pose_barrier, barrier)
            self._change_mode_locked(TUNNEL if self.context is not None
                                     and self.context.dark_tunnel else FRONT, force=True)
            self._epoch_since = max(self._epoch_since, barrier)

    def front_pose_ready(self, barrier):
        """Only sources after the confirmed forward pose and settle interval count."""
        with self.control_read():
            self._front_pose_barrier = max(self._front_pose_barrier, barrier)
            self._change_mode_locked(TUNNEL if self.context is not None
                                     and self.context.dark_tunnel else FRONT, force=True)
            self._epoch_since = max(self._epoch_since, barrier)

    def begin_front_clear(self, barrier):
        with self.control_read():
            if self.context is not None and self.context.dark_tunnel:
                self._post_turn_probe = True
                self._change_mode_locked(FRONT, force=True)
            self._epoch_since = max(self._epoch_since, barrier)

    def finish_front_clear(self):
        with self.control_read():
            self._post_turn_probe = False
            if self.context is not None and self.context.dark_tunnel:
                self._change_mode_locked(TUNNEL)

    def front_clear_token(self, barrier):
        with self.control_read():
            if (self.mode != FRONT or not self._ready or self._current is None
                    or not self._current.available
                    or (self._yolo_stop_eligible if self.yolo_obstacles else self._current.detected)
                    or self._confirmed is not None or self._candidate.active
                    or self._captured_at is None
                    or self._captured_at <= max(barrier, self._front_pose_barrier)
                    or not self._fresh(self._captured_at)
                    or not self._source_generation_current_locked()
                    or self._runtime_outage_active
                    or getattr(self.camera, "fault_generation", None) != self._source_fault_generation
                    or getattr(self.camera, "last_error", None)):
                return None
            if (self.yolo_obstacles and self.recovery_token(
                    max(barrier, self._front_pose_barrier)) is None):
                return None
            return self._source_generation, self._runtime_stop_generation

    def peek_obstacle(self):
        """Pure read for the final UART guard, under the publication lock.

        This does not poll a driver, run a watchdog, refresh evidence or consume
        a fact. ``motion_write()`` holds this same reentrant condition lock.
        """
        with self.control_read():
            return self._obstacle_events[0] if self._obstacle_events else None

    def peek_obstacle_fact(self):
        """A committed immutable stop is visible even while publication is busy.

        The detector only appends; the one control owner removes entries.
        Reading this atomic deque head neither consumes nor refreshes evidence.
        """
        return self._obstacle_events[0] if self._obstacle_events else None

    def take_obstacle(self):
        """Consume one committed fact even after camera failure or road change."""
        with self.control_read():
            return self._obstacle_events.popleft() if self._obstacle_events else None

    def bind_obstacle_location(self, event, resolver=None):
        """The control owner freezes location after its complete UART batch.

        Detection has already committed the parking fact. Location lookup and
        its diagnostics never hold the publication lock or change its source.
        """
        if event.location_bound or event.location_evidence is not None:
            return event
        resolver = self.resolve_obstacle_location if resolver is None else resolver
        location = None
        if resolver is not None:
            try:
                location = resolver(event.source_at)
            except Exception as exc:
                runlog.exception("competition_obstacle_location", "障碍已确认，源时刻定位失败，保留停车事件",
                                 exc, source_at=event.source_at,
                                 frame_sequence=event.frame_sequence, road=event.road_id)
        return replace(event, location_evidence=location, location_bound=True)

    @property
    def approach_active(self):
        if self.yolo_obstacles:
            with self.control_read():
                return self.mode == FRONT and self._yolo_candidate_present
        return super().approach_active

    def approach_speed(self, requested_speed):
        if self.yolo_obstacles:
            return min(requested_speed, 100) if self.approach_active else requested_speed
        if self.mode == FRONT and self.approach_active:
            return min(requested_speed, stage2.STAGE2_OBSTACLE_APPROACH_SPEED_MMPS)
        return requested_speed

    def motion_health_reason(self, token=None, *, now=None):
        """Two seconds of independent raw/detection supervision, without I/O."""
        with self.control_read():
            now = time.monotonic() if now is None else now
            if token is None or token[1] != self._runtime_stop_generation:
                return "等待当前摄像头代次的两张成功新帧"
            raw_source = getattr(self, "_last_reliable_capture_at", None)
            if raw_source is None:
                raw_source = self._last_successful_capture_at
            if raw_source is None or not 0 <= now - raw_source < 2.:
                return "连续2秒没有可靠新图像"
            if self.mode == FRONT and self.context is not None and self.context.active:
                source = max(self._business_enabled_since,
                             self._last_successful_capture_at or self._business_enabled_since)
                if not 0 <= now - source < 2.:
                    return "连续2秒没有成功业务检测"
            return None

    def recovery_token(self, barrier):
        """Two successful current-source frames measured after strict parking.

        Each frame was fresh when published. The first need not still be within
        500 ms when a low-rate camera completes the second; the original one
        second confirmation window and the latest frame's freshness still apply.
        """
        with self.control_read():
            now = time.monotonic()
            if (getattr(self.camera, "last_error", None)
                    or not self._source_generation_current_locked()
                    or getattr(self.camera, "fault_generation", None) != self._source_fault_generation):
                return None
            current = self._source_generation, self._runtime_stop_generation
            evidence = (self._successful_evidence if self.mode == FRONT
                        and self.context is not None and self.context.active else self._raw_successful_evidence)
            accepted = [(sequence, source) for generation, sequence, source, stop in evidence
                        if (generation, stop) == current and source > barrier]
            if len(accepted) < 2:
                return None
            first, last = accepted[-2:]
            if (last[0] <= first[0] or last[1] <= first[1]
                    or last[1] - first[1] > stage2.STAGE2_OBSTACLE_VERIFY_TIMEOUT_S + 1e-9
                    or not self._fresh(last[1], now)):
                return None
            if evidence is self._raw_successful_evidence:
                # Tunnel/scan modes have no business completions to end an
                # outage. Only this strict post-stop two-source grant may do
                # so; another later raw interruption starts a new stop.
                self._soft_ready = True
                self._runtime_outage_active = False
                self._timeout_capture_key = None
            return current

    def latest_frame(self, *, after=None):
        with self.control_read():
            self._watchdog_locked(time.monotonic())
            frame = self._raw_latest
            if (frame is None or not self._fresh(frame.captured_monotonic)
                    or frame.source_generation != self._source_generation
                    or getattr(self.camera, "last_error", None)
                    or after is not None and frame.captured_monotonic <= after):
                return None
            return frame

    def _runtime_timeout_locked(self, now):
        # Capture and detection describe the same interrupted camera task.
        # Once parked, a second family cannot invalidate its recovery frames.
        if self._runtime_outage_active:
            return
        clocks = [("capture", getattr(self, "_last_reliable_capture_at", None))]
        if self.mode == FRONT and self.context is not None and self.context.active:
            clocks.append(("detection", max(self._business_enabled_since,
                           self._last_successful_capture_at or self._business_enabled_since)))
        for family, source in clocks:
            if source is None or now - source < 2.:
                continue
            key = "competition", family, source
            if key != self._timeout_capture_key:
                self._interrupt_yolo_observation_locked()
                self._timeout_capture_key = key
                self._latch_runtime_stop_locked("连续2秒没有可靠新图像" if family == "capture"
                                               else "连续2秒没有成功业务检测", ("timeout", key))
                self._availability_kind = "fault"
            return

    def _ingest_frame(self, captured):
        token = super()._ingest_frame(captured)
        if token is None:
            return None
        with self._condition:
            self._raw_successful_evidence.append((captured.source_generation, captured.sequence,
                                                 captured.captured_monotonic,
                                                 self._runtime_stop_generation))
            if self.mode != FRONT or self.context is None or not self.context.active:
                self._detect_pending = None
                self._ready, self._startup_reason = True, None
                self._availability_kind, self._active_fault_key = "ready", None
                self._sequence = captured.sequence
                self._captured_at = captured.captured_monotonic
            return token

    def _detect_frame(self, captured, token):
        generation, visual, source_generation, _started, stop_generation = token
        with self._condition:
            if (self.mode != FRONT or generation != self._generation
                    or stop_generation != self._runtime_stop_generation
                    or captured.captured_monotonic <= self._epoch_since):
                return
            context = self._at(captured.captured_monotonic)
            diagnostic_generation = self._yolo_diagnostic_generation
            captured_mode = self.mode
        if context is None:
            return
        detail = {}
        started = time.monotonic()
        if self.yolo_obstacles:
            result = self._detect_yolo_obstacle(captured.bgr, detail, expected_generation=generation)
        elif self._distance_detector is not None:
            with self._condition:
                detecting_candidate = self._candidate
                continuation = bool(self._epoch is not None and generation == self._generation
                    and visual == self._visual_generation
                    and stop_generation == self._runtime_stop_generation
                    and source_generation == self._source_generation and context == self.context
                    and self._candidate.active
                    and not self._candidate.expired(captured.captured_monotonic)
                    and self._candidate.last is not None)
            result = self._distance_detector.detect(captured.bgr, diagnostics=detail,
                                                    continuation_active=continuation)
        else:
            result = stage2.detect_stage2_obstacle(captured.bgr, diagnostics=detail)
            result = self._track_approach_target(captured, result, detail)
        now = time.monotonic()
        if self.yolo_obstacles:
            self._observe_yolo_result(captured, detail, context, token, diagnostic_generation,
                                      captured_mode, now)
        timing_due = now >= self._next_detection_diagnostic_at
        with self._condition:
            if timing_due:
                self._next_detection_diagnostic_at = now + 1.
                self._diagnostic_locked("competition_detection_timing", "采集、排队与检测耗时分别记录",
                    detection_duration_s=now-started, source_age_s=now-captured.captured_monotonic,
                    queue_age_s=started-captured.captured_monotonic,
                    source_sequence=captured.sequence, distance_profile=self.distance_profile,
                    terminal=not self.yolo_obstacles,
                    **({"inference_attempted": detail["inference_attempted"],
                        "inference_status": detail["inference_status"],
                        "detection_available": result.available} if self.yolo_obstacles else {}))
            self._sync_source_generation_locked()
            if (self._closed or self.mode != FRONT or generation != self._generation
                    or visual != self._visual_generation or stop_generation != self._runtime_stop_generation
                    or source_generation != self._source_generation or context != self.context
                    or self.yolo_obstacles and captured.captured_monotonic <= self._epoch_since):
                return
            self._watchdog_locked(now)
            if stop_generation != self._runtime_stop_generation:
                return
            if self._distance_detector is not None and detecting_candidate is not self._candidate:
                return
            if not result.available:
                self._recover_startup(result.reason or "obstacle detector unavailable")
                return
            if not self._fresh(captured.captured_monotonic, now) or getattr(self.camera, "last_error", None):
                return
            self._ready, self._startup_reason = True, None
            self._availability_kind, self._active_fault_key = "ready", None
            self._current, self._sequence = result, captured.sequence
            if self.yolo_obstacles:
                self._yolo_candidate_present = result.detected
                self._yolo_stop_eligible = detail.get("stop_eligible") is True
            self._set_obstacle_metrics_locked(captured, result, detail)
            self._captured_at = captured.captured_monotonic
            self._record_success_locked(captured)
            self._completed.append(now)
            while len(self._completed) > 1 and self._completed[0] < now - 2.:
                self._completed.popleft()
            elapsed = now - self._completed[0]
            self._fps = (len(self._completed) - 1) / elapsed if elapsed > 0 else 0.
            phase = "preview"
            if self._epoch is not None and self._confirmed is None:
                key = captured.sequence, captured.captured_monotonic
                if self.yolo_obstacles:
                    previous = self._candidate.last
                    same = (previous is not None and previous.bbox is not None
                            and result.bbox is not None
                            and obstacle._bbox_iou(previous.bbox, result.bbox)
                            >= config.OBSTACLE_CONTINUITY_MIN_IOU)
                    phase = self._candidate.update(
                        replace(result, detected=detail.get("stop_eligible") is True),
                        key, captured.captured_monotonic, same_target=same)
                elif self._distance_detector is not None:
                    same = (self._candidate.last is not None
                            and self._candidate.last.detected and result.detected)
                    phase = self._candidate.update(result, key, captured.captured_monotonic, same_target=same)
                else:
                    phase = self._update_approach_locked(result, key)
                if phase == "confirmed":
                    if context.dark_tunnel:
                        # The parked exit check may inspect one frame at the
                        # known tunnel entrance. Tunnel business remains off:
                        # no new closure or scanning event can be published.
                        phase = "pending"
                        self._condition.notify_all()
                        return
                    self._confirmed = result
                    leg = (context.road_id, context.from_node, context.to_node, context.identity,
                           context.leg_id if context.leg_id is not None else context.source_barrier)
                    if leg not in self._obstacle_offered_legs:
                        self._obstacle_offered_legs.add(leg)
                        self._obstacle_events.append(ObstacleEvent(
                            context, result, captured.captured_monotonic,
                            source_generation, captured.sequence, self._target_kind))
                    if self.preview_enabled:
                        self._pending_confirmation = captured, result, self._fps, phase, visual
                elif phase == "timeout":
                    self._timed_out = True
                    self._clear_unconfirmed_locked()
            if self.preview_enabled and self._raw_latest is captured:
                self._pending = captured, result, self._fps, phase, visual
            evidence_epoch = (context.road_id, context.from_node, context.to_node,
                              context.identity, context.leg_id)
            self._condition.notify_all()
        self._flush_diagnostics()
        if self._evidence_writer is not None and not self.yolo_obstacles:
            try:
                self._evidence_writer.observe(captured, result, detail, phase, epoch=evidence_epoch)
            except Exception as exc:
                runlog.exception("competition_obstacle_evidence", "障碍证据入队失败", exc)

    def _observe_yolo_result(self, captured, detail, context, token, observation_generation,
                             captured_mode, completed_at):
        writer = self._evidence_writer
        if writer is None:
            return
        generation, visual, source_generation, _started, stop_generation = token
        with self._condition:
            self._sync_source_generation_locked()
            diagnostics = dict(detail,
                observation_generation=observation_generation,
                captured_context=context, current_context=self.context,
                captured_mode=captured_mode, current_mode=self.mode,
                captured_pose_generation=generation, current_pose_generation=self._generation,
                captured_visual_generation=visual, current_visual_generation=self._visual_generation,
                current_source_generation=self._source_generation,
                completed_monotonic=completed_at,
                source_age_s=completed_at-captured.captured_monotonic,
                stale_frame=not self._fresh(captured.captured_monotonic, completed_at),
                road_changed=context != self.context,
                pose_or_mode_changed=(generation != self._generation or visual != self._visual_generation
                                      or self.mode != captured_mode),
                source_generation_changed=source_generation != self._source_generation,
                continuity_broken=(observation_generation != self._yolo_diagnostic_generation
                    or stop_generation != self._runtime_stop_generation or self._closed
                    or bool(getattr(self.camera, "last_error", None))
                    or self._observed_at is None or not self._fresh(self._observed_at, completed_at)))
        # Save returned candidates before any business freshness/context rejection.
        # The observation generation also rejects a late empty result after a turn.
        try:
            writer.observe_yolo(captured, detail.get("yolo_candidates", ()), diagnostics)
        except Exception as exc:
            runlog.exception("competition_obstacle_evidence", "障碍诊断证据入队失败", exc)

    def _detect_yolo_obstacle(self, bgr, detail, *, expected_generation):
        """Use one clipped original-image box; classes share one obstacle policy."""
        detail.update(inference_attempted=False, inference_status="unavailable", yolo_candidates=[])
        if self.yolo_recovery_needed or not self._yolo_lock.acquire(blocking=False):
            return obstacle.ObstacleResult.unavailable(self.yolo_prepare_reason or "YOLO障碍模型尚未准备完成")
        try:
            with self._condition:
                if (self._closed or self._navigation_turn_active or self.mode != FRONT
                        or expected_generation != self._generation):
                    detail["inference_status"] = "skipped"
                    return obstacle.ObstacleResult.unavailable("障碍识别已暂停或图像代次已失效")
                detail["inference_attempted"] = True
            detections = self._yolo_detector.infer_bgr(bgr, timeout_s=.5)
            detail["inference_status"] = "returned"
        except Exception as exc:
            detail["inference_status"] = "timeout" if isinstance(exc, TimeoutError) else "error"
            reason = "YOLO障碍检测失败：" + str(exc)
            if isinstance(exc, TimeoutError) or getattr(self._yolo_detector, "_disabled", False) is True:
                with self._condition:
                    self._yolo_prepared = False
                    self._yolo_prepare_failed = True
                    self.yolo_prepare_reason = reason
                    self._latch_runtime_stop_locked(reason, ("yolo-detector", id(self._yolo_detector)))
                    self._invalidate_yolo_evidence_locked()
                    self._recover_startup(reason)
            return obstacle.ObstacleResult.unavailable(reason)
        finally:
            self._yolo_lock.release()
        height, width = bgr.shape[:2]
        choices = []
        for detection in detections:
            score = float(detection.score)
            box = tuple(float(value) for value in detection.box)
            if (detection.class_id not in (0, 1, 2) or not math.isfinite(score)
                    or score < .6 or not all(math.isfinite(value) for value in box)):
                continue
            x1, y1, x2, y2 = box
            x1, x2 = max(0., min(width, x1)), max(0., min(width, x2))
            y1, y2 = max(0., min(height, y1)), max(0., min(height, y2))
            area = max(0., x2 - x1) * max(0., y2 - y1)
            if area <= 0.:
                continue
            detail["yolo_candidates"].append(dict(class_id=int(detection.class_id), score=score,
                box=(x1, y1, x2, y2), area_ratio=area / (width * height)))
            if score < .8:
                continue
            result = obstacle.ObstacleResult(True, "yolo_obstacle", area,
                (int(x1), int(y1), math.ceil(x2) - int(x1), math.ceil(y2) - int(y1)),
                ((x1 + x2) / 2., (y1 + y2) / 2.), True)
            choices.append((y2 >= height * .95, area, score, result))
        if not choices:
            detail.update(target_kind=None, stop_eligible=False)
            return obstacle.ObstacleResult(False, None, 0., None, None, True)
        stop_eligible, area, score, result = max(choices, key=lambda item: item[:3])
        detail.update(profile="yolo", target_kind="yolo_obstacle", stop_eligible=stop_eligible,
                      score=score, judgment_area_ratio=area / (width * height))
        return result

    def publish_scan(self, captured, detections, fps=0.):
        """Publish boxed parked evidence without feeding it to obstacle recognition."""
        if not self.preview_enabled:
            return
        import cv2
        output = annotate(captured.bgr, tuple(detections), fps)
        ok, encoded = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self._condition:
            if (self.mode == SCAN and captured.source_generation == self._source_generation
                    and self._source_generation_current_locked()):
                self._last_obstacle_jpeg = encoded.tobytes()
                self._last_obstacle_id += 1

    def _capture_loop(self):
        while not self._stop.is_set():
            try:
                captured = self.camera.grab_frame()
                if captured is not None:
                    self._ingest_frame(captured)
                self.poll()
                if self.preview_enabled and self._server is None and time.monotonic() >= self._preview_retry_at:
                    self._preview_retry_at = time.monotonic() + 1.
                    try:
                        server = stage2._Stage2PreviewServer((self.host, self.port), self)
                        with self._condition:
                            if self._closed:
                                server.server_close()
                                return
                            self._server, self.port = server, server.server_port
                            self._preview_error = None
                        self._http_thread = threading.Thread(target=self._serve, name="competition-http", daemon=True)
                        self._threads.append(self._http_thread)
                        self._http_thread.start()
                    except Exception as exc:
                        self._preview_failed("HTTP预览等待恢复：" + str(exc))
            except Exception as exc:
                self._fault("摄像头取帧异常：" + str(exc))
            self._flush_diagnostics()
            self._stop.wait(.005)


@dataclass
class ScanSideState:
    status: str = "PENDING"
    class_id: int | None = None
    announced: bool = False
    healthy_elapsed_s: float = 0.
    request_id: str | None = None
    started_at: float | None = None
    deadline_at: float | None = None
    outcome: str = "PENDING"
    reason: str | None = None

    @property
    def completed(self):
        return self.status == "DONE"


@dataclass
class ScanState:
    road_id: str
    sides: dict[str, ScanSideState] = field(default_factory=lambda: {
        "left": ScanSideState(), "right": ScanSideState()})
    completed: bool = False
    forward_restored: bool = False
    observed: bool = True
    scan_status: str = "PENDING"
    skip_reason: str | None = None


@dataclass(frozen=True)
class ScanOutcome:
    road_id: str
    completed: bool
    class_ids: tuple[int, ...]
    forward_restored: bool
    failed_sides: tuple[str, ...] = ()


class CompetitionScanner:
    """Resumable parked Stage3 scan; the poll hook owns strict motor safety."""

    def __init__(self, vision, gimbal, detector, audio_callback=None, *, confidence=.8,
                 pose_callback=None):
        if not math.isfinite(confidence) or not .8 <= confidence <= 1.:
            raise ValueError("confidence must be 0.80..1")
        self.vision, self.gimbal, self.detector = vision, gimbal, detector
        self.audio_callback, self.confidence = audio_callback, confidence
        self.states = {}
        self.completed_roads = set()
        self._inference = None
        self.pose_callback = pose_callback

    def observe_road(self, road_id):
        return self.states.setdefault(road_id, ScanState(road_id))

    def skip_road(self, road_id, reason):
        state = self.states.setdefault(road_id, ScanState(road_id))
        if state.completed:
            return
        state.scan_status, state.skip_reason = "SKIPPED", str(reason)

    def _tick(self, hook):
        hook()
        self.vision.poll()

    def _move_camera(self, pan, hook, *, tilt=SCAN_TILT_US):
        self._tick(hook)
        try:
            if self.pose_callback is not None:
                self.pose_callback(pan, tilt)
            else:
                self.gimbal.set_pwm(pan, tilt)
        except Exception as exc:
            # A real motor/link fault is rechecked by the hook and propagates.
            hook()
            raise VisionUnavailable("云台等待恢复：" + str(exc)) from exc
        barrier = time.monotonic() + SCAN_SETTLE_S
        while time.monotonic() < barrier:
            self._tick(hook)
            time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
        return barrier, getattr(self.vision.camera, "generation", None)

    def _infer(self, frame, token):
        if self._inference is not None:
            if not self._inference["done"].is_set():
                raise VisionUnavailable("上一项推理尚未退出，保持停车等待", kind="inference")
            self._inference = None
        request = {"frame": frame, "token": token, "done": threading.Event()}
        self._inference = request
        def infer():
            try:
                request["hits"] = self.detector.infer_bgr(frame.bgr, timeout_s=config.DETECTOR_INFER_TIMEOUT_S)
            except Exception as exc:
                request["error"] = exc
            finally:
                request["done"].set()
        threading.Thread(target=infer, name="competition-parked-inference", daemon=True).start()
        return request

    def _scan_side(self, state, pan, hook):
        if state.deadline_at is not None and time.monotonic() >= state.deadline_at:
            state.status, state.outcome = "DONE", "FAILED"
            state.reason = state.reason or "识别期限已到，无可靠目标"
            return
        barrier, generation = self._move_camera(pan, hook)
        if state.deadline_at is None:
            state.started_at = barrier
            state.deadline_at = barrier + SCAN_SIDE_TIMEOUT_S
        streak = _Stage3Streak(self.confidence)
        previous_source = None
        last_sequence = None
        completed = deque(maxlen=240)
        while time.monotonic() < state.deadline_at:
            self._tick(hook)
            now = time.monotonic()
            if now >= state.deadline_at:
                break
            if generation != getattr(self.vision.camera, "generation", None):
                generation = getattr(self.vision.camera, "generation", None)
                barrier = now
                previous_source = last_sequence = None
                streak = _Stage3Streak(self.confidence)
                state.reason = "扫描期间相机代次变化"
            frame = self.vision.latest_frame(after=barrier)
            if frame is None:
                state.reason = "扫描期限内未取得新鲜相机帧"
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
                continue
            if time.monotonic() >= state.deadline_at:
                break
            if (frame.source_generation != generation or frame.captured_monotonic <= barrier
                    or not stage2.Stage2Vision._fresh(frame.captured_monotonic)):
                state.reason = "扫描帧不属于当前相机代次或姿态屏障"
                streak = _Stage3Streak(self.confidence)
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
                continue
            if last_sequence is not None and frame.sequence <= last_sequence:
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
                continue
            last_sequence = frame.sequence
            try:
                request = self._infer(frame, (generation, barrier))
            except VisionUnavailable as exc:
                state.reason = str(exc)
                streak = _Stage3Streak(self.confidence)
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
                continue
            while not request["done"].is_set():
                self._tick(hook)
                if (time.monotonic() >= state.deadline_at
                        or generation != getattr(self.vision.camera, "generation", None)):
                    break
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
            if time.monotonic() >= state.deadline_at:
                state.reason = "推理未在该侧识别期限内完成"
                break
            if request["done"].is_set():
                self._inference = None
            self._tick(hook)
            now = time.monotonic()
            if now >= state.deadline_at:
                state.reason = "推理结果晚于该侧识别期限"
                break
            if generation != getattr(self.vision.camera, "generation", None):
                state.reason = "推理完成时相机代次变化，废止旧结果"
                streak = _Stage3Streak(self.confidence)
                continue
            if request.get("error") is not None:
                state.reason = "目标识别失败：" + str(request["error"])
                streak = _Stage3Streak(self.confidence)
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
                continue
            if not stage2.Stage2Vision._fresh(frame.captured_monotonic, now):
                state.reason = "识别完成时图像已超龄"
                streak = _Stage3Streak(self.confidence)
                continue
            hits = request.get("hits")
            if not isinstance(hits, (list, tuple)):
                state.reason = "推理未返回合法目标列表"
                streak = _Stage3Streak(self.confidence)
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
                continue
            if previous_source is not None:
                if stage2.Stage2Vision._fresh(previous_source, frame.captured_monotonic):
                    state.healthy_elapsed_s += max(0., frame.captured_monotonic - previous_source)
                else:
                    streak = _Stage3Streak(self.confidence)
            previous_source = frame.captured_monotonic
            completed.append(now)
            elapsed = now - completed[0]
            fps = (len(completed) - 1) / elapsed if elapsed > 0 else 0.
            try:
                found = streak.observe(frame, hits, barrier, now)
            except (AttributeError, TypeError, ValueError) as exc:
                state.reason = "推理目标字段非法：" + str(exc)
                streak = _Stage3Streak(self.confidence)
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
                continue
            try:
                self.vision.publish_scan(frame, hits, fps)
            except Exception as exc:
                runlog.exception("competition_scan_preview", "扫描预览不可用，识别证据仍有效", exc)
            if found is not None:
                state.class_id = found
                state.status = "AUDIO_PENDING"
                state.outcome, state.reason = "SUCCESS", None
                return
        state.status, state.outcome = "DONE", "FAILED"
        state.reason = state.reason or "5秒内无可靠目标"

    def _restore(self, hook):
        barrier, _generation = self._move_camera(PAN_FORWARD_US, hook, tilt=TILT_US)
        return barrier

    def scan(self, context, poll_hook, *, audio_callback=None):
        context = _context(context)
        if context is None or context.dark_tunnel or not context.active:
            raise ValueError("only a localized ordinary road can be scanned")
        state = self.states.setdefault(context.road_id, ScanState(context.road_id))
        if state.scan_status == "SKIPPED":
            return ScanOutcome(context.road_id, False, tuple(
                side.class_id for side in state.sides.values() if side.class_id is not None),
                state.forward_restored)
        callback = audio_callback if audio_callback is not None else self.audio_callback
        self._tick(poll_hook)
        self.vision.set_mode(SCAN)
        state.forward_restored = False
        for vehicle_side, pan in (("left", 2500), ("right", 500)):
            canonical = context.canonical_side(vehicle_side)
            side = state.sides[canonical]
            if side.completed:
                continue
            if side.class_id is None:
                self._scan_side(side, pan, poll_hook)
                runlog.emit("competition_scan_side_result", "涵洞侧识别结束，任务结果与识别结果分别记录",
                            road=context.road_id, side=canonical, outcome=side.outcome,
                            reason=side.reason, class_id=side.class_id,
                            started_at=side.started_at, deadline_at=side.deadline_at)
            if side.class_id is not None and not side.announced:
                self._tick(poll_hook)
                side.request_id = side.request_id or f"culvert:{context.road_id}:{canonical}:{side.class_id}"
                if callback is None:
                    raise VisionUnavailable("目标音频服务未连接")
                try:
                    played = callback(side.request_id, side.class_id)
                except Exception as exc:
                    poll_hook()
                    raise VisionUnavailable("目标播报等待恢复：" + str(exc)) from exc
                if played is not True:
                    raise VisionUnavailable("目标播报尚未取得完成证据")
                side.announced = True
                side.status = "DONE"
                side.outcome = "SUCCESS"
        state.completed = all(side.completed for side in state.sides.values())
        if state.completed:
            state.scan_status = "DONE"
            self.completed_roads.add(context.road_id)
        barrier = self._restore(poll_hook)
        state.forward_restored = True
        front_ready = getattr(self.vision, "front_pose_ready", None)
        if front_ready is not None:
            front_ready(barrier)
        else:
            self.vision.set_mode(FRONT)
        return ScanOutcome(context.road_id, state.completed,
                           tuple(side.class_id for side in state.sides.values() if side.class_id is not None),
                           state.forward_restored,
                           tuple(name for name, side in state.sides.items() if side.outcome == "FAILED"))
