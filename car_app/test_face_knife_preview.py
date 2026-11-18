"""RDK静止人脸/刀具识别：固定PWM、带框照片保存及编号播报。"""
from __future__ import annotations
import argparse
from collections import deque
from datetime import datetime, timezone
import io
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import uuid
import audio
import clues
import config
import detector
import mcu
import motion_speed_input
import runlog
import test_motion_sequence as sequence
import test_stage2_obstacle as stage2
from drivers.protocol_v2 import (
    Capability, MessageType, MotionActionCode, StateFlag, StopReason, WireGimbalPose,
)
import ctypes
import multiprocessing
import signal
import zlib
from detector import Detection
import queue

PAN_US = 1500
TILT_US = 800
CONFIDENCE = .80
ABSENCE_S = 5.0
DEFAULT_OUTPUT_DIR = config.APP_ROOT / "资料" / "静止识别测试"
MODEL_DIRECTORY = config.APP_ROOT / "assets" / "peopleknife_yolo"


def operator_message(message):
    stage2._stage2_operator_message(str(message))


class ParkedLink(stage2.Stage2Link):
    """Use the existing command evidence without road/upper-range triggers."""

    def _handle(self, frame):
        if frame.message_type in (MessageType.DIRECTIONAL_RANGE4,
                                  MessageType.CULVERT_UPPER2, MessageType.HAZARD_STATE):
            return
        return super()._handle(frame)

    def _laser_stale_reason(self, *, require_upper=True):
        return None

    def receive_shutdown_tail(self, *, check_parked=None):
        if check_parked is None:
            return super().receive_shutdown_tail()
        if self._ser is None or not self.has_negotiated_capability(Capability.WHEEL_TRACE):
            return
        deadline = time.monotonic() + config.WHEEL_TRACE_TAIL_S
        complete = False
        try:
            while time.monotonic() < deadline:
                # Consume stop evidence while it is fresh, throughout the tail.
                check_parked()
                time.sleep(min(.005, max(0., deadline - time.monotonic())))
            complete = True
        finally:
            self._capture_call("update_metadata", shutdown_tail_complete=complete)
            self._capture_call("event", "shutdown_tail_end", complete=complete)

    def send_motion(self, linear_mmps, angular_mradps, **kwargs):
        if linear_mmps != 0 or angular_mradps != 0:
            raise mcu.McuError("静止识别测试禁止非零底盘命令")
        return super().send_motion(0, 0, **kwargs)

    def send_motion_action(self, action, speed_mms=0, **kwargs):
        if action is not MotionActionCode.BRAKE or speed_mms != 0:
            raise mcu.McuError("静止识别测试只允许零速BRAKE动作")
        return super().send_motion_action(action, 0, **kwargs)

    def send_gimbal(self, pose, pan_us, tilt_us):
        if (pose, pan_us, tilt_us) != (WireGimbalPose.LEVEL_CENTER, PAN_US, TILT_US):
            raise mcu.McuError("静止识别测试固定上下800、左右1500")
        return super().send_gimbal(pose, pan_us, tilt_us)


class ParkedController:
    """Receive every 20ms; no inference, disk write or audio wait owns UART."""

    def __init__(self, link, input_source=None):
        self.link, self.input_source = link, input_source
        self.tracker = None
        self.barrier = None
        self.identity = None
        self.pwm_confirmed = False

    def cancel_check(self):
        sequence._check_input_exit(self.input_source)

    def _receive(self):
        self.link.pump()
        states = self.link.drain_state_samples()
        traces = self.link.drain_wheel_trace_samples()
        self.link.drain_diagnostics_samples()
        return states, traces

    def _check_state(self, states=None, *, require_pwm=None):
        link = self.link
        if not link.connected or (link.session_nonce, link.boot_id) != self.identity:
            raise mcu.McuError("停车监测期间MCU连接或会话变化")
        state = link.state
        if state is None or not link._sample_is_fresh("state"):
            raise mcu.McuError("停车监测缺少新鲜STATE")
        for candidate in [state, *(item[0] for item in (states or []))]:
            if (candidate.session_nonce, candidate.boot_id) != self.identity:
                raise mcu.McuError("停车STATE身份不匹配")
            if candidate.target_linear_mms or candidate.target_angular_mrads:
                raise mcu.McuError("停车期间底盘目标非零")
            if candidate.state_flags & StateFlag.TURN_ACTIVE:
                raise mcu.McuError("停车期间仍有转弯动作")
            if candidate.odom_status != 0 or not candidate.state_flags & StateFlag.ODOM_VALID:
                raise mcu.McuError("停车期间轮程无效")
            check_pwm = self.pwm_confirmed if require_pwm is None else require_pwm
            if check_pwm:
                reason = stage2.Stage2Link._stage2_gimbal_reason(
                    candidate, int(WireGimbalPose.LEVEL_CENTER), self.identity)
                if reason:
                    raise mcu.McuError(reason)

    def tick(self):
        self.cancel_check()
        states, traces = self._receive()
        reason = self.link.motion_health_reason()
        if reason:
            raise mcu.McuError(reason)
        self._check_state(states)
        if self.tracker is None:
            raise mcu.McuError("尚未取得停车证据")
        # Inspect all events, so a move then return within one UART drain fails.
        tail_before = self.tracker.left_sum, self.tracker.right_sum
        stable = self.tracker.update(self.link, states, traces)
        moving_trace = any(trace.left_encoder_delta or trace.right_encoder_delta
                           or getattr(trace, "left_applied_output", 0)
                           or getattr(trace, "right_applied_output", 0)
                           or getattr(trace, "left_target_command", 0)
                           or getattr(trace, "right_target_command", 0)
                           for trace, at in traces
                           if self.tracker.source_barrier < at <= time.monotonic())
        if (not stable or tail_before != (self.tracker.left_sum, self.tracker.right_sum)
                or moving_trace):
            raise mcu.McuError("停车证据失效或车轮发生运动")
        return self.barrier

    def brake(self, *, allow_fault=False, cancellable=True):
        link = self.link
        if cancellable:
            self.cancel_check()
        link.stage2_fault_stopping = allow_fault
        if allow_fault:
            link.stage2_gimbal_monitor = None
            link.stage2_gimbal_pending = None
        display = SimpleNamespace(action="静止识别BRAKE", set_stopped=lambda: None)
        sequence._apply_brake(link, display, time.monotonic() + 5.0)
        tracker = sequence._StopTracker(link, source_barrier=link._brake_evidence_barrier)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if cancellable:
                self.cancel_check()
            states, traces = self._receive()
            if not allow_fault:
                reason = link.motion_health_reason()
                if reason:
                    raise mcu.McuError(reason)
            self._check_state(states, require_pwm=False)
            if tracker.update(link, states, traces):
                self.tracker = link._stop_tracker = tracker
                self.barrier = link._stable_barrier = time.monotonic()
                runlog.emit("static_vision_parked", "双轮连续500ms停稳已确认",
                            source_barrier=tracker.source_barrier, stable_barrier=self.barrier,
                            odom_mm=link.odom_mm, observed_ms=tracker.observed_ms)
                return
            time.sleep(.02)
        raise mcu.McuError("5秒内未确认双轮连续500ms停稳")

    def prepare(self):
        link = self.link
        self.cancel_check()
        link.open()
        link.hello()
        self.identity = link.session_nonce, link.boot_id
        link.set_stage2_mode("stopped")
        link.synchronize_clock()
        # BRAKE first; allow startup telemetry calibration to settle at rest.
        display = SimpleNamespace(action="静止识别启动BRAKE", set_stopped=lambda: None)
        sequence._apply_brake(link, display, time.monotonic() + 5.0)
        deadline = time.monotonic() + config.MCU_SELFCHECK_READY_TIMEOUT_S
        while True:
            self.cancel_check()
            self._receive()
            self._check_state(require_pwm=False)
            reason = link.motion_health_reason()
            if reason is None:
                break
            if time.monotonic() >= deadline or link._hard_fault_reason:
                raise mcu.McuError(reason)
            time.sleep(.02)
        link.motion_checks_enabled = True
        self.brake()
        self.configure_pwm()

    def configure_pwm(self):
        self.tick()
        command = self.link.send_gimbal(WireGimbalPose.LEVEL_CENTER, PAN_US, TILT_US)
        self.link.wait_for_control_applied(command, timeout_s=config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
        self.pwm_confirmed = True
        end = time.monotonic() + config.SERVO_SETTLE_S
        while time.monotonic() < end:
            self.tick()
            time.sleep(.02)
        # PWM changed applied sequence; obtain a new BRAKE and stop anchor.
        self.brake()
        self.tick()

    def shutdown(self, *, failed=False):
        if not self.link.connected:
            return False
        try:
            self.brake(allow_fault=failed, cancellable=False)
            if self.pwm_confirmed:
                self._check_state(require_pwm=True)
            return True
        except (Exception, KeyboardInterrupt) as exc:
            runlog.exception("static_vision_exit_park_failed", "退出停车未确认", exc)
            self.link.send_stop(StopReason.CLOSE)
            return False


def annotate(bgr, detections, fps):
    """Draw even a valid subpixel/border box; leave the captured image intact."""
    import cv2
    image = bgr.copy()
    height, width = image.shape[:2]
    for hit in detections:
        x1, y1, x2, y2 = hit.box
        x1 = max(0, min(width - 1, math.floor(x1)))
        y1 = max(0, min(height - 1, math.floor(y1)))
        x2 = max(x1, min(width - 1, math.ceil(x2)))
        y2 = max(y1, min(height - 1, math.ceil(y2)))
        suspect = hit.class_id <= 10
        kind, number = ("Suspect", hit.class_id) if suspect else ("Knife", hit.class_id - 10)
        color = (80, 220, 80) if suspect else (40, 180, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f"{kind} {number} | class {hit.class_id} | {hit.score:.2f}",
                    (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, color, 2, cv2.LINE_AA)
    cv2.putText(image, f"Inference FPS: {fps:.1f}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2, cv2.LINE_AA)
    return image


class CameraFeed:
    """Read camera IPC away from the UART loop; publish one immutable reference."""

    def __init__(self, source):
        self.source = source
        self.latest = None
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name="static-camera-feed", daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            try:
                self.latest = self.source.grab_frame()
                self.error = None
            except Exception as exc:
                self.latest = None
                self.error = str(exc)
                self.source.request_retry(self.error)
            self.stop.wait(.01)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=.2)





class _StaticMessageSlot:
    """Single-writer bounded mailbox with version and CRC, never a shared lock."""

    def __init__(self, context, capacity=32 * 1024 * 1024):
        self.capacity = capacity
        self.buffer = context.RawArray(ctypes.c_ubyte, capacity)
        self.version = context.RawValue(ctypes.c_uint64, 0)
        self.length = context.RawValue(ctypes.c_uint32, 0)
        self.crc = context.RawValue(ctypes.c_uint32, 0)

    def publish(self, fields):
        data = json.dumps(fields, ensure_ascii=False, allow_nan=False,
                          separators=(",", ":")).encode("utf-8")
        if len(data) > self.capacity:
            raise RuntimeError(f"识别结果容量超限：{len(data)} > {self.capacity} 字节；未截断")
        version = int(self.version.value)
        odd = version + 1 if not version % 2 else version + 2
        self.version.value = odd
        ctypes.memmove(ctypes.addressof(self.buffer), data, len(data))
        self.length.value = len(data)
        self.crc.value = zlib.crc32(data)
        self.version.value = odd + 1

    def read(self):
        for _ in range(2):
            before = int(self.version.value)
            length = int(self.length.value)
            crc = int(self.crc.value)
            if not before or before % 2 or not 0 < length <= self.capacity:
                continue
            payload = ctypes.string_at(ctypes.addressof(self.buffer), length)
            if before != int(self.version.value) or zlib.crc32(payload) != crc:
                continue
            try:
                return json.loads(payload)
            except (UnicodeError, ValueError):
                continue
        return None


def _static_detector_factory():
    from detector import Detector
    return Detector(model_bin=str(MODEL_DIRECTORY / "peopleknife_bayese_640x640_nv12.bin"),
                    manifest_path=MODEL_DIRECTORY / "runtime-manifest.json",
                    conf_thresh=CONFIDENCE, profile_timing=True)


def _static_detection_rows(detections):
    rows = []
    for detection in detections:
        class_id = int(detection.class_id)
        score = float(detection.score)
        box = tuple(float(value) for value in detection.box)
        if (not 1 <= class_id <= 20 or not math.isfinite(score) or not 0 <= score <= 1
                or len(box) != 4 or not all(math.isfinite(value) for value in box)):
            raise RuntimeError("YOLO返回非法类别、分数或识别框")
        rows.append([class_id, score, box])
    return rows


def _static_detector_process(factory, frame_slot, request_slot, result_slot,
                             status, stop, generation, load_timeout, infer_timeout,
                             log_directory=None):
    """One process owns all model/native calls, including model destruction."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    current = None
    owner_log = None
    try:
        if log_directory is not None:
            owner_log = runlog.RunLog(
                mode="static_face_knife_detector",
                directory=Path(log_directory) / f"detector-{generation}",
                terminal=io.StringIO(), capture=False, periodic_status=False).__enter__()
        status.publish(phase="loading", started_at=time.monotonic())
        current = factory()
        current.load(timeout_s=load_timeout)
        if factory is _static_detector_factory:
            # The shared Detector honors manifest max_detections (normally 300).
            # This entrypoint promises every qualifying post-NMS target. The
            # sum of class-logit candidates is a strict upper bound, so its
            # private instance never silently applies the manifest top-k cap.
            current._max_detections = sum(
                spec.shape[1] * spec.shape[2] * spec.shape[3]
                for spec in current._output_schema[::2])
        status.publish(phase="ready")
        seen = 0
        while not stop.value:
            request = request_slot.read()
            if request is None or request.get("request_id", 0) <= seen:
                time.sleep(.005)
                continue
            seen = request["request_id"]
            if request.get("owner_generation") != generation:
                raise RuntimeError("推理请求进程代次不匹配")
            captured = frame_slot.read(request["source_generation"])
            if (captured is None or captured.sequence != request["sequence"]
                    or captured.captured_monotonic != request["captured_monotonic"]
                    or captured.source_generation != request["source_generation"]):
                raise RuntimeError("推理请求帧身份不匹配或帧已过期")
            status.publish(phase="inferring", request_id=seen, started_at=time.monotonic())
            detections = current.infer_bgr(captured.bgr, timeout_s=infer_timeout)
            if stop.value:
                break
            result_slot.publish(dict(request, detections=_static_detection_rows(detections),
                                     diagnostics=dict(getattr(current, "last_diagnostics", {}))))
            status.publish(phase="ready", request_id=seen)
    except Exception as exc:
        import traceback
        status.publish(phase="failed", error=str(exc), error_type=type(exc).__name__,
                       traceback=traceback.format_exc())
    finally:
        if current is not None:
            try:
                current.close()
            except Exception as exc:
                status.publish(phase="failed", error=f"关闭YOLO失败：{exc}")
        if owner_log is not None:
            owner_log.__exit__(None, None, None)


class DetectorOwner:
    """Background supervision keeps the stationary UART loop outside native/IPC waits."""

    RETRY_INTERVAL_S = 1.0
    OWNER_JOIN_S = .2
    IPC_GRACE_S = .5
    STARTUP_GRACE_S = 2.0
    RESULT_CAPACITY = 32 * 1024 * 1024

    def __init__(self, factory=None, context=None):
        self._factory = factory or _static_detector_factory
        self._context = context or multiprocessing.get_context("spawn")
        self._load_timeout = float(getattr(config, "DETECTOR_LOAD_TIMEOUT_S", 15.0))
        self._infer_timeout = 1.0
        self._lock = threading.Lock()  # Parent-only; the child never receives it.
        self._stop = threading.Event()
        self._thread = self._process = None
        self._frame_slot = self._request_slot = self._result_slot = None
        self._status = self._owner_stop = None
        self._inflight = self._result = None
        self._ready = False
        self._generation = 0
        self._last_error = "正在启动YOLO识别"
        self._request_id = 0
        self._retry_at = 0.0
        self._started_at = 0.0
        self._last_status = {}
        self._last_status_at = None

    @property
    def ready(self):
        return self._ready and not self._stop.is_set()

    @property
    def generation(self):
        return self._generation

    @property
    def last_error(self):
        return self._last_error

    def start(self):
        if self._thread is None and not self._stop.is_set():
            self._thread = threading.Thread(target=self._run, name="static-yolo-supervisor", daemon=True)
            self._thread.start()

    def submit(self, captured):
        if not self.ready or not self._lock.acquire(blocking=False):
            return False
        try:
            if (not self.ready or self._inflight is not None or self._result is not None
                    or self._frame_slot is None):
                return False
            source_generation = getattr(captured, "source_generation", None)
            if (isinstance(source_generation, bool) or not isinstance(source_generation, int)
                    or not 0 <= source_generation < 2 ** 64):
                raise ValueError("摄像头代次非法")
            if not self._frame_slot.publish(captured):
                raise ValueError(f"推理请求帧无效：{self._frame_slot.rejection}")
            self._request_id += 1
            identity = dict(request_id=self._request_id, owner_generation=self._generation,
                            source_generation=source_generation, sequence=captured.sequence,
                            captured_monotonic=captured.captured_monotonic)
            self._inflight = (captured, identity, time.monotonic())
            self._ready = False
            self._request_slot.publish(identity)
            return True
        except Exception as exc:
            self._inflight = None
            self._ready = False
            self._last_error = str(exc)
            # A bad/torn publication retires this generation as well. The
            # supervisor consumes the failure without blocking the caller.
            if self._owner_stop is not None:
                self._owner_stop.value = 1
            return False
        finally:
            self._lock.release()

    def poll(self):
        if self._stop.is_set() or not self._lock.acquire(blocking=False):
            return None
        try:
            result = self._result
            self._result = None
            if result is not None and result.owner_generation == self._generation:
                self._ready = self._last_error is None
                return result
            return None
        finally:
            self._lock.release()

    def _fault(self, reason):
        with self._lock:
            self._ready = False
            self._last_error = str(reason)
            self._inflight = self._result = None
        self._retry_at = time.monotonic() + self.RETRY_INTERVAL_S

    def _launch(self):
        frame_slot = stage2._Stage2FrameSlot(self._context)
        request_slot = _StaticMessageSlot(self._context, 4096)
        result_slot = _StaticMessageSlot(self._context, self.RESULT_CAPACITY)
        status = stage2._Stage2CameraStatus(self._context)
        owner_stop = self._context.RawValue(ctypes.c_int, 0)
        generation = self._generation + 1
        log_directory = runlog.current_directory()
        process = self._context.Process(
            target=_static_detector_process,
            args=(self._factory, frame_slot, request_slot, result_slot, status, owner_stop,
                  generation, self._load_timeout, self._infer_timeout,
                  str(log_directory) if log_directory is not None else None),
            name=f"static-yolo-owner-{generation}", daemon=True)
        with self._lock:
            self._generation = generation
            self._frame_slot, self._request_slot, self._result_slot = frame_slot, request_slot, result_slot
            self._status, self._owner_stop = status, owner_stop
            self._process = process
            self._started_at = time.monotonic()
            self._last_status = {}
            self._last_status_at = None
            self._last_error = "正在加载YOLO模型"
        process.start()

    def _retire(self):
        process = self._process
        if process is None:
            return True
        self._owner_stop.value = 1
        try:
            process.join(self.OWNER_JOIN_S)
            if process.is_alive():
                process.terminate()
                process.join(self.OWNER_JOIN_S)
            if process.is_alive():
                process.kill()
                process.join(self.OWNER_JOIN_S)
            if process.is_alive():
                self._last_error = "旧YOLO进程尚未退出，保持停车，暂不重建"
                return False
            process.close()
        except Exception as exc:
            # A process whose start failed has no native owner to retire.
            if getattr(process, "pid", None) is not None:
                self._last_error = f"回收YOLO进程失败：{exc}"
                return False
        with self._lock:
            self._process = self._frame_slot = self._request_slot = self._result_slot = None
            self._status = self._owner_stop = None
        return True

    def _supervise(self):
        now = time.monotonic()
        status = self._status.read()
        phase = status.get("phase")
        if phase is not None:
            self._last_status, self._last_status_at = status, now
        elif self._last_status_at is not None:
            # A concurrent status publication can make one read inconclusive.
            status = self._last_status
            phase = status.get("phase")
        if self._owner_stop.value:
            return self._last_error or "YOLO推理进程请求重启"
        if phase == "failed":
            return status.get("error") or "YOLO推理失败"
        if not self._process.is_alive():
            return "YOLO进程意外退出"
        if self._last_status_at is not None and now - self._last_status_at >= self.IPC_GRACE_S:
            return "YOLO状态IPC超时"
        if phase is None and now - self._started_at >= self._load_timeout + self.STARTUP_GRACE_S:
            return "YOLO进程启动或IPC超时"
        if phase == "loading" and now - status["started_at"] >= self._load_timeout + self.IPC_GRACE_S:
            return "YOLO模型加载超时"
        with self._lock:
            pending = self._inflight
        if pending is not None:
            captured, identity, submitted_at = pending
            result = self._result_slot.read()
            if result is not None and all(result.get(key) == value for key, value in identity.items()):
                detections = tuple(Detection(row[0], row[1], tuple(row[2]))
                                   for row in result["detections"])
                with self._lock:
                    if self._inflight is pending and not self._stop.is_set():
                        self._result = SimpleNamespace(captured=captured, detections=detections,
                                                       owner_generation=self._generation,
                                                       diagnostics=dict(result.get("diagnostics", {})))
                        self._inflight = None
                return None
            if now - submitted_at >= self._infer_timeout + self.IPC_GRACE_S:
                return "YOLO推理或结果IPC超时"
        elif phase == "ready":
            with self._lock:
                self._last_error = None
                self._ready = (self._result is None and self._inflight is None
                               and not self._stop.is_set())
        return None

    def _run(self):
        retiring = False
        try:
            while not self._stop.is_set():
                if retiring:
                    if self._retire():
                        retiring = False
                    else:
                        self._stop.wait(.05)
                        continue
                if self._process is None:
                    if time.monotonic() < self._retry_at:
                        self._stop.wait(.01)
                        continue
                    try:
                        self._launch()
                    except Exception as exc:
                        self._fault(f"启动YOLO进程失败：{exc}")
                        retiring = True
                        continue
                try:
                    failure = self._supervise()
                except Exception as exc:
                    failure = f"YOLO监督失败：{exc}"
                if failure:
                    self._fault(failure)
                    retiring = True
                self._stop.wait(.005)
        finally:
            self._ready = False
            self._retire()

    def close(self):
        self._stop.set()
        self._ready = False
        if self._thread is not None:
            self._thread.join(timeout=3 * self.OWNER_JOIN_S + .5)
        if self._lock.acquire(blocking=False):
            try:
                self._inflight = self._result = None
            finally:
                self._lock.release()




class RecognitionPolicy:
    """Only two advancing, reliable winner frames authorize an announcement."""

    def __init__(self, confidence=.8, absence_s=5.0):
        self.confidence = float(confidence)
        self.absence_s = float(absence_s)
        self._generation = None
        self._sequence = None
        self._captured_at = None
        self._observed_at = None
        self._winner = None
        self._streak = 0
        self._confirmation_barrier = float('-inf')
        self._announced = {}
        self._completed = set()

    @property
    def announced_ids(self):
        return frozenset(self._completed)

    @property
    def active_ids(self):
        return frozenset(self._announced)

    def reset_confirmation(self, after_monotonic=None):
        self._winner = None
        self._streak = 0
        if after_monotonic is not None:
            self._confirmation_barrier = max(self._confirmation_barrier, float(after_monotonic))

    def suspend(self):
        self.reset_confirmation()
        for class_id in self._announced:
            self._announced[class_id] = None
        self._observed_at = None

    def mark_announced(self, class_id):
        self.mark_triggered(class_id)
        self._completed.add(int(class_id))

    def mark_triggered(self, class_id):
        self._announced.setdefault(int(class_id), None)
        self.reset_confirmation()

    def valid_detections(self, detections, bgr):
        try:
            height, width = bgr.shape[:2]
            if height <= 0 or width <= 0:
                return ()
        except (AttributeError, TypeError, ValueError):
            return ()
        valid = []
        for hit in detections or ():
            try:
                raw_id = hit.class_id
                class_id = int(raw_id)
                score = float(hit.score)
                box = tuple(float(value) for value in hit.box)
                if (isinstance(raw_id, bool) or raw_id != class_id or not 1 <= class_id <= 20
                        or not math.isfinite(score) or not self.confidence <= score <= 1.
                        or len(box) != 4 or not all(math.isfinite(value) for value in box)):
                    continue
                x1, y1, x2, y2 = box
                if x1 >= x2 or y1 >= y2:
                    continue
                box = (max(0., min(float(width), x1)), max(0., min(float(height), y1)),
                       max(0., min(float(width), x2)), max(0., min(float(height), y2)))
                if box[0] >= box[2] or box[1] >= box[3]:
                    continue
                valid.append(Detection(class_id, score, box))
            except (AttributeError, TypeError, ValueError, OverflowError):
                continue
        return tuple(valid)

    def observe(self, captured, detections, now=None, blocked=False):
        now = time.monotonic() if now is None else float(now)
        try:
            at = float(captured.captured_monotonic)
            generation = int(captured.source_generation)
            sequence = int(captured.sequence)
            if (not math.isfinite(at) or not math.isfinite(now)
                    or not 0. <= now - at <= config.CAMERA_FRAME_MAX_AGE_S):
                self.suspend()
                return None
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.suspend()
            return None
        if self._generation is not None and generation < self._generation:
            return None
        if generation == self._generation:
            if sequence <= self._sequence or at <= self._captured_at:
                return None
            if (at - self._captured_at > .5 or self._observed_at is None
                    or now - self._observed_at > .5):
                self.suspend()
        else:
            self.suspend()
        self._generation, self._sequence = generation, sequence
        self._captured_at, self._observed_at = at, now
        hits = self.valid_detections(detections, captured.bgr)
        present = {hit.class_id for hit in hits}
        for class_id, absent_since in tuple(self._announced.items()):
            if class_id in present:
                self._announced[class_id] = None
            elif absent_since is None:
                self._announced[class_id] = at
            elif at - absent_since >= self.absence_s:
                del self._announced[class_id]
                self._completed.discard(class_id)
        if blocked or at <= self._confirmation_barrier or not hits:
            self.reset_confirmation()
            return None
        winner = min(hits, key=lambda hit: (-hit.score, hit.class_id))
        if winner.class_id in self._completed:
            self.reset_confirmation()
            return None
        if self._winner == winner.class_id:
            self._streak += 1
        else:
            self._winner, self._streak = winner.class_id, 1
        return winner if self._streak >= 2 else None


class AnnouncementController:
    """Keep photo and real audio completion separate for each appearance."""

    def __init__(self, speaker, clues_source, snapshots, emitter=None, policy=None):
        self.speaker = speaker
        self.clues_source = clues_source
        self.snapshots = snapshots
        self.emitter = operator_message if emitter is None else emitter
        self.policy = policy or RecognitionPolicy()
        self._ticket = None
        self._effects = {}
        self._pending = {}
        self._retry_at = 0.
        self._error_report_at = 0.
        self._closed = False
        self._unowned_active = False

    @property
    def busy(self):
        return self._ticket is not None or self._unowned_active

    @property
    def blocks_confirmation(self):
        return self._ticket is not None and not self._unowned_active

    def _failed(self, message):
        now = time.monotonic()
        self._retry_at = now + 1.
        self.policy.reset_confirmation(after_monotonic=now)
        if now >= self._error_report_at:
            self.emitter("播报失败：" + message)
            self._error_report_at = now + 1.
        runlog.emit('static_preview_audio_failed', message, terminal=False)

    def _complete(self, state):
        class_id = state['class_id']
        if (state['photo_done'] and state['audio_done']
                and self._effects.get(class_id) is state and class_id in self.policy.active_ids):
            self.policy.mark_announced(class_id)
            self.policy.reset_confirmation(after_monotonic=time.monotonic())

    def _drain_snapshots(self):
        for receipt in self.snapshots.drain_receipts():
            state = self._pending.pop(receipt['request_id'], None)
            if state is None:
                continue
            state['photo_pending'] = False
            if receipt['ok']:
                state['photo_done'] = True
                self._complete(state)
            else:
                self._retry_at = max(self._retry_at, time.monotonic() + 1.)
                self.policy.reset_confirmation(after_monotonic=time.monotonic())

    def tick(self):
        if self._closed:
            return
        self._drain_snapshots()
        try:
            active = bool(self.speaker.poll())
        except Exception as exc:
            if not self._unowned_active:
                self._failed(str(exc))
            # Keep ownership until close proves the old player has exited.
            self._unowned_active = True
            return
        ticket = self._ticket
        if ticket is None:
            self._unowned_active = active
            return
        if self.speaker.failed_count != ticket['failed']:
            if not ticket['failure_reported']:
                ticket['failure_reported'] = True
                self._failed(str(self.speaker.last_error or '播放器失败'))
            if not active:
                self._ticket = None
                self._unowned_active = False
                self.policy.reset_confirmation(after_monotonic=time.monotonic())
            return
        if active:
            return
        self._ticket = None
        self._unowned_active = False
        if self.speaker.completed_count == ticket['completed'] + 1:
            ticket['state']['audio_done'] = True
            self._complete(ticket['state'])
            self.policy.reset_confirmation(after_monotonic=time.monotonic())
            runlog.emit('static_preview_audio_completed', '目标播报实际完成', terminal=False,
                        class_id=ticket['state']['class_id'])
        else:
            self._failed('缺少音频实际完成证据')

    def offer(self, hit, captured, annotated):
        if (self._closed or self.blocks_confirmation
                or (not self._unowned_active and time.monotonic() < self._retry_at)):
            return False
        class_id = int(hit.class_id)
        state = self._effects.get(class_id)
        if class_id not in self.policy.active_ids or state is None:
            state = dict(class_id=class_id, photo_done=False, audio_done=False,
                         photo_pending=False)
            self._effects[class_id] = state
            self.policy.mark_triggered(class_id)
        self.policy.reset_confirmation()
        clue = self.clues_source.detection(class_id)
        metadata = dict(class_id=class_id,
                        category=getattr(clue, 'category', 'SUSPECT' if class_id <= 10 else 'KNIFE'),
                        number=getattr(clue, 'number', class_id if class_id <= 10 else class_id - 10),
                        score=float(hit.score), sequence=int(captured.sequence),
                        source_generation=int(captured.source_generation),
                        captured_monotonic=float(captured.captured_monotonic),
                        captured_wall_time=datetime.fromtimestamp(
                            time.time() - max(0., time.monotonic() - captured.captured_monotonic),
                            timezone.utc).isoformat(),
                        wall_time=datetime.now(timezone.utc).isoformat())
        started_effect = False
        if not state['photo_done'] and not state['photo_pending']:
            request_id = uuid.uuid4().hex
            try:
                accepted = self.snapshots.submit(annotated, dict(metadata, snapshot_request_id=request_id))
            except Exception as exc:
                accepted = False
                self.emitter('照片保存失败：' + str(exc))
                runlog.exception('static_preview_snapshot_submit_failed', '照片任务提交失败', exc,
                                 terminal=False)
            if accepted:
                state['photo_pending'] = True
                self._pending[request_id] = state
                started_effect = True
            else:
                self._retry_at = time.monotonic() + 1.
                self.policy.reset_confirmation(after_monotonic=time.monotonic())
        if state['audio_done'] or self.busy:
            return started_effect
        path = self.clues_source.detection_audio_path(class_id)
        if clue is None or path is None or not Path(path).is_file():
            self._failed(f'编号{class_id}没有可用音频，照片仍独立保存')
            return started_effect
        completed, failed, started = (self.speaker.completed_count,
                                      self.speaker.failed_count, self.speaker.started_count)
        try:
            accepted = self.speaker.play(path)
        except Exception as exc:
            self._failed(str(exc))
            return started_effect
        actually_started = self.speaker.started_count == started + 1
        if not accepted or not actually_started or self.speaker.failed_count != failed:
            # A queued player is not completion evidence; do not accumulate work.
            try:
                self.speaker.close()
            except Exception as exc:
                self._unowned_active = True
                runlog.exception('static_preview_audio_close_failed', '未启动音频清理失败', exc,
                                 terminal=False)
            self._failed(str(self.speaker.last_error or '目标音频没有实际启动'))
            return started_effect
        self._ticket = dict(state=state, completed=completed, failed=failed,
                            failure_reported=False)
        self.emitter(f'开始播报：{clue.text}')
        runlog.emit('static_preview_audio_started', '目标音频实际启动', terminal=False, **metadata)
        return True

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._ticket = None
        self._unowned_active = False
        self.policy.reset_confirmation()
        self.speaker.close()


class SnapshotWriter:
    """Bound outstanding disk work and deliver receipts on the control thread."""

    def __init__(self, directory, emitter=None, capacity=8):
        self.directory = Path(directory).resolve()
        self.emitter = operator_message if emitter is None else emitter
        capacity = max(1, int(capacity))
        self._queue = queue.Queue(maxsize=capacity)
        self._receipts = queue.Queue(maxsize=capacity)
        self._slots = threading.BoundedSemaphore(capacity)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._errors = []
        self._saved_count = 0
        self._closed = False
        self._thread = threading.Thread(target=self._run, name='static-preview-snapshots', daemon=True)
        self._thread.start()

    @property
    def errors(self):
        with self._lock:
            return tuple(self._errors)

    @property
    def saved_count(self):
        with self._lock:
            return self._saved_count

    def _error(self, message):
        with self._lock:
            self._errors.append(message)
        self.emitter('照片保存失败：' + message)
        runlog.emit('static_preview_snapshot_failed', message, terminal=False)

    def submit(self, image, metadata):
        if self._closed:
            self._error('保存器已经关闭')
            return False
        if not self._slots.acquire(blocking=False):
            self._error('照片保存队列已满，此次照片未保存')
            return False
        try:
            document = dict(metadata)
            document.setdefault('snapshot_request_id', uuid.uuid4().hex)
            self._queue.put_nowait((image.copy(), document))
        except Exception as exc:
            self._slots.release()
            self._error(str(exc))
            return False
        return True

    def _write(self, image, metadata):
        import cv2
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            raise OSError('JPEG编码失败')
        diagnostic = metadata.get('snapshot_kind') == 'diagnostic_no_hit'
        directory = self.directory / 'diagnostics' if diagnostic else self.directory
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S-%f')
        label = 'diagnostic-no-hit' if diagnostic else f"class{int(metadata['class_id']):02d}"
        stem = (f"{stamp}-{label}"
                f"-g{int(metadata['source_generation'])}-s{int(metadata['sequence'])}-{uuid.uuid4().hex}")
        image_path = directory / (stem + '.jpg')
        metadata_path = directory / (stem + '.json')
        document = dict(metadata, image_file=image_path.name,
                        saved_wall_time=datetime.now(timezone.utc).isoformat())
        json_bytes = json.dumps(document, ensure_ascii=False, allow_nan=False, indent=2).encode('utf-8')
        created = []
        try:
            with image_path.open('xb') as output:
                created.append(image_path)
                output.write(encoded.tobytes())
            with metadata_path.open('xb') as output:
                created.append(metadata_path)
                output.write(json_bytes)
        except Exception:
            for path in created:
                path.unlink(missing_ok=True)
            raise
        if not diagnostic:
            with self._lock:
                self._saved_count += 1
        return dict(path=str(image_path), metadata_path=str(metadata_path))

    def drain_receipts(self):
        receipts = []
        while True:
            try:
                receipt = self._receipts.get_nowait()
            except queue.Empty:
                break
            self._slots.release()
            receipts.append(receipt)
            if receipt['ok']:
                message = ('诊断原图已保存' if receipt.get('snapshot_kind') == 'diagnostic_no_hit'
                           else '识别照片已保存')
                self.emitter(message + '：' + receipt['path'])
                runlog.emit('static_preview_snapshot_saved', message, terminal=False,
                            **receipt)
            else:
                self._error(receipt['error'])
        return tuple(receipts)

    def _run(self):
        while not self._stop.is_set() or not self._queue.empty():
            try:
                image, metadata = self._queue.get(timeout=.05)
            except queue.Empty:
                continue
            receipt = dict(request_id=metadata['snapshot_request_id'], class_id=metadata['class_id'])
            if 'snapshot_kind' in metadata:
                receipt['snapshot_kind'] = metadata['snapshot_kind']
            try:
                receipt.update(ok=True, **self._write(image, metadata))
            except Exception as exc:
                receipt.update(ok=False, error=str(exc))
            finally:
                self._receipts.put_nowait(receipt)
                self._queue.task_done()

    def close(self, timeout_s=1.):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        self._thread.join(max(0., float(timeout_s)))
        self.drain_receipts()
        if not self._thread.is_alive():
            return
        dropped = 0
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
            self._slots.release()
            self._queue.task_done()
            dropped += 1
        self._error(f'退出等待保存超过时限，已取消{dropped}张排队照片；正在写入任务由后台收尾')


class StaticRunner:
    def __init__(self, parked, camera_source, owner, policy, announcer, snapshots,
                 input_source=None, emitter=None):
        self.parked, self.camera, self.owner = parked, camera_source, owner
        self.policy, self.announcer, self.snapshots = policy, announcer, snapshots
        self.input_source = input_source
        self.emitter = operator_message if emitter is None else emitter
        self.feed = CameraFeed(camera_source)
        self.last_key = None
        self.health_epoch = None
        self.last_error = None
        self.completions = deque()
        self.last_success = None
        self.barrier = time.monotonic()
        self._diagnostics_started_at = self.barrier
        self._diagnostics = {}
        self._phase = 'loading'
        self._ready_announced = False
        self._completed_results = 0
        self._no_hit_started_at = self._no_hit_last_at = None
        self._diagnostic_snapshot_count = 0

    def _publish_status(self, phase=None, error=None):
        if phase is not None:
            self._phase = phase
        if self._phase == 'recognizing' and not self._healthy_epoch(self.health_epoch):
            self._phase = 'recovering'
            error = self.feed.error or self.camera.last_error or self.owner.last_error or '识别代次已变化'
        runlog.status(mode='static_face_knife', phase=self._phase, error=error,
                      camera_generation=self.camera.generation,
                      detector_generation=self.owner.generation,
                      completed_results=self._completed_results,
                      saved_count=self.snapshots.saved_count,
                      diagnostic_snapshot_count=self._diagnostic_snapshot_count)

    def _reset_no_hit(self):
        self._no_hit_started_at = self._no_hit_last_at = None

    def _save_no_hit_diagnostic(self, result, valid):
        if valid:
            self._reset_no_hit()
            return
        at = result.captured.captured_monotonic
        if self._no_hit_last_at is not None:
            if at <= self._no_hit_last_at:
                return
            if at - self._no_hit_last_at > config.CAMERA_FRAME_MAX_AGE_S:
                self._reset_no_hit()
        if self._no_hit_started_at is None:
            self._no_hit_started_at = at
        self._no_hit_last_at = at
        elapsed = at - self._no_hit_started_at
        if (self._diagnostic_snapshot_count >= 3
                or elapsed < (0., 5., 10.)[self._diagnostic_snapshot_count]):
            return
        captured = result.captured
        metadata = dict(snapshot_kind='diagnostic_no_hit', class_id=None,
                        sequence=int(captured.sequence),
                        source_generation=int(captured.source_generation),
                        owner_generation=int(result.owner_generation),
                        captured_monotonic=float(at),
                        captured_wall_time=datetime.fromtimestamp(
                            time.time() - max(0., time.monotonic() - at), timezone.utc).isoformat(),
                        no_hit_duration_s=elapsed, confidence_threshold=CONFIDENCE,
                        input_bgr_shape=list(captured.bgr.shape),
                        diagnostics=dict(getattr(result, 'diagnostics', {})))
        if self.snapshots.submit(captured.bgr, metadata):
            # Reserve on acceptance so queued writes cannot exceed the run-wide cap.
            self._diagnostic_snapshot_count += 1

    def _record_diagnostic(self, result=None, discard_reason=None):
        data = self._diagnostics
        if result is not None:
            if not data:
                self._diagnostics = data = dict(
                    completed_results=0, returned_candidates=0, valid_candidates=0,
                    highest_returned_class_id=None, highest_returned_score=None,
                    latest_result_age_s=None, max_result_age_s=None, discarded_results={})
            data["completed_results"] += 1
            data["returned_candidates"] += len(result.detections)
            data["valid_candidates"] += len(self.policy.valid_detections(
                result.detections, result.captured.bgr))
            best = min(result.detections, key=lambda hit: (-hit.score, hit.class_id), default=None)
            if best is not None and (data["highest_returned_score"] is None or
                    (-best.score, best.class_id) <
                    (-data["highest_returned_score"], data["highest_returned_class_id"])):
                data["highest_returned_class_id"] = best.class_id
                data["highest_returned_score"] = best.score
            age = time.monotonic() - result.captured.captured_monotonic
            data["latest_result_age_s"] = age
            data["latest_result_frame"] = dict(
                sequence=result.captured.sequence,
                source_generation=result.captured.source_generation,
                owner_generation=result.owner_generation,
                captured_monotonic=result.captured.captured_monotonic)
            data["latest_model_diagnostics"] = dict(getattr(result, "diagnostics", {}))
            data["max_result_age_s"] = max(age, data["max_result_age_s"] if
                                           data["max_result_age_s"] is not None else age)
        if discard_reason is not None and data:
            counts = data["discarded_results"]
            counts[discard_reason] = counts.get(discard_reason, 0) + 1

    def _flush_diagnostics(self, force=False):
        now = time.monotonic()
        if not self._diagnostics or not force and now - self._diagnostics_started_at < 1.:
            return
        runlog.emit("static_vision_recognition_summary", "静止识别结果汇总", terminal=False,
                    window_s=now - self._diagnostics_started_at, **self._diagnostics,
                    confirmation_class_id=self.policy._winner,
                    confirmation_streak=self.policy._streak,
                    confirmation_blocked=self.announcer.blocks_confirmation)
        self._diagnostics = {}
        self._diagnostics_started_at = now

    def _healthy_epoch(self, epoch):
        return (epoch == (self.camera.generation, self.camera.fault_generation, self.owner.generation)
                and not (self.feed.error or self.camera.last_error or self.owner.last_error))

    def _reject_changed_epoch(self):
        self.policy.suspend()
        self._reset_no_hit()
        self.barrier = time.monotonic()
        self._publish_status('recovering', self.feed.error or self.camera.last_error
                             or self.owner.last_error or '等待健康新鲜推理结果')

    def step(self):
        self._flush_diagnostics()
        self.parked.tick()
        self.camera.authorize_stationary(True, barrier=self.parked.barrier)
        self.announcer.tick()
        now = time.monotonic()
        epoch = (self.camera.generation, self.camera.fault_generation, self.owner.generation)
        error = self.feed.error or self.camera.last_error or self.owner.last_error
        current = self.feed.latest
        if (current is None or getattr(current, "source_generation", None) != self.camera.generation
                or not 0 <= now - current.captured_monotonic <= config.CAMERA_FRAME_MAX_AGE_S):
            error = error or "等待摄像头新鲜画面"
        if epoch != self.health_epoch or error:
            if epoch != self.health_epoch or error != self.last_error:
                if self.health_epoch is not None:
                    self._phase = 'recovering'
                self.policy.suspend()
                self.completions.clear()
                self.last_success = None
                self._reset_no_hit()
                self.barrier = now
                self.last_key = None
                if error and error != self.last_error:
                    self.emitter("保持停车，正在恢复：" + str(error))
                self.health_epoch = epoch
            self.last_error = error
        if error:
            # Consume and discard any completed result from before the failure.
            discarded = self.owner.poll()
            if discarded is not None:
                self._record_diagnostic(discarded, discard_reason="unavailable")
            self.policy.suspend()
            self._reset_no_hit()
            self.barrier = now
            self._publish_status('recovering', str(error))
            return
        self.last_error = None
        if self.last_success is not None and now - self.last_success > config.CAMERA_FRAME_MAX_AGE_S:
            self._reset_no_hit()
            self._publish_status('recovering', '等待新鲜推理结果')
        else:
            self._publish_status()
        result = self.owner.poll()
        if result is not None:
            self._record_diagnostic(result)
            captured = result.captured
            at = captured.captured_monotonic
            if (result.owner_generation == self.owner.generation
                    and captured.source_generation == self.camera.generation
                    and at > self.barrier and 0 <= now - at <= config.CAMERA_FRAME_MAX_AGE_S
                    and self._healthy_epoch(epoch)):
                if self.last_success is not None and now - self.last_success > config.CAMERA_FRAME_MAX_AGE_S:
                    self.policy.suspend()
                    self._reset_no_hit()
                self.completions.append(now)
                while self.completions and self.completions[0] < now - 2.0:
                    self.completions.popleft()
                fps = ((len(self.completions) - 1) / (now - self.completions[0])
                       if len(self.completions) > 1 and now > self.completions[0] else 0.0)
                valid = self.policy.valid_detections(result.detections, captured.bgr)
                image = annotate(captured.bgr, valid, fps)
                self.parked.tick()
                now = time.monotonic()
                if not self._healthy_epoch(epoch):
                    self._record_diagnostic(discard_reason="fault_changed")
                    self._reject_changed_epoch()
                    return
                if not 0 <= now - captured.captured_monotonic <= config.CAMERA_FRAME_MAX_AGE_S:
                    self._record_diagnostic(discard_reason="expired")
                    self.policy.suspend()
                    self._reset_no_hit()
                    self._publish_status('recovering', '推理结果已过期')
                    return
                hit = self.policy.observe(captured, valid, now=now,
                                          blocked=self.announcer.blocks_confirmation)
                if (self._healthy_epoch(epoch)
                        and 0 <= time.monotonic() - captured.captured_monotonic
                        <= config.CAMERA_FRAME_MAX_AGE_S):
                    self.last_success = now
                    self._completed_results += 1
                    if not self._ready_announced:
                        self.emitter('识别已就绪')
                        self._ready_announced = True
                    self._save_no_hit_diagnostic(result, valid)
                    self._publish_status('recognizing')
                    if hit is not None:
                        self.announcer.offer(hit, captured, image)
                else:
                    self._record_diagnostic(discard_reason=("fault_changed" if
                        not self._healthy_epoch(epoch) else "expired"))
                    self._reject_changed_epoch()
            else:
                reason = ("owner_generation" if result.owner_generation != self.owner.generation else
                          "camera_generation" if captured.source_generation != self.camera.generation else
                          "pre_barrier" if at <= self.barrier else
                          "expired" if not 0 <= now - at <= config.CAMERA_FRAME_MAX_AGE_S else
                          "fault_changed")
                self._record_diagnostic(discard_reason=reason)
                self._reset_no_hit()
                self._publish_status('recovering', reason)
                if not self._healthy_epoch(epoch):
                    self._reject_changed_epoch()
        key = (current.source_generation, current.sequence, current.captured_monotonic)
        if key != self.last_key and current.captured_monotonic > self.barrier:
            if self.owner.submit(current):
                self.last_key = key

    def run(self):
        try:
            while True:
                self.step()
                time.sleep(.02)
        finally:
            self.camera.authorize_stationary(False)
            self._flush_diagnostics(force=True)


class _SnapshotOnlyClues:
    def detection(self, class_id):
        return None

    def detection_audio_path(self, class_id):
        return None


class _UnavailableAudio:
    completed_count = failed_count = started_count = 0

    def __init__(self, error):
        self.last_error = str(error)

    def poll(self):
        return False

    def play(self, path):
        return False

    def close(self):
        pass


def _check_detection_assets(clues_source, emitter=None):
    write = operator_message if emitter is None else emitter
    errors = []
    for class_id in range(1, 21):
        try:
            if clues_source.detection(class_id) is None:
                errors.append(f"缺少识别类别映射：{class_id}")
            path = clues_source.detection_audio_path(class_id)
            if path is None or not Path(path).is_file():
                errors.append(f"缺少识别音频：{class_id}，{path}")
        except Exception as exc:
            errors.append(f"编号{class_id}播报资源检查失败：{exc}")
    if errors:
        write("播报资源不完整，照片仍独立保存：" + "；".join(errors))
    return tuple(errors)


def run_static_test(*, components=None, input_source=None, emitter=None,
                    camera_index=0, camera_device=None, output_dir=None):
    write = operator_message if emitter is None else emitter
    c = {} if components is None else dict(components)
    parked = runner = None
    failed = False
    stopped = False
    runlog.status(mode='static_face_knife', phase='loading')
    try:
        if input_source is None:
            input_source = motion_speed_input.TerminalSpeedInput(write_line=write)
        c.setdefault("mcu", ParkedLink())
        parked = ParkedController(c["mcu"], input_source)
        write("静止识别测试准备；q并回车或Ctrl+C退出。")
        parked.prepare()
        write("已确认停车；摄像头上下PWM=800、左右PWM=1500。")
        if "clues" not in c:
            try:
                c["clues"] = clues.Clues()
            except Exception as exc:
                write("播报映射加载失败，照片仍独立保存：" + str(exc))
                runlog.exception("static_vision_clues_unavailable", "播报映射加载失败", exc, terminal=False)
                c["clues"] = _SnapshotOnlyClues()
        _check_detection_assets(c["clues"], write)
        if "audio" not in c:
            try:
                c["audio"] = audio.Audio()
            except Exception as exc:
                write("播放器初始化失败，照片仍独立保存：" + str(exc))
                runlog.exception("static_vision_audio_unavailable", "播放器初始化失败", exc, terminal=False)
                c["audio"] = _UnavailableAudio(exc)
        if "camera" not in c:
            c["camera"] = stage2._Stage2CameraSource(camera_index=camera_index,
                                                    camera_device=camera_device)
        if "detector" not in c:
            c["detector"] = DetectorOwner()
        if "snapshots" not in c:
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
            c["snapshots"] = SnapshotWriter(Path(output_dir or DEFAULT_OUTPUT_DIR) / stamp, emitter=write)
        write("照片保存目录：" + str(Path(c["snapshots"].directory).resolve()))
        c["camera"].authorize_stationary(True, barrier=parked.barrier)
        c["camera"].start()
        c["detector"].start()
        policy = RecognitionPolicy(confidence=CONFIDENCE, absence_s=ABSENCE_S)
        announcer = AnnouncementController(c["audio"], c["clues"], c["snapshots"],
                                           emitter=write, policy=policy)
        c["announcer"] = announcer
        runner = StaticRunner(parked, c["camera"], c["detector"], policy,
                              announcer, c["snapshots"], input_source, write)
        runner.run()
    except (sequence._UserQuit, KeyboardInterrupt):
        write("收到退出请求，正在确认停车。")
    except Exception as exc:
        failed = True
        runlog.status(mode='static_face_knife', phase='failed', error=str(exc))
        write("静止识别测试失败：" + str(exc))
        runlog.exception("static_vision_failed", "静止识别测试失败", exc)
    finally:
        runlog.status(mode='static_face_knife', phase='failed' if failed else 'stopping')
        if c.get("camera") is not None:
            c["camera"].authorize_stationary(False)
        if parked is not None:
            try:
                stopped = parked.shutdown(failed=failed)
            except (Exception, KeyboardInterrupt) as exc:
                failed = True
                write("退出停车失败：" + str(exc))
                runlog.exception("static_vision_exit_failed", "退出停车失败", exc)
        link = c.get("mcu")
        exit_input = parked.input_source if parked is not None else None
        if parked is not None:
            parked.input_source = None
        close_errors = []

        def close_resources():
            resources = [("feed", runner.feed if runner is not None else None)]
            resources.extend((name, c.get(name)) for name in
                             ("announcer", "detector", "camera", "snapshots", "audio"))
            resources.append(("input", input_source))
            for name, resource in resources:
                if resource is not None:
                    try:
                        resource.close()
                    except (Exception, KeyboardInterrupt) as exc:
                        close_errors.append((name, exc))

        cleanup = threading.Thread(target=close_resources, name="static-vision-cleanup", daemon=True)
        cleanup_monitor_failed = False
        cleanup.start()
        while cleanup.is_alive():
            try:
                if stopped and not cleanup_monitor_failed:
                    parked.tick()
                elif parked is not None and link is not None and link.connected:
                    parked._receive()
                cleanup.join(.02)
            except (Exception, KeyboardInterrupt) as exc:
                failed = True
                if not cleanup_monitor_failed:
                    cleanup_monitor_failed = True
                    write("退出清理期间停车/PWM证据失效：" + str(exc))
                    runlog.exception("static_vision_cleanup_park_failed", "退出清理停车证据失效", exc)
                    if parked is not None:
                        try:
                            parked.shutdown(failed=True)
                        except (Exception, KeyboardInterrupt) as stop_exc:
                            runlog.exception("static_vision_cleanup_brake_failed", "退出清理制动失败", stop_exc)
        for name, exc in close_errors:
            failed = True
            write(f"关闭{name}失败：{exc}")
            runlog.exception("static_vision_close_failed", "资源关闭失败", exc, component=name)
        if link is not None:
            try:
                link.receive_shutdown_tail(check_parked=parked.tick if stopped else None)
            except (Exception, KeyboardInterrupt) as exc:
                failed = True
                runlog.exception("static_vision_tail_failed", "串口接收收尾失败", exc)
                if stopped and not cleanup_monitor_failed:
                    cleanup_monitor_failed = True
                    try:
                        parked.shutdown(failed=True)
                    except (Exception, KeyboardInterrupt) as stop_exc:
                        runlog.exception("static_vision_tail_brake_failed", "尾段制动失败", stop_exc)
            finally:
                if stopped:
                    try:
                        parked.tick()
                    except (Exception, KeyboardInterrupt) as exc:
                        failed = True
                        write("退出收尾停车/PWM证据失效：" + str(exc))
                        runlog.exception("static_vision_final_park_failed", "退出收尾停车证据失效", exc)
                        if not cleanup_monitor_failed:
                            try:
                                parked.shutdown(failed=True)
                            except (Exception, KeyboardInterrupt) as stop_exc:
                                runlog.exception("static_vision_final_brake_failed", "最终制动失败", stop_exc)
                try:
                    link.close()
                except (Exception, KeyboardInterrupt) as exc:
                    failed = True
                    runlog.exception("static_vision_serial_close_failed", "关闭串口失败", exc)
        if parked is not None:
            parked.input_source = exit_input
        success = not failed and stopped
        if success:
            result = ("停车已确认，保持上下800、左右1500。" if parked.pwm_confirmed
                      else "停车已确认；固定PWM设置尚未完成。")
        else:
            result = "失败，请检查日志与停车状态。"
        runlog.status(mode='static_face_knife', phase='stopped' if success else 'failed',
                      parked=stopped)
        write("测试结束：" + result)
    return success


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--camera-index", type=int, default=0)
    group.add_argument("--camera-device", help="指定USB摄像头设备路径")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.camera_index < 0:
        parser.error("camera-index必须非负")
    with stage2._Stage2RunLog(mode="static_face_knife"):
        return 0 if run_static_test(camera_index=args.camera_index,
            camera_device=args.camera_device, output_dir=args.output_dir) else 1


if __name__ == "__main__":
    raise SystemExit(main())
