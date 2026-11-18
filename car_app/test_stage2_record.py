"""实车直行录像：150 mm/s，q回车切换停车/行走，a回车拍10张原始画面。"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, is_dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid

import config
import mcu
import motion_speed_input as speed_input
import runlog
import test_motion_sequence as sequence
import test_stage2_obstacle as stage2


class _RecordingTerminalInput(speed_input.TerminalSpeedInput):
    """Keep startup prompt rules; retain every complete runtime command line."""

    def __init__(self, **kwargs):
        self._recording_runtime = False
        self._recording_lines = deque()
        super().__init__(**kwargs)

    def begin_runtime(self):
        with self._lock:
            if self._error is not None:
                raise self._error
            if self._quit:
                raise sequence._UserQuit("启动末尾收到q，保持停车并结束录像")
            self._active = False
            self._response = None
            self._clear_partial()
            self._recording_runtime = True

    def _consume_chunk(self, chunk, *, generation):
        if not self._recording_runtime:
            return super()._consume_chunk(chunk, generation=generation)
        text = self._decoder.decode(chunk) if isinstance(chunk, bytes) else chunk
        for character in text:
            if character in ("\x03", "\x1a"):
                self._error = KeyboardInterrupt() if character == "\x03" else EOFError("标准输入已关闭")
                return
            if character == "\n" and self._skip_lf:
                self._skip_lf = False
                continue
            self._skip_lf = character == "\r"
            if character in ("\r", "\n"):
                line = "" if self._overflow else self._pending
                self._pending, self._overflow = "", False
                if line.strip().lower() in ("q", "a"):
                    if len(self._recording_lines) >= 256:
                        self._error = OSError("录像操作输入积压过多，停车退出")
                        return
                    self._recording_lines.append(line.strip().lower())
            elif character in ("\b", "\x7f"):
                if not self._overflow:
                    self._pending = self._pending[:-1]
            elif not self._overflow:
                if len(self._pending) < speed_input._MAX_LINE:
                    self._pending += character
                else:
                    self._pending, self._overflow = "", True

    def poll_runtime_commands(self):
        with self._lock:
            if self._error is not None:
                raise self._error
            lines = tuple(self._recording_lines)
            self._recording_lines.clear()
            return lines


class _RecordingPauseInterrupt(Exception):
    """Yield a pending nonzero write to the FIFO pause/resume handler."""


def _json_value(value):
    if is_dataclass(value):
        return asdict(value)
    if hasattr(value, "name"):
        return value.name
    if hasattr(value, "item"):
        return value.item()
    raise TypeError(type(value).__name__)


class RawRecorder:
    """A bounded raw-frame writer; neither capture nor control waits for disk IO."""

    def __init__(self, root=None, *, writer_factory=None, photo_writer=None, fps=30.0, capacity=16):
        if not math.isfinite(fps) or fps <= 0 or capacity < 1:
            raise ValueError("录像fps和队列容量必须为正数")
        self.root = Path(root) if root is not None else config.APP_ROOT / "recordings"
        self.directory = self.root / (datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8])
        self.fps, self.writer_factory = fps, writer_factory
        self.photo_writer = photo_writer
        self._queue = queue.Queue(maxsize=capacity)
        self._lock = threading.Lock()
        self._context, self._last_key = {}, None
        self._closing = threading.Event()
        self._writing = threading.Event()
        self._thread = None
        self._error = None
        self.metadata_provider = None
        self._last_progress = time.monotonic()
        self.segments, self.frames_written, self.dropped = [], 0, 0
        self.photo_batches = []
        self._photo_batch = None
        self._photo_reports = queue.SimpleQueue()

    @property
    def error(self):
        with self._lock:
            return self._error

    def _fail(self, reason):
        with self._lock:
            if self._error is None:
                self._error = str(reason)

    def update_context(self, **context):
        # Serialize before handing the record to another thread: no live MCU objects.
        detached = json.loads(json.dumps(context, default=_json_value, ensure_ascii=False))
        with self._lock:
            self._context = detached

    def request_photos(self):
        """Reserve ten subsequent fresh source frames without waiting for disk."""
        with self._lock:
            if self._photo_batch is not None:
                return False, self._photo_batch["directory"]
            if self._closing.is_set() or self._error:
                return False, None
            directory = self.directory / "photos" / (
                datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8])
            batch = dict(directory=str(directory.resolve()), requested_at=time.monotonic(),
                         queued=0, written=0, complete=False, paths=[], source_keys=[])
            self.photo_batches.append(batch)
            self._photo_batch = batch
            return True, batch["directory"]

    def drain_photo_reports(self):
        reports = []
        while True:
            try:
                reports.append(self._photo_reports.get_nowait())
            except queue.Empty:
                return reports

    def submit(self, frame):
        if self._closing.is_set() or self.error:
            return
        key = (getattr(frame, "source_generation", 0), frame.sequence, frame.captured_monotonic)
        if self.metadata_provider is not None:
            try:
                self.update_context(**self.metadata_provider())
            except Exception as exc:
                self._fail("逐帧元数据采集失败：" + str(exc))
                return
        with self._lock:
            if key == self._last_key:
                return
            self._last_key = key
            context = self._context.copy()
            batch = self._photo_batch
            photo = None
            if (batch is not None and batch["queued"] < 10
                    and key not in batch["source_keys"]
                    and frame.captured_monotonic > batch["requested_at"]
                    and stage2.Stage2Vision._fresh(frame.captured_monotonic)):
                index = batch["queued"] + 1
                photo = batch, str(Path(batch["directory"]) / f"{index:02d}.jpg")
            if not self._writing.is_set() and self._queue.empty():
                # Parked preparation or camera recovery may leave the writer
                # idle for minutes; the next job starts a new progress window.
                self._last_progress = time.monotonic()
            if self._thread is None:
                self._thread = threading.Thread(target=self._write_loop, name="raw-video-writer", daemon=True)
                self._thread.start()
            try:
                self._queue.put_nowait((frame.bgr.copy(), key, context, photo))
                if photo is not None:
                    batch["queued"] += 1
                    batch["source_keys"].append(key)
            except queue.Full:
                self.dropped += 1
                if self._error is None:
                    self._error = "原始录像队列溢出，停车保存已有录像"

    def check_health(self):
        if (self._writing.is_set() or not self._queue.empty()) and time.monotonic() - self._last_progress > 2.0:
            self._fail("录像写入超过2秒没有进展，停车保存已有录像")
        return self.error

    def _new_writer(self, path, size):
        if self.writer_factory is not None:
            return self.writer_factory(path, self.fps, size)
        import cv2
        return cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), self.fps, size)

    def _write_photo(self, image, photo):
        batch, path = photo
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        if self.photo_writer is None:
            import cv2
            written, encoded = cv2.imencode(".jpg", image)
            if written:
                Path(path).write_bytes(encoded.tobytes())
        else:
            written = self.photo_writer(path, image)
        if not written:
            raise RuntimeError("原始照片写入失败：" + path)
        with self._lock:
            batch["written"] += 1
            batch["paths"].append(str(Path(path).resolve()))
            if batch["written"] == 10:
                batch["complete"] = True
                self._photo_batch = None
                self._photo_reports.put("10张原始照片已保存：" + batch["directory"])

    def _seal_segment(self, writer, segment):
        writer.release()
        if segment is None:
            return
        if self.writer_factory is not None:
            segment["integrity"] = "injected_writer"
            return
        import cv2
        path = Path(segment["path"])
        segment["bytes"] = path.stat().st_size
        capture = cv2.VideoCapture(str(path))
        try:
            if not capture.isOpened():
                raise RuntimeError("封存录像无法重新打开：" + str(path))
            count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
            segment["container_frames"] = count
            if count != segment["frames"]:
                raise RuntimeError(f"封存录像帧数不符：{path}，写入{segment['frames']}，文件{count}")
            for index in sorted({0, count - 1}):
                if index and not capture.set(cv2.CAP_PROP_POS_FRAMES, index):
                    raise RuntimeError("封存录像无法定位末帧：" + str(path))
                ok, image = capture.read()
                if (not ok or image.shape[:2] != (segment["height"], segment["width"])):
                    raise RuntimeError("封存录像首末帧或分辨率不完整：" + str(path))
            segment["integrity"] = "frame_count_and_first_last_verified"
        finally:
            capture.release()

    def _write_loop(self):
        writer, signature, segment = None, None, None
        try:
            self.directory.mkdir(parents=True, exist_ok=False)
            with (self.directory / "frames.jsonl").open("w", encoding="utf-8") as metadata:
                while not self._closing.is_set() or not self._queue.empty():
                    try:
                        image, key, context, photo = self._queue.get(timeout=.05)
                    except queue.Empty:
                        continue
                    self._writing.set()
                    generation, source_sequence, captured = key
                    height, width = image.shape[:2]
                    current = generation, width, height
                    if current != signature:
                        if writer is not None:
                            previous, writer = writer, None
                            self._seal_segment(previous, segment)
                        path = self.directory / f"raw-{len(self.segments) + 1:03d}.avi"
                        writer = self._new_writer(path, (width, height))
                        if not writer.isOpened():
                            raise RuntimeError("RDK不支持MJPEG/AVI写入：" + str(path))
                        segment = {"path": str(path.resolve()), "generation": generation,
                                   "width": width, "height": height, "fps": self.fps, "frames": 0}
                        self.segments.append(segment)
                        signature = current
                    writer.write(image)
                    segment["frames"] += 1
                    self.frames_written += 1
                    if photo is not None:
                        self._write_photo(image, photo)
                    record = {"segment": Path(segment["path"]).name,
                              "video_frame": segment["frames"], "sequence": source_sequence,
                              "captured_monotonic": captured, "source_generation": generation,
                              **context}
                    metadata.write(json.dumps(record, ensure_ascii=False, default=_json_value) + "\n")
                    metadata.flush()
                    self._last_progress = time.monotonic()
                    self._writing.clear()
        except Exception as exc:
            self._fail("录像写入失败：" + str(exc))
        finally:
            self._writing.clear()
            if writer is not None:
                try:
                    self._seal_segment(writer, segment)
                except Exception as exc:
                    self._fail("录像封存失败：" + str(exc))

    def close(self, *, motion_stopped=False, reason=None):
        self._closing.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                self._fail("录像线程未在5秒内完成封存")
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self._fail("录像目录无法封存：" + str(exc))
        with self._lock:
            batches = json.loads(json.dumps(self.photo_batches))
        result = {"directory": str(self.directory.resolve()), "segments": [item.copy() for item in self.segments],
                  "metadata": str((self.directory / "frames.jsonl").resolve()),
                  "frames": self.frames_written, "dropped": self.dropped,
                  "photos": batches,
                  "complete": bool(motion_stopped and self.frames_written and not self.error and not reason),
                  "motion_stopped": motion_stopped, "error": self.error, "exit_reason": reason,
                  "timing": "AVI使用标称fps；精确采集时间以frames.jsonl为准"}
        path = self.directory / "manifest.json"
        try:
            path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        except OSError as exc:
            self._fail("录像清单无法写入：" + str(exc))
            result["error"], result["complete"] = self.error, False
        result["manifest"] = str(path.resolve())
        return result


class Stage2RecordingRunner(stage2.Stage2Runner):
    """Forward-only control with observation decisions deliberately disabled."""

    def __init__(self, *args, recorder, **kwargs):
        super().__init__(*args, **kwargs)
        self.recorder = recorder
        self.phase, self.road, self.reference_fixed = "RECORD", "直行录像", True
        self._last_recorded_opening = None
        self._recording_commands = deque()
        self._recording_paused = False
        self._handling_recording_commands = False
        self._recording_input = (self.input_source.source
            if isinstance(self.input_source, stage2._Stage2Console) else self.input_source)
        disable_approach = getattr(self.vision, "enable_obstacle_approach", None)
        if callable(disable_approach):
            disable_approach(False)
        begin_runtime = getattr(self._recording_input, "begin_runtime", None)
        if callable(begin_runtime):
            begin_runtime()

    def _read_recording_commands(self):
        source = self._recording_input
        if source is None:
            return
        poll_many = getattr(source, "poll_runtime_commands", None)
        if callable(poll_many):
            lines = poll_many()
        else:
            line = source.poll()
            lines = () if line is None else (line,)
        if getattr(source, "eof", False):
            raise EOFError("终端输入结束，尝试停车")
        self._recording_commands.extend(line.strip().lower() for line in lines
            if isinstance(line, str) and line.strip().lower() in ("q", "a"))

    def _report_photos(self):
        drain = getattr(self.recorder, "drain_photo_reports", None)
        if callable(drain):
            for message in drain():
                self.display.emitter(message)

    def _checkpoint(self):
        # Leave FIFO/input errors untouched until BRAKE has fully settled.
        if self._safety_stop:
            consume = getattr(self.vision, "consume_control_snapshot", None)
            self._apply_visual_snapshot(consume() if callable(consume) else self.vision.poll())
            return
        reason = self.recorder.check_health()
        if reason:
            raise mcu.McuError(reason)
        self._read_recording_commands()
        if ("q" in self._recording_commands and not self._recording_paused
                and not self._handling_recording_commands):
            raise _RecordingPauseInterrupt()
        self._report_photos()
        consume = getattr(self.vision, "consume_control_snapshot", None)
        self._apply_visual_snapshot(consume() if callable(consume) else self.vision.poll())

    def _guard_visual_motion(self):
        self._checkpoint()
        if self._recording_paused:
            raise _RecordingPauseInterrupt()
        if self._vision_pause_pending:
            raise stage2._Stage2VisionInterrupt()

    def _guard_visual_snapshot(self, snapshot):
        if self._recording_paused or "q" in self._recording_commands:
            raise _RecordingPauseInterrupt()
        self._apply_visual_snapshot(snapshot)
        if self._vision_pause_pending:
            raise stage2._Stage2VisionInterrupt()

    def _apply_visual_snapshot(self, snapshot):
        super()._apply_visual_snapshot(snapshot)
        # HSV recognition remains visible/logged, but cannot authorize a maneuver.
        self.visual_confirmed = None
        self.visual_candidate = self.visual_timeout = False

    def _action(self, text):
        super()._action(text.replace("q退出", "q回车切换停车/行走"))

    def _resume_recording(self):
        """Use stop-after STATE/image evidence; retain road reference and distance."""
        barrier = self.link._stable_barrier
        self.display.reset_averages(source_barrier=barrier, reset_controller=False)
        self.armed = False
        self.display.centering_target = None
        self.sent_at = None
        self._action("保持停车；等待停后新鲜STATE与画面，再恢复150mm/s")
        while True:
            self._checkpoint()
            self.display.tick()
            samples = self._consume(update=True, means=False)
            if not sequence._assert_still_stable(self.link, self.display):
                self._strict_brake("恢复前静止证据失效：BRAKE并重新严格停稳",
                                   reason="recording_resume_stationary_lost")
                barrier = self.link._stable_barrier
                self.display.reset_averages(source_barrier=barrier, reset_controller=False)
            self._arm_walls(samples, time.monotonic())
            token, healthy, captured = self._visual_health(self.visual_snapshot)
            state_at = self.display.controller._state_at
            if (healthy and bool(stage2._stage2_value(self.visual_snapshot, "ready", False))
                    and stage2.Stage2Vision._fresh(captured) and captured > barrier
                    and state_at is not None and state_at > barrier
                    and 0 <= time.monotonic() - state_at <= config.TELEMETRY_STALE_MS / 1000.0):
                self._visual_health_token = token
                self._vision_pause_pending = False
                self._vision_pause_reason = None
                self._recording_paused = False
                self.link.set_stage2_mode("normal")
                self.link.side_motion_active = True
                self.response.pause(self.link, time.monotonic())
                self.next_send = time.monotonic()
                self.display.emitter("录像继续；恢复150mm/s前进。")
                runlog.emit("stage2_record_resumed", "手动恢复直行录像",
                            stable_barrier=barrier, captured_at=captured,
                            state_at=state_at, reference_deg=self.display.controller.heading_reference_deg,
                            position_mm=self.progress.position_mm)
                return
            time.sleep(sequence.CONTROL_INTERVAL_S)

    def _handle_recording_commands(self):
        self._handling_recording_commands = True
        reports = []
        try:
            while self._recording_commands:
                command = self._recording_commands.popleft()
                if command == "a":
                    accepted, directory = self.recorder.request_photos()
                    reports.append("开始保存随后10张原始画面。" if accepted else
                        "上一批10张照片尚未完成，请稍候。" if directory else "录像不可用，无法开始拍照。")
                elif not self._recording_paused:
                    self._recording_paused = True
                    self._strict_brake("q暂停：BRAKE并严格停稳；录像和预览继续",
                                       reason="recording_manual_pause")
                    self.link.set_stage2_mode("stopped")
                    self.link.side_motion_active = False
                    self.display.emitter("已严格停稳，录像和预览继续；再次输入q并回车恢复行走。")
                    runlog.emit("stage2_record_paused", "手动暂停直行录像",
                                stable_barrier=self.link._stable_barrier,
                                position_mm=self.progress.position_mm)
                else:
                    self._resume_recording()
            for report in reports:
                self.display.emitter(report)
        finally:
            self._handling_recording_commands = False

    def _arm_walls(self, samples, now):
        controller = self.display.controller
        for sample in samples:
            if self._fresh_walls(sample, now):
                controller.observe_side(sample)
        controller.retry_pending_pair(now=now)
        self.armed = self._fresh_position_feedback(now)

    def _record_context(self, samples):
        link, snapshot = self.link, self.visual_snapshot
        state = getattr(link, "state", None)
        sample = getattr(link, "_latest_side_sample", None)
        for observation in samples:
            if observation.candidates and observation.source_keys != self._last_recorded_opening:
                self._last_recorded_opening = observation.source_keys
                runlog.emit("stage2_record_opening", "录像路口只记录，保持前进",
                            source_keys=observation.source_keys, sides=tuple(observation.candidates))
        self.recorder.update_context(
            phase=self.phase, motion_command=self.display.command,
            wheel_targets=self.display.wheel_targets, wheel_speeds=getattr(link, "wheel_speeds", None),
            odometry=None if state is None else (state.left_mm, state.right_mm), yaw_deg=link.yaw_deg,
            laser=None if sample is None else {"left_mm": sample.left_mm, "right_mm": sample.right_mm,
                "left_status": sample.left_status, "right_status": sample.right_status,
                "source_times": sample.source_times, "source_keys": sample.source_keys},
            detection=stage2._stage2_value(snapshot, "current"),
            detection_sequence=stage2._stage2_value(snapshot, "sequence"),
            detection_captured_at=stage2._stage2_value(snapshot, "captured_at"))

    def _run_loop(self):
        link, display, controller = self.link, self.display, self.display.controller
        link.side_motion_active, link.side_candidates_enabled, link.stage2_return = True, False, False
        link.set_stage2_mode("normal")
        display._distance_observer, display.road_progress = self._observe_distance, self.progress
        self.progress.update(link.state, controller._state_at or time.monotonic())
        link.stage2_runtime_poll = self._checkpoint
        link.stage2_visual_health_gate = self._guard_visual_motion
        link.stage2_visual_snapshot_gate = self._guard_visual_snapshot
        link.stage2_visual_stop_pending = lambda: False
        self._epoch(None)
        while True:
            try:
                self._handle_recording_commands()
                self._checkpoint()
                self._handle_recording_commands()
                display.tick()
                self._checkpoint()
                self._handle_recording_commands()
                if self._recording_paused:
                    samples = self._consume(update=False, means=False)
                    self._record_context(samples)
                    if not sequence._assert_still_stable(link, display):
                        self._strict_brake("暂停中检测到位移：BRAKE并重新严格停稳",
                                           reason="recording_paused_motion")
                    self._action("停车录像；q回车恢复行走，a回车拍10张，Ctrl+C结束")
                    display.maybe_print()
                    time.sleep(sequence.CONTROL_INTERVAL_S)
                    continue
                if self._vision_pause_pending:
                    self._recover_vision()
                    continue
                samples = self._consume(update=True, means=False)
                now = time.monotonic()
                self._arm_walls(samples, now)
                self._record_context(samples)
                executing, confirmed, applied = sequence._confirmed_wheel_response(
                    link, self.sent_at, display.wheel_targets)
                self.response.update(link, executing, now, confirmed, display.state_samples, applied_state=applied)
                self._action("150mm/s直行录像；q回车暂停/恢复，a回车拍10张，Ctrl+C结束")
                if now + 1e-9 >= self.next_send:
                    target = self._target(time.monotonic())
                    if any(target.wheel_targets):
                        if sequence._wheel_transition_requires_stop(display.wheel_targets, target.wheel_targets):
                            raise mcu.McuError("录像前进目标产生非法归零或换向")
                        display.centering_target = target
                        self._send_target(target)
                        self._mark_started()
                        self.sent_at = time.monotonic()
                        controller.confirm_target_sent(target)
                        self.next_send = self.sent_at + sequence.COMMAND_INTERVAL_S
                    elif any(display.wheel_targets):
                        self._strict_brake("录像控制目标不可用，停车等待新鲜证据")
                display.maybe_print()
                time.sleep(sequence.CONTROL_INTERVAL_S)
            except (sequence._UserQuit, KeyboardInterrupt, EOFError):
                self._strict_brake("手动结束录像：BRAKE并严格停稳", reason="recording_user_stop")
                return True
            except _RecordingPauseInterrupt:
                continue
            except sequence._CenteringProposalExpired:
                self.next_send = time.monotonic()
            except stage2._Stage2VisionInterrupt:
                continue


def run_recording_test(*, components=None, input_source=None, host="0.0.0.0", port=8080,
                       camera_index=0, camera_device=None, output_root=None, emitter=None,
                       recorder_factory=RawRecorder, vision_factory=stage2.Stage2Vision):
    write = emitter or stage2._stage2_operator_message
    recorder = recorder_factory(output_root)
    c, display, source, vision = {}, None, None, None
    stopped, reason = False, None
    try:
        c = components if components is not None else stage2._components(
            camera_index=camera_index, camera_device=camera_device)
        source = stage2._Stage2Console(input_source or _RecordingTerminalInput(write_line=write), emitter=write)
        display = stage2.Stage2Display(c["mcu"], emitter=write)
        stage2._prepare(c, display, source)
        def capture_context():
            link = c["mcu"]
            state = getattr(link, "state", None)
            sample = getattr(link, "_latest_side_sample", None)
            snapshot = vision.poll() if vision is not None else None
            return dict(metadata_monotonic=time.monotonic(), phase=display.action,
                motion_command=display.command, wheel_targets=display.wheel_targets,
                wheel_speeds=getattr(link, "wheel_speeds", None), yaw_deg=link.yaw_deg,
                odometry=None if state is None else (state.left_mm, state.right_mm),
                laser=None if sample is None else dict(left_mm=sample.left_mm, right_mm=sample.right_mm,
                    left_status=sample.left_status, right_status=sample.right_status,
                    source_times=sample.source_times, source_keys=sample.source_keys),
                detection=stage2._stage2_value(snapshot, "current"),
                detection_sequence=stage2._stage2_value(snapshot, "sequence"),
                detection_captured_at=stage2._stage2_value(snapshot, "captured_at"))
        recorder.metadata_provider = capture_context
        vision = vision_factory(c["camera"], host=host, port=port, frame_observer=recorder.submit)
        vision._status_emitter = write
        source.vision, source.require_preview = vision, True
        vision.start()
        c["mcu"].stage2_vision = vision
        def startup_poll():
            if recorder.check_health():
                raise mcu.McuError(recorder.error)
            sequence._check_input_exit(source.cancellation_input)
        c["mcu"].stage2_runtime_poll = startup_poll
        stage2._wait_preview(c, display, source, vision, write, host)
        stage2._configure_gimbal(c, display, source, write)
        source.require_detection = True
        stage2._sample_stage2_start(c, display, source, vision)
        source.require_preview = source.require_detection = False
        c["mcu"].stage2_runtime_poll = None
        write("开始150mm/s直行录像；q并回车暂停/恢复，a并回车拍10张；Ctrl+C结束并保存。")
        stopped = Stage2RecordingRunner(c["mcu"], display, vision, 150,
                                        input_source=source, recorder=recorder).run()
    except (KeyboardInterrupt, EOFError, sequence._UserQuit) as exc:
        reason = "手动退出" if isinstance(exc, KeyboardInterrupt) else str(exc)
    except Exception as exc:
        reason = str(exc)
        runlog.exception("stage2_record_failed", "直行录像结束", exc)
        write("录像失败，尝试停车：" + reason)
    finally:
        if c.get("mcu") is not None:
            link = c["mcu"]
            link.stage2_runtime_poll = None
            if not stopped and display is not None and getattr(link, "connected", False):
                try:
                    sequence._brake(link, display, time.monotonic() + 5.0)
                    stopped = sequence._assert_still_stable(link, display)
                except Exception as exc:
                    reason = (reason or "") + "; 停稳未确认：" + str(exc)
        stage2._shutdown(c, display, source, vision, failed=not stopped, reason=reason)
        result = recorder.close(motion_stopped=stopped, reason=reason)
        for segment in result["segments"]:
            write("RDK录像文件：" + segment["path"])
        for batch in result.get("photos", ()):
            write(("RDK照片目录：" if batch["complete"] else
                   f"RDK未完成照片批次（已写{batch['written']}/10）：") + batch["directory"])
        write("RDK逐帧记录：" + result["metadata"])
        write("RDK录像清单：" + result["manifest"])
        write("录像保存完整。" if result["complete"] else "录像未完整完成：" + str(result["error"] or reason or "尚无画面"))
    return bool(result["complete"] and not reason)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=stage2._port, default=8080)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--camera-index", type=stage2._camera_index, default=0)
    selection.add_argument("--camera-device")
    args = parser.parse_args(argv)
    with stage2._Stage2RunLog():
        return 0 if run_recording_test(host=args.host, port=args.port,
            camera_index=args.camera_index, camera_device=args.camera_device) else 1


if __name__ == "__main__":
    raise SystemExit(main())
