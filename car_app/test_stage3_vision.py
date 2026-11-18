"""阶段 3：赛道行驶触发涵洞扫描，浏览器查看前向画面与扫描结果。"""
from __future__ import annotations

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time

import audio
import camera
import clues
import config
import culvert
import detector
import gimbal
import mcu
import runlog
import selfcheck
import test_stage2_obstacle as stage2


DEFAULT_DURATION_S = None
SCAN_SETTLE_S = 0.6
SCAN_SIDE_TIMEOUT_S = 5.0
SCAN_TILT_US = 800


PAGE = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>小车人脸与刀具赛道测试</title><style>
body{font-family:system-ui;margin:24px;background:#16191d;color:#eee}
img{max-width:100%;height:auto} #state{margin:12px 0} .error{color:#ff8b80}
</style><h1>人脸 / 刀具赛道识别</h1>
<div>Suspect = 嫌疑人（类别 1–10）；Knife = 刀具（类别 11–20，编号为类别减 10）。</div>
<div id="state">等待新画面…</div><img id="video" src="/stream.mjpg" alt="实时检测画面">
<div>最近一帧停车识别结果：</div><img id="scan" alt="停车识别画面" hidden>
<script>
const state=document.getElementById('state'), video=document.getElementById('video');
const scan=document.getElementById('scan'); let scanId=0;
async function poll(){
  try{
    const r=await fetch('/status',{cache:'no-store',signal:AbortSignal.timeout(2000)});
    if(!r.ok) throw new Error('服务不可用');
    const s=await r.json();
    state.className=s.live?'':'error';
    state.textContent=s.live?(s.mode==='scanning'
      ?`停车扫描 · 新帧推理 ${s.fps.toFixed(1)} FPS · 图像年龄 ${s.age_s.toFixed(2)} 秒`
      :`前向新鲜画面 · 行驶中未进行目标推理 · 图像年龄 ${s.age_s.toFixed(2)} 秒`)
      :(s.error||'等待新鲜画面，当前图像不可用于判断');
    video.style.opacity=s.live?'1':'.3';
    if(s.last_obstacle_id>scanId){scanId=s.last_obstacle_id;scan.hidden=false;
      scan.src=`/last-obstacle.jpg?frame=${scanId}`;}
  }catch(e){state.className='error';state.textContent='连接已中断，请检查 RDK 终端';video.style.opacity='.3';}
  setTimeout(poll,500);
} poll();
</script></html>""".encode("utf-8")


class PreviewState:
    """所有浏览器共享一张最新 JPEG；慢客户端不积压图像队列。"""

    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.captured_at = None
        self.fps = 0.0
        self.error = None
        self.stopped = False
        self.mode = "driving"

    def publish(self, jpeg, captured_at, fps, mode="scanning"):
        with self.condition:
            self.jpeg = jpeg
            self.sequence += 1
            self.captured_at = captured_at
            self.fps = fps
            self.mode = mode
            self.condition.notify_all()

    def next_frame(self, after_sequence):
        with self.condition:
            self.condition.wait_for(
                lambda: self.stopped or self.sequence > after_sequence, timeout=1.0)
            if self.stopped or self.sequence <= after_sequence:
                return None
            return self.sequence, self.jpeg

    def status(self):
        with self.condition:
            age = None if self.captured_at is None else max(0.0, time.monotonic() - self.captured_at)
            live = not self.stopped and age is not None and age <= config.CAMERA_FRAME_MAX_AGE_S
            return {"live": live, "fps": self.fps if live else 0.0,
                    "age_s": age, "mode": self.mode, "error": self.error}

    def close(self, error=None):
        with self.condition:
            self.stopped = True
            self.error = error
            self.condition.notify_all()


class PreviewServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    allow_reuse_address = True
    request_queue_size = 8

    def __init__(self, address, state, max_clients=8):
        self.state = state
        self.slots = threading.BoundedSemaphore(max_clients)
        super().__init__(address, PreviewHandler)

    def process_request(self, request, client_address):
        # 未发请求或停止读流的连接也占用名额；数量和 socket 等待都有限。
        request.settimeout(2.0)
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class PreviewHandler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def _reply(self, data, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        try:
            if self.path == "/":
                self._reply(PAGE, "text/html; charset=utf-8")
            elif self.path == "/status":
                self._reply(json.dumps(self.server.state.status()).encode("utf-8"), "application/json")
            elif self.path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                sequence = -1
                while not self.server.state.stopped:
                    result = self.server.state.next_frame(sequence)
                    if result is None:
                        continue
                    sequence, jpeg = result
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpeg)).encode("ascii") + b"\r\n\r\n" + jpeg + b"\r\n")
            else:
                self.send_error(404)
        except (OSError, TimeoutError):
            # 浏览器关页或读取过慢只结束该客户端，采集和其他客户端继续。
            pass


def annotate(bgr, detections, fps):
    import cv2

    output = bgr.copy()
    height, width = output.shape[:2]
    for detection in detections:
        if not 1 <= detection.class_id <= 20 or not math.isfinite(detection.score):
            continue
        if not all(math.isfinite(value) for value in detection.box):
            continue
        x1, y1, x2, y2 = (int(round(value)) for value in detection.box)
        x1, x2 = max(0, min(width - 1, x1)), max(0, min(width - 1, x2))
        y1, y2 = max(0, min(height - 1, y1)), max(0, min(height - 1, y2))
        if x2 <= x1 or y2 <= y1:
            continue
        suspect = detection.class_id <= 10
        kind = "Suspect" if suspect else "Knife"
        number = detection.class_id if suspect else detection.class_id - 10
        color = (80, 220, 80) if suspect else (40, 180, 255)
        label = f"{kind} {number} | class {detection.class_id} | {detection.score:.2f}"
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        cv2.putText(output, label, (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 2, cv2.LINE_AA)
    cv2.putText(output, f"Inference FPS: {fps:.1f}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return output


class PreviewPublisher:
    """只编码最新缓存帧；行驶控制和扫描推理线程只提交引用。"""

    def __init__(self, state: PreviewState, quality: int = 80):
        self.state, self.quality = state, quality
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.last_forward_sequence = None
        self.last_scan_sequence = None
        self.inference_completed = deque(maxlen=240)
        self.thread = threading.Thread(target=self._run, name="vision-preview", daemon=True)
        self.thread.start()

    def submit_forward(self, captured):
        if not captured.is_fresh(config.CAMERA_FRAME_MAX_AGE_S):
            return
        with self.condition:
            if self.closed or (self.last_forward_sequence is not None
                               and captured.sequence <= self.last_forward_sequence):
                return
            self.last_forward_sequence = captured.sequence
            self.pending = ("driving", captured.bgr, (), captured.captured_monotonic, 0.0)
            self.condition.notify()

    def submit_scan(self, frame, detections, sequence, captured_monotonic):
        with self.condition:
            if self.closed or sequence is None or (self.last_scan_sequence is not None
                                                   and sequence <= self.last_scan_sequence):
                return
            self.last_scan_sequence = sequence
            now = time.monotonic()
            self.inference_completed.append(now)
            while len(self.inference_completed) > 1 and self.inference_completed[0] < now - 2.0:
                self.inference_completed.popleft()
            elapsed = now - self.inference_completed[0]
            fps = (len(self.inference_completed) - 1) / elapsed if elapsed > 0 else 0.0
            self.pending = ("scanning", frame, detections, captured_monotonic, fps)
            self.condition.notify()

    def _run(self):
        import cv2

        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                mode, frame, detections, captured_at, fps = self.pending
                self.pending = None
            try:
                if mode == "scanning":
                    output = annotate(frame, detections, fps)
                else:
                    output = frame.copy()
                    cv2.putText(output, "Forward preview - no inference", (10, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                ok, encoded = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if not ok:
                    raise RuntimeError("无法编码预览图像")
                with self.condition:
                    if self.closed or self.pending is not None:
                        continue
                self.state.publish(encoded.tobytes(), captured_at, fps, mode)
            except Exception as exc:
                runlog.exception("vision_preview_failed", "预览编码停止", exc)
                with self.condition:
                    self.closed = True
                    self.condition.notify_all()
                self.state.error = str(exc)
                return

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()
        self.thread.join(timeout=config.IO_CLOSE_TIMEOUT_S)


class _Stage3PreviewHandler(stage2._Stage2PreviewHandler):
    def do_GET(self):
        if self.path.split("?", 1)[0] == "/":
            try:
                self._reply(PAGE, "text/html; charset=utf-8")
            except (OSError, TimeoutError):
                pass
        else:
            super().do_GET()


class Stage3Vision(stage2.Stage2Vision):
    """One camera owner; forward frames establish health without color inference."""

    def __init__(self, camera, *, host="0.0.0.0", port=8080, jpeg_quality=80):
        super().__init__(camera, host=host, port=port)
        self.jpeg_quality = jpeg_quality
        self.scanning = False

    def start(self):
        super().start()
        if self._server is not None:
            self._server.RequestHandlerClass = _Stage3PreviewHandler

    def _status(self):
        result = super()._status()
        result.update(mode="scanning" if self.scanning else "driving",
                      error=result.get("fault_reason") or result.get("startup_reason"))
        return result

    def latest_frame(self):
        with self._condition:
            self._sync_source_generation_locked()
            frame = self._raw_latest
            return frame if frame is not None and self._fresh(frame.captured_monotonic) else None

    def _detect_frame(self, captured, token):
        # Keep Stage2's camera continuity contract, but do not run HSV or YOLO
        # while driving. YOLO belongs exclusively to the strictly parked scan.
        _epoch, visual, source_generation, _started, stop_generation = token
        with self._condition:
            self._sync_source_generation_locked()
            now = time.monotonic()
            self._runtime_timeout_locked(now)
            if (self._closed or visual != self._visual_generation
                    or stop_generation != self._runtime_stop_generation
                    or not self._source_generation_current_locked()
                    or source_generation is not None and source_generation != self._source_generation
                    or not self._fresh(captured.captured_monotonic, now)
                    or getattr(self.camera, "last_error", None)):
                return
            self._ready, self._startup_reason = True, None
            self._availability_kind, self._active_fault_key = "ready", None
            self._current = stage2.obstacle.ObstacleResult(False, None, None, None, None, True)
            self._sequence, self._captured_at = captured.sequence, captured.captured_monotonic
            self._record_success_locked(captured)
            self._condition.notify_all()

    def publish_scan(self, captured, detections, fps):
        with self._condition:
            self._sync_source_generation_locked()
            if (not self._closed and self._source_generation_current_locked()
                    and getattr(captured, "source_generation", None) == self._source_generation):
                self._pending_confirmation = (captured, tuple(detections), fps,
                                              "scanning", self._visual_generation)
                self._condition.notify_all()

    def _encode_loop(self):
        import cv2
        while not self._stop.is_set():
            with self._condition:
                self._condition.wait_for(lambda: self._stop.is_set() or self._pending is not None
                                         or self._pending_confirmation is not None, timeout=.2)
                if self._stop.is_set():
                    return
                scan = self._pending_confirmation is not None
                if scan:
                    item, self._pending_confirmation = self._pending_confirmation, None
                else:
                    item, self._pending = self._pending, None
            if item is None:
                continue
            captured, detections, fps, _phase, generation = item
            try:
                output = annotate(captured.bgr, detections, fps) if scan else captured.bgr.copy()
                ok, encoded = cv2.imencode(".jpg", output,
                                          [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                if not ok:
                    raise RuntimeError("无法编码JPEG画面")
                with self._condition:
                    self._sync_source_generation_locked()
                    if self._closed or generation != self._visual_generation:
                        continue
                    if not self._source_generation_current_locked():
                        continue
                    jpeg = encoded.tobytes()
                    if scan:
                        self._last_obstacle_jpeg = jpeg
                        self._last_obstacle_id += 1
                    if (self._fresh(captured.captured_monotonic) and
                            (self._jpeg_captured_at is None
                             or captured.captured_monotonic >= self._jpeg_captured_at)):
                        self._jpeg, self._jpeg_captured_at = jpeg, captured.captured_monotonic
                        self._jpeg_sequence += 1
                        self._jpeg_generation = generation
                        self._fps = fps if scan else 0.
                    self._condition.notify_all()
            except Exception as exc:
                self._preview_failed("JPEG预览编码失败：" + str(exc))


class Stage3Link(stage2.Stage2Link):
    """A fresh CH5 distance commits one entry; empty upper space is normal."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stage3_trigger_enabled = False
        self.stage3_trigger_pending = None
        self.stage3_scan_started = False
        self._stage3_upper_key = None
        self._stage3_upper_identity = None

    def _reset_link_state(self):
        super()._reset_link_state()
        self.stage3_trigger_pending = None
        self._stage3_upper_key = self._stage3_upper_identity = None

    def _accept_upper(self, upper):
        previous = self.culvert_upper
        super()._accept_upper(upper)
        if self.culvert_upper is not upper or previous is upper:
            return
        identity = upper.session_nonce, upper.boot_id
        if identity != self._stage3_upper_identity:
            self._stage3_upper_identity = identity
            self._stage3_upper_key = None
        age = upper.age_ms[1]
        if not isinstance(age, int) or isinstance(age, bool) or age < 0:
            return
        value = upper.distance_mm[1]  # Only the right upper slot, CH5.
        numeric = (isinstance(value, (int, float)) and not isinstance(value, bool)
                   and math.isfinite(value) and 0 <= value < 65535)
        key = (upper.uptime_ms - age) & 0xFFFFFFFF
        source = self._sample_time("upper") - age / 1000.
        fresh = 0 <= time.monotonic() - source <= .250 + 1e-9
        new_source = stage2.sequence._source_key_newer(key, self._stage3_upper_key)
        if new_source:
            self._stage3_upper_key = key
        if (self.stage3_trigger_enabled and not self.stage3_scan_started
                and self.stage3_trigger_pending is None and numeric and new_source and fresh):
            self.stage3_trigger_pending = {"identity": identity, "source_key": key,
                                           "source_at": source, "cycle": upper.upper_cycle_seq,
                                           "distance_mm": value}
            runlog.emit("stage3_right_upper_entry", "CH5首次取得真新鲜有效距离，停车扫描",
                        **self.stage3_trigger_pending)


class _Stage3Streak:
    """Each photograph side has one highest-scoring class and a two-frame streak."""

    def __init__(self, confidence=.8):
        if not math.isfinite(confidence) or not .8 <= confidence <= 1.:
            raise ValueError("confidence必须是0.80..1的有限数")
        self.confidence = confidence
        self.class_id, self.hits = None, 0
        self.last_sequence = self.last_source = None

    def observe(self, captured, detections, barrier, now):
        seq, source = captured.sequence, captured.captured_monotonic
        if (source <= barrier or not stage2.Stage2Vision._fresh(source, now)
                or self.last_sequence is not None and seq <= self.last_sequence
                or self.last_source is not None and source <= self.last_source):
            return None
        self.last_sequence, self.last_source = seq, source
        eligible = [hit for hit in detections
                    if isinstance(hit.class_id, int) and not isinstance(hit.class_id, bool)
                    and 1 <= hit.class_id <= 20 and math.isfinite(hit.score)
                    and self.confidence <= hit.score <= 1.
                    and len(hit.box) == 4 and all(math.isfinite(value) for value in hit.box)
                    and hit.box[2] > hit.box[0] and hit.box[3] > hit.box[1]]
        best = max(eligible, key=lambda hit: hit.score, default=None)
        if best is None:
            self.class_id, self.hits = None, 0
            return None
        self.hits = self.hits + 1 if best.class_id == self.class_id else 1
        self.class_id = best.class_id
        return self.class_id if self.hits >= 2 else None


class Stage3Runner(stage2.Stage2Runner):
    """One right-upper entry scan, then strict stop at the next lower junction."""

    def __init__(self, link, display, vision, xx, *, components, confidence=.8,
                 input_source=None, duration_s=None):
        if not math.isfinite(confidence) or not .8 <= confidence <= 1.:
            raise ValueError("confidence必须是0.80..1的有限数")
        super().__init__(link, display, vision, xx, input_source=input_source, duration_s=duration_s)
        self.components, self.confidence = components, confidence
        self.scanned, self.reported = False, []
        self.junction_after_scan = False
        self._stage3_loop_initialized = False
        self._scan_requested = self._junction_requested = False
        self._scan_side_results = {}
        self._active_scan_side = None
        self._scan_audio_requests = {}

    def _recover_encoder_fault(self):
        # Retain accepted business decisions, never the old measurement itself.
        self._scan_requested |= self.link.stage3_trigger_pending is not None
        self._junction_requested |= self.link.stage2_opening is not None
        super()._recover_encoder_fault()
        self.link.stage2_return = True
        self.link.stage3_trigger_enabled = not self.scanned

    def _checkpoint(self):
        # A previously consumed q/EOF must not interrupt the already-authorized
        # BRAKE confirmation and strict-stop proof.
        if self._safety_stop:
            self._apply_visual_snapshot(self.vision.poll())
            return
        super()._checkpoint()

    def _stationary_tick(self, *, consume=True):
        try:
            return self._stationary_tick_checked(consume=consume)
        except mcu.McuError:
            if not self._known_encoder_fault():
                raise
            self._recover_encoder_fault()
            return False

    def _stationary_tick_checked(self, *, consume=True):
        self._checkpoint()
        self.display.tick()
        if consume:
            self._consume(update=False, means=False)
        if not stage2.sequence._assert_still_stable(self.link, self.display):
            raise mcu.McuError("阶段三扫描／播报期间车轮发生变化，保持停车")
        authorize = getattr(self.vision.camera, "authorize_stationary", None)
        if callable(authorize):
            authorize(True, barrier=self.link._stable_barrier)
        if self._vision_pause_pending:
            self._recover_vision()
            return False
        return True

    def _move_camera(self, pan):
        while True:
            if self._stationary_tick() is False:
                continue
            token = self._visual_health_token
            self.components["gimbal"].set_pwm(pan, SCAN_TILT_US, deadline=self.deadline)
            barrier = time.monotonic() + SCAN_SETTLE_S
            interrupted = False
            while time.monotonic() < barrier:
                if self._stationary_tick() is False:
                    interrupted = True
                    break
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
            if not interrupted and token == self._visual_health_token:
                return barrier
            # A recovered camera needs the applied pose and settle barrier again.

    def _scan_side(self, pan):
        barrier = self._move_camera(pan)
        expires = barrier + SCAN_SIDE_TIMEOUT_S
        streak = _Stage3Streak(self.confidence)
        last_submitted = None
        pending = None
        token = self._visual_health_token
        completed = deque(maxlen=240)
        while True:
            continuous = self._stationary_tick()
            if not continuous or token != self._visual_health_token:
                barrier = self._move_camera(pan)
                expires = barrier + SCAN_SIDE_TIMEOUT_S
                streak = _Stage3Streak(self.confidence)
                last_submitted, token = None, self._visual_health_token
                # An old asynchronous completion is discarded after recovery.
                if pending is not None:
                    pending["invalid"] = True
            now = time.monotonic()
            if pending is not None and pending["done"].is_set():
                result, pending = pending, None
                if not result.get("invalid") and result["token"] == token:
                    if result.get("error") is not None:
                        raise mcu.McuError("阶段三识别故障：" + str(result["error"]))
                    frame, hits = result["frame"], result["hits"]
                    completed.append(now)
                    elapsed = now - completed[0]
                    fps = (len(completed) - 1) / elapsed if elapsed > 0 else 0.
                    self.vision.publish_scan(frame, hits, fps)
                    if now <= expires:
                        found = streak.observe(frame, hits, barrier, now)
                        if found is not None:
                            return found
            if now >= expires:
                if pending is not None and not pending["done"].is_set():
                    raise mcu.McuError("阶段三推理未在本侧5秒内结束，保持停车")
                return None
            if pending is None:
                frame = self.vision.latest_frame()
                if (frame is not None and frame.captured_monotonic > barrier
                        and (last_submitted is None or frame.sequence > last_submitted)):
                    last_submitted = frame.sequence
                    pending = {"frame": frame, "token": token, "done": threading.Event()}
                    request = pending
                    def infer(request=request):
                        try:
                            request["hits"] = self.components["detector"].infer_bgr(
                                request["frame"].bgr, timeout_s=config.DETECTOR_INFER_TIMEOUT_S)
                        except Exception as exc:
                            request["error"] = exc
                        finally:
                            request["done"].set()
                    threading.Thread(target=infer, name="stage3-parked-inference", daemon=True).start()
            time.sleep(stage2.sequence.CONTROL_INTERVAL_S)

    def _broadcast(self, class_id):
        clues_source, speaker = self.components["clues"], self.components["audio"]
        if clues_source.detection(class_id) is None:
            raise mcu.McuError(f"阶段三编号{class_id}没有对应播报配置")
        side = self._active_scan_side or "standalone"
        ticket = self._scan_audio_requests.get(side)
        if ticket is None:
            ticket = dict(class_id=class_id, speaker=speaker, token=self._visual_health_token,
                          continuous=True, completed=False,
                          before_completed=speaker.completed_count, before_failed=speaker.failed_count)
            speaker.set_deadline(None if math.isinf(self.deadline) else self.deadline)
            if not speaker.play(clues_source.detection_audio_path(class_id)):
                raise mcu.McuError(f"阶段三编号{class_id}音频未入队")
            self._scan_audio_requests[side] = ticket
        elif ticket["speaker"] is not speaker or ticket["class_id"] != class_id:
            raise mcu.McuError("阶段三未完成音频请求的所有者或编号改变，保持停车")
        if ticket["completed"]:
            return ticket["continuous"]
        if "local_limit" not in ticket:
            ticket["local_limit"] = time.monotonic() + speaker.remaining_duration_s() + 1.
        while True:
            try:
                healthy = self._stationary_tick()
            except BaseException:
                ticket["continuous"] = False
                raise
            ticket["continuous"] = bool(ticket["continuous"] and healthy
                                         and ticket["token"] == self._visual_health_token)
            active = speaker.poll()
            if speaker.failed_count != ticket["before_failed"]:
                raise mcu.McuError("阶段三播报失败：" + str(speaker.last_error))
            if speaker.completed_count == ticket["before_completed"] + 1 and not active:
                ticket["completed"] = True
                self.reported.append(class_id)
                runlog.emit("stage3_audio_completed", "该侧目标语音实际播放完成", class_id=class_id)
                return ticket["continuous"]
            if not active:
                raise mcu.McuError("阶段三播报结束但缺少实际播放完成证据")
            if time.monotonic() >= ticket["local_limit"]:
                raise mcu.McuError("阶段三播报超过该音频的局部期限，保持停车")
            time.sleep(stage2.sequence.CONTROL_INTERVAL_S)

    def _scan(self):
        link = self.link
        self._scan_requested = True
        self.junction_after_scan |= self._junction_requested or link.stage2_opening is not None
        link.stage3_scan_started = True
        self._strict_brake("CH5涵洞入口：BRAKE并严格停稳后左右扫描", reason="right_upper_entry")
        self.vision.scanning = True
        try:
            for side, pan in (("left", 2500), ("right", 500)):
                if side in self._scan_side_results:
                    continue
                self._active_scan_side = side
                self._action(f"停车识别{side}侧；连续两帧最高编号一致且置信度≥{self.confidence}")
                ticket = self._scan_audio_requests.get(side)
                spoken = ticket["class_id"] if ticket is not None and ticket["completed"] else None
                while True:
                    ticket = self._scan_audio_requests.get(side)
                    if ticket is not None and not ticket["completed"]:
                        found = ticket["class_id"]
                        continuous = self._broadcast(found)
                        spoken = found
                    else:
                        found = self._scan_side(pan)
                        if found is None or spoken is not None:
                            break
                        continuous = self._broadcast(found)
                        spoken = found
                    if continuous is not False:
                        break
                    # Playback already occurred and cannot be undone. Verify
                    # this side anew after recovery without announcing it twice.
                    runlog.emit("stage3_side_rescan", "播报期间相机恢复，当前侧重新识别",
                                side=side, played_class_id=spoken)
                runlog.emit("stage3_side_completed", "单侧扫描完成", side=side,
                            class_id=found, played_class_id=spoken)
                self._scan_side_results[side] = {"class_id": found, "played_class_id": spoken}
            barrier = self._move_camera(1500)
            while True:
                if not self._stationary_tick():
                    barrier = self._move_camera(1500)
                frame = self.vision.latest_frame()
                if frame is not None and frame.captured_monotonic > barrier:
                    break
                if time.monotonic() >= barrier + SCAN_SIDE_TIMEOUT_S:
                    raise mcu.McuError("阶段三云台归中后5秒没有前向新画面，保持停车")
                time.sleep(stage2.sequence.CONTROL_INTERVAL_S)
            self.scanned = True
            self._scan_requested = False
            link.stage3_trigger_pending = None
            self.next_send = time.monotonic()
            runlog.emit("stage3_scan_completed", "左右扫描结束且前向新画面恢复", class_ids=self.reported)
        finally:
            self._active_scan_side = None
            self.vision.scanning = False

    def _recover(self, reason=None):
        self._strict_brake("控制反馈不可用：严格停稳后等新反馈", reason=reason)
        barrier, previous, pairs = self.link._stable_barrier, None, []
        while True:
            self._stationary_tick(consume=False)
            if (self._scan_requested or self._junction_requested
                    or self.link.stage3_trigger_pending is not None or self.link.stage2_opening is not None):
                return
            samples = self._consume(update=True, means=False)
            now = time.monotonic()
            for sample in samples:
                if sample.dual_wall and stage2.sequence._fresh_post_stop_pair(sample, now, barrier, previous):
                    previous = sample
                    pairs.append(sample)
            if len(pairs) >= 2:
                self.display.controller.retry_pending_pair(now=now)
                if all(self._position_at(source) is not None for pair in pairs[-2:] for source in pair.source_times):
                    stage2.sequence._set_ready_feedback(self.link, pairs[-2:], barrier)
                    stage2.sequence._clear_side_recovery(self.link)
                    self.display.controller.begin_alignment(now, source_barrier=barrier)
                    self.next_send = now
                    return
            time.sleep(stage2.sequence.CONTROL_INTERVAL_S)

    def _run_loop(self):
        link, display, controller = self.link, self.display, self.display.controller
        link.side_motion_active, link.side_candidates_enabled = True, False
        link.stage2_return = True  # Receive-time opening means stop, never a Stage2 passage.
        link.set_stage2_mode("normal")
        display._distance_observer = self._observe_distance
        display.road_progress = self.progress
        self.progress.update(link.state, controller._state_at or time.monotonic())
        link.stage2_runtime_poll = self._checkpoint
        link.stage2_visual_health_gate = self._guard_visual_motion
        link.stage2_visual_snapshot_gate = self._guard_visual_snapshot
        link.stage2_visual_stop_pending = lambda: (link.stage3_trigger_pending is not None
                                                   or self._scan_requested or self._junction_requested
                                                   or link.stage2_opening is not None)
        if not self._stage3_loop_initialized:
            link.stage3_trigger_pending = None
            link.stage3_trigger_enabled = True
            self._epoch("outbound")
            self._stage3_loop_initialized = True
        while True:
            self._checkpoint()
            display.tick()
            self._checkpoint()
            samples = self._consume(update=self.armed, means=self.armed)
            now = time.monotonic()
            if not self.armed:
                self._arm_walls(samples, now)
            self._detect_junction(samples, now)
            if (self._scan_requested or link.stage3_trigger_pending is not None) and not self.scanned:
                self._scan()
                if self.junction_after_scan or link.stage2_opening is not None:
                    return True
                continue
            if self._junction_requested or link.stage2_opening is not None:
                self._strict_brake("下一路口：BRAKE并严格停稳，阶段三结束", reason="stage3_junction")
                if not self.scanned:
                    self._action("路口先到：已停稳，本次涵洞扫描未触发")
                return self.scanned
            if self._vision_pause_pending:
                self._recover_vision()
                continue
            if link.side_recovery_active:
                self._recover(reason=getattr(link, "side_recovery_reason", None))
                continue
            executing, confirmed, applied = stage2.sequence._confirmed_wheel_response(
                link, self.sent_at, display.wheel_targets)
            self.response.update(link, executing, now, confirmed, display.state_samples, applied_state=applied)
            self._sample_reference(samples, now)
            self._action(f"{'扫描完成后' if self.scanned else '等待CH5入口'}前进居中；目标{self.xx}mm/s")
            if now + 1e-9 >= self.next_send:
                try:
                    self._checkpoint()
                    target = self._target(time.monotonic())
                    if not any(target.wheel_targets):
                        if any(display.wheel_targets):
                            self._recover(reason="zero_control_target")
                    else:
                        if stage2.sequence._wheel_transition_requires_stop(display.wheel_targets, target.wheel_targets):
                            raise mcu.McuError("阶段三前进目标产生非法归零／换向")
                        display.centering_target = target
                        self._send_target(target)
                        self._mark_started()
                        self.sent_at = time.monotonic()
                        controller.confirm_target_sent(target)
                        self.next_send = self.sent_at + stage2.sequence.COMMAND_INTERVAL_S
                except stage2.sequence._CenteringProposalExpired:
                    self.next_send = time.monotonic()
                except stage2._Stage2VisionInterrupt:
                    continue
                except stage2.sequence._SideRecoveryRequired as exc:
                    self._recover(reason=str(exc))
            display.maybe_print()
            time.sleep(stage2.sequence.CONTROL_INTERVAL_S)

    def run(self):
        try:
            return super().run()
        finally:
            self.link.stage3_trigger_enabled = False


class _Stage3Gimbal(stage2._Stage2Gimbal):
    """Confirm once; Stage3Runner pumps strict parked checks while settling."""

    def set_pwm(self, pan_us, tilt_us, *, deadline=None):
        if self._deadline_expired(deadline):
            raise mcu.McuError("阶段三云台调整已截止")
        command = self.mcu.send_gimbal(stage2.WireGimbalPose.LEVEL_CENTER, pan_us, tilt_us)
        timeout = config.MCU_COMMAND_CONFIRM_TIMEOUT_S
        if deadline is not None:
            record = self.mcu._command_ledger[(stage2.MessageType.CONTROL_TARGET, command)]
            timeout = min(timeout, deadline - record[0])
            if timeout <= 0:
                raise mcu.McuError("阶段三云台确认时间已用尽")
        self.mcu.wait_for_control_applied(command, timeout_s=timeout)
        self.mcu.check_stage2_gimbal_output()
        runlog.emit("stage3_gimbal_applied", "PWM应用已确认，由停车扫描循环等待机械到位",
                    sequence=command, pan_us=pan_us, tilt_us=tilt_us)
        return command


def _components(camera_index, confidence, camera_device=None):
    link = Stage3Link()
    return {"mcu": link, "gimbal": _Stage3Gimbal(link),
            "camera": stage2._Stage2CameraSource(camera_index=camera_index, camera_device=camera_device),
            "detector": detector.Detector(conf_thresh=confidence), "audio": audio.Audio(),
            "clues": clues.Clues()}


def _require(result):
    if not result.ok:
        raise mcu.McuError(f"阶段三初始化失败：{result.name}: {result.message}")


def _prepare(c, display, source):
    stage2._prepare(c, display, source)
    deadline = time.monotonic() + 30.
    _require(selfcheck._check_clues(c["clues"], deadline))
    _require(selfcheck._check_model(c["detector"], deadline, infer=False))
    _require(selfcheck._initialize_audio(c["audio"], deadline, c["clues"]))


def run_vision_test(*, duration_s=DEFAULT_DURATION_S, host="0.0.0.0", port=8080,
                    camera_index=config.CAMERA_INDEX, camera_device=None, confidence=.8,
                    jpeg_quality=80, components=None, input_source=None,
                    vision_factory=None, emitter=None):
    if duration_s is not None and (not math.isfinite(duration_s) or duration_s <= 0):
        raise ValueError("duration必须是有限正数或None")
    if not math.isfinite(confidence) or not .8 <= confidence <= 1.:
        raise ValueError("confidence必须是0.80..1的有限数")
    if not 0 <= port <= 65535 or not 1 <= jpeg_quality <= 100:
        raise ValueError("port必须为0..65535，jpeg-quality必须为1..100")
    if camera_index < 0 or camera_device is not None and camera_index != config.CAMERA_INDEX:
        raise ValueError("camera-index非负且不能与camera-device同时指定")
    write = emitter or stage2._stage2_operator_message
    c, display, source, vision = {}, None, None, None
    succeeded, reason = False, None
    try:
        c = components if components is not None else _components(camera_index, confidence, camera_device)
        source = stage2._Stage2Console(input_source if input_source is not None else
                                      stage2.TerminalSpeedInput(write_line=write), emitter=write)
        display = stage2.Stage2Display(c["mcu"], emitter=write)
        _prepare(c, display, source)
        vision = (vision_factory or Stage3Vision)(c["camera"], host=host, port=port,
                                                  jpeg_quality=jpeg_quality)
        vision._status_emitter = write
        source.vision, source.require_preview = vision, True
        vision.start()
        c["mcu"].stage2_vision = vision
        c["mcu"].stage2_runtime_poll = lambda: stage2.sequence._check_input_exit(source.cancellation_input)
        stage2._wait_preview(c, display, source, vision, write, host)
        stage2._configure_gimbal(c, display, source, write)
        source.require_detection = True
        stage2._wait_startup_vision(c, display, source, vision)
        xx = stage2.sequence._wait_speed(c["mcu"], display, source)
        stage2._sample_stage2_start(c, display, source, vision)
        source.require_preview = source.require_detection = False
        c["mcu"].stage2_runtime_poll = None
        write("阶段三开始：按输入速度前进居中；上方无数值可正常行驶，CH5新鲜有效距离触发左右扫描，下一路口停稳结束。")
        succeeded = Stage3Runner(c["mcu"], display, vision, xx, components=c,
                                 confidence=confidence, input_source=source, duration_s=duration_s).run()
        write("阶段三完成：左右扫描结束，下一路口已确认停稳。" if succeeded else
              "阶段三未完成：已停稳，本次未触发涵洞扫描。")
    except (KeyboardInterrupt, EOFError, stage2.sequence._UserQuit) as exc:
        reason = "收到Ctrl-C" if isinstance(exc, KeyboardInterrupt) else str(exc)
        write("阶段三未完成：" + reason + "；尝试安全停车。")
    except Exception as exc:
        reason = str(exc)
        write("阶段三未完成：" + reason + "；尝试安全停车。")
        runlog.exception("stage3_failed", "阶段三执行失败", exc)
    finally:
        stage2._shutdown(c, display, source, vision, failed=not succeeded, reason=reason)
        for name in ("detector", "audio"):
            resource = c.get(name)
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    runlog.exception("stage3_close_failed", "阶段三资源关闭失败", exc, component=name)
        directory = runlog.current_directory()
        if directory is not None:
            write(f"本次日志：{directory}")
    return succeeded


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="HTTP监听地址")
    parser.add_argument("--port", type=int, default=8080)
    camera_group = parser.add_mutually_exclusive_group()
    camera_group.add_argument("--camera-index", type=stage2._camera_index, default=config.CAMERA_INDEX)
    camera_group.add_argument("--camera-device", help="指定USB相机节点，例如/dev/video0")
    parser.add_argument("--confidence", type=float, default=.8)
    parser.add_argument("--duration", type=stage2._duration, default=None,
                        help="可选运动、扫描及播报总时限；默认到下一路口停稳结束")
    parser.add_argument("--jpeg-quality", type=int, default=80)
    return parser


def _parse_cli(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 1 <= args.jpeg_quality <= 100:
        parser.error("port必须为1..65535，jpeg-quality必须为1..100")
    if not math.isfinite(args.confidence) or not .8 <= args.confidence <= 1.:
        parser.error("confidence必须是0.80..1的有限数")
    return args


def _run_cli(args):
    return 0 if run_vision_test(duration_s=args.duration, host=args.host, port=args.port,
                                 camera_index=args.camera_index, camera_device=args.camera_device,
                                  confidence=args.confidence, jpeg_quality=args.jpeg_quality) else 1


def main(argv=None):
    args = _parse_cli(argv)
    with stage2._Stage2RunLog(mode="stage3_vision"):
        return _run_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
