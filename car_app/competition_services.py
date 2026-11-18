"""Competition RFID and logical audio requests; this service never drives motors."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import shutil
import threading
import time
import wave

import rfid
import runlog
from test_stage1_patrol import PatrolAudio, PatrolRfidReader as _PatrolRfidReader, _record_stage1_rfid_event


_UNSET = object()


class CompetitionRfidReader(_PatrolRfidReader):
    """Stage 1 pass semantics plus actual healthy transaction recovery evidence."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.competition_last_healthy_at = None
        self.competition_consecutive_errors = 0

    def read_patrol_card(self):
        event = super().read_patrol_card()
        if event.transaction_kind != "transport_error":
            self.competition_last_healthy_at = event.monotonic_s
            self.competition_consecutive_errors = 0
        else:
            self.competition_consecutive_errors += 1
        return event

    def stop_polling(self):
        thread = self._thread
        if thread is not None and thread.ident is None and not thread.is_alive():
            self._stop.set()
            self._thread = None
            return
        return super().stop_polling()


# A compatible import from this module uses the stronger competition reader.
PatrolRfidReader = CompetitionRfidReader


class CompetitionAudio(PatrolAudio):
    """Keep child-process references even if the legacy close path detaches them."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._competition_children = []
        self._competition_children_lock = threading.Lock()
        original_emit, original_close = self._backend._emit, self._backend.close

        def remember():
            child = self._backend._proc
            with self._competition_children_lock:
                if child is not None and all(child is not known for known in self._competition_children):
                    self._competition_children.append(child)

        def emit(event, message, **fields):
            remember()
            return original_emit(event, message, **fields)

        def close():
            remember()
            return original_close()

        self._backend._emit, self._backend.close = emit, close

    def children_exited(self):
        try:
            with self._competition_children_lock:
                self._competition_children[:] = [child for child in self._competition_children
                                                 if child.poll() is None]
                return not self._competition_children
        except Exception:
            return False

    def quiescent(self):
        return not self.poll() and self.children_exited()

    def worker_stopped(self):
        return self._thread is None or not self._thread.is_alive()

    def close(self):
        # Thread.start can fail after PatrolAudio records a pending request. A
        # never-started thread cannot be joined, but its queue still needs a
        # terminal result before a replacement owner may be created.
        with self._lock:
            if self._thread is not None and self._thread.ident is None and not self._thread.is_alive():
                self._thread = None
        super().close()
        if self.worker_stopped():
            self._backend.close()


class CompetitionServices:
    """Preserve every RFID pass and retry an audio ticket without replaying successes.

    RFID recovery runs in one background owner while navigation continues.
    The coordinator owns parked audio retries and strict motor stop evidence.
    """

    def __init__(self, c, ledger, active_step=None, display=None, *, stage3_enabled=True):
        self.c, self.ledger, self.active_step, self.display = c, ledger, active_step, display
        self.stage3_enabled = bool(stage3_enabled)
        self.distance_mm, self.phase = None, "启动准备"
        self.rfid_context_at = None
        self.audio_request_outcomes = {}
        self.ledger.competition_audio_request_outcomes = self.audio_request_outcomes
        if not hasattr(ledger, "stage1_announced_patrol_nos"):
            ledger.stage1_announced_patrol_nos = set()
        self._faults = {}
        self._reported_failures = set()
        self.failures = []
        self._backend_requests = {}
        self._terminal_backend_requests = set()
        self._audio_generation = 0
        self._rfid_counter = 0
        self._rfid_ready = self._polling_started = self._closed = False
        self._polling_requested = self._finishing = False
        self._rfid_fault_at = None
        self._deferred_rfid = deque()
        self.rfid_missing_intervals = []
        self._rfid_consecutive_errors = 0
        self._rfid_retry_at = 0.
        self._rfid_recovery_thread = None
        self._rfid_recovery_work = None
        self._rfid_start_stop_lock = threading.Lock()
        self._pending_audio = deque()
        self._failed_audio = set()
        self._audio_event_budget = None
        self._audio_incomplete_count = 0

    @property
    def fault_reason(self):
        # RFID affects task completeness, not the independent motor UART.
        return "；".join(reason for kind, reason in self._faults.items()
                        if not kind.startswith("rfid")) or None

    @property
    def rfid_fault_reason(self):
        return "；".join(reason for kind, reason in self._faults.items()
                        if kind.startswith("rfid")) or None

    @property
    def all_audio_requests_completed(self):
        return self._audio_incomplete_count == 0

    def _message(self, text):
        if self.display is not None:
            self.display(text)

    def _failure(self, kind, reason, *, request_key=None, attempt=None):
        self._faults[kind] = reason
        audit_key = kind, reason, request_key, attempt
        if audit_key not in self._reported_failures:
            self._reported_failures.add(audit_key)
            self.failures.append(reason)
            if kind.startswith("rfid"):
                self.ledger.rfid_errors.append(reason)
            else:
                self.ledger.record_audio_failure(reason)
            runlog.emit("competition_service_failed", reason, terminal=False, kind=kind, request_key=request_key,
                        attempt=attempt, phase=self.phase,
                        road=getattr(self.active_step, "road_id", None))
        if kind.startswith("rfid") and self._rfid_fault_at is None:
            self._rfid_fault_at = time.monotonic()
            self.rfid_missing_intervals.append(dict(start_at=self._rfid_fault_at, end_at=None,
                road_id=getattr(self.active_step, "road_id", None), reason=reason))

    @staticmethod
    def _valid_wav(path):
        if path is None:
            raise ValueError("音频路径缺失")
        with wave.open(str(path), "rb") as source:
            if (source.getframerate() <= 0 or source.getnchannels() <= 0
                    or source.getnframes() <= 0 or not source.readframes(1)):
                raise ValueError("WAV没有有效音频样本")

    def _audio_ready(self):
        player, mapping = self.c.get("audio"), self.c.get("clues")
        if (player is None or mapping is None or getattr(player, "mode", None) != "aplay"
                or not getattr(player, "device", None) or shutil.which("aplay") is None
                or not all(callable(getattr(player, name, None)) for name in
                           ("consume_event", "quiescent", "worker_stopped", "set_deadline"))):
            raise RuntimeError("逐请求播放器、aplay或线索映射不可用")
        if getattr(player, "_closed", False):
            raise RuntimeError("逐请求播放器已关闭")
        for number in range(1, 13):
            clue = mapping.patrol(number)
            if clue is None or clue.number != number or not isinstance(clue.text, str) or not clue.text.strip():
                raise ValueError(f"巡逻点{number}播报映射无效")
            self._valid_wav(mapping.patrol_audio_path(number))
        for class_id in range(1, 21) if self.stage3_enabled else ():
            clue = mapping.detection(class_id)
            if clue is None or not isinstance(clue.text, str) or not clue.text.strip():
                raise ValueError(f"识别类别{class_id}播报映射无效")
            self._valid_wav(mapping.detection_audio_path(class_id))
        player.set_deadline(None)

    def prepare(self):
        if self._closed:
            return False
        try:
            self._audio_ready()
            self._faults.pop("audio_prepare", None)
        except Exception as exc:
            self._failure("audio_prepare", f"比赛音频准备失败：{exc}")
        reader = self.c.get("rfid")
        try:
            if reader is None:
                raise RuntimeError("RFID组件不可用")
            if not self._rfid_ready:
                reader.open()
                self._rfid_ready = True
            self._faults.pop("rfid_prepare", None)
        except Exception as exc:
            self._failure("rfid_prepare", f"RFID初始化失败：{exc}")
        return self.fault_reason is None

    def set_step(self, step, *, distance_mm=None, phase=None):
        self.active_step, self.distance_mm = step, distance_mm
        if phase is not None:
            self.phase = phase

    def start_polling(self):
        with self._rfid_start_stop_lock:
            if self._closed or self._finishing:
                return False
            self._polling_requested = True
            if not self._rfid_ready:
                return False
            if self.active_step is None:
                self._failure("rfid_context", "RFID轮询缺少实际道路上下文")
                return False
            if self._polling_started:
                return True
            try:
                self.c["rfid"].start_polling()
                self._polling_started = True
                if not self._deferred_rfid:
                    self._faults.pop("rfid_context", None)
                return True
            except Exception as exc:
                self._failure("rfid", f"RFID轮询启动失败：{exc}")
                return False

    def queue_audio(self, request_key, path, patrol_no=None, class_id=None):
        hash(request_key)
        path_text = None if path is None else str(Path(path))
        existing = self.audio_request_outcomes.get(request_key)
        if existing is not None:
            if (existing["path"], existing["patrol_no"], existing["class_id"]) != (path_text, patrol_no, class_id):
                raise ValueError("同一音频请求键不能改为不同任务或路径")
            return existing["status"] != "failed"
        record = dict(path=path_text, patrol_no=patrol_no, class_id=class_id,
                      status="waiting", attempts=[])
        self.audio_request_outcomes[request_key] = record
        self._audio_incomplete_count += 1
        self._submit(request_key, record)
        if record["status"] == "waiting":
            self._pending_audio.append(request_key)
        return record["status"] != "failed"

    def audio_completed(self, request_key):
        return self.audio_request_outcomes.get(request_key, {}).get("status") == "completed"

    def _submit(self, request_key, record):
        if (self._closed or record["status"] != "waiting"
                or any(kind.startswith("audio") for kind in self._faults)):
            return
        player = self.c.get("audio")
        attempt = dict(generation=self._audio_generation, backend_id=None, event="submitting",
                       returncode=None, error=None)
        record["attempts"].append(attempt)
        try:
            if player is None or record["path"] is None:
                raise RuntimeError("逐请求音频组件或路径不可用")
            accepted = player.play(record["path"])
            backend_id = getattr(player, "last_request_id", None)
            if isinstance(backend_id, bool) or not isinstance(backend_id, int) or backend_id <= 0:
                raise RuntimeError("播放器没有提供唯一请求编号")
            key = self._audio_generation, backend_id
            if key in self._backend_requests:
                raise RuntimeError("播放器重复使用请求编号")
            attempt["backend_id"] = backend_id
            self._backend_requests[key] = request_key, attempt
            if not accepted:
                raise RuntimeError(getattr(player, "last_error", None) or "播放器拒绝请求")
            record["status"], attempt["event"] = "queued", "queued"
            runlog.emit("competition_audio_requested", "音频逻辑请求进入队列", terminal=False,
                        request_key=request_key, backend_id=backend_id, generation=self._audio_generation,
                        patrol_no=record["patrol_no"], class_id=record["class_id"], path=record["path"])
        except Exception as exc:
            record["status"], attempt["event"], attempt["error"] = "failed", "failed", str(exc)
            self._failure("audio", f"音频请求失败：{exc}", request_key=request_key,
                          attempt=len(record["attempts"]))
            self._failed_audio.add(request_key)

    def _event_available(self):
        budget = self._audio_event_budget
        return budget is None or budget[0] < 16 and time.monotonic() < budget[1]

    def _event_consumed(self):
        if self._audio_event_budget is not None:
            self._audio_event_budget[0] += 1

    def _submit_pending_audio(self):
        for _ in range(len(self._pending_audio)):
            if not self._event_available():
                break
            self._event_consumed()
            key = self._pending_audio.popleft()
            record = self.audio_request_outcomes[key]
            self._submit(key, record)
            if record["status"] == "waiting":
                self._pending_audio.append(key)

    def _consume_audio_events(self):
        player = self.c.get("audio")
        consume = getattr(player, "consume_event", None)
        if not callable(consume):
            return
        while self._event_available():
            try:
                event = consume()
            except Exception as exc:
                self._failure("audio", f"音频事件消费失败：{exc}")
                return
            if event is None:
                return
            self._event_consumed()
            if (not isinstance(event, dict)
                    or type(event.get("request_id")) is not int
                    or event.get("event") not in ("queued", "started", "completed", "failed")):
                self._failure("audio", "音频事件缺少合法逐请求身份或状态")
                continue
            key = self._audio_generation, event.get("request_id")
            binding = self._backend_requests.get(key)
            if binding is None:
                self._failure("audio", "音频结果缺少本次逻辑请求身份")
                continue
            if key in self._terminal_backend_requests:
                continue
            request_key, attempt = binding
            record = self.audio_request_outcomes[request_key]
            if attempt["event"] == "failed":
                # A rejected submission cannot be repaired by delayed started or
                # completed events from that rejected attempt.
                continue
            kind = event.get("event")
            reason = None
            if str(event.get("path")) != record["path"]:
                reason = "音频结果路径与本次请求不一致"
            elif kind == "completed" and (type(event.get("returncode")) is not int
                                           or event.get("returncode") != 0):
                reason = "音频结果没有成功返回码0"
            elif kind == "failed":
                reason = str(event.get("message") or "播放器实际播放失败")
            if reason is not None:
                self._terminal_backend_requests.add(key)
                record["status"], attempt["event"] = "failed", "failed"
                attempt["error"], attempt["returncode"] = reason, event.get("returncode")
                self._failure("audio", reason, request_key=request_key, attempt=len(record["attempts"]))
                self._failed_audio.add(request_key)
                continue
            if kind == "completed":
                self._terminal_backend_requests.add(key)
                record["status"], attempt["event"], attempt["returncode"] = "completed", "completed", 0
                self._audio_incomplete_count -= 1
                self._failed_audio.discard(request_key)
                if record["patrol_no"] is not None:
                    self.ledger.stage1_announced_patrol_nos.add(record["patrol_no"])
                runlog.emit("competition_audio_completed", "本次逻辑请求实际播放完成", terminal=False,
                            request_key=request_key, backend_id=key[1], generation=key[0],
                            patrol_no=record["patrol_no"], class_id=record["class_id"])
            elif kind in ("queued", "started"):
                record["status"], attempt["event"] = kind, kind

    def _consume_rfid_events(self):
        reader = self.c.get("rfid")
        consume = getattr(reader, "consume_event", None)
        if not callable(consume):
            if self._polling_started:
                self._failure("rfid", "RFID缺少非阻塞事件接口")
            return
        while self._event_available():
            try:
                event = self._deferred_rfid.popleft() if self._deferred_rfid else consume()
            except Exception as exc:
                self._failure("rfid", f"RFID事件消费失败：{exc}")
                return
            if event is None:
                return
            self._event_consumed()
            if getattr(event, "transaction_kind", None) not in (None, "transport_error"):
                self._rfid_healthy(event.monotonic_s)
            if (getattr(event, "uid", None) is None and getattr(event, "error", None) is None
                    and getattr(event, "reader_status", None) == rfid.STATUS_NO_CARD):
                self._rfid_healthy(event.monotonic_s)
                continue  # A healthy no-card observation is not a missed task.
            step, distance = self.active_step, self.distance_mm
            if self.rfid_context_at is not None:
                try:
                    step, distance = self.rfid_context_at(event.monotonic_s)
                except Exception as exc:
                    self._deferred_rfid.appendleft(event)
                    self._failure("rfid_context", f"RFID源时刻道路上下文不可用：{exc}")
                    return
            if step is None:
                self._deferred_rfid.appendleft(event)
                self._failure("rfid_context", "RFID事件缺少实际道路上下文")
                return
            self._faults.pop("rfid_context", None)
            try:
                record = _record_stage1_rfid_event(self.ledger, event, step, distance)
                if record.error is not None or record.patrol_no is None:
                    raise RuntimeError(record.error or "RFID没有合法巡逻号")
                self._rfid_healthy(event.monotonic_s)
                self._rfid_counter += 1
                number = record.patrol_no
                mapping = self.c["clues"]
                self._message(mapping.patrol(number).text)
                self.queue_audio(f"rfid:{self._rfid_counter}", mapping.patrol_audio_path(number), patrol_no=number)
                runlog.emit("competition_rfid_event", "实际RFID读块已记录，每次经过独立播报", terminal=False,
                            uid=record.uid_hex, patrol_no=number, road=record.road_id,
                            step=record.step_index, source_at=record.monotonic_s)
            except Exception as exc:
                if getattr(event, "transaction_kind", None) == "transport_error":
                    self._rfid_consecutive_errors += 1
                self._failure("rfid", f"RFID实际读块失败：{exc}")

    def _rfid_healthy(self, source_at):
        if self._rfid_fault_at is not None and source_at <= self._rfid_fault_at:
            return
        self._rfid_consecutive_errors = 0
        self._faults.pop("rfid", None)
        if self._rfid_ready:
            self._faults.pop("rfid_prepare", None)
        if self._rfid_fault_at is not None:
            self.rfid_missing_intervals[-1]["end_at"] = source_at
            runlog.emit("competition_rfid_recovered", "RFID合法交易恢复，结束可能漏读时段", terminal=False,
                        source_at=source_at)
            self._rfid_fault_at = None

    def _poll_rfid_recovery(self):
        """Only this worker may restart the reader; the motor pump never waits."""
        work = self._rfid_recovery_work
        if work is not None:
            if not work["done"].is_set():
                return
            self._rfid_recovery_work = None
            if "error" in work:
                self._failure("rfid", "RFID后台恢复失败：" + work["error"])
            else:
                self._rfid_ready = True
                self._polling_started = work["polling"]
                self._faults.pop("rfid_prepare", None)
        reader = self.c.get("rfid")
        healthy = getattr(reader, "competition_last_healthy_at", None)
        if (healthy is not None and getattr(reader, "_last_read_error", _UNSET) is None):
            self._rfid_healthy(healthy)
        thread = getattr(reader, "_thread", None)
        errors = getattr(reader, "transport_consecutive_errors",
                         getattr(reader, "competition_consecutive_errors", self._rfid_consecutive_errors))
        needs_restart = (reader is None or not self._rfid_ready or thread is None or not thread.is_alive()
                         or errors >= 3)
        now = time.monotonic()
        if (not self._polling_requested or self._closed or self._finishing or not needs_restart
                or now < self._rfid_retry_at):
            return
        self._rfid_retry_at = now + 1.
        self._rfid_consecutive_errors = 0
        work = {"done": threading.Event(), "polling": False}
        self._rfid_recovery_work = work

        def recover():
            rfid.recover_polling_owner(self.c, reader, work, lambda: self._closed or self._finishing,
                                      self._rfid_start_stop_lock)
        self._rfid_recovery_thread = threading.Thread(target=recover, name="competition-rfid-recovery", daemon=True)
        try:
            self._rfid_recovery_thread.start()
        except Exception as exc:
            work["error"] = str(exc)
            work["done"].set()
            self._failure("rfid", "RFID后台恢复线程启动失败：" + str(exc))

    def tick(self, *, step=None, distance_mm=_UNSET, phase=None):
        if self._closed:
            return
        if step is not None:
            self.active_step = step
        if distance_mm is not _UNSET:
            self.distance_mm = distance_mm
        if phase is not None:
            self.phase = phase
        player = self.c.get("audio")
        try:
            if player is not None:
                player.poll()
        except Exception as exc:
            self._failure("audio", f"播放器轮询失败：{exc}")
        self._audio_event_budget = [0, time.monotonic() + .002]
        self._last_event_budget = self._audio_event_budget
        try:
            self._consume_audio_events()
            self._consume_rfid_events()
            self._submit_pending_audio()
        finally:
            self._audio_event_budget = None
        reader = self.c.get("rfid")
        thread = getattr(reader, "_thread", None)
        if (self._polling_started and self._rfid_recovery_work is None
                and (thread is None or not thread.is_alive())):
            self._failure("rfid", "RFID后台轮询已停止")
        self._poll_rfid_recovery()

    def _player_idle(self, player):
        if player is None:
            return True
        quiescent = getattr(player, "quiescent", None)
        return bool(quiescent()) if callable(quiescent) else not player.poll()

    def recover_pending(self, *, reuse_tick=False):
        """Retry active faults while parked; historical failures remain in the ledger."""
        if self._closed:
            return False
        if not reuse_tick or not hasattr(self, "_last_event_budget"):
            self.tick()
        self._poll_rfid_recovery()
        player = self.c.get("audio")
        if any(key.startswith("audio") for key in self._faults):
            try:
                backend = getattr(player, "_backend", None)
                thread = getattr(player, "_thread", None)
                replace = (player is None or getattr(player, "_closed", False)
                           or getattr(backend, "_disabled", False)
                           or thread is not None and not thread.is_alive())
                if replace and player is not None:
                    player.close()
                    stopped = getattr(player, "worker_stopped", None)
                    if not self._player_idle(player) or callable(stopped) and not stopped():
                        return False
                    self._audio_event_budget = self._last_event_budget
                    try:
                        self._consume_audio_events()
                        if not self._event_available():
                            return False
                    finally:
                        self._audio_event_budget = None
                elif not self._player_idle(player):
                    return False
                if replace:
                    factory = self.c.get("_audio_factory")
                    player = (factory() if factory is not None else CompetitionAudio(
                        device=getattr(player, "device", None), mode=getattr(player, "mode", None)))
                    self.c["audio"] = player
                    self._audio_generation += 1
                self._audio_ready()
                self._faults.pop("audio", None)
                self._faults.pop("audio_prepare", None)
                for key in tuple(self._failed_audio):
                    record = self.audio_request_outcomes[key]
                    if record["status"] == "failed":
                        record["status"] = "waiting"
                        self._pending_audio.append(key)
                self._failed_audio.clear()
                self._audio_event_budget = self._last_event_budget
                try:
                    self._submit_pending_audio()
                finally:
                    self._audio_event_budget = None
            except Exception as exc:
                self._failure("audio", f"音频恢复失败：{exc}")
        return self.fault_reason is None

    def finish_after_stop(self, timeout_s=None, *, on_poll=None):
        with self._rfid_start_stop_lock:
            if self._closed:
                return False
            self._finishing, self._polling_requested = True, False
        reader = self.c.get("rfid")
        try:
            if reader is not None:
                reader.stop_polling()
            self._polling_started = False
        except Exception as exc:
            self._failure("rfid", f"停车后RFID停止失败：{exc}")
        end = None if timeout_s is None else time.monotonic() + max(0., timeout_s)
        while True:
            previous_budget = getattr(self, "_last_event_budget", None)
            if on_poll is not None:
                on_poll()
            if getattr(self, "_last_event_budget", None) is previous_budget:
                self.tick(phase="停车播报收尾")
            if self.fault_reason is not None:
                self.recover_pending(reuse_tick=True)
            reader = self.c.get("rfid")
            thread = getattr(reader, "_thread", None)
            recovery = self._rfid_recovery_work
            pending_count = getattr(reader, "pending_event_count", None)
            pending_rfid = (pending_count() if callable(pending_count)
                            else len(getattr(reader, "events", ())))
            if (self.all_audio_requests_completed and self.fault_reason is None
                    and not pending_rfid and not self._deferred_rfid
                    and (thread is None or not thread.is_alive())
                    and (recovery is None or recovery["done"].is_set())):
                return True
            if end is not None and time.monotonic() >= end:
                self._failure("audio", "停车后音频收尾超过局部观察期限")
                return False
            time.sleep(.005)

    def close(self):
        with self._rfid_start_stop_lock:
            if self._closed:
                return
            self._closed = True
        for key in ("rfid", "audio"):
            if key == "rfid" and self._rfid_recovery_work is not None and not self._rfid_recovery_work["done"].is_set():
                # Its sole owner closes after the in-flight open/read exits.
                continue
            component = self.c.get(key)
            if component is not None:
                try:
                    component.close()
                except Exception as exc:
                    self._failure(key, f"{key}关闭失败：{exc}")


__all__ = ["CompetitionAudio", "CompetitionRfidReader", "CompetitionServices", "PatrolRfidReader"]
