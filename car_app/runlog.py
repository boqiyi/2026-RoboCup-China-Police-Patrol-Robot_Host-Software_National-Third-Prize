"""Non-blocking, per-run terminal and JSONL event logging."""

from __future__ import annotations

from contextlib import AbstractContextManager
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
import traceback
import uuid

import config

_active = None


def current_directory() -> Path | None:
    """Directory shared by event logs and the current MCU wire capture."""
    return _active.directory if _active is not None else None


def emit(event: str, message: str, *, terminal: bool = True, **fields) -> None:
    current = _active
    if current is not None:
        current.emit(event, message, terminal=terminal, **fields)


def exception(event: str, message: str, exc: BaseException, *, terminal: bool = True, **fields) -> None:
    emit(event, message, terminal=terminal, error=str(exc), exception_type=type(exc).__name__,
         traceback="".join(traceback.format_exception(type(exc), exc, exc.__traceback__)), **fields)


def status(**fields) -> None:
    current = _active
    if current is not None:
        current.status(**fields)


class _Sink:
    def __init__(self, write, limit: int, on_close=None) -> None:
        self.queue = queue.Queue(maxsize=limit)
        self.write = write
        self.on_close = on_close
        self.dropped = 0
        self.error = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True, name="car-log-sink")
        self.thread.start()

    def put(self, item) -> None:
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        try:
            while not self.stopping.is_set() or not self.queue.empty():
                try:
                    item = self.queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                try:
                    self.write(item)
                except Exception as exc:
                    self.error = f"{type(exc).__name__}: {exc}"
                finally:
                    self.queue.task_done()
        finally:
            if self.on_close is not None:
                self.on_close()

    def close(self, timeout: float) -> None:
        self.stopping.set()
        self.thread.join(timeout)


class _Stream:
    def __init__(self, owner, original, event: str) -> None:
        self.owner, self.original, self.event = owner, original, event
        self.buffers = {}
        self.lock = threading.Lock()
        self.encoding = "utf-8"

    def write(self, text: str) -> int:
        with self.lock:
            key = threading.get_ident()
            value = self.buffers.get(key, "") + text
            lines = value.split("\n")
            self.buffers[key] = lines.pop()
            for line in lines:
                if line.strip():
                    self.owner.emit(self.event, line.rstrip("\r"))
        return len(text)

    def flush(self) -> None:
        with self.lock:
            for key, value in list(self.buffers.items()):
                if value:
                    self.owner.emit(self.event, value)
                    self.buffers[key] = ""

    def isatty(self) -> bool:
        return False


class RunLog(AbstractContextManager):
    def __init__(self, mode: str, *, directory=None, terminal=None, capture=True,
                 periodic_status=True) -> None:
        self.mode = mode
        self.run_id = datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
        self.directory = Path(directory if directory is not None else config.LOG_DIR) / self.run_id
        self.terminal = terminal
        self.capture = capture
        self.periodic_status = periodic_status
        self.started = time.monotonic()
        self.last_status = float("-inf")
        self.sequence = 0
        self.lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._status_snapshot = {"mode": mode, "phase": "initializing"}
        self._status_source_at = self.started
        self._status_stop = threading.Event()
        self._status_thread = None
        self._closed = False
        self.terminal_sink = None
        self.file_sink = None
        self._files = []
        self._file_error = None

    def __enter__(self):
        global _active
        self.previous = _active
        self.stdout, self.stderr = sys.stdout, sys.stderr
        stream = self.terminal if self.terminal is not None else self.stdout

        def write_terminal(record):
            stream.write(self._text(record))
            stream.flush()

        self.terminal_sink = _Sink(write_terminal, getattr(config, "LOG_TERMINAL_QUEUE_LIMIT", 4096))

        def write_file(record):
            if not self._files:
                self.directory.mkdir(parents=True, exist_ok=True)
                events = (self.directory / "events.jsonl").open("a", encoding="utf-8")
                try:
                    readable = (self.directory / "run.log").open("a", encoding="utf-8")
                except Exception:
                    events.close()
                    raise
                self._files = [events, readable]
            self._files[0].write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
            self._files[1].write(self._text(record))
            for handle in self._files:
                handle.flush()

        def close_files():
            for handle in self._files:
                try:
                    handle.close()
                except OSError as exc:
                    self.file_sink.error = f"{type(exc).__name__}: {exc}"

        self.file_sink = _Sink(write_file, getattr(config, "LOG_FILE_QUEUE_LIMIT", 16384), close_files)
        _active = self
        if self.capture:
            sys.stdout = _Stream(self, self.stdout, "console")
            sys.stderr = _Stream(self, self.stderr, "stderr")
        self.emit("session_start", "本次运行已开始；本场任务状态重新建立", mode=self.mode,
                  log_directory=str(self.directory), run_id=self.run_id)
        if self.periodic_status:
            self._status_thread = threading.Thread(target=self._status_loop, daemon=True, name="car-log-status")
            self._status_thread.start()
        return self

    @staticmethod
    def _text(record) -> str:
        fields = record["fields"]
        suffix = " " + json.dumps(fields, ensure_ascii=False, default=str) if fields else ""
        return f"[{record['elapsed_s']:8.2f}s] [{record['event']}] {record['message']}{suffix}\n"

    def emit(self, event: str, message: str, *, terminal: bool = True, **fields) -> None:
        with self.lock:
            if self._closed:
                return
            self.sequence += 1
            record = {"seq": self.sequence, "run_id": self.run_id,
                      "time": datetime.now(timezone.utc).isoformat(),
                      "elapsed_s": round(time.monotonic() - self.started, 3),
                      "event": event, "message": message, "fields": fields}
            for sink in (self.terminal_sink if terminal else None, self.file_sink):
                if sink is not None:
                    sink.put(record)
            if event == "session_end":
                self._closed = True

    def status(self, **fields) -> None:
        """Replace the business snapshot; source_age_s never implies fresh telemetry."""
        now = time.monotonic()
        with self._status_lock:
            self._status_snapshot = dict(fields)
            self._status_source_at = now
        self._publish_status(now)

    def _status_loop(self) -> None:
        interval = max(0.01, getattr(config, "LOG_STATUS_INTERVAL_S", 1.0))
        while not self._status_stop.is_set():
            with self._status_lock:
                remaining = self.last_status + interval - time.monotonic()
            wait_s = interval if self.last_status == float("-inf") else max(0.001, remaining)
            if self._status_stop.wait(wait_s):
                break
            self._publish_status(time.monotonic())

    def _publish_status(self, now: float) -> None:
        if self._status_stop.is_set():
            return
        interval = max(0.01, getattr(config, "LOG_STATUS_INTERVAL_S", 1.0))
        with self._status_lock:
            if now - self.last_status < interval:
                return
            self.last_status = now
            fields = dict(self._status_snapshot)
            fields["source_age_s"] = round(max(0.0, now - self._status_source_at), 3)
        deadline = fields.get("deadline_monotonic")
        if isinstance(deadline, (int, float)) and math.isfinite(deadline):
            fields["remaining_s"] = max(0.0, deadline - now)
            fields["after_deadline"] = now >= deadline
        fields["log_terminal_dropped"] = self.terminal_sink.dropped
        fields["log_file_dropped"] = self.file_sink.dropped
        fields["log_file_error"] = self.file_sink.error
        fields["log_terminal_error"] = self.terminal_sink.error
        self.emit("status", "运行状态", **fields)

    def __exit__(self, exc_type, exc, tb):
        global _active
        self._status_stop.set()
        if self._status_thread is not None:
            self._status_thread.join(1.0)
        if exc is not None and not (isinstance(exc, SystemExit) and exc.code in (0, None)):
            exception("unhandled_exception", "运行异常退出", exc)
        if self.capture:
            sys.stdout.flush()
            sys.stderr.flush()
            sys.stdout, sys.stderr = self.stdout, self.stderr
        self.emit("session_end", "本次运行结束", terminal_dropped=self.terminal_sink.dropped,
                  file_dropped=self.file_sink.dropped, file_error=self.file_sink.error,
                  terminal_error=self.terminal_sink.error)
        _active = self.previous
        self.file_sink.close(1.0)
        self.terminal_sink.close(1.0)
        return False


def session(mode: str, *, periodic_status: bool = True) -> RunLog:
    return RunLog(mode, periodic_status=periodic_status)
