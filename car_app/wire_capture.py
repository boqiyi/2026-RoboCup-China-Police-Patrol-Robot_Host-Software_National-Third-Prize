"""Bounded, asynchronous MCU wire evidence; never opens a device itself."""

from __future__ import annotations

import csv
from dataclasses import fields, is_dataclass
from datetime import datetime, timezone
from enum import Enum
import hashlib
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
from typing import Mapping
import uuid

import config
from drivers import protocol_v2 as protocol


WHEEL_TRACE_COLUMNS = (
    "host_rx_monotonic_ms", "frame_seq", "session_nonce", "boot_id", "uptime_ms",
    "sample_sequence", "applied_control_sequence", "elapsed_ms",
    "left_encoder_delta", "right_encoder_delta", "left_target_counts",
    "right_target_counts", "left_target_command", "right_target_command",
    "left_pi_output", "right_pi_output", "left_applied_output",
    "right_applied_output", "reference_yaw_tenth_deg", "yaw_tenth_deg",
    "heading_correction", "flags",
)


def _jsonable(value):
    if isinstance(value, Enum):
        return _jsonable(value.value)
    if is_dataclass(value) and not isinstance(value, type):
        return {field.name: _jsonable(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes(value).hex()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def _decode(frame):
    names = {
        0x10: "decode_hello_request", 0x11: "decode_control_target",
        0x12: "decode_stop", 0x13: "decode_light_control",
        0x14: "decode_motion_action", 0x90: "decode_hello_response",
        0x91: "decode_state", 0x92: "decode_ack", 0x93: "decode_hazard_state",
        0x94: "decode_directional_range4", 0x95: "decode_culvert_upper2",
        0x96: "decode_light_state", 0x97: "decode_diagnostics",
        0x98: "decode_wheel_trace",
    }
    decoder = getattr(protocol, names.get(int(frame.message_type), ""), None)
    return decoder(frame.payload) if decoder is not None else None


class WireCapture:
    """One writer owns all files; producers only submit to a bounded queue.

    ``directory`` is an exact run directory, not its parent. ``status`` exposes
    errors and loss for the caller to report once; capture never prints or emits
    per-frame runlog events. Only the worker performs filesystem operations.
    """

    def __init__(self, directory=None, metadata=None, queue_limit=4096,
                 clock=time.monotonic):
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + uuid.uuid4().hex[:12]
        self.directory = (Path(directory) if directory is not None else
                          Path(config.LOG_DIR) / self.run_id).absolute()
        self._directory_reserved = False
        self._clock = clock
        self._queue = queue.Queue(maxsize=max(1, int(queue_limit)))
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._closed = False
        self._accepting = True
        self._close_timed_out = False
        self._close_deadline = None
        self._finished_monotonic = None
        self._metadata = _jsonable(metadata or {})
        self._source_hashes = {}
        self._stats = dict(accepted_records=0, written_records=0, dropped_records=0,
                           failed_records=0, errors_count=0, first_error=None,
                           rx_bytes=0, tx_bytes=0, frame_records=0,
                           wheel_trace_rows=0, decode_errors=0,
                           trace_decode_errors=0, trace_gap_events=0,
                           tx_failures=0)
        self._sessions = []
        self._session_count = 0
        self._errors = []
        self._offsets = {"rx": 0, "tx": 0}
        self._created_utc = datetime.now(timezone.utc).isoformat()
        self._thread = threading.Thread(target=self._run, daemon=True, name="mcu-wire-capture")
        self._thread.start()

    def _problem(self, stage, exc, *, fatal=False):
        description = f"{stage}: {type(exc).__name__}: {exc}"
        with self._lock:
            self._stats["errors_count"] += 1
            if self._stats["first_error"] is None:
                self._stats["first_error"] = description
            if len(self._errors) < 16:
                self._errors.append(description)
            if fatal:
                self._accepting = False

    def _submit(self, item):
        item[1]["capture_id"] = self.run_id
        with self._lock:
            if not self._accepting:
                self._stats["dropped_records"] += 1
                return
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                self._stats["dropped_records"] += 1
            else:
                self._stats["accepted_records"] += 1

    @staticmethod
    def _direction(direction):
        value = str(direction).lower()
        if value not in ("rx", "tx"):
            raise ValueError("direction must be rx or tx")
        return value

    def record_bytes(self, direction, data, host_monotonic_ms, **context):
        try:
            direction = self._direction(direction)
            data = bytes(data)
            record = _jsonable(context)
            record.update(event="bytes", direction=direction,
                          host_monotonic_ms=host_monotonic_ms, length=len(data))
            if (direction == "tx" and "requested_bytes" in record
                    and record.get("written_bytes") != record["requested_bytes"]):
                with self._lock:
                    self._stats["tx_failures"] += 1
            self._submit(("bytes", record, data))
        except Exception as exc:
            self._problem("record_bytes", exc)

    def record_frame(self, direction, frame, host_monotonic_ms, decoded=None, **context):
        try:
            direction = self._direction(direction)
            decode_allowed = context.get("decode_allowed", True)
            decode_error = context.get("decode_error")
            if not decode_allowed or decode_error:
                decoded = None
            elif decoded is None:
                try:
                    decoded = _decode(frame)
                except Exception as exc:
                    decode_error = f"{type(exc).__name__}: {exc}"
            record = _jsonable(context)
            value = _jsonable(decoded)
            record.update(event="frame", direction=direction,
                          host_monotonic_ms=host_monotonic_ms,
                          frame_seq=int(frame.sequence),
                          message_type=int(frame.message_type),
                          message_type_name=getattr(frame.message_type, "name", None),
                          payload_hex=bytes(frame.payload).hex(), decoded=value,
                          # A command or ACK has no actual MCU timestamp. A
                          # caller's latest STATE belongs in the reference key.
                          uptime_ms=value.get("uptime_ms") if isinstance(value, dict) else None)
            if isinstance(value, dict):
                for key in ("session_nonce", "boot_id"):
                    if key in value:
                        record[key] = value[key]
            if decode_error is not None:
                record["decode_error"] = decode_error
            if decode_error:
                with self._lock:
                    self._stats["decode_errors"] += 1
                    if int(frame.message_type) == 0x98:
                        self._stats["trace_decode_errors"] += 1
            trace_type = getattr(protocol, "WheelTraceTelemetry", None)
            trace = (value if direction == "rx" and int(frame.message_type) == 0x98 and trace_type is not None
                     and isinstance(decoded, trace_type) else None)
            self._submit(("frame", record, trace))
        except Exception as exc:
            self._problem("record_frame", exc)

    def event(self, name, **fields):
        try:
            record = _jsonable(fields)
            record.update(event=str(name))
            record.setdefault("host_monotonic_ms", self._clock() * 1000.0)
            with self._lock:
                if name in ("tx_failed", "tx_partial_write", "partial_write"):
                    self._stats["tx_failures"] += 1
                if name == "trace_sample_gap":
                    self._stats["trace_gap_events"] += 1
                if name == "hello_accepted":
                    self._session_count += 1
                    self._sessions.append(dict(record))
                    del self._sessions[:-128]
            self._submit(("event", record, None))
        except Exception as exc:
            self._problem("event", exc)

    def update_metadata(self, **fields):
        try:
            value = _jsonable(fields)
            with self._lock:
                self._metadata.update(value)
        except Exception as exc:
            self._problem("update_metadata", exc)

    def status(self):
        with self._lock:
            value = dict(self._stats)
            value.update(directory=str(self.directory), run_id=self.run_id,
                         queue_limit=self._queue.maxsize, queued_records=self._queue.qsize(),
                         closing=self._stopping.is_set(), closed=self._closed,
                         close_timed_out=self._close_timed_out, errors=list(self._errors))
            value.update(self._completeness_locked(self._closed))
        value["worker_alive"] = self._thread.is_alive()
        return value

    def _complete_locked(self, final):
        finished = self._finished_monotonic if self._closed else time.monotonic()
        return bool(final and self._stopping.is_set() and not self._close_timed_out
                    and not self._stats["errors_count"] and not self._stats["dropped_records"]
                    and self._stats["accepted_records"] == self._stats["written_records"]
                    and not self._metadata.get("capture_close_interrupted", False)
                    and (self._close_deadline is None or finished <= self._close_deadline))

    def _completeness_locked(self, final):
        capture_complete = self._complete_locked(final)
        trace_complete = (not self._stats["trace_decode_errors"]
                          and not self._stats["trace_gap_events"]
                          and self._metadata.get("shutdown_tail_complete", True)
                          and not self._metadata.get("trace_sample_gaps", 0))
        commands_complete = not self._stats["tx_failures"] and not self._stats["decode_errors"]
        return dict(capture_complete=capture_complete,
                    trace_samples_complete=bool(trace_complete),
                    commands_complete=bool(commands_complete),
                    complete=capture_complete and trace_complete and commands_complete)

    def close(self, timeout=1.0):
        try:
            timeout = max(0.0, float(timeout))
            if not math.isfinite(timeout):
                timeout = 1.0
            with self._lock:
                self._accepting = False
                if not self._stopping.is_set():
                    self._close_deadline = time.monotonic() + timeout
                self._stopping.set()
            self._thread.join(timeout)
            if self._thread.is_alive():
                with self._lock:
                    self._close_timed_out = True
        except Exception as exc:
            self._problem("close", exc)

    def _fingerprint_sources(self):
        root = Path(__file__).resolve().parent
        paths = [Path(__file__), root / "mcu.py", root / "drivers" / "protocol_v2.py"]
        entry = Path(sys.argv[0])
        if entry.suffix.lower() == ".py" and entry.is_file():
            paths.append(entry)
        with self._lock:
            extra = self._metadata.get("source_paths", [])
        if isinstance(extra, str):
            extra = [extra]
        paths.extend(Path(path) for path in extra)
        for path in dict.fromkeys(path.resolve() for path in paths):
            try:
                self._source_hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
            except OSError as exc:
                self._source_hashes[str(path)] = {"error": f"{type(exc).__name__}: {exc}"}

    def _manifest(self, final):
        with self._lock:
            stats = dict(self._stats)
            stats.update(self._completeness_locked(final))
            stats.update(closed=final,
                         close_timed_out=self._close_timed_out,
                         queued_records=self._queue.qsize(), errors=list(self._errors))
            metadata = dict(self._metadata)
            sessions = list(self._sessions)
            session_count = self._session_count
        return dict(format_version=1, run_id=self.run_id, created_utc=self._created_utc,
                    directory=str(self.directory), metadata=metadata,
                    sessions=sessions, session_count=session_count,
                    source_sha256=dict(self._source_hashes), capture=stats,
                    files=dict(rx="rx.bin", tx="tx.bin", wire="wire.jsonl",
                               wheel_trace="wheel_trace.csv"))

    def _write_manifest(self, final=False):
        path = self.directory / "manifest.json"
        temporary = self.directory / "manifest.json.tmp"
        value = self._manifest(final)
        temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2), encoding="utf-8")
        # A slow filesystem may have crossed the close deadline while writing.
        # Publish an incomplete manifest in that case, even if all data drained.
        if final and value["capture"]["complete"] and not self._manifest(final)["capture"]["complete"]:
            value = self._manifest(final)
            temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2), encoding="utf-8")
        temporary.replace(path)
        if final and value["capture"]["complete"] and not self._manifest(final)["capture"]["complete"]:
            # A delayed atomic replacement is also a shutdown timeout. Only
            # this worker rewrites the manifest; close() never touches handles.
            value = self._manifest(final)
            temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2), encoding="utf-8")
            temporary.replace(path)

    def _write_item(self, item, streams, csv_writer):
        kind, record, data = item
        if kind == "bytes":
            direction = record["direction"]
            record["offset"] = self._offsets[direction]
            count = streams[direction].write(data)
            if count is not None:
                self._offsets[direction] += count
                with self._lock:
                    self._stats[f"{direction}_bytes"] += count
            if count != len(data):
                raise OSError(f"short {direction} capture write: {count}/{len(data)}")
        streams["wire"].write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        if kind == "frame":
            with self._lock:
                self._stats["frame_records"] += 1
            if data is not None:
                row = {key: data[key] for key in WHEEL_TRACE_COLUMNS[2:]}
                row.update(host_rx_monotonic_ms=record["host_monotonic_ms"],
                           frame_seq=record["frame_seq"])
                csv_writer.writerow(row)
                with self._lock:
                    self._stats["wheel_trace_rows"] += 1

    def _discard_pending(self):
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return
            with self._lock:
                self._stats["failed_records"] += 1
            self._queue.task_done()

    def _reserve_directory(self):
        requested = self.directory
        candidate = requested
        filenames = ("rx.bin", "tx.bin", "wire.jsonl", "wheel_trace.csv", "manifest.json")
        attempt = 0
        while True:
            candidate.mkdir(parents=True, exist_ok=True)
            if not any((candidate / filename).exists() for filename in filenames):
                try:
                    stream = (candidate / "rx.bin").open("xb", buffering=0)
                except FileExistsError:
                    pass
                else:
                    with self._lock:
                        self.directory = candidate
                        self._directory_reserved = True
                    return stream
            attempt += 1
            suffix = "" if attempt == 1 else f"-{attempt}"
            candidate = requested / f"mcu-{self.run_id}{suffix}"

    def _run(self):
        streams = {}
        try:
            streams["rx"] = self._reserve_directory()
            self._fingerprint_sources()
            self._write_manifest()
            streams["tx"] = (self.directory / "tx.bin").open("xb", buffering=0)
            streams["wire"] = (self.directory / "wire.jsonl").open("x", encoding="utf-8", newline="")
            streams["csv"] = (self.directory / "wheel_trace.csv").open("x", encoding="utf-8", newline="")
            csv_writer = csv.DictWriter(streams["csv"], fieldnames=WHEEL_TRACE_COLUMNS)
            csv_writer.writeheader()
            while not self._stopping.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=0.02)
                except queue.Empty:
                    continue
                try:
                    self._write_item(item, streams, csv_writer)
                except Exception:
                    with self._lock:
                        self._stats["failed_records"] += 1
                    raise
                else:
                    with self._lock:
                        self._stats["written_records"] += 1
                finally:
                    self._queue.task_done()
        except Exception as exc:
            self._problem("writer", exc, fatal=True)
            self._discard_pending()
        finally:
            for name, stream in streams.items():
                try:
                    stream.close()
                except Exception as exc:
                    self._problem(f"close_{name}", exc, fatal=True)
            with self._lock:
                if (self._stopping.is_set() and self._close_deadline is not None
                        and time.monotonic() > self._close_deadline):
                    self._close_timed_out = True
            try:
                if self._directory_reserved:
                    self._write_manifest(final=True)
            except Exception as exc:
                self._problem("manifest", exc, fatal=True)
            with self._lock:
                self._finished_monotonic = time.monotonic()
                if (self._stopping.is_set() and self._close_deadline is not None
                        and self._finished_monotonic > self._close_deadline):
                    self._close_timed_out = True
                self._closed = True
