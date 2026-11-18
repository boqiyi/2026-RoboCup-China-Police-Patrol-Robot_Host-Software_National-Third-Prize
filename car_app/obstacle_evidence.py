"""Bounded asynchronous evidence for color events and YOLO detections."""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, is_dataclass
import json
from pathlib import Path
import queue
import threading


def _json_value(value):
    if is_dataclass(value):
        return _json_value(asdict(value))
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()
                if key != 'completion_mask'}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if hasattr(value, 'item'):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


def _obstacle_identity(result, detail):
    return 'color_obstacle' if detail.get('target_kind') == 'color_obstacle' else result.color


def _area_diagnostics(detail):
    return _json_value({key: detail.get(key) for key in
        ('target_kind', 'dominant_channel', 'judgment_area_ratio', 'area_interval', 'perchannel')})


class DistanceEvidenceWriter:
    """Control never waits for image encoding or evidence disk writes."""
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        self._recent = deque(maxlen=3)
        self._queue = queue.Queue(maxsize=8)
        self._lock = threading.Lock()
        self._saved_keys = set()
        self._events = 0
        self._closed = False
        self._last_phase = None
        self._last_context = None
        self.errors = []
        self.saved_paths = []
        self.dropped_events = 0
        self._yolo_recent = deque(maxlen=128)
        self._yolo_encounter = 0
        self._yolo_active = None
        self._yolo_empty_since = None
        self._yolo_generation = None
        self._yolo_latest_source = None
        self._yolo_observation_generation = None
        self._yolo_saved_scores = {}
        self._yolo_pending_scores = {}
        self._worker = threading.Thread(target=self._run, name='obstacle-evidence', daemon=True)
        self._worker.start()

    def observe(self, captured, result, detail, phase, *, epoch=None):
        """Call on first/pending/confirmed detections, passing camera generation.

        Buffer the latest three raw observations. At confirmation submit the
        distinct matching obstacle observations, without copying/encoding on
        the serial control path. Acquired BGR arrays are immutable snapshots.
        """
        if self._closed or not result.available or not result.detected:
            return
        key = (epoch, getattr(captured, 'source_generation', 0),
               getattr(captured, 'sequence', None), getattr(captured, 'captured_monotonic', None))
        with self._lock:
            if self._closed:
                return
            identity = _obstacle_identity(result, detail)
            context = (epoch, key[1], detail.get('event_id'), identity)
            repeated_confirmation = (phase == 'confirmed' and self._last_phase == 'confirmed'
                                     and context == self._last_context)
            self._last_phase, self._last_context = phase, context
            if repeated_confirmation:
                return
            if phase == 'first':
                # A new candidate can follow a short detector failure without
                # a camera-owner or road change. Its previous hits are invalid
                # even if they are still inside the original 0.5-second window.
                self._recent.clear()
            if not any(item['key'] == key for item in self._recent):
                self._recent.append(dict(key=key, captured=captured, result=result,
                                         detail=dict(detail), phase=phase))
            if phase != 'confirmed' or key in self._saved_keys:
                return
            matching = [item for item in self._recent
                        if _obstacle_identity(item['result'], item['detail']) == identity
                        and item['key'][:2] == key[:2]
                        and 0 <= key[3] - item['key'][3] <= .5]
            self._events += 1
            job = self._events, matching, _json_value(detail)
            try:
                self._queue.put_nowait(job)
                self._saved_keys.add(key)
            except queue.Full:
                self.dropped_events += 1

    def observe_yolo(self, captured, detections, diagnostics):
        """Keep each class's best frame per encounter, independently of braking.

        Only advancing, healthy successful empty observations prove a one-second
        disappearance. Missing observations reset that timer, never the encounter.
        Frames remain immutable; encoding and file replacement run on the worker.
        """
        with self._lock:
            if self._closed:
                return
            observation_generation = diagnostics.get('observation_generation')
            old_observation = False
            if observation_generation is not None:
                if (self._yolo_observation_generation is None
                        or observation_generation > self._yolo_observation_generation):
                    self._yolo_empty_since = None
                    self._yolo_observation_generation = observation_generation
                else:
                    old_observation = observation_generation < self._yolo_observation_generation
            if captured is None or diagnostics.get('inference_status', 'returned') != 'returned':
                self._yolo_empty_since = None
                return
            generation = getattr(captured, 'source_generation', 0)
            source_time = captured.captured_monotonic
            key = (generation, getattr(captured, 'sequence', None), source_time)
            if key in self._yolo_recent:
                return
            self._yolo_recent.append(key)
            candidates = tuple(_json_value(item) for item in detections
                               if item['class_id'] in (0, 1, 2) and item['score'] >= .60)
            if not candidates and old_observation:
                return
            generation_changed = (not old_observation and self._yolo_generation is not None
                                  and generation != self._yolo_generation)
            if generation_changed:
                self._yolo_empty_since = None
                self._yolo_latest_source = None
            if not old_observation:
                self._yolo_generation = generation
            advancing = (self._yolo_latest_source is None
                         or source_time > self._yolo_latest_source)
            if advancing and not old_observation:
                self._yolo_latest_source = source_time
            if not candidates:
                unreliable = generation_changed or not advancing or any(diagnostics.get(flag)
                    for flag in ('stale_frame', 'road_changed', 'pose_or_mode_changed',
                                 'source_generation_changed', 'continuity_broken'))
                if unreliable:
                    self._yolo_empty_since = None
                elif self._yolo_active is not None:
                    if self._yolo_empty_since is None:
                        self._yolo_empty_since = source_time
                    elif source_time - self._yolo_empty_since >= 1.0:
                        self._yolo_active = None
                        self._yolo_empty_since = None
                return
            self._yolo_empty_since = None
            if self._yolo_active is None:
                self._yolo_encounter += 1
                self._yolo_active = self._yolo_encounter
            for class_id in (0, 1, 2):
                matches = [item for item in candidates if item['class_id'] == class_id]
                if not matches:
                    continue
                detection = max(matches, key=lambda item: item['score'])
                score = detection['score']
                record_key = (self._yolo_active, class_id)
                pending = self._yolo_pending_scores.get(record_key, [])
                if score <= max([self._yolo_saved_scores.get(record_key, -1), *pending]):
                    continue
                job = dict(record_type='yolo_detection', encounter=self._yolo_active,
                    class_id=class_id, score=score, detection=detection, detections=candidates,
                    captured=captured, diagnostics=_json_value(diagnostics))
                try:
                    self._queue.put_nowait(job)
                    self._yolo_pending_scores.setdefault(record_key, []).append(score)
                except queue.Full:
                    self.dropped_events += 1

    def _run(self):
        while True:
            try:
                job = self._queue.get(timeout=.1)
            except queue.Empty:
                if self._closed:
                    return
                continue
            try:
                if isinstance(job, dict):
                    self._write_yolo(job)
                else:
                    self._write(*job)
            except Exception as exc:
                with self._lock:
                    self.errors.append(str(exc))
            finally:
                if isinstance(job, dict):
                    with self._lock:
                        key = (job['encounter'], job['class_id'])
                        pending = self._yolo_pending_scores[key]
                        pending.remove(job['score'])
                        if not pending:
                            del self._yolo_pending_scores[key]
                self._queue.task_done()

    def _write_yolo(self, job):
        import cv2

        record_key = (job['encounter'], job['class_id'])
        with self._lock:
            if job['score'] <= self._yolo_saved_scores.get(record_key, -1):
                return
        captured = job['captured']
        image = captured.bgr
        overlay = image.copy()
        colors = ((0, 200, 255), (0, 220, 0), (255, 120, 0))
        for detection in job['detections']:
            x0, y0, x1, y1 = (round(value) for value in detection['box'])
            color = colors[detection['class_id']]
            cv2.rectangle(overlay, (x0, y0), (x1 - 1, y1 - 1), color, 2)
            label = f"class-{detection['class_id']} conf={detection['score']:.3f} ({detection['score']:.1%})"
            font_scale = .5
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
            font_scale *= min(1., (image.shape[1] - 4) / text_width,
                              (image.shape[0] - 4) / (text_height + baseline))
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
            text_x = max(2, min(x0, image.shape[1] - text_width - 2))
            text_y = max(text_height + 2, min(y0 - 5, image.shape[0] - baseline - 2))
            cv2.putText(overlay, label, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 1)
        flags = [flag for flag in ('stale_frame', 'road_changed', 'pose_or_mode_changed',
                                  'source_generation_changed', 'continuity_broken')
                 if job['diagnostics'].get(flag)]
        for index, flag in enumerate(flags, 1):
            cv2.putText(overlay, flag, (8, max(14, image.shape[0] - 20 * index)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (20, 20, 240), 1)
        target = self.directory / f"encounter-{job['encounter']:06d}" / f"class-{job['class_id']}"
        summary_path = target / 'summary.json'
        record = {key: _json_value(value) for key, value in job.items() if key != 'captured'}
        record.update(source_sequence=getattr(captured, 'sequence', None),
            captured_monotonic=captured.captured_monotonic,
            source_generation=getattr(captured, 'source_generation', 0),
            original=str(target / 'original.png'), annotated=str(target / 'annotated.jpg'))
        payloads = {}
        for name, pixels, suffix in (('original.png', image, '.png'),
                                      ('annotated.jpg', overlay, '.jpg')):
            success, encoded = cv2.imencode(suffix, pixels)
            if not success:
                raise OSError(f'Image encoding failed: {target / name}')
            payloads[name] = encoded.tobytes()
        payloads['summary.json'] = json.dumps(record, ensure_ascii=False, indent=2).encode('utf-8')
        target.mkdir(parents=True, exist_ok=True)
        staged = []
        replaced = []
        try:
            # Stage every encoded file before replacing any existing best image.
            for name, payload in payloads.items():
                temporary = target / f'.{name}.tmp'
                staged.append(temporary)
                temporary.write_bytes(payload)
            previous = {name: (target / name).read_bytes() if (target / name).exists() else None
                        for name in payloads}
            for temporary, name in zip(staged, payloads):
                temporary.replace(target / name)
                replaced.append(name)
        except Exception as exc:
            rollback_errors = []
            for name in replaced:
                try:
                    if previous[name] is None:
                        (target / name).unlink(missing_ok=True)
                    else:
                        temporary = target / f'.{name}.tmp'
                        temporary.write_bytes(previous[name])
                        temporary.replace(target / name)
                except OSError as rollback_exc:
                    rollback_errors.append(str(rollback_exc))
            if rollback_errors:
                raise OSError(f'{exc}; evidence rollback failed: {rollback_errors}') from exc
            raise
        finally:
            for temporary in staged:
                temporary.unlink(missing_ok=True)
        with self._lock:
            if record_key not in self._yolo_saved_scores:
                self.saved_paths.append(str(summary_path))
            self._yolo_saved_scores[record_key] = job['score']

    def _write(self, event, frames, confirmation):
        import cv2
        import numpy as np

        target = self.directory / f'event-{event:04d}'
        target.mkdir(parents=True, exist_ok=False)
        records = []
        for index, frame in enumerate(frames, 1):
            captured, result, detail = frame['captured'], frame['result'], frame['detail']
            original = target / f'{index:02d}-original.png'
            annotated = target / f'{index:02d}-annotated.jpg'
            mask_path = target / f'{index:02d}-color-mask.png'
            image = captured.bgr
            mask = detail.get('completion_mask')
            overlay = image.copy()
            if mask is not None:
                selected = mask != 0
                overlay[selected] = (overlay[selected].astype(np.float32) * .45
                    + np.array((0, 220, 255), np.float32) * .55).astype(np.uint8)
                self._save(mask_path, mask, cv2)
            roi = detail.get('roi')
            if roi:
                x0, y0, x1, y1 = roi
                cv2.rectangle(overlay, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 0), 1)
            if result.bbox:
                x, y, width, height = result.bbox
                cv2.rectangle(overlay, (x, y), (x + width - 1, y + height - 1), (0, 0, 255), 2)
            dominant = detail.get('dominant_channel')
            area = detail.get('judgment_area_ratio', detail.get('area_ratio', 0))
            label = (f"{detail.get('profile')} obstacle | max={dominant} | A={area:.5%}"
                     if dominant else f"{detail.get('profile')} {result.color}: {area:.5%}")
            cv2.putText(overlay, label,
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, .6, (20, 20, 240), 2)
            if dominant:
                channels = detail.get('perchannel', {})
                text = ' '.join(f"{channel}={channels.get(channel, {}).get('area_ratio', 0):.5%}"
                                for channel in ('blue_red', 'orange', 'brown'))
                cv2.putText(overlay, text, (10, 48), cv2.FONT_HERSHEY_SIMPLEX,
                            .5, (20, 20, 240), 1)
                interval = detail.get('area_interval')
                if interval:
                    cv2.putText(overlay, f"interval=[{interval[0]:.5%}, {interval[1]:.5%}]",
                                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, .5, (20, 20, 240), 1)
            self._save(original, image, cv2)
            self._save(annotated, overlay, cv2)
            records.append(dict(source_key=frame['key'], phase=frame['phase'], result=_json_value(result),
                diagnostics=_json_value(detail), original=str(original), annotated=str(annotated),
                color_mask=str(mask_path) if mask is not None else None, **_area_diagnostics(detail)))
        summary = target / 'summary.json'
        summary.write_text(json.dumps(dict(event=event, frames=records, confirmation=confirmation,
                                          **_area_diagnostics(confirmation)),
                                     ensure_ascii=False, indent=2), encoding='utf-8')
        with self._lock:
            self.saved_paths.append(str(summary))

    @staticmethod
    def _save(path, image, cv2):
        success, encoded = cv2.imencode(path.suffix, image)
        if not success:
            raise OSError(f'Image encoding failed: {path}')
        path.write_bytes(encoded.tobytes())

    def close(self, *, timeout_s=3.0):
        """Flush normally; a hung encoder/disk cannot block shutdown forever."""
        with self._lock:
            self._closed = True
        self._worker.join(timeout=max(0.0, float(timeout_s)))
        with self._lock:
            summary = dict(directory=str(self.directory), saved_paths=list(self.saved_paths),
                errors=list(self.errors), dropped_events=self.dropped_events,
                incomplete=self._worker.is_alive() or not self._queue.empty(),
                worker_alive=self._worker.is_alive(),
                queued_events=self._queue.qsize())
        # The final summary itself may hit an unavailable disk. Give that I/O
        # a short independent deadline, using a serialized immutable snapshot
        # so a late writer cannot mutate the returned summary concurrently.
        payload = json.dumps(summary, ensure_ascii=False, indent=2)
        summary_errors = []
        def save_summary():
            try:
                self.directory.mkdir(parents=True, exist_ok=True)
                (self.directory / 'summary.json').write_text(payload, encoding='utf-8')
            except OSError as exc:
                summary_errors.append(f'Could not save evidence summary: {exc}')
        summary_worker = threading.Thread(target=save_summary, name='obstacle-evidence-summary', daemon=True)
        summary_worker.start()
        summary_worker.join(timeout=.25)
        summary['summary_write_incomplete'] = summary_worker.is_alive()
        if summary_worker.is_alive():
            summary['incomplete'] = True
            summary['errors'].append('Evidence summary disk write did not finish before shutdown deadline')
        else:
            summary['errors'].extend(summary_errors)
        return summary
