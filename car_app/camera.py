"""USB UVC 摄像头抓帧（OpenCV）。

抓帧失败返回 ``None``。``grab_frame`` 还返回递增序号和单调时间，供安全关键
调用方拒绝陈旧帧；``grab_bgr`` 保留给现有调用方。
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

import config
import runlog


@dataclass(frozen=True)
class CameraFrame:
    bgr: object
    sequence: int
    captured_monotonic: float

    def is_fresh(self, max_age_s: float, now: float | None = None) -> bool:
        if max_age_s < 0:
            return False
        reference = time.monotonic() if now is None else now
        return 0.0 <= reference - self.captured_monotonic <= max_age_s


class Camera:
    def __init__(self, index: int | None = None) -> None:
        self.index = index if index is not None else config.CAMERA_INDEX
        self._cap = None
        self._sequence = 0
        self._latest: CameraFrame | None = None
        self._ready = threading.Event()
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._closed = False
        self.last_error: str | None = None

    def _start(self) -> None:
        with self._condition:
            if self._closed:
                raise RuntimeError("camera is closed")
            if self._thread is None:
                self._thread = threading.Thread(target=self._capture_loop, name="camera-capture", daemon=True)
                self._thread.start()

    def open(self, timeout_s: float | None = None) -> None:
        self._start()
        timeout = getattr(config, "CAMERA_OPEN_TIMEOUT_S", 3.0) if timeout_s is None else timeout_s
        if not self._ready.wait(max(0.0, timeout)):
            self.last_error = "camera open timed out"
            self._stop.set()
            runlog.emit("camera_failed", self.last_error, index=self.index)
            raise TimeoutError(self.last_error)
        if self._cap is None or self._stop.is_set():
            raise RuntimeError(self.last_error or "camera unavailable")

    def _capture_loop(self) -> None:
        cap = None
        previous_error = None
        try:
            import cv2

            cap = cv2.VideoCapture(self.index)
            if not cap.isOpened():
                raise RuntimeError(f"无法打开摄像头 index={self.index}（/dev/video{self.index}）")
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._cap = cap
            self._ready.set()
            while not self._stop.is_set():
                # A delayed read must not make an old acquisition look fresh.
                acquired = time.monotonic()
                ok, bgr = cap.read()
                if self._stop.is_set():
                    break
                if not ok or bgr is None:
                    self.last_error = "camera read returned no frame"
                    if self.last_error != previous_error:
                        runlog.emit("camera_read_failed", self.last_error, index=self.index)
                        previous_error = self.last_error
                    self._stop.wait(0.02)
                    continue
                with self._condition:
                    self._sequence += 1
                    self._latest = CameraFrame(bgr, self._sequence, acquired)
                    self.last_error = None
                    previous_error = None
                    self._condition.notify_all()
                self._stop.wait(0.001)
        except Exception as exc:
            self.last_error = str(exc)
            self._stop.set()
            runlog.exception("camera_failed", "camera capture stopped", exc, index=self.index)
        finally:
            self._ready.set()
            if cap is not None:
                try:
                    cap.release()
                except Exception as exc:
                    runlog.exception("camera_close_failed", "camera release failed", exc)
            self._cap = None
            with self._condition:
                self._condition.notify_all()

    def grab_frame(self) -> CameraFrame | None:
        """抓取一帧并附加递增序号与采集单调时间；失败返回 ``None``。"""
        if self._closed:
            return None
        self._start()
        with self._condition:
            if self._latest is None and not self._stop.is_set():
                self._condition.wait(max(0.0, getattr(config, "CAMERA_READ_WAIT_S", 0.02)))
            return None if self._stop.is_set() else self._latest

    def grab_bgr(self):
        """兼容现有调用：抓一帧 BGR，失败返回 ``None``。"""
        captured = self.grab_frame()
        return captured.bgr if captured is not None else None

    def close(self) -> None:
        self._closed = True
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=max(0.0, getattr(config, "IO_CLOSE_TIMEOUT_S", 0.2)))
            if thread.is_alive():
                runlog.emit("camera_close_pending", "capture worker is still in a driver call")
        with self._condition:
            self._latest = None
            self._condition.notify_all()
