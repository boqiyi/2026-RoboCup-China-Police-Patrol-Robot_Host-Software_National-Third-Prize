"""语音播报：用 aplay 串行、非阻塞地播放 WAV 文件。"""

from __future__ import annotations

from collections import deque
import shutil
import subprocess
import time
from pathlib import Path
from tempfile import TemporaryDirectory, TemporaryFile
import wave

import config
import runlog


class Audio:
    def __init__(self, device: str | None = None, mode: str | None = None) -> None:
        self.device = device or config.AUDIO_DEVICE
        self.mode = mode or config.AUDIO_MODE
        self._proc: subprocess.Popen | None = None
        self._queue: deque[tuple[Path, float]] = deque()
        self._active_path: Path | None = None
        self._active_deadline = 0.0
        self._stderr = None
        self._terminating_at: float | None = None
        self._timed_out = False
        self._kill_sent = False
        self._disabled = False
        self._deadline_monotonic: float | None = None
        self.last_error: str | None = None
        self.last_returncode: int | None = None
        self.completed_count = 0
        self.failed_count = 0
        self.started_count = 0

    def set_deadline(self, deadline_monotonic: float | None) -> None:
        self._deadline_monotonic = deadline_monotonic

    def _emit(self, event: str, message: str, **fields) -> None:
        late = self._deadline_monotonic is not None and time.monotonic() >= self._deadline_monotonic
        runlog.emit(event, message, after_deadline=late, **fields)

    def _failure(self, path: Path | None, message: str) -> None:
        self.last_error = message
        self.failed_count += 1
        self._emit("audio_failed", message, path=str(path) if path is not None else None)

    @staticmethod
    def _duration(path: Path) -> float:
        try:
            with wave.open(str(path), "rb") as source:
                rate = source.getframerate()
                duration = source.getnframes() / rate if rate else float("inf")
        except (OSError, EOFError, wave.Error):
            # aplay reports malformed/unsupported WAVs; the fallback is still bounded.
            return min(getattr(config, "AUDIO_UNKNOWN_DURATION_S", 10.0),
                       getattr(config, "AUDIO_MAX_CLIP_S", 30.0))
        if duration > getattr(config, "AUDIO_MAX_CLIP_S", 30.0):
            raise ValueError(f"audio duration exceeds limit: {duration:.2f}s")
        return max(0.01, duration)

    def play(self, wav_path: str | Path) -> bool:
        """把音频加入串行队列；不会阻塞运动控制循环。"""
        if self.mode == "none":
            self._failure(Path(wav_path), "音频模式为 none")
            return False
        if self.mode != "aplay":
            self._failure(Path(wav_path), f"未知 audio 模式 {self.mode}")
            return False
        path = Path(wav_path)
        if self._disabled:
            self._failure(path, "audio player is disabled")
            return False
        if not path.is_file():
            self._failure(path, f"音频文件不存在: {path}")
            return False
        if len(self._queue) >= getattr(config, "AUDIO_QUEUE_LIMIT", 128):
            self._failure(path, "audio queue is full")
            return False
        try:
            duration = self._duration(path)
        except ValueError as exc:
            self._failure(path, str(exc))
            return False
        self._queue.append((path, duration))
        self._emit("audio_queued", "audio queued", path=str(path), duration_s=duration)
        active = self.poll()
        return active or bool(self._queue)

    def poll(self) -> bool:
        """回收已完成播放器，并在空闲时启动下一段音频。"""
        self._reap()
        if self._proc is not None:
            return True
        # Limit failed launch work per poll so a broken player cannot stall control.
        if self._queue and not self._disabled:
            path, duration = self._queue.popleft()
            try:
                self._stderr = TemporaryFile(mode="w+b")
                self._proc = subprocess.Popen(
                    ["aplay", "-q", "-D", self.device, str(path)],
                    stdout=subprocess.DEVNULL,
                    stderr=self._stderr,
                    stdin=subprocess.DEVNULL,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                self._close_stderr()
                self._failure(path, f"aplay 启动失败: {exc}")
                return bool(self._queue)
            self._active_path = path
            self._active_deadline = time.monotonic() + duration + getattr(config, "AUDIO_TIMEOUT_MARGIN_S", 2.0)
            self._terminating_at = None
            self._timed_out = False
            self._kill_sent = False
            self.started_count += 1
            self._emit("audio_started", "audio playback started", path=str(path))
            return True
        return False

    def _close_stderr(self) -> str:
        source, self._stderr = self._stderr, None
        if source is None:
            return ""
        try:
            source.seek(0, 2)
            size = source.tell()
            source.seek(max(0, size - 4096))
            return source.read(4096).decode("utf-8", errors="replace").strip()
        except OSError as exc:
            runlog.exception("audio_stderr_failed", "cannot read player stderr", exc)
            return str(exc)
        finally:
            try:
                source.close()
            except OSError as exc:
                runlog.exception("audio_stderr_close_failed", "cannot close player stderr", exc)

    def remaining_duration_s(self) -> float:
        margin = getattr(config, "AUDIO_TIMEOUT_MARGIN_S", 2.0)
        # Every clip can need both terminate/kill grace periods and a final poll.
        cleanup_s = 0.5
        queued = sum(duration + margin + cleanup_s for _path, duration in self._queue)
        active = (max(0.0, self._active_deadline - time.monotonic()) + cleanup_s
                  if self._proc is not None else 0.0)
        return queued + active

    def drain(self, timeout_s: float | None = None, *, deadline_monotonic: float | None = None) -> bool:
        if deadline_monotonic is not None:
            self.set_deadline(deadline_monotonic)
        failed_before = self.failed_count
        if timeout_s is None:
            # Queue length and per-clip durations already provide a finite bound.
            timeout_s = self.remaining_duration_s() + 1.0
        end = time.monotonic() + max(0.0, timeout_s)
        while self.poll():
            if time.monotonic() >= end:
                self._emit("audio_drain_timeout", "audio drain timed out")
                self.close()
                return False
            time.sleep(0.01)
        return self.failed_count == failed_before and not self._queue and self._proc is None

    def finish_after_stop(self, deadline_monotonic: float | None = None, timeout_s: float | None = None) -> bool:
        return self.drain(timeout_s, deadline_monotonic=deadline_monotonic)

    def close(self) -> None:
        """停止当前播报并清空尚未开始的队列。"""
        while self._queue:
            path, _duration = self._queue.popleft()
            self._failure(path, "audio cancelled before playback")
        proc = self._proc
        self._proc = None
        if proc is None or proc.poll() is not None:
            if proc is not None:
                self._proc = proc
                self._reap()
            self._close_stderr()
            return
        if not self._timed_out:
            self._failure(self._active_path, "audio playback cancelled")
        try:
            proc.terminate()
            proc.wait(timeout=getattr(config, "IO_CLOSE_TIMEOUT_S", 0.2))
        except (OSError, subprocess.SubprocessError):
            try:
                proc.kill()
                proc.wait(timeout=getattr(config, "IO_CLOSE_TIMEOUT_S", 0.2))
            except (OSError, subprocess.SubprocessError) as exc:
                runlog.exception("audio_close_failed", "player did not exit after kill", exc)
        self._close_stderr()
        self._active_path = None

    def _reap(self) -> None:
        """回收已结束的 aplay 子进程，并保留失败返回码供运行日志检查。"""
        if self._proc is None:
            return
        returncode = self._proc.poll()
        if returncode is None:
            now = time.monotonic()
            if not self._timed_out and now >= self._active_deadline:
                self._timed_out = True
                self._terminating_at = now
                self._failure(self._active_path, "aplay 播放超时")
                try:
                    self._proc.terminate()
                except OSError as exc:
                    runlog.exception("audio_terminate_failed", "cannot terminate player", exc)
            elif self._terminating_at is not None and now - self._terminating_at >= 0.2:
                if not self._kill_sent:
                    try:
                        self._proc.kill()
                    except OSError as exc:
                        runlog.exception("audio_kill_failed", "cannot kill player", exc)
                    self._kill_sent = True
                    self._terminating_at = now
                else:
                    # A stuck child must never allow a second player to overlap it.
                    self._terminating_at = None
                    self._disabled = True
                    while self._queue:
                        path, _duration = self._queue.popleft()
                        self._failure(path, "audio cancelled: previous player did not exit after kill")
            return
        path = self._active_path
        stderr = self._close_stderr()
        self._proc = None
        self._active_path = None
        self.last_returncode = int(returncode)
        if self._timed_out:
            if stderr:
                self.last_error = f"{self.last_error}: {stderr}"
                self._emit("audio_player_stderr", self.last_error, path=str(path))
            return
        if returncode == 0:
            self.completed_count += 1
            self._emit("audio_completed", "audio playback completed", path=str(path))
            return
        self._failure(path, f"aplay 返回码 {returncode}: {stderr}")

    def test(self, wav_path: str | Path | None = None) -> tuple[bool, str]:
        """自检声卡链路；默认用临时静音 WAV 验证配置的 ALSA 设备。"""
        if self.mode == "none":
            return False, "audio=none 模式，无法完成比赛播报"
        if self.mode != "aplay":
            return False, f"未知 audio 模式 {self.mode}"
        if wav_path is None:
            if shutil.which("aplay") is None:
                return False, "aplay 不可用"
            with TemporaryDirectory(prefix="car-audio-selfcheck-") as directory:
                path = Path(directory) / "silence.wav"
                with wave.open(str(path), "wb") as output:
                    output.setnchannels(1)
                    output.setsampwidth(2)
                    output.setframerate(8000)
                    output.writeframes(b"\x00\x00" * 80)
                ok, message = self.test(path)
            if ok:
                return True, f"设备 {self.device} 静音输出已确认"
            return False, message

        path = Path(wav_path)
        if not path.exists():
            return False, f"测试音频不存在: {path}"
        cmd = ["aplay", "-q", "-D", self.device, str(path)]
        try:
            proc = subprocess.run(cmd, capture_output=True, timeout=10)
        except subprocess.TimeoutExpired:
            return False, "aplay 播放超时"
        except OSError as exc:
            return False, f"aplay 不可用: {exc}"
        if proc.returncode != 0:
            err = proc.stderr.decode("utf-8", errors="ignore").strip()
            return False, f"aplay 返回码 {proc.returncode}: {err[:200]}"
        return True, f"设备 {self.device} 可播放"
