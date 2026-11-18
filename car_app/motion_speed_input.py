"""Stoppable terminal input for manually released motion phases.

The reader never touches the MCU or Python's buffered stdin reader. Speed lines
belong to one prompt only; cancellation and input failures remain visible while
the caller is driving, too.
"""

from __future__ import annotations

import codecs
import os
import sys
import threading
from typing import Callable


_POLL_S = 0.02
_JOIN_S = 0.20
_CHUNK_SIZE = 4096
_MAX_LINE = 256
_MAX_DRAIN_BYTES = 65536
_EOF = object()


class _NativeStdin:
    """Read only immediately available raw bytes or Windows console keys."""

    def __init__(self, stream) -> None:
        self.descriptor = stream.fileno()
        self.console = False
        if os.name != "nt":
            return
        import ctypes
        from ctypes import wintypes
        import msvcrt

        self.kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        self.handle = msvcrt.get_osfhandle(self.descriptor)
        self.kernel.GetFileType.argtypes = (wintypes.HANDLE,)
        self.kernel.GetFileType.restype = wintypes.DWORD
        self.kernel.GetConsoleMode.argtypes = (
            wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD),
        )
        self.kernel.GetConsoleMode.restype = wintypes.BOOL
        self.kernel.PeekNamedPipe.argtypes = (
            wintypes.HANDLE, wintypes.LPVOID, wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD), ctypes.POINTER(wintypes.DWORD),
            ctypes.POINTER(wintypes.DWORD),
        )
        self.kernel.PeekNamedPipe.restype = wintypes.BOOL
        mode = wintypes.DWORD()
        self.console = bool(self.kernel.GetConsoleMode(self.handle, ctypes.byref(mode)))
        self.file_type = self.kernel.GetFileType(self.handle)

    def read_available(self):
        if os.name == "nt":
            return self._read_windows()
        import select

        readable, _writable, _exceptional = select.select([self.descriptor], [], [], 0)
        if not readable:
            return None
        return os.read(self.descriptor, _CHUNK_SIZE) or _EOF

    def _read_windows(self):
        import ctypes
        from ctypes import wintypes
        import msvcrt

        if self.console:
            characters = []
            for _ in range(_CHUNK_SIZE):
                if not msvcrt.kbhit():
                    break
                character = msvcrt.getwche()
                if character in ("\x00", "\xe0"):
                    if msvcrt.kbhit():
                        msvcrt.getwch()
                    continue
                characters.append(character)
            return "".join(characters) if characters else None
        if self.file_type == 3:  # FILE_TYPE_PIPE; Windows select cannot poll stdin pipes.
            available = wintypes.DWORD()
            if not self.kernel.PeekNamedPipe(
                self.handle, None, 0, None, ctypes.byref(available), None,
            ):
                error = ctypes.get_last_error()
                if error in (109, 232):  # ERROR_BROKEN_PIPE / ERROR_NO_DATA
                    return _EOF
                raise ctypes.WinError(error)
            if not available.value:
                return None
            count = min(available.value, _CHUNK_SIZE)
        elif self.file_type == 1:  # FILE_TYPE_DISK
            count = _CHUNK_SIZE
        elif self.file_type == 2:  # A non-console character device such as NUL.
            return _EOF
        else:
            raise OSError("不支持的标准输入类型")
        return os.read(self.descriptor, count) or _EOF

    def discard_partial_input(self) -> None:
        # Canonical POSIX terminals hide unfinished lines from select/os.read.
        # Flush those digits as well, so typing before a prompt cannot release it.
        if os.name != "nt" and os.isatty(self.descriptor):
            import termios

            termios.tcflush(self.descriptor, termios.TCIFLUSH)


class TerminalSpeedInput:
    """One bounded response slot per prompt, with globally visible cancellation."""

    def __init__(self, *, stream=None, write_line: Callable[[str], None] | None = None) -> None:
        stream = sys.stdin if stream is None else stream
        self._native = _NativeStdin(stream)
        self._decoder = codecs.getincrementaldecoder(
            getattr(stream, "encoding", None) or "utf-8",
        )(errors="replace")
        self._write_line = write_line or (lambda line: print(line, flush=True))
        self._lock = threading.RLock()
        self._stopping = threading.Event()
        self._generation = 0
        self._active = False
        self._response: tuple[int, str] | None = None
        self._error: BaseException | None = None
        self._quit = False
        self._pending = ""
        self._overflow = False
        self._skip_lf = False
        self._thread = threading.Thread(
            target=self._run, name="motion-speed-input", daemon=True,
        )
        self._thread.start()

    def begin_request(self, prompt: str) -> None:
        """Clear old input, print the prompt, then accept its first response."""
        with self._lock:
            self._active = False
            self._response = None
            self._clear_partial()
            drained = 0
            while not self._stopping.is_set() and self._error is None:
                try:
                    chunk = self._native.read_available()
                except BaseException as exc:
                    self._error = exc
                    break
                if chunk is None:
                    break
                if chunk is _EOF:
                    self._error = EOFError("标准输入已关闭")
                    break
                # Preserve q/Ctrl+C, but discard all pending speed lines.
                self._consume_chunk(chunk, generation=None)
                drained += len(chunk)
                if drained >= _MAX_DRAIN_BYTES:
                    self._error = OSError("标准输入积压过多，保持停车并退出")
                    break
            if self._error is None:
                try:
                    self._native.discard_partial_input()
                except BaseException as exc:
                    self._error = exc
            self._clear_partial()
            self._generation += 1
            self._write_line(prompt)
            self._active = not (self._stopping.is_set() or self._error is not None or self._quit)

    def poll(self) -> str | None:
        """Return one speed line/q without waiting; input failures propagate."""
        with self._lock:
            if self._error is not None:
                raise self._error
            if self._quit:
                self._quit = False
                return "q"
            response, self._response = self._response, None
            if response is not None and response[0] == self._generation:
                return response[1]
            return None

    def end_request(self) -> None:
        with self._lock:
            self._active = False
            self._response = None
            self._clear_partial()

    def close(self) -> None:
        self._stopping.set()
        self.end_request()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=_JOIN_S)

    def _clear_partial(self) -> None:
        self._pending = ""
        self._overflow = False
        self._skip_lf = False
        self._decoder.reset()

    def _consume_chunk(self, chunk, *, generation: int | None) -> None:
        text = self._decoder.decode(chunk) if isinstance(chunk, bytes) else chunk
        for character in text:
            if character == "\x03":
                self._error = KeyboardInterrupt()
                self._active = False
                self._response = None
                return
            if character == "\x1a":
                self._error = EOFError("标准输入已关闭")
                self._active = False
                self._response = None
                return
            if character == "\n" and self._skip_lf:
                self._skip_lf = False
                continue
            self._skip_lf = character == "\r"
            if character in ("\r", "\n"):
                line = "" if self._overflow else self._pending
                self._pending, self._overflow = "", False
                if line.strip().lower() == "q":
                    self._quit = True
                    self._active = False
                    self._response = None
                elif generation == self._generation and self._active:
                    self._response = (generation, line)
                    self._active = False
            elif character in ("\b", "\x7f"):
                if not self._overflow:
                    self._pending = self._pending[:-1]
            elif not self._overflow:
                if len(self._pending) < _MAX_LINE:
                    self._pending += character
                else:
                    self._pending, self._overflow = "", True

    def _run(self) -> None:
        while not self._stopping.is_set():
            try:
                # Reading and assigning the request token are atomic with gate changes.
                with self._lock:
                    chunk = self._native.read_available()
                    if chunk is _EOF:
                        self._error = EOFError("标准输入已关闭")
                        return
                    if chunk is not None:
                        generation = self._generation if self._active else None
                        self._consume_chunk(chunk, generation=generation)
                        if self._error is not None:
                            return
            except BaseException as exc:
                with self._lock:
                    self._error = exc
                return
            self._stopping.wait(_POLL_S)
