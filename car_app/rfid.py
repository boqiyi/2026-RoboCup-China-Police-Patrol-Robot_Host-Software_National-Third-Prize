"""RFID 读卡（U13T，优灵科技 0x7F 私有协议，非 Modbus）。

后台读取 0 扇区 1 块，以块数据第 0 字节的 10、20……C0 识别巡逻点。
UID 仅作卡片身份；导航层负责把读块证据绑定到当前道路、步骤和里程。
"""

from __future__ import annotations

from collections import Counter, deque
from contextlib import nullcontext
from dataclasses import dataclass
import threading
import time

import config
import runlog

SYNC = 0x7F
CMD_READ_CARD = 0x10
CMD_READ_CARD_RESP = 0x90
CMD_READ_BLOCK = 0x11
CMD_READ_BLOCK_RESP = 0x91
STATUS_NO_CARD = 0xFF


@dataclass(frozen=True)
class RfidEvent:
    """一次读卡或读卡器异常事件。"""

    uid: bytes | None
    patrol_no: int | None
    monotonic_s: float
    error: str | None = None
    block_no: int | None = None
    block_data: bytes | None = None
    reader_status: int | None = None
    transaction_kind: str | None = None


def xor_checksum(data: bytes) -> int:
    """异或校验。"""
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def frame_command(addr: int, cmd: int, params: bytes = b"") -> bytes:
    """构造 0x7F 帧：帧头 + [长度][地址][命令][参数][XOR校验]，0x7F 转义。"""
    body = bytes([addr, cmd]) + params
    data = bytes([1 + len(body)]) + body
    full = data + bytes([xor_checksum(data)])
    out = bytearray([SYNC])
    for b in full:
        out.append(b)
        if b == SYNC:
            out.append(SYNC)
    return bytes(out)


class _FrameDecoder:
    """声明长度分帧；串口分片和转义字节可跨 feed 保存。"""

    def __init__(self):
        self.frame = None
        self.escape = False

    def feed(self, raw: bytes) -> list[bytes]:
        frames = []
        for byte in raw:
            if self.frame is None:
                if byte == SYNC:
                    self.frame = bytearray()
                continue
            if self.escape:
                self.escape = False
                if byte != SYNC:
                    self.frame = bytearray()  # 前一帧未完整，新帧头重新同步。
            elif byte == SYNC:
                self.escape = True
                continue
            self.frame.append(byte)
            if self.frame[0] < 3:
                self.frame = None
            elif len(self.frame) == self.frame[0] + 1:
                frames.append(bytes(self.frame))
                self.frame = None
        return frames


def _split_frames(raw: bytes) -> list[bytes]:
    return _FrameDecoder().feed(raw)


def _is_valid_response(frame: bytes, command: int) -> bool:
    """验证去转义后的完整声明长度、校验、地址和响应命令。"""
    if len(frame) < 5:
        return False
    length = frame[0]
    if length != len(frame) - 1 or frame[length] != xor_checksum(frame[:length]):
        return False
    return frame[1] == 0 and frame[2] == command


def _is_valid_read_card_response(frame: bytes) -> bool:
    """验证 0x10 身份探测的成功或明确无卡响应。"""
    if not _is_valid_response(frame, CMD_READ_CARD_RESP):
        return False
    if frame[3] == 0:
        return len(frame) == 11  # 读到卡时固定携带 4 字节 UID
    return frame[3] == STATUS_NO_CARD and len(frame) == 5  # 当前无卡


def extract_uid(raw: bytes) -> bytes | None:
    """从读卡响应里提取卡号 UID（4 字节）；无有效卡返回 None。"""
    for frame in _split_frames(raw):
        # U13T 的读卡成功响应固定为 length=10、含校验共 11 字节。
        # 声明长度之外的噪声不属于有效响应；有效 UID 仍须完整校验。
        if len(frame) != 11 or not _is_valid_read_card_response(frame) or frame[3] != 0:
            continue
        return bytes(frame[6:10])
    return None


def parse_patrol_no(block_data: bytes) -> int | None:
    """仅从完整数据块取 10、20……C0；不解码 UID、01..0C 或 ASCII。"""
    if not isinstance(block_data, bytes) or len(block_data) != 16:
        return None
    idx = config.RFID_PATROL_BYTE_INDEX
    if not 0 <= idx < len(block_data):
        return None
    no = block_data[idx]
    return no >> 4 if 0x10 <= no <= 0xC0 and no & 0x0F == 0 else None


def _response_failure_reason(raw: bytes, command: int) -> str:
    if not raw:
        return "no_response"
    decoder = _FrameDecoder()
    frames = decoder.feed(raw)
    if not frames:
        return "incomplete_response" if decoder.frame is not None else "invalid_bytes"
    for frame in reversed(frames):
        if len(frame) < 5 or frame[0] != len(frame) - 1:
            reason = "invalid_length"
        elif xor_checksum(frame) != 0:
            reason = "invalid_checksum"
        elif frame[1] != 0:
            reason = "unexpected_address"
        elif frame[2] != command:
            reason = "unexpected_response_command"
        else:
            return "invalid_response_payload"
    return reason


class RfidReader:
    """U13T 读卡器；轮询线程独占串口并输出带读块证据的非阻塞事件。"""

    def __init__(self, port: str | None = None, baud: int | None = None) -> None:
        self.port = port or config.RFID_PORT
        self.baud = baud or config.RFID_BAUD
        self._ser = None
        self._latest_uid: bytes | None = None
        self._latest_no: int | None = None
        self._latest_seen_at = 0.0
        self._last_consumed_no: int | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._events: deque[RfidEvent] = deque()
        self._emitted_uid: bytes | None = None
        self._last_read_error: str | None = None
        self._last_emitted_error: tuple | None = None
        self._io_lock = threading.Lock()
        self._last_transaction_key: tuple | None = None
        self._response_frames: list[tuple[bytes, float]] = []
        self._no_card_streak = 0
        self.transport_consecutive_errors = 0
        self.last_healthy_at = None
        self._response_classification = None
        self._stats_started_at = time.monotonic()
        self._transaction_counts = Counter()
        self._awaiting_recovery_response = False

    def open(self) -> None:
        import serial

        if self._ser is not None:
            return
        self._ser = serial.Serial(
            self.port,
            self.baud,
            timeout=config.RFID_TIMEOUT_S,
            write_timeout=config.RFID_WRITE_TIMEOUT_S,
        )

    def close(self) -> None:
        self.stop_polling()
        if self._thread is not None and self._thread.is_alive():
            runlog.emit("rfid_close_pending", "RFID worker is still in a serial call", terminal=False)
            return
        self._close_port()

    def _close_port(self) -> None:
        if self._ser is not None:
            ser = self._ser
            self._ser = None
            try:
                ser.close()
            except Exception as exc:
                runlog.exception("rfid_close_failed", "RFID serial close failed", exc, terminal=False)

    def _drain_input(self) -> None:
        """旧数据只作诊断；未在字节和时间边界内排完时不发下一笔命令。"""
        raw = bytearray()
        deadline = time.monotonic() + 0.05
        waiting = self._ser.in_waiting
        while waiting and len(raw) < 4096 and time.monotonic() < deadline:
            chunk = self._ser.read(min(waiting, 4096 - len(raw)))
            if not chunk:
                break
            raw.extend(chunk)
            waiting = self._ser.in_waiting
        if raw or waiting:
            decoder = _FrameDecoder()
            frames = decoder.feed(raw)
            runlog.emit("rfid_preexisting_rx", "发送前残留仅记录，不作为本次响应", terminal=False,
                        raw_hex=raw.hex(), received_bytes=len(raw), pending_bytes=waiting,
                        complete_frames=len(frames), incomplete=decoder.frame is not None)
        if waiting:
            self._response_classification = "preexisting_rx_limit"
            raise RuntimeError("RFID preexisting RX exceeded bounded drain; request not sent")

    def _read_response(self, budget_s: float = 0.15, *, response_command=None) -> bytes:
        """成功读块立即交付；无卡或残帧仍等待原有接收窗口。"""
        raw = bytearray()
        ser = self._ser
        deadline = time.monotonic() + budget_s
        decoder = _FrameDecoder()
        self._response_frames = []
        while not self._stop.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            n = min(4096 - len(raw), ser.in_waiting)
            if n > 0:
                chunk = ser.read(n)
                received_at = time.monotonic()
                raw.extend(chunk)
                if received_at <= deadline:
                    frames = decoder.feed(chunk)
                    self._response_frames.extend((frame, received_at) for frame in frames)
                    if response_command == CMD_READ_BLOCK_RESP and any(
                            len(frame) == 27 and _is_valid_response(frame, response_command)
                            and frame[3] == 0 for frame in frames):
                        return bytes(raw)
                else:
                    self._response_classification = "late_response"
                if len(raw) >= 4096:
                    self._response_classification = "response_rx_limit"
                    raise RuntimeError("RFID response exceeded 4096-byte bound")
            else:
                time.sleep(min(0.002, remaining / 2))
        return bytes(raw)

    def _write_command(self, request: bytes) -> None:
        """单次有界写入必须完整；短写不能作为已发送命令继续读响应。"""
        written = self._ser.write(request)
        if written != len(request):
            self._response_classification = "short_write"
            raise RuntimeError(f"RFID command short write: {written}/{len(request)} bytes")

    def _log_transaction(self, command: int, request: bytes, raw: bytes,
                         frame: bytes | None, event: RfidEvent) -> None:
        """变化时记录详情，并每五秒保留包含重复结果的交易统计。"""
        now = time.monotonic()
        classification = (self._response_classification if event.transaction_kind == "transport_error"
                          else "invalid_patrol_number" if event.reader_status == 0 and event.error
                          else event.transaction_kind)
        classification = classification or "serial_error"
        self._transaction_counts[classification] += 1
        if self._awaiting_recovery_response and event.transaction_kind != "transport_error":
            self._awaiting_recovery_response = False
            runlog.emit("rfid_recovery_stage", "重启后取得首笔真实合法响应", terminal=False,
                        stage="first_valid_response", reader_status=event.reader_status,
                        transaction_kind=event.transaction_kind, source_at=event.monotonic_s)
        if now - self._stats_started_at >= 5.:
            runlog.emit("rfid_statistics", "RFID五秒交易统计", terminal=False,
                        window_s=now - self._stats_started_at,
                        transactions=sum(self._transaction_counts.values()),
                        classifications=dict(self._transaction_counts),
                        thread_alive=self._thread is not None and self._thread.is_alive(),
                        transport_consecutive_errors=self.transport_consecutive_errors,
                        seconds_since_healthy=(None if self.last_healthy_at is None
                                               else max(0., now - self.last_healthy_at)))
            self._stats_started_at = now
            self._transaction_counts.clear()
        key = (command, event.uid, event.patrol_no, event.block_no, event.block_data,
               event.reader_status, event.error, classification)
        if key == self._last_transaction_key:
            return
        self._last_transaction_key = key
        runlog.emit(
            "rfid_transaction", "RFID transaction result changed", terminal=False, command=command,
            request_hex=request.hex(), response_hex=raw.hex(),
            response_frame_hex=frame.hex() if frame is not None else None,
            reader_status=event.reader_status, block_no=event.block_no,
            uid=event.uid.hex() if event.uid is not None else None,
            block_data=event.block_data.hex() if event.block_data is not None else None,
            patrol_no=event.patrol_no, error=event.error,
            source_at=event.monotonic_s, transaction_kind=event.transaction_kind,
            classification=classification,
        )

    def read_uid(self) -> bytes | None:
        """发读卡命令，返回 UID；无卡返回 None，串口异常写入 last_read_error。"""
        if not self._io_lock.acquire(blocking=False):
            raise RuntimeError("RFID serial transaction already in progress")
        try:
            if self._ser is None:
                self.open()
            self._drain_input()
            self._write_command(frame_command(0, CMD_READ_CARD))
            self._response_frames = None
            raw = self._read_response()
            frames = (_split_frames(raw) if self._response_frames is None
                      else [frame for frame, _ in self._response_frames])
            if not any(_is_valid_read_card_response(frame) for frame in frames):
                raise RuntimeError("RFID response missing or invalid")
        except Exception as exc:  # noqa: BLE001 - 轮询异常不打断比赛
            self._last_read_error = str(exc)
            return None
        finally:
            self._io_lock.release()
        self._last_read_error = None
        return next((bytes(frame[6:10]) for frame in frames
                     if _is_valid_read_card_response(frame) and frame[3] == 0), None)

    def ping(self) -> bool:
        """读卡器是否返回了完整、校验通过的读卡或无卡响应。"""
        if not self._io_lock.acquire(blocking=False):
            self._last_read_error = "RFID serial transaction already in progress"
            return False
        try:
            if self._ser is None:
                self.open()
            self._drain_input()
            self._write_command(frame_command(0, CMD_READ_CARD))
            self._response_frames = None
            raw = self._read_response()
            frames = (_split_frames(raw) if self._response_frames is None
                      else [frame for frame, _ in self._response_frames])
            healthy = any(
                _is_valid_read_card_response(frame)
                for frame in frames
            )
            self._last_read_error = None if healthy else "RFID response missing or invalid"
            return healthy
        except Exception as exc:  # noqa: BLE001
            self._last_read_error = str(exc)
            runlog.exception("rfid_ping_failed", "RFID ping failed", exc, terminal=False)
            return False
        finally:
            self._io_lock.release()

    def read_patrol_card(self) -> RfidEvent:
        """读配置的数据块；返回身份、块数据、状态和错误，不修改密钥或卡片。"""
        block_no = config.RFID_PATROL_BLOCK
        if not isinstance(block_no, int) or not 0 <= block_no <= 255:
            error = "RFID patrol block must be an integer in 0..255"
            self._last_read_error = error
            return RfidEvent(None, None, time.monotonic(), error, transaction_kind="card_error")
        request = frame_command(0, CMD_READ_BLOCK, bytes((block_no,)))
        raw = b""
        frame = None
        uid = None
        block_data = None
        status = None
        patrol_no = None
        error = None
        transaction_kind = "transport_error"
        received_at = None
        if not self._io_lock.acquire(blocking=False):
            error = "RFID serial transaction already in progress"
            self._last_read_error = error
            return RfidEvent(None, None, time.monotonic(), error, block_no,
                             transaction_kind="transport_error")
        try:
            self._response_classification = None
            if self._ser is None:
                self.open()
            self._drain_input()
            self._write_command(request)
            self._response_frames = None
            raw = self._read_response(response_command=CMD_READ_BLOCK_RESP)
            timed_frames = ([(candidate, time.monotonic()) for candidate in _split_frames(raw)]
                            if self._response_frames is None else self._response_frames)
            responses = [(candidate, stamp) for candidate, stamp in timed_frames
                         if _is_valid_response(candidate, CMD_READ_BLOCK_RESP)]
            if not responses:
                self._response_classification = (self._response_classification
                    or _response_failure_reason(raw, CMD_READ_BLOCK_RESP))
                raise RuntimeError("RFID block response " + self._response_classification)
            # A successful complete reply is stronger evidence than a no-card reply
            # received in the same bounded transaction. Never take payload from errors.
            frame, received_at = next(((candidate, stamp) for candidate, stamp in responses
                                      if candidate[3] == 0 and len(candidate) == 27), responses[-1])
            status = frame[3]
            transaction_kind = "card_error"
            if status == 0:
                if len(frame) != 27:
                    transaction_kind = "transport_error"
                    self._response_classification = "invalid_success_payload_length"
                    raise RuntimeError(f"RFID successful block response has invalid length: {len(frame)}")
                uid = bytes(frame[6:10])
                block_data = bytes(frame[10:26])
                patrol_no = parse_patrol_no(block_data)
                if patrol_no is None:
                    idx = config.RFID_PATROL_BYTE_INDEX
                    value = f"0x{block_data[idx]:02X}" if 0 <= idx < 16 else "invalid offset"
                    error = f"RFID invalid patrol number: block={block_no} offset={idx} value={value}"
                else:
                    transaction_kind = "card"
            elif status == STATUS_NO_CARD:
                # 非零状态的载荷长度未由手册限定；只认可结构完整的无卡状态，
                # 不把附带字节解读为 UID、巡逻号或成功读块证据。
                transaction_kind = "no_card"
            elif status == 0xFE:
                error = "RFID block operation failed (status=0xFE; card operation or RF interference)"
            elif status == 0xFB:
                error = "RFID device rejected command checksum (status=0xFB)"
                transaction_kind = "transport_error"
                self._response_classification = "reader_command_checksum_rejected"
            else:
                error = f"RFID block operation failed (status=0x{status:02X})"
        except Exception as exc:  # noqa: BLE001 - RFID 故障由账本记录，不阻塞运动
            error = str(exc)
        finally:
            self._io_lock.release()
        self._last_read_error = error
        event = RfidEvent(uid, patrol_no, received_at if received_at is not None else time.monotonic(),
                          error, block_no, block_data, status, transaction_kind)
        if transaction_kind == "transport_error":
            self.transport_consecutive_errors += 1
        else:
            self.transport_consecutive_errors = 0
            self.last_healthy_at = event.monotonic_s
        self._log_transaction(CMD_READ_BLOCK, request, raw, frame, event)
        return event

    def read_patrol_no(self) -> int | None:
        """读数据块中的巡逻点号；无卡或任何错误返回 None。"""
        event = self.read_patrol_card()
        return event.patrol_no if event.error is None else None

    def start_polling(self) -> None:
        """启动后台轮询，避免 RFID 串口阻塞运动租约。"""
        if self._thread is not None and self._thread.is_alive():
            if self._stop.is_set():
                raise RuntimeError("previous RFID worker has not stopped")
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._poll_loop, name="rfid-poll", daemon=True)
        self._thread.start()

    def _poll_loop(self) -> None:
        try:
            self._poll()
        except Exception as exc:
            now = time.monotonic()
            self._last_read_error = str(exc)
            with self._lock:
                self._append_event(RfidEvent(None, None, now, str(exc), transaction_kind="transport_error"))
            runlog.exception("rfid_worker_failed", "RFID polling stopped", exc, terminal=False)
        finally:
            self._close_port()

    def _append_event(self, event: RfidEvent) -> None:
        self._events.append(event)

    def _poll(self) -> None:
        while not self._stop.is_set():
            read_started = time.monotonic()
            event = self.read_patrol_card()
            if self._stop.is_set():
                break
            now = event.monotonic_s
            uid = event.uid
            error = event.error
            with self._lock:
                if error:
                    # Failure and invalid data do not prove departure, and must not
                    # consume the UID's eventual successful recognition.
                    self._no_card_streak = 0
                    error_key = (error, uid, event.block_data, event.reader_status)
                    if error_key != self._last_emitted_error:
                        self._append_event(event)
                        self._last_emitted_error = error_key
                        runlog.emit("rfid_read_failed", error, terminal=False, reader_status=event.reader_status,
                                    block_no=event.block_no,
                                    uid=uid.hex() if uid is not None else None)
                else:
                    self._last_emitted_error = None
                if not error and uid is not None:
                    no = event.patrol_no
                    self._latest_uid = uid
                    self._latest_no = no
                    self._latest_seen_at = now
                    self._no_card_streak = 0
                    if uid != self._emitted_uid:
                        self._append_event(event)
                        self._emitted_uid = uid
                        runlog.emit("rfid_card", "RFID patrol block received", terminal=False, uid=uid.hex(),
                                    patrol_no=no, block_no=event.block_no,
                                    block_data=event.block_data.hex(), reader_status=event.reader_status)
                elif not error and event.reader_status == STATUS_NO_CARD:
                    self._no_card_streak += 1
                    if self._no_card_streak >= config.RFID_NO_CARD_DEBOUNCE:
                        self._latest_uid = None
                        self._latest_no = None
                        self._emitted_uid = None
                        self._last_consumed_no = None
            elapsed = time.monotonic() - read_started
            self._stop.wait(max(0.01, getattr(config, "RFID_POLL_INTERVAL_S", 0.3) - elapsed))

    def latest_patrol_no(self) -> int | None:
        """非阻塞取最新巡逻号（兼容旧调用）。"""
        with self._lock:
            return self._latest_no

    def consume_event(self) -> RfidEvent | None:
        """按发生顺序取一条读块／异常事件，供任务账本绑定道路上下文。"""
        with self._lock:
            return self._events.popleft() if self._events else None

    def pending_event_count(self) -> int:
        with self._lock:
            return len(self._events)

    def consume_patrol_no(self) -> int | None:
        """兼容旧调用：仅在无卡→有卡或换卡后返回巡逻号。"""
        with self._lock:
            no = self._latest_no
            if no == self._last_consumed_no:
                return None
            self._last_consumed_no = no
            return no

    def stop_polling(self) -> None:
        """停止轮询线程（先停再 close，避免 close 与线程读卡竞争串口）。"""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, getattr(config, "IO_CLOSE_TIMEOUT_S", 0.2)))
            if not self._thread.is_alive():
                self._thread = None


def recover_polling_owner(components, reader, work, stopping, start_stop_lock=None):
    """单个服务恢复线程调用；旧读取所有者退出后才允许重开串口。"""
    current = reader
    try:
        if stopping():
            return
        runlog.emit("rfid_recovery_stage", "开始恢复RFID读取所有者", terminal=False, stage="starting")
        if current is None:
            factory = components.get("_factories", {}).get("rfid")
            if factory is None:
                raise RuntimeError("RFID组件不可用")
            current = components["rfid"] = factory()
        current.stop_polling()
        owner = getattr(current, "_thread", None)
        if owner is not None and owner.is_alive():
            raise RuntimeError("旧RFID读取线程尚未退出")
        runlog.emit("rfid_recovery_stage", "旧RFID读取线程已退出", terminal=False, stage="stopped")
        current.close()
        runlog.emit("rfid_recovery_stage", "旧RFID串口已关闭", terminal=False, stage="closed")
        if stopping():
            return
        current.open()
        runlog.emit("rfid_recovery_stage", "RFID串口已重新打开", terminal=False, stage="opened")
        # The service publishes shutdown intent under this same lock. Serial
        # open/read and old-owner joins stay outside it; only authorization and
        # starting the replacement reader must be indivisible with shutdown.
        with start_stop_lock if start_stop_lock is not None else nullcontext():
            if stopping():
                return
            for name in ("transport_consecutive_errors", "competition_consecutive_errors"):
                if hasattr(current, name):
                    setattr(current, name, 0)
            current._awaiting_recovery_response = True
            current.start_polling()
            work["polling"] = True
            runlog.emit("rfid_recovery_stage", "RFID读取线程已启动，等待真实合法响应", terminal=False,
                        stage="polling_started")
    except Exception as exc:
        work["error"] = str(exc)
        runlog.emit("rfid_recovery_stage", "RFID恢复失败", terminal=False, stage="failed", error=str(exc))
    finally:
        if current is not None and stopping():
            try:
                current.close()
            except Exception as exc:
                work["error"] = str(exc)
        work["done"].set()

