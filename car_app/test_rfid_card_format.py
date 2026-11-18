"""U13T 实卡读块诊断；只打开指定 RFID 串口，不播放、不控制小车、不写卡。

在 RDK 运行 ``python3 -u test_rfid_card_format.py``，完整证据实时打印到终端并保存。
复制 RFID_DIAGNOSTIC_BEGIN 到 RFID_DIAGNOSTIC_END 之间的全部输出即可回传。
与比赛读卡实现独立，按声明长度累积帧；诊断结果用于后续确定读卡修复。
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid

import config
import rfid

SYNC = 0x7F
READ_CARD = 0x10
READ_BLOCK = 0x11
BLOCK_NUMBER = 1
TRANSACTION_SECONDS = 0.250
MAX_RX_BYTES = 4096


def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def command_frame(command: int, parameters: bytes = b"") -> bytes:
    if command not in (READ_CARD, READ_BLOCK):
        raise ValueError("diagnostic only permits read-card/read-block commands")
    if parameters != (b"\x01" if command == READ_BLOCK else b""):
        raise ValueError("diagnostic only reads sector 0, global block 1")
    body = bytes((3 + len(parameters), 0, command)) + parameters
    escaped = bytearray((SYNC,))
    for byte in body + bytes((xor_checksum(body),)):
        escaped.append(byte)
        if byte == SYNC:
            escaped.append(byte)
    return bytes(escaped)


def describe_frame(frame: bytes, expected_command: int | None = None) -> dict:
    """Interpret status/payload only after full length, XOR, address and command checks."""
    result = {"frame_hex": frame.hex(), "valid": False, "status": None,
              "uid_hex": None, "block_hex": None, "first_byte_hex": None,
              "patrol_no": None}
    if len(frame) < 5 or frame[0] != len(frame) - 1:
        result["reason"] = "invalid_declared_length"
    elif xor_checksum(frame) != 0:
        result["reason"] = "invalid_xor"
    elif frame[1] != 0:
        result["reason"] = "unexpected_address"
    elif frame[2] not in (0x90, 0x91) or (expected_command is not None
                                        and frame[2] != expected_command | 0x80):
        result["reason"] = "unexpected_response_command"
    else:
        status = frame[3]
        result.update(valid=True, command=frame[2], status=status)
        if status == 0:
            expected_length = 11 if frame[2] == 0x90 else 27
            if len(frame) != expected_length:
                result.update(valid=False, reason="invalid_success_payload_length")
                return result
            result["uid_hex"] = frame[6:10].hex()
            result["reason"] = "read_card_success" if frame[2] == 0x90 else "read_block_success"
            if frame[2] == 0x91:
                block = frame[10:26]
                result.update(block_hex=block.hex(), first_byte_hex=f"{block[0]:02X}",
                              patrol_no=rfid.parse_patrol_no(block),
                              block_no=BLOCK_NUMBER)
        elif status == 0xFF:
            result["reason"] = "no_card"
        elif status == 0xFE:
            result["reason"] = "reader_reported_operation_failure"
        elif status == 0xFB:
            result["reason"] = "reader_reported_command_checksum_failure"
        else:
            result["reason"] = "reader_reported_status"
    return result


class FrameAccumulator:
    """Length-delimited, doubled-7F decoder preserving incomplete/noisy wire bytes."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, chunk: bytes) -> list[dict]:
        self.buffer.extend(chunk)
        records = []
        while self.buffer:
            if self.buffer[0] != SYNC:
                stop = self.buffer.find(bytes((SYNC,)))
                if stop < 0:
                    stop = len(self.buffer)
                records.append({"kind": "noise", "raw_hex": self.buffer[:stop].hex()})
                del self.buffer[:stop]
                continue
            decoded = bytearray()
            position = 1
            expected_length = None
            restart = False
            while position < len(self.buffer):
                byte = self.buffer[position]
                if byte == SYNC:
                    if position + 1 == len(self.buffer):
                        break  # A doubled escape or a new header may arrive next chunk.
                    if self.buffer[position + 1] != SYNC:
                        records.append({"kind": "incomplete", "reason": "new_header_before_declared_length",
                                        "raw_hex": self.buffer[:position].hex(),
                                        "frame_hex": decoded.hex()})
                        del self.buffer[:position]
                        restart = True
                        break
                    position += 1
                decoded.append(byte)
                position += 1
                if expected_length is None:
                    expected_length = byte + 1
                    if expected_length < 5:
                        records.append({"kind": "malformed", "reason": "invalid_declared_length",
                                        "raw_hex": self.buffer[:position].hex(), "frame_hex": decoded.hex()})
                        del self.buffer[:position]
                        restart = True
                        break
                if len(decoded) == expected_length:
                    records.append({"kind": "frame", "raw_hex": self.buffer[:position].hex(),
                                    "frame_hex": decoded.hex()})
                    del self.buffer[:position]
                    restart = True
                    break
            if not restart:
                break
        return records

    def finish(self) -> list[dict]:
        if not self.buffer:
            return []
        raw = bytes(self.buffer)
        self.buffer.clear()
        return [{"kind": "incomplete", "reason": "transaction_ended_before_complete_frame",
                 "raw_hex": raw.hex()}]


class DiagnosticLog:
    """Save and print every evidence event, including bounded RX fragments."""

    def __init__(self, output_dir: Path | str) -> None:
        self.run_id = datetime.now().strftime("%Y%m%d-%H%M%S") + "-rfid-" + uuid.uuid4().hex[:8]
        self.directory = Path(output_dir) / self.run_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.events = (self.directory / "events.jsonl").open("w", encoding="utf-8")
        self.readable = (self.directory / "run.log").open("w", encoding="utf-8")
        self.sequence = 0

    def emit(self, event: str, **fields) -> None:
        self.sequence += 1
        record = {"seq": self.sequence, "run_id": self.run_id,
                  "time": datetime.now(timezone.utc).isoformat(), "event": event, "fields": fields}
        serialized = json.dumps(record, ensure_ascii=False)
        self.events.write(serialized + "\n")
        self.readable.write(f"[{self.sequence}] {event} {json.dumps(fields, ensure_ascii=False)}\n")
        self.events.flush()
        self.readable.flush()
        print(serialized, flush=True)

    def close(self, summary: dict) -> None:
        try:
            self.emit("diagnostic_end", **summary)
            (self.directory / "summary.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        finally:
            self.events.close()
            self.readable.close()


def read_transaction(ser, command: int, log: DiagnosticLog, transaction_no: int,
                     run_deadline: float, *, clock=time.monotonic, sleep=time.sleep) -> dict:
    """One serial transaction, <=250 ms and <=4096 total received bytes.

    Preexisting bytes are captured separately and cannot grant this transaction a UID
    or block. No reset_input_buffer() silently discards diagnostic evidence.
    """
    started = clock()
    deadline = min(run_deadline, started + TRANSACTION_SECONDS)
    request = command_frame(command, b"\x01" if command == READ_BLOCK else b"")
    decoder = FrameAccumulator()
    result = {"transaction_no": transaction_no, "command": command, "request_hex": request.hex(),
              "response_hex": "", "frames": [], "preexisting_hex": "", "error": None}
    raw = bytearray()
    remaining_rx = MAX_RX_BYTES

    def record_chunks(chunk: bytes) -> None:
        raw.extend(chunk)
        log.emit("rfid_rx_chunk", transaction_no=transaction_no, command=command,
                 elapsed_s=clock() - started, raw_hex=chunk.hex())
        for record in decoder.feed(chunk):
            if record["kind"] == "frame":
                record.update(describe_frame(bytes.fromhex(record["frame_hex"]), command))
                result["frames"].append(record)
            log.emit("rfid_rx_parse", transaction_no=transaction_no, request_command=command, **record)

    try:
        waiting = max(0, int(ser.in_waiting))
        if waiting:
            preexisting = ser.read(min(waiting, MAX_RX_BYTES))
            result["preexisting_hex"] = preexisting.hex()
            remaining_rx -= len(preexisting)
            log.emit("rfid_preexisting_rx", transaction_no=transaction_no, raw_hex=preexisting.hex(),
                     pending_bytes=waiting, bounded=waiting > MAX_RX_BYTES)
            # Never spend more than the transaction bound draining or reading.
            if remaining_rx <= 0 or waiting > MAX_RX_BYTES or int(ser.in_waiting) > 0:
                raise RuntimeError("preexisting RX exceeds bounded drain; request not sent")
        if clock() >= deadline:
            raise RuntimeError("transaction deadline reached before write")
        ser.write_timeout = min(0.050, max(0.001, deadline - clock()))
        log.emit("rfid_tx", transaction_no=transaction_no, command=command, raw_hex=request.hex())
        written = ser.write(request)
        if written != len(request):
            raise RuntimeError(f"short serial write: {written}/{len(request)}")
        while clock() < deadline:
            waiting = max(0, int(ser.in_waiting))
            if waiting:
                chunk = ser.read(min(waiting, remaining_rx - len(raw)))
                if chunk:
                    record_chunks(chunk)
                else:
                    sleep(min(0.002, max(0.0, deadline - clock())))
                if len(raw) >= remaining_rx:
                    result["error"] = "RX reached 4096-byte transaction bound"
                    break
            else:
                sleep(min(0.002, max(0.0, deadline - clock())))
    except KeyboardInterrupt:
        result["error"] = "interrupted"
        raise
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        for record in decoder.finish():
            log.emit("rfid_rx_parse", transaction_no=transaction_no, request_command=command, **record)
        result["response_hex"] = raw.hex()
        result["received_bytes"] = MAX_RX_BYTES - remaining_rx + len(raw)
        result["elapsed_s"] = clock() - started
        result["complete_response"] = any(frame["valid"] for frame in result["frames"])
        result["classification"] = "complete_response" if result["complete_response"] else "incomplete_response"
        log.emit("rfid_transaction", **result)
    return result


def collect(args, *, serial_factory=None, clock=time.monotonic, sleep=time.sleep) -> tuple[int, Path]:
    """Collect finite evidence; device construction is restricted to one RFID serial port."""
    log = DiagnosticLog(args.output_dir)
    summary = {"port": args.port, "baud": args.baud, "requested_duration_s": args.duration,
               "block_no": BLOCK_NUMBER, "transactions": 0, "complete_responses": 0,
               "successful_block_reads": 0, "patrol_numbers": [], "uid_hex_values": [],
               "errors": [], "interrupted": False, "log_directory": str(log.directory)}
    started = clock()
    deadline = started + args.duration
    ser = None
    exit_code = 0
    print("===== RFID_DIAGNOSTIC_BEGIN =====", flush=True)
    print(f"RFID 只读诊断：{args.port}，{args.baud} baud，{args.duration:g} 秒；不播报、不控制小车。",
          flush=True)
    print("请将卡放在读卡器感应区；原始 TX/RX、分片、残留、完整帧、UID、块数据及错误均逐条打印。",
          flush=True)
    print(f"日志副本目录：{log.directory}；直接复制终端输出即可，无需上传日志文件。", flush=True)
    try:
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        log.emit("diagnostic_start", **summary)
        ser = serial_factory(args.port, args.baud, timeout=0, write_timeout=0.050)
        while clock() < deadline:
            for command in (READ_CARD, READ_BLOCK):
                if clock() >= deadline:
                    break
                summary["transactions"] += 1
                transaction = read_transaction(ser, command, log, summary["transactions"], deadline,
                                               clock=clock, sleep=sleep)
                summary["complete_responses"] += int(transaction["complete_response"])
                if transaction["error"]:
                    summary["errors"].append({"transaction_no": summary["transactions"],
                                              "error": transaction["error"]})
                for frame in transaction["frames"]:
                    if not frame["valid"]:
                        continue
                    if frame["uid_hex"] is not None and frame["uid_hex"] not in summary["uid_hex_values"]:
                        summary["uid_hex_values"].append(frame["uid_hex"])
                    if frame["block_hex"] is not None:
                        summary["successful_block_reads"] += 1
                        no = frame["patrol_no"]
                        if no is not None and no not in summary["patrol_numbers"]:
                            summary["patrol_numbers"].append(no)
                # Bad/no-response transactions also yield to the finite run deadline.
                sleep(min(0.010, max(0.0, deadline - clock())))
    except KeyboardInterrupt:
        summary["interrupted"] = True
        exit_code = 130
    except Exception as exc:
        summary["errors"].append({"error": f"{type(exc).__name__}: {exc}"})
        exit_code = 1
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception as exc:
                summary["errors"].append({"error": f"serial close: {type(exc).__name__}: {exc}"})
                exit_code = 1
        summary["elapsed_s"] = clock() - started
        summary["exit_code"] = exit_code
        log.close(summary)
    print("===== RFID_SUMMARY_BEGIN =====", flush=True)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    print("===== RFID_SUMMARY_END =====", flush=True)
    print(f"诊断结束：完整响应 {summary['complete_responses']}，完整读块 {summary['successful_block_reads']}，"
          f"编号 {summary['patrol_numbers']}。", flush=True)
    print("===== RFID_DIAGNOSTIC_END =====", flush=True)
    print("请复制从 RFID_DIAGNOSTIC_BEGIN 到 RFID_DIAGNOSTIC_END 的全部输出粘贴到对话中；"
          "内容较长可分多条发送。", flush=True)
    return exit_code, log.directory


def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return number


def _positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.RFID_PORT, help="RFID serial port; no automatic device scan")
    parser.add_argument("--baud", type=_positive_int, default=115200)
    parser.add_argument("--duration", type=_positive_float, default=30.0)
    parser.add_argument("--output-dir", type=Path, default=config.LOG_DIR,
                        help="root containing a new diagnostic run directory")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    return collect(parse_arguments(argv))[0]


if __name__ == "__main__":
    raise SystemExit(main())
