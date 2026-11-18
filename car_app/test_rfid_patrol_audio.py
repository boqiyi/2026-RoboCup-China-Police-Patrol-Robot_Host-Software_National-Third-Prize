"""人工搬运读卡播报：只连接 RFID 与播放器，不要求姿态、激光或摄像头。"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import math
from pathlib import Path
import time
from types import SimpleNamespace

import config
from clues import Clues
from competition_services import CompetitionAudio, CompetitionRfidReader, CompetitionServices
from mission_ledger import MissionLedger
from motion_speed_input import TerminalSpeedInput
import runlog


class _DiagnosticLedger:
    """Reuse raw-card validation without constructing a map or a vehicle position."""

    record_audio_failure = MissionLedger.record_audio_failure

    def __init__(self, output):
        self.output = output
        self.rfid_records, self.rfid_errors, self.audio_failures = [], [], []
        self.confirmed_patrol_nos, self.confirmed_patrol_uids = set(), {}

    def record_rfid_event(self, event, step, distance_mm):
        record = MissionLedger.record_rfid_event(self, event, step, distance_mm)
        self.output(f"UID={record.uid_hex or '未知'}；块原字节={record.block_data_hex or '无'}；"
                    f"巡逻点={record.patrol_no if record.patrol_no is not None else '无效'}"
                    + (f"；错误={record.error}" if record.error else ""))
        runlog.emit("rfid_audio_check_card", "独立检查实际读块证据", terminal=False, **asdict(record))
        return record


def run_check(*, duration_s=None, components=None, input_source=None, emitter=None):
    output = emitter or (lambda text: print(text, flush=True))
    c = components if components is not None else {}
    ledger, services = _DiagnosticLedger(output), None
    reported, last_fault = {}, None

    def report():
        nonlocal last_fault
        for key, record in services.audio_request_outcomes.items():
            state = record["status"], len(record["attempts"])
            if reported.get(key) == state:
                continue
            reported[key] = state
            status = {"waiting": "等待提交", "queued": "已排队", "started": "正在播放",
                      "completed": "实际播放完成（返回码0）", "failed": "播放失败"}[record["status"]]
            detail = record["attempts"][-1].get("error") if record["attempts"] else None
            output(f"播报请求 {key}；巡逻点{record['patrol_no']}：{status}"
                   + (f"；{detail}" if detail else ""))
        fault = services.rfid_fault_reason or services.fault_reason
        if fault and fault != last_fault:
            output(f"RFID/播报异常：{fault}")
        last_fault = fault

    try:
        if duration_s is not None and (isinstance(duration_s, bool)
                or not isinstance(duration_s, (int, float))
                or not math.isfinite(duration_s) or duration_s <= 0):
            raise ValueError("duration必须是有限正数秒数")
        output("人工搬运读卡播报：1～9对应1～9号，A/B/C对应10/11/12号巡逻点。")
        output("只连接RFID与播放器；同卡连续只播一次，连续3次合法无卡后可再次播报；q/Ctrl-C/EOF结束。")
        output("块首字节格式为10、20、…、90、A0、B0、C0；实际声音请现场听验。")
        if components is None:
            c["clues"] = Clues()
            c["audio"] = CompetitionAudio()
            c["rfid"] = CompetitionRfidReader()
        step = SimpleNamespace(index=-1, road_id="RFID_AUDIO_CHECK")
        services = CompetitionServices(c, ledger, active_step=step, display=output, stage3_enabled=False)
        c["_ledger"], c["_services"] = ledger, services
        if not services.prepare() or not services.start_polling():
            report()
            output("RFID/播报初始化失败，请核对读卡器串口、音频文件及aplay设备。")
            return False
        if input_source is None:
            input_source = TerminalSpeedInput(write_line=output)
        deadline = math.inf if duration_s is None else time.monotonic() + duration_s
        reason = "达到duration"
        try:
            while time.monotonic() < deadline:
                line = input_source.poll()
                if isinstance(line, str) and line.strip().lower() == "q":
                    reason = "收到q"
                    break
                services.tick(distance_mm=None, phase="人工搬运读卡播报")
                report()
                if services.fault_reason or services.rfid_fault_reason:
                    services.recover_pending(reuse_tick=True)
                time.sleep(.020)
        except (KeyboardInterrupt, EOFError) as exc:
            reason = "收到Ctrl-C" if isinstance(exc, KeyboardInterrupt) else "终端EOF"
        output(f"{reason}，停止新增读卡；已排队播报最多收尾30秒。")

        def finish_poll():
            services.tick(phase="独立播报收尾")
            report()

        finished = services.finish_after_stop(timeout_s=30., on_poll=finish_poll)
        report()
        outcomes = services.audio_request_outcomes
        completed = sum(record["status"] == "completed" for record in outcomes.values())
        runlog.emit("rfid_audio_check_summary", "独立读卡与实际播放结果", terminal=False,
                    requests=outcomes, rfid_errors=ledger.rfid_errors,
                    audio_failures=ledger.audio_failures, finished=finished)
        output(f"检查结束：读到巡逻点{sorted(ledger.confirmed_patrol_nos)}；"
               f"实际播报完成{completed}/{len(outcomes)}；历史播报失败{len(ledger.audio_failures)}次。")
        return finished and bool(outcomes) and not ledger.rfid_errors
    except KeyboardInterrupt:
        output("播报收尾被Ctrl-C中断，取消未完成请求。")
        return False
    except Exception as exc:
        runlog.exception("rfid_audio_check_failed", "独立读卡播报失败", exc, terminal=False)
        output(f"RFID/播报检查失败：{type(exc).__name__}: {exc}")
        return False
    finally:
        if services is not None:
            services.close()
            services._consume_audio_events()
            report()
        else:
            for name in ("rfid", "audio"):
                if c.get(name) is not None:
                    c[name].close()
        if input_source is not None:
            input_source.close()


def _duration(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("duration必须是有限正数秒数")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=_duration, default=None, help="可选运行秒数；默认持续读卡")
    parser.add_argument("--output-dir", type=Path, default=config.LOG_DIR)
    args = parser.parse_args(argv)
    with runlog.RunLog("rfid_patrol_audio", directory=args.output_dir, periodic_status=False):
        return 0 if run_check(duration_s=args.duration) else 1


if __name__ == "__main__":
    raise SystemExit(main())
