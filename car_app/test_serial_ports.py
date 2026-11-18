"""端口身份识别：确认哪个 ttyUSB 是 STM32、哪个是 RFID 读卡器，并打印逐口结论。

用法（在 RDK 上）::

    cd /root/car_app
    python3 -u test_serial_ports.py              # 完整识别
    python3 -u test_serial_ports.py --readonly   # 零写入，只列证据

安全规则（对应验收条款 A01"不得把 RFID 当 MCU 试写"）：
先对每个口发**读卡器自己的**私有协议读卡命令（0x7F/0x10）；确认读卡器后，
只有当剩下恰好一个候选口时，才允许向它发 HELLO 确认 STM32。任何情况下都不向
身份未排除的口写 MCU 帧。

本脚本只做识别与打印，不修改任何配置、不驱动运动/云台/灯；
实际 MCU HELLO 会保存 v9 原始采集日志，--readonly 不写日志，
也不打开摄像头或播放声音（只列清单）。改哪两行由人依据输出决定。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import glob
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Mapping, Sequence

import config
import mcu as mcu_mod
import rfid as rfid_mod
import selfcheck
import runlog


@dataclass(frozen=True)
class IdentityPlan:
    """识别计划：哪些口已确认、哪些口允许发 HELLO、哪些口无法判定。"""

    rfid_port: str | None
    hello_targets: tuple[str, ...]
    undecided: tuple[str, ...]


def plan_identification(ports: Sequence[str], rfid_ok: Mapping[str, bool]) -> IdentityPlan:
    """根据读卡器协议探针结果，决定后续允许对哪些口发 HELLO。

    闸门：只有当某个口**已被读卡器协议确认为读卡器**、且剩下恰好一个候选口时，
    才允许向该候选口发 HELLO。否则一律不发 —— HELLO 是 MCU 帧，发给身份未排除的
    设备就是验收条款 A01 禁止的"把 RFID 当 MCU 试写"。
    """
    ordered = tuple(ports)
    readers = [port for port in ordered if rfid_ok.get(port)]
    if len(readers) != 1:
        return IdentityPlan(rfid_port=None, hello_targets=(), undecided=ordered)

    rfid_port = readers[0]
    remaining = tuple(port for port in ordered if port != rfid_port)
    if len(remaining) != 1:
        return IdentityPlan(rfid_port=rfid_port, hello_targets=(), undecided=remaining)

    return IdentityPlan(rfid_port=rfid_port, hello_targets=remaining, undecided=())


@dataclass(frozen=True)
class PortReport:
    """单个口的最终结论：它是什么、凭什么这么判、config 里该写成哪一行。"""

    port: str
    role: str
    basis: str
    config_line: str | None


def build_reports(
    plan: IdentityPlan,
    rfid_ok: Mapping[str, bool],
    hello_ok: Mapping[str, bool],
) -> tuple[PortReport, ...]:
    """把识别计划与实测结果整理成逐口结论。"""
    ordered = list(dict.fromkeys([
        *rfid_ok,
        *plan.hello_targets,
        *(() if plan.rfid_port is None else (plan.rfid_port,)),
        *plan.undecided,
    ]))

    reports: list[PortReport] = []
    for port in ordered:
        if port == plan.rfid_port:
            reports.append(PortReport(
                port=port,
                role="RFID 读卡器",
                basis="回应了读卡器私有协议（0x7F 读卡命令回 0x90）",
                config_line=f'RFID_PORT = "{port}"',
            ))
        elif port in plan.hello_targets and hello_ok.get(port):
            reports.append(PortReport(
                port=port,
                role="STM32",
                basis="读卡器身份已排除，且 HELLO 握手成功（v9 / protocol 2.3 / 0xEDB）",
                config_line=f'MCU_PORT = "{port}"',
            ))
        elif port in plan.hello_targets:
            reports.append(PortReport(
                port=port,
                role="未知",
                basis="读卡器身份已排除，但未回应 HELLO（检查供电与接线）",
                config_line=None,
            ))
        else:
            reports.append(PortReport(
                port=port,
                role="未知",
                basis="未确认读卡器身份，无法排除它是不是读卡器，因此未发 HELLO",
                config_line=None,
            ))
    return tuple(reports)


# ---------------------------------------------------------------------------
# 硬件探测（薄封装，复用已测代码；不在这里重造枚举或协议逻辑）
# ---------------------------------------------------------------------------


def discover_ports() -> list[str]:
    """列出当前存在的 USB 串口，按名称排序。"""
    return sorted(glob.glob("/dev/ttyUSB*"))


def probe_rfid(port: str) -> tuple[bool, str]:
    """发读卡器自己的私有协议读卡命令，问它是不是读卡器。"""
    reader = rfid_mod.RfidReader(port=port)
    try:
        return reader.ping(), ""
    except Exception as exc:  # noqa: BLE001 - 单口异常不中断整体识别
        return False, str(exc)
    finally:
        try:
            reader.close()
        except Exception:  # noqa: BLE001
            pass


def probe_hello(port: str) -> tuple[bool, str]:
    """HELLO 握手确认 STM32。只允许发往 plan 批准的口。"""
    with runlog.session("serial_port_identity", periodic_status=False):
        link = mcu_mod.McuLink(port=port)
        try:
            link.open()
            link.hello()
            return True, f"build={link.build_id} capabilities=0x{link.capabilities:08X}"
        except Exception as exc:  # noqa: BLE001 - 单口异常不中断整体识别
            return False, str(exc)
        finally:
            try:
                link.close()  # 故意不走 shutdown()：识别阶段不发 STOP
            except Exception:  # noqa: BLE001
                pass


def list_cameras() -> list[dict[str, str]]:
    """只读列出摄像头设备节点，不打开设备。"""
    cameras: list[dict[str, str]] = []
    for device in sorted(glob.glob("/dev/video*")):
        record = {"device": device, "by_id": ""}
        by_id_root = Path("/dev/v4l/by-id")
        if by_id_root.is_dir():
            for link in sorted(by_id_root.iterdir()):
                try:
                    if os.path.realpath(link) == os.path.realpath(device):
                        record["by_id"] = str(link)
                except OSError:
                    continue
        cameras.append(record)
    return cameras


def list_audio_cards() -> tuple[list[str], str]:
    """只读列出声卡，不播放任何声音。"""
    if shutil.which("aplay") is None:
        return [], "未找到 aplay"
    try:
        completed = subprocess.run(
            ["aplay", "-l"], capture_output=True, text=True, timeout=10.0, check=False,
        )
    except Exception as exc:  # noqa: BLE001
        return [], f"aplay -l 失败: {exc}"
    lines = [line.rstrip() for line in completed.stdout.splitlines() if line.strip()]
    return lines, ""


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------

_RULE = "=" * 68


def format_report(reports: Sequence[PortReport], *, readonly: bool) -> str:
    lines = [_RULE, "端口身份识别结论", _RULE]
    if readonly:
        lines.append("（--readonly：未做任何写入，因此没有识别设备身份）")
    lines.append("")
    for report in reports:
        lines.append(f"{report.port}  ->  {report.role}")
        lines.append(f"    依据: {report.basis}")
        if report.config_line:
            lines.append(f"    改这一行: {report.config_line}")
        lines.append("")

    suggested = [report.config_line for report in reports if report.config_line]
    if suggested:
        lines.append("--- config.py 需要改的行 ---")
        lines.extend(suggested)
    else:
        lines.append("--- 无法给出 config.py 修改建议 ---")
        lines.append("请接好两个 CH340 后重跑；或在断电状态下拔掉一根线对比端口列表。")
    lines.append("")
    return "\n".join(lines)


def format_evidence(ports_records: Sequence[Mapping[str, object]]) -> str:
    lines = ["--- 设备证据（只读）---"]
    if not ports_records:
        lines.append("(无)")
    for record in ports_records:
        lines.append(
            f"{record.get('device')}  exists={record.get('exists')}  "
            f"vid:pid={record.get('vid')}:{record.get('pid')}  "
            f"location={record.get('location')}  serial={record.get('serial_number')}"
        )
        by_id = record.get("by_id") or []
        if by_id:
            lines.append(f"    by-id: {', '.join(str(item) for item in by_id)}")
        openers = record.get("openers") or []
        if openers:
            lines.append(f"    被占用: {openers}")
    return "\n".join(lines)


def build_json(
    *,
    ports: list[str],
    readonly: bool,
    reports: Sequence[PortReport],
    evidence: Mapping[str, object],
    cameras: Sequence[Mapping[str, str]],
    audio_cards: Sequence[str],
) -> str:
    suggested = {report.port: report.config_line for report in reports if report.config_line}
    payload = {
        "tool": "test_serial_ports",
        "readonly": readonly,
        "ports": ports,
        "reports": [
            {
                "port": report.port,
                "role": report.role,
                "basis": report.basis,
                "config_line": report.config_line,
            }
            for report in reports
        ],
        "config_current": {
            "MCU_PORT": config.MCU_PORT,
            "RFID_PORT": config.RFID_PORT,
        },
        "config_suggested": suggested,
        "evidence": evidence,
        "cameras": list(cameras),
        "audio_cards": list(audio_cards),
    }
    return json.dumps(payload, ensure_ascii=False, indent=2, default=str)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="识别哪个 ttyUSB 是 STM32、哪个是 RFID 读卡器，并打印需要修改的 config 行。",
    )
    parser.add_argument(
        "--readonly",
        action="store_true",
        help="零写入：只列端口证据与摄像头/声卡清单，不调用读卡器协议也不发 HELLO。",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    print(_RULE)
    print("端口身份识别")
    print(_RULE)
    print("规则：先用读卡器自己的协议确认读卡器，再对唯一剩下的候选口发 HELLO。")
    print("      不向身份未排除的口写 MCU 帧（验收条款 A01）。")
    print()

    if os.name != "posix":
        print("本脚本需要在 RDK（Linux）上运行；当前系统不是 posix。")
        return 2

    ports = discover_ports()
    if not ports:
        print("没有发现任何 /dev/ttyUSB* 设备。请检查两个 CH340 是否插好、供电是否正常。")
        return 1
    print(f"发现 {len(ports)} 个 USB 串口: {' '.join(ports)}")
    print()

    rfid_ok: dict[str, bool] = {port: False for port in ports}
    rfid_errors: dict[str, str] = {}
    if not args.readonly:
        print("--- 第 1 步：用读卡器私有协议逐口确认读卡器 ---")
        for port in ports:
            ok, error = probe_rfid(port)
            rfid_ok[port] = ok
            if error:
                rfid_errors[port] = error
            print(f"  {port}: {'是读卡器' if ok else '无读卡器响应'}"
                  f"{f'（{error}）' if error else ''}")
        print()

    plan = plan_identification(ports, rfid_ok)
    hello_ok: dict[str, bool] = {}
    hello_detail: dict[str, str] = {}
    if plan.hello_targets:
        print("--- 第 2 步：对已排除读卡器身份的唯一候选口发 HELLO ---")
        for port in plan.hello_targets:
            ok, detail = probe_hello(port)
            hello_ok[port] = ok
            hello_detail[port] = detail
            print(f"  {port}: {'HELLO 成功' if ok else 'HELLO 失败'}（{detail}）")
        print()
    elif not args.readonly:
        print("--- 第 2 步：跳过 HELLO ---")
        if plan.rfid_port is None:
            print("  没有口能确认读卡器身份，无法排除读卡器，故不发 HELLO。")
            print("  请确认读卡器已接好并供电，然后重跑。")
        elif plan.undecided:
            print(f"  已确认读卡器在 {plan.rfid_port}，但剩余候选口不止一个，无法区分："
                  f"{' '.join(plan.undecided)}")
        else:
            print(f"  已确认读卡器在 {plan.rfid_port}，但没有其他候选口。")
            print("  请检查 STM32 是否已接好并上电，然后重跑。")
        print()

    reports = build_reports(plan, rfid_ok, hello_ok)

    evidence: dict[str, object] = {}
    try:
        evidence = selfcheck._serial_diagnostics(ports)  # 复用既有只读枚举，避免重造
    except Exception as exc:  # noqa: BLE001 - 证据采集失败不影响结论
        evidence = {"error": str(exc)}
    ports_records = evidence.get("ports") if isinstance(evidence, dict) else None
    if not isinstance(ports_records, list):
        ports_records = []

    cameras = list_cameras()
    audio_cards, audio_error = list_audio_cards()

    print(format_report(reports, readonly=args.readonly))
    print(format_evidence(ports_records))
    print()
    print("--- 摄像头（只读清单，未打开设备）---")
    if cameras:
        for camera in cameras:
            by_id = f"  by-id={camera['by_id']}" if camera["by_id"] else ""
            print(f"{camera['device']}{by_id}")
    else:
        print("(未发现 /dev/video*)")
    print()
    print("--- 声卡（只读清单，未播放声音）---")
    if audio_cards:
        for line in audio_cards:
            print(line)
    else:
        print(f"(无){f' {audio_error}' if audio_error else ''}")
    print()
    print("--- JSON（整段复制即可）---")
    print(build_json(
        ports=ports,
        readonly=args.readonly,
        reports=reports,
        evidence=evidence,
        cameras=cameras,
        audio_cards=audio_cards,
    ))
    print()

    if args.readonly:
        return 0
    decided = {report.port for report in reports if report.role != "未知"}
    return 0 if len(decided) == len(ports) else 1


if __name__ == "__main__":
    sys.exit(main())
