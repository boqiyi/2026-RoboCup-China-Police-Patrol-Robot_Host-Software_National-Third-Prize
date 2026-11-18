"""顶层入口：上位机自检、导航、任务账本与安全重规划。"""

from __future__ import annotations

import argparse
import math
import sys
import time

import config
from drivers.protocol_v2 import Capability, MotionActionCode, StopReason

import audio as audio_mod
import camera as camera_mod
import clues as clues_mod
import coverage
import culvert as culvert_mod
import detector as detector_mod
import gimbal as gimbal_mod
import gimbal_test as gimbal_test_mod
import mcu as mcu_mod
import mission_ledger as mission_mod
import mission_planner
import nav
import obstacle
import rfid as rfid_mod
import selfcheck
import runlog
import telemetry_display


def build_components() -> dict:
    m = mcu_mod.McuLink()
    components = {"mcu": m, "_init_errors": {}}
    factories = {"gimbal": lambda: gimbal_mod.Gimbal(m), "camera": camera_mod.Camera,
                 "detector": detector_mod.Detector, "audio": audio_mod.Audio,
                 "clues": clues_mod.Clues, "rfid": rfid_mod.RfidReader}
    for key, factory in factories.items():
        try:
            components[key] = factory()
        except Exception as exc:
            components[key] = None
            components["_init_errors"][key] = str(exc)
            runlog.exception("component_init_failed", "组件初始化失败", exc, component=key)
    return components


def build_passive_components() -> dict:
    """被动诊断/自检只创建 STM32 串口对象，避免初始化任何可动作外设。"""
    return {"mcu": mcu_mod.McuLink()}


def _safe_shutdown(c: dict) -> None:
    """异常/Ctrl-C/正常退出统一清理：先停车关灯，再关闭资源。"""
    runlog.status(phase="shutdown")
    mcu = c.get("mcu")
    if mcu is not None:
        try:
            mcu.send_stop(StopReason.CLOSE)
        except Exception as exc:
            runlog.exception("stop_failed", "退出时停车命令失败", exc)
        try:
            has_capability = getattr(mcu, "has_negotiated_capability", None)
            if callable(has_capability) and has_capability(Capability.LIGHT_CONTROL):
                sequence = mcu.send_light(0)
                confirm = getattr(mcu, "wait_for_light_applied", None)
                if callable(confirm) and sequence is not None:
                    confirm(sequence, 0)
        except Exception as exc:
            runlog.exception("light_shutdown_failed", "退出时关灯未确认", exc)
        try:
            mcu.shutdown()
        except Exception as exc:
            runlog.exception("mcu_close_failed", "关闭STM32连接失败", exc)
    rfid = c.get("rfid")
    if rfid is not None:
        try:
            rfid.stop_polling()
        except Exception as exc:
            runlog.exception("rfid_stop_failed", "读卡线程停止失败", exc)
    for key in ("camera", "detector", "rfid", "audio"):
        obj = c.get(key)
        if obj is not None:
            try:
                obj.close()
            except Exception as exc:
                runlog.exception("component_close_failed", "资源关闭失败", exc, component=key)


def cmd_selfcheck(*, passive: bool = False) -> int:
    c = {}
    try:
        c = build_passive_components() if passive else build_components()
        results = selfcheck.run_selfcheck(components=c, passive=passive)
        selfcheck.print_results(results)
        log_path = selfcheck.write_log(results)
        prefix = "被动" if passive else "完整"
        print(f"\n{prefix}自检日志已写入: {log_path}")
        critical_fail = [result for result in results if result.critical and not result.ok]
        if critical_fail:
            print("\n关键项未通过；车辆不会启动。保持静止后可运行: python3 main.py diagnose --duration 20")
            return 1
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"\n组件初始化/自检失败: {exc}")
        return 2
    finally:
        if passive:
            mcu = c.get("mcu")
            if mcu is not None:
                mcu.close()
        else:
            _safe_shutdown(c)


def cmd_diagnose(duration_s: float) -> int:
    """无动作采集 STM32 证据；唯一串口写入是 HELLO_REQ。"""
    mcu = mcu_mod.McuLink()
    try:
        print(f"开始 {duration_s:g} 秒被动诊断：请保持车辆静止；仅发送 HELLO_REQ 并读取遥测。")
        report = selfcheck.run_passive_diagnose(mcu, duration_s=duration_s)
        text_path, json_path = selfcheck.write_diagnostic_report(report)
        selfcheck.print_diagnostic_summary(report)
        print(f"\n诊断文本已写入: {text_path}")
        print(f"诊断 JSON 已写入: {json_path}")
        mcu_report = report.get("mcu", {})
        hello_ok = bool(mcu_report.get("hello_ok")) if isinstance(mcu_report, dict) else False
        return 0 if hello_ok else 1
    except Exception as exc:  # noqa: BLE001
        print(f"\n被动诊断失败: {exc}")
        return 2
    finally:
        mcu.close()


def cmd_gimbal_test(*, port: str | None = None) -> int:
    """只构造 MCU/云台，交互标定上下水平位置，不启动导航。"""
    c = {}
    try:
        c["mcu"] = mcu_mod.McuLink(port=port)
        c["gimbal"] = gimbal_mod.Gimbal(c["mcu"])
        gimbal_test_mod.run(c["mcu"], c["gimbal"])
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，停止云台测试并关闭连接。")
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"\n云台测试失败: {exc}")
        runlog.exception("gimbal_test.failed", "云台测试初始化或执行失败", exc)
        return 2
    finally:
        _safe_shutdown(c)


def cmd_start() -> int:
    # Keep the diagnostic/legacy helpers independent of the v12 competition
    # owner. Import lazily so merely importing main never constructs a device.
    from competition_runtime import run_competition
    return run_competition(profile="combined", stage3_enabled=True)


def _yaw_delta_deg(current_yaw_deg: float, expected_yaw_deg: float) -> float:
    """返回两个航向的最小夹角，处理 -180/180 度跨界。"""
    return abs(((current_yaw_deg - expected_yaw_deg + 180.0) % 360.0) - 180.0)


def _emit_telemetry(c: dict, *, force: bool = False, **context) -> None:
    display = c.get("_telemetry_display")
    if display is None:
        return
    fields = c.setdefault("_telemetry_context", {})
    fields.update(context)
    try:
        block = display.poll(force=force, **fields)
        if block is not None:
            runlog.emit("telemetry_group", block)
    except Exception as exc:
        runlog.exception("telemetry_display_failed", "遥测显示失败，继续原控制流程", exc)


def _notify_telemetry(on_poll, **context) -> None:
    if on_poll is not None:
        try:
            on_poll(**context)
        except Exception as exc:
            runlog.exception("telemetry_display_failed", "遥测显示回调失败，继续原控制流程", exc)


def _motion_is_stable(
    mcu,
    deadline: float,
    *,
    expected_yaw_deg: float | None = None,
    require_ranges: bool = True,
    require_upper: bool = False,
    on_poll=None,
) -> bool:
    """用连续新鲜编码器里程确认底盘已停稳，必要时同时核验航向。"""
    previous = mcu.odom_mm
    if previous is None:
        return False
    token_missing = object()
    sample_token = getattr(mcu, "state_sample_token", token_missing)
    requires_fresh_state = sample_token is not token_missing
    previous_token = sample_token if requires_fresh_state else None
    if requires_fresh_state and previous_token is None:
        return False
    end = min(deadline, time.monotonic() + getattr(config, "STOP_STABLE_TIMEOUT_S", 0.30))
    required_samples = getattr(config, "STOP_STABLE_SAMPLES", 3)
    max_delta_mm = getattr(config, "STOP_STABLE_DELTA_MM", 2.0)
    stable_samples = 0
    movement = [0.0, 0.0]
    if not _zero_motion(mcu):
        return False
    while time.monotonic() < end:
        try:
            mcu.pump()
        except mcu_mod.McuError:
            return False
        if _telemetry_fault(
            mcu,
            require_ranges=require_ranges,
            require_upper=require_upper,
        ) is not None:
            return False
        current = mcu.odom_mm
        if current is None:
            return False
        if expected_yaw_deg is not None:
            yaw = mcu.yaw_deg
            if yaw is None or _yaw_delta_deg(yaw, expected_yaw_deg) > config.TURN_YAW_TOLERANCE_DEG:
                runlog.emit("stop_heading_unconfirmed", "停车后的航向不满足目标，禁止继续行驶",
                            expected_yaw_deg=expected_yaw_deg, actual_yaw_deg=yaw,
                            tolerance_deg=config.TURN_YAW_TOLERANCE_DEG)
                return False
        _notify_telemetry(on_poll)
        current_token = getattr(mcu, "state_sample_token", None) if requires_fresh_state else None
        if requires_fresh_state:
            if current_token is None:
                return False
            if current_token == previous_token:
                # pump 未收到新 STATE，不能用同一份里程重复证明“已停稳”。
                time.sleep(0.02)
                continue
        if all(math.isfinite(v) for v in (*previous, *current)):
            movement = [movement[i] + abs(current[i] - previous[i]) for i in (0, 1)]
        else:
            return False
        if all(delta <= max_delta_mm for delta in movement):
            stable_samples += 1
            if stable_samples >= required_samples:
                return True
        else:
            stable_samples = 0
            movement = [0.0, 0.0]
        previous = current
        previous_token = current_token
        time.sleep(0.02)
    return False


def _confirm_motion_applied(mcu, sequence: int, deadline: float) -> None:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise mcu_mod.McuError("比赛截止，无剩余运动确认时间")
    confirm = mcu.wait_for_control_applied
    if gimbal_mod._accepts_keyword(confirm, "timeout_s"):
        confirm(sequence, timeout_s=min(config.MCU_COMMAND_CONFIRM_TIMEOUT_S, remaining))
    else:
        confirm(sequence)


def _execute_turn(
    turner: nav.TurnExecutor, turn: str, deadline: float, ramp: nav.VelocityRamp,
    *, on_poll=None,
) -> bool:
    """执行原地转弯；on_poll 接收仅用于显示的上下文关键字。"""
    expected_yaw = None

    def failed():
        _zero_motion(turner.mcu)
        ramp.reset()
        target = "" if expected_yaw is None else f"，目标 {expected_yaw:.1f}°"
        _notify_telemetry(on_poll, phase="转弯失败", command=(0, 0),
                          confirmation="已尝试停车", turn_result=f"{turn_name}失败{target}")
        return False

    runlog.status(phase="turn", turn=turn, deadline_monotonic=deadline)
    angle = coverage.turn_angle_deg(turn)
    turn_name = ({coverage.TURN_LEFT: "左", coverage.TURN_RIGHT: "右",
                  coverage.TURN_UTURN: "左掉头"}.get(turn, str(turn)) + f"{abs(angle):.0f}°")
    if angle == 0.0:
        return True

    def observe_stop():
        _notify_telemetry(on_poll, phase="转弯停车确认", command=(0, 0),
                          confirmation="双轮停稳核验", turn_result="")

    def observe_complete(ok):
        _notify_telemetry(on_poll, phase="转弯完成" if ok else "转弯停稳未确认",
                          reference_yaw_deg=expected_yaw,
                          command=(0, 0), confirmation="停稳" if ok else "停稳未确认",
                          turn_result=f"{turn_name}{'完成' if ok else '停稳未确认'}，目标 {expected_yaw:.1f}°")
        return ok
    send_action = getattr(turner.mcu, "send_motion_action", None)
    if callable(send_action):
        action_code = {
            coverage.TURN_LEFT: MotionActionCode.TURN_LEFT,
            coverage.TURN_RIGHT: MotionActionCode.TURN_RIGHT,
            coverage.TURN_UTURN: MotionActionCode.UTURN_LEFT,
        }[turn]
        turn_end = min(deadline, time.monotonic() + config.TURN_TIMEOUT_S)
        try:
            if not _zero_motion(turner.mcu):
                return failed()
            if not _motion_is_stable(turner.mcu, turn_end, require_ranges=False,
                                     require_upper=False, on_poll=observe_stop):
                return failed()
            yaw = turner.mcu.yaw_deg
            if yaw is None or time.monotonic() >= turn_end:
                return failed()
            expected_yaw = nav._unwrap(yaw + angle)
            # Firmware holds a completed action against accidental replay. BRAKE
            # clears that latch before another identical turn is requested.
            brake_sequence = send_action(MotionActionCode.BRAKE, require_upper=False)
            turner.mcu.wait_for_motion_action_applied(brake_sequence)
            if not _motion_is_stable(turner.mcu, turn_end, require_ranges=False,
                                     require_upper=False, on_poll=observe_stop):
                return failed()
            sequence = send_action(action_code, config.TURN_ACTION_SPEED_MMPS,
                                   brake_after_turn=True, require_ranges=False, require_upper=False)

            def check_turn():
                if time.monotonic() >= turn_end:
                    raise mcu_mod.McuError("转向超过比赛或动作截止时间")
                fault = _telemetry_fault(turner.mcu, require_ranges=False, require_upper=False)
                if fault is not None:
                    raise mcu_mod.McuError(f"转向遥测故障：{fault}")
                _notify_telemetry(on_poll, phase=f"转弯 {angle:+.0f}°",
                                  reference_yaw_deg=expected_yaw, command=None,
                                  confirmation="等待固件完成", turn_result="")

            turner.mcu.wait_for_motion_action_complete(
                sequence, timeout_s=max(0.0, turn_end - time.monotonic()),
                require_ranges=False, require_upper=False, on_poll=check_turn)
            if not _zero_motion(turner.mcu):
                return failed()
            ramp.reset()
            turner._target_yaw_deg = expected_yaw
            return observe_complete(_motion_is_stable(
                turner.mcu, deadline, expected_yaw_deg=expected_yaw,
                require_ranges=False, require_upper=False,
                on_poll=lambda: _notify_telemetry(
                    on_poll, phase="转弯后停稳确认", reference_yaw_deg=expected_yaw,
                    command=(0, 0), confirmation="双轮停稳核验", turn_result="")))
        except (mcu_mod.McuError, TimeoutError, ValueError):
            return failed()
    try:
        stop_sequence = _send_motion(turner.mcu, 0, 0, require_upper=False)
        confirm = getattr(turner.mcu, "wait_for_control_applied", None)
        if callable(confirm) and stop_sequence is not None:
            _confirm_motion_applied(turner.mcu, stop_sequence, deadline)
    except mcu_mod.McuError:
        return failed()
    if not _motion_is_stable(turner.mcu, deadline, require_ranges=False,
                             require_upper=False, on_poll=observe_stop):
        return failed()
    ramp.reset()
    turner.begin(angle)
    while time.monotonic() < deadline:
        try:
            turner.mcu.pump()
            if time.monotonic() >= deadline:
                _zero_motion(turner.mcu)
                return failed()
            if _telemetry_fault(turner.mcu, require_ranges=False, require_upper=False) is not None:
                return failed()
            status = turner.tick()
            if status == "done":
                # TurnExecutor 已把 ramp 降到零，但仍需确认最终零速命令已被下位机
                # 应用，并用后续 STATE 排除停车后的回摆。
                stop_sequence = _send_motion(turner.mcu, 0, 0, require_upper=False)
                confirm = getattr(turner.mcu, "wait_for_control_applied", None)
                if callable(confirm) and stop_sequence is not None:
                    _confirm_motion_applied(turner.mcu, stop_sequence, deadline)
                ramp.reset()
                expected_yaw = turner.target_yaw_deg
                if expected_yaw is None:
                    return failed()
                return observe_complete(_motion_is_stable(
                    turner.mcu,
                    deadline,
                    expected_yaw_deg=expected_yaw,
                    require_ranges=False,
                    require_upper=False,
                    on_poll=lambda: _notify_telemetry(
                        on_poll, phase="转弯后停稳确认", reference_yaw_deg=expected_yaw,
                        command=(0, 0), confirmation="双轮停稳核验", turn_result=""),
                ))
            if status in ("timeout", "no_yaw", "misaligned"):
                return failed()
            _notify_telemetry(on_poll, phase=f"转弯 {angle:+.0f}°",
                              reference_yaw_deg=turner.target_yaw_deg, command=None,
                              confirmation="航向转弯进行中", turn_result="")
        except mcu_mod.McuError:
            return failed()
        time.sleep(0.02)
    return failed()


def _safe_audio(audio, path) -> bool:
    if audio is None or path is None:
        runlog.emit("audio_unavailable", "音频组件或素材不可用", path=str(path))
        return False
    try:
        queued = bool(audio.play(path))
        if not queued:
            runlog.emit("audio_enqueue_failed", "播报未能加入队列", path=str(path),
                        reason=getattr(audio, "last_error", None))
        return queued
    except Exception as exc:
        runlog.exception("audio_enqueue_failed", "播报入队异常", exc, path=str(path))
        return False


def _odom_distance_mm(
    entry: tuple[int, int] | None,
    current: tuple[int, int] | None,
) -> float | None:
    """按双轮里程差绝对值求走过距离，兼容正负编码器坐标或累计里程。"""
    if entry is None or current is None or not all(math.isfinite(v) for v in (*entry, *current)):
        return None
    return (
        (abs(current[0] - entry[0]) + abs(current[1] - entry[1]))
        * 0.5
        * config.BRAKE_ODOM_SCALE
    )


def _forward_distance_mm(
    entry: tuple[int, int] | None,
    current: tuple[int, int] | None,
) -> float | None:
    """Standalone tests count signed encoder advance, excluding pivot wheel travel."""
    if entry is None or current is None:
        return None
    return ((current[0] - entry[0]) + (current[1] - entry[1])) / 2.0


def _drain_rfid_events(
    c: dict,
    reader,
    ledger: mission_mod.MissionLedger | None,
    step: coverage.Step,
    distance_mm: float | None,
) -> None:
    """按实际巡逻块数据播报，并把读卡事件绑定到当前道路账本。"""
    if ledger is None:
        return
    consume = getattr(reader, "consume_event", None)
    if not callable(consume):
        return
    # The producer is independent; bound draining so a fast reader cannot starve motion.
    for _ in range(64):
        event = consume()
        if event is None:
            return
        record = ledger.record_rfid_event(event, step, distance_mm)
        runlog.emit("rfid_event", "读卡结果", uid=record.uid_hex, patrol_no=record.patrol_no,
                    error=record.error, road=step.road_id, distance_mm=distance_mm,
                    block_no=record.block_no, block_data_hex=record.block_data_hex,
                    reader_status=record.reader_status)
        if record.error is None and record.patrol_no is not None:
            _announce_patrol_confirmation(c, (record.patrol_no,))


def _grab_camera_frame(camera, frame_state: dict | None = None):
    """抓取新鲜相机帧；色块确认必须有递增序号和采集时间。"""
    grab_frame = getattr(camera, "grab_frame", None)
    if not callable(grab_frame):
        frame = camera.grab_bgr()
        if frame_state is not None:
            frame_state["is_new"] = False
            frame_state["metadata_error"] = "摄像头接口未提供帧序号和采集时间"
        return frame
    captured = grab_frame()
    if captured is None:
        return None
    fresh = getattr(captured, "is_fresh", None)
    if callable(fresh) and not fresh(config.CAMERA_FRAME_MAX_AGE_S):
        return None
    if frame_state is not None:
        frame_state["captured"] = captured
        sequence = getattr(captured, "sequence", None)
        captured_monotonic = getattr(captured, "captured_monotonic", None)
        metadata_valid = (
            isinstance(sequence, int)
            and not isinstance(sequence, bool)
            and isinstance(captured_monotonic, (int, float))
            and not isinstance(captured_monotonic, bool)
            and math.isfinite(captured_monotonic)
        )
        if not metadata_valid:
            frame_state["is_new"] = False
            frame_state["metadata_error"] = "摄像头帧缺少递增序号或采集时间"
            return getattr(captured, "bgr", captured)
        previous_sequence = frame_state.get("sequence")
        previous_captured_monotonic = frame_state.get("captured_monotonic")
        frame_state["is_new"] = (
            previous_sequence is None
            or (
                sequence > previous_sequence
                and captured_monotonic > previous_captured_monotonic
            )
        )
        frame_state["sequence"] = sequence
        frame_state["captured_monotonic"] = captured_monotonic
        frame_state.pop("metadata_error", None)
    return getattr(captured, "bgr", captured)


def _minimum_junction_distance(road_len_mm: float | None) -> float:
    """防止刚离开原路口时的侧向跳变被误判为下一节点。"""
    length = max(0.0, float(road_len_mm or 0.0))
    if length == 0.0:
        return 0.0
    candidate = max(config.JUNCTION_MIN_TRAVEL_MM, length * config.JUNCTION_MIN_TRAVEL_RATIO)
    return min(candidate, length * 0.5)


def _junction_matches(c, step, channels, distance_mm, road_len_mm):
    """侧开口是到达证据；地图和行程仅作一致性核验。"""
    if distance_mm is None or distance_mm < _minimum_junction_distance(road_len_mm):
        return False
    expected = _expected_junction_sides(c, step)
    if expected is not None and set(channels) != expected:
        return False
    # 名义尺寸仅用于拒绝明显错节点，仍须侧开口确认。
    if road_len_mm:
        margin = min(float(road_len_mm) * 0.5, max(150.0, float(road_len_mm) * 0.20))
        return abs(distance_mm - float(road_len_mm)) <= margin
    return False


def _expected_junction_sides(c, step):
    fm = c.get("_field_map")
    if fm is None:
        return None
    dr, dc = coverage.direction_between(fm, step.from_node, step.to_node)
    directions = {"left": (-dc, dr), "right": (dc, -dr)}
    outgoing = {coverage.direction_between(fm, step.to_node, neighbor)
                for _, neighbor in fm.adjacency[step.to_node] if neighbor != step.from_node}
    return {side for side, direction in directions.items() if direction in outgoing}


def _telemetry_fault(
    mcu,
    *,
    require_ranges: bool = True,
    require_upper: bool = False,
) -> str | None:
    """基础运动保留侧向保护；阶段三可显式要求上层健康。"""
    check = getattr(mcu, "telemetry_ok", None)
    if not callable(check):
        return None
    if gimbal_mod._accepts_keyword(check, "require_upper"):
        return check(require_ranges=require_ranges, require_upper=require_upper)
    if gimbal_mod._accepts_keyword(check, "require_ranges"):
        return check(require_ranges=require_ranges)
    return check()


def _update_upper_scan_health(
    c: dict,
    jump: nav.JumpDetector,
    selector: nav.CeilingSourceSelector,
    ledger: mission_mod.MissionLedger | None = None,
    step: coverage.Step | None = None,
) -> bool:
    """维护跨道路的上层可用性；恢复的新帧只建立顶棚基线。"""
    mcu = c["mcu"]
    healthy, reason = selfcheck._upper_tof_health(mcu, required=True)
    upper = getattr(mcu, "culvert_upper", None)
    sequence = getattr(upper, "upper_cycle_seq", None)
    available = healthy and sequence is not None
    previous = c.get("_upper_scan_available")
    if not available:
        c["_upper_scan_lost_seq"] = sequence
        if healthy:
            reason += "；缺少上层周期序号"
    elif previous is False and sequence in (
        c.get("_upper_scan_lost_seq"), c.get("_upper_scan_last_seq")
    ):
        available = False
        reason += "；等待失效后的新上层样本"
    if available:
        c["_upper_scan_last_seq"] = sequence
    c["_upper_scan_available"] = available
    if previous != available and (not available or previous is False):
        jump.reset_upper()
        selector.reset()
        now = time.monotonic()
        road_id = getattr(step, "road_id", None)
        if ledger is not None:
            ledger.record_upper_availability(available, reason, road_id, now)
        runlog.emit(
            "upper_scan_recovered" if available else "upper_scan_degraded",
            "上层已恢复，新样本重建涵洞基线" if available else "上层不可用，继续基础运行并暂停新涵洞触发",
            road=road_id, reason=reason, upper_cycle_seq=sequence,
            monotonic_s=now,
        )
    return available


def _send_motion(
    mcu,
    linear_mmps: float,
    angular_mradps: float,
    *,
    require_ranges: bool = True,
    require_upper: bool = False,
):
    """将阶段测距策略传给 McuLink，同时兼容旧测试替身。"""
    if linear_mmps != 0 or angular_mradps != 0:
        if angular_mradps and abs(angular_mradps) * config.FW_WHEEL_BASE_MM < 2000:
            angular_mradps = 0
        wheel_delta = angular_mradps * config.FW_WHEEL_BASE_MM / 2000.0
        wheels = (linear_mmps - wheel_delta, linear_mmps + wheel_delta)
        if any(abs(wheel) > config.FW_MAX_LINEAR_SPEED_MM_S or
               0 < abs(wheel) < config.FW_MIN_NONZERO_WHEEL_SPEED_MM_S for wheel in wheels):
            raise mcu_mod.McuError("目标轮速超出固件允许范围")
    send = mcu.send_motion
    if require_ranges and require_upper:
        return send(linear_mmps, angular_mradps)
    if gimbal_mod._accepts_keyword(send, "require_upper"):
        return send(
            linear_mmps,
            angular_mradps,
            require_ranges=require_ranges,
            require_upper=require_upper,
        )
    if gimbal_mod._accepts_keyword(send, "require_ranges"):
        return send(linear_mmps, angular_mradps, require_ranges=require_ranges)
    return send(linear_mmps, angular_mradps)


def _home_stop_is_stable(mcu, entry_yaw_deg: float, deadline: float, *, on_poll=None) -> bool:
    """确认 n01 终段已停稳且车头保持朝场外。"""
    try:
        stop_sequence = _send_motion(mcu, 0, 0, require_upper=False)
        confirm = getattr(mcu, "wait_for_control_applied", None)
        if not callable(confirm) or stop_sequence is None:
            return False
        _confirm_motion_applied(mcu, stop_sequence, deadline)
        if not _zero_motion(mcu):
            return False
    except mcu_mod.McuError:
        return False

    previous_odom = mcu.odom_mm
    if previous_odom is None:
        return False
    end = min(deadline, time.monotonic() + getattr(config, "STOP_STABLE_TIMEOUT_S", 0.30))
    samples: list[float] = []
    movement = [0.0, 0.0]
    token_missing = object()
    previous_token = getattr(mcu, "state_sample_token", token_missing)
    requires_fresh_state = previous_token is not token_missing
    if requires_fresh_state and previous_token is None:
        return False
    max_delta_mm = getattr(config, "STOP_STABLE_DELTA_MM", 2.0)
    required_samples = getattr(config, "STOP_STABLE_SAMPLES", 3)
    while time.monotonic() < end:
        try:
            mcu.pump()
        except mcu_mod.McuError:
            return False
        if _telemetry_fault(mcu, require_upper=False) is not None:
            return False
        _notify_telemetry(on_poll)
        if requires_fresh_state:
            current_token = getattr(mcu, "state_sample_token", None)
            if current_token is None:
                return False
            if current_token == previous_token:
                time.sleep(0.02)
                continue
        current_odom = mcu.odom_mm
        if current_odom is None or not all(math.isfinite(v) for v in (*previous_odom, *current_odom)):
            return False
        movement = [movement[i] + abs(current_odom[i] - previous_odom[i]) for i in (0, 1)]
        yaw = mcu.yaw_deg
        if yaw is None:
            return False
        if _yaw_delta_deg(yaw, entry_yaw_deg) > config.TURN_YAW_TOLERANCE_DEG:
            runlog.emit("home_heading_unconfirmed", "出发区终点航向不满足朝场外要求",
                        expected_yaw_deg=entry_yaw_deg, actual_yaw_deg=yaw,
                        tolerance_deg=config.TURN_YAW_TOLERANCE_DEG)
            return False
        if all(delta <= max_delta_mm for delta in movement):
            samples.append(yaw)
        else:
            samples.clear()
            movement = [0.0, 0.0]
        if len(samples) >= required_samples:
            reference = samples[0]
            if all(abs(((sample - reference + 180.0) % 360.0) - 180.0) <= config.CENTER_DEADBAND_DEG for sample in samples[1:]):
                return True
            samples = [yaw]
        previous_odom = current_odom
        if requires_fresh_state:
            previous_token = current_token
        time.sleep(0.02)
    return False


def _drive_until_junction(
    c: dict,
    follower: nav.CenterFollower,
    jump: nav.JumpDetector,
    culvert_flow: culvert_mod.CulvertFlow | None,
    rf: rfid_mod.RfidReader,
    step: coverage.Step,
    deadline: float,
    ramp: nav.VelocityRamp,
    road_len_mm: float | None,
    detect_obstacles: bool = True,
    require_camera: bool = True,
    terminal_home: bool = False,
    rollback: bool = False,
    *,
    ledger: mission_mod.MissionLedger | None = None,
    road_progress: mission_mod.RoadProgress | None = None,
    ceiling_selector: nav.CeilingSourceSelector | None = None,
    allow_scans: bool = True,
    use_upper_for_scans: bool = True,
    planner=None,
    action=None,
    budget=None,
    expected_yaw_deg: float | None = None,
    recenter: bool = False,
    observer=None,
) -> mission_mod.DriveResult:
    """居中行驶至侧向路口，末段回出发区则用里程和航向确认。

    上层健康只决定新涵洞触发，基础运动保留通信、里程、陀螺仪和侧向
    测距门禁。``use_upper_for_scans=False`` 完全忽略上层事件。
    """
    mcu = c["mcu"]
    cam = c.get("camera")
    ramp.reset()
    brake = nav.ApproachBrake(
        road_len_mm if road_len_mm is not None else 0.0,
        enabled=config.BRAKE_ENABLED,
        brake_short_roads=True,
    )
    odom0 = mcu.odom_mm
    use_odom = odom0 is not None and config.BRAKE_ODOM_SCALE > 0.0
    distance_estimate: float | None = 0.0
    v_sent_prev = 0.0
    last_command = (0, 0)
    last_tick = time.monotonic()
    entered_at = last_tick
    last_progress_at = last_tick
    last_progress_mm = 0.0
    scan_elapsed = 0.0
    return_requested = False
    frame_state = {}
    junction_since = None
    junction_pending = False
    junction_stop_confirmed = False
    junction_confirmed = set()
    junction_candidates = set()
    junction_trigger_mm = None
    side_armed = False
    recenter_timeout_logged = False
    sample_progress = 0.0
    previous_projected_odom = odom0
    entry_yaw = expected_yaw_deg if expected_yaw_deg is not None else mcu.yaw_deg
    side_observer = nav.SideObserver(c.get("_side_total_range"))
    def observe(event, **fields):
        if observer is not None:
            observer(event, **fields)

    def update_projected_progress():
        nonlocal sample_progress, previous_projected_odom
        yaw_error = (_yaw_delta_deg(mcu.yaw_deg, entry_yaw)
                     if mcu.yaw_deg is not None and entry_yaw is not None else None)
        if use_odom and mcu.odom_mm is not None and previous_projected_odom is not None:
            delta = _forward_distance_mm(previous_projected_odom, mcu.odom_mm)
            if delta is not None and yaw_error is not None:
                sample_progress += (delta * config.BRAKE_ODOM_SCALE
                                    * max(0.0, math.cos(math.radians(yaw_error))))
            previous_projected_odom = mcu.odom_mm
        return yaw_error

    def observe_sample(side_observation, candidates, confirmed, speed):
        if observer is None:
            return
        observe("sample", progress_mm=sample_progress,
                odom_distance_mm=_odom_distance_mm(odom0, mcu.odom_mm) if use_odom else distance_estimate,
                left_mm=mcu.left_mm, right_mm=mcu.right_mm,
                left_status=getattr(mcu.left_status, "name", None),
                right_status=getattr(mcu.right_status, "name", None),
                upper_left_mm=mcu.upper_left_mm, upper_right_mm=mcu.upper_right_mm,
                upper_left_status=getattr(mcu.upper_left_status, "name", None),
                upper_right_status=getattr(mcu.upper_right_status, "name", None),
                yaw_error_deg=update_projected_progress(), yaw_deg=mcu.yaw_deg, odom_mm=mcu.odom_mm,
                directional_seq=mcu.directional.directional_cycle_seq, monotonic_s=time.monotonic(),
                sample_age_ms=getattr(mcu, "directional_age_ms", None), speed_mmps=speed,
                side_candidates=list(candidates), dual_wall=side_observation.dual_wall,
                side_reason=side_observation.reason, side_confirmed=list(confirmed),
                recenter_elapsed_s=follower.recenter_elapsed_s,
                recenter_complete=follower.recenter_complete)
    road_timeout = (max(0.0, float(road_len_mm or 0.0)) / max(1, config.BRAKE_MIN_MMPS)
                    * config.ROAD_TIMEOUT_FACTOR + config.ROAD_TIMEOUT_EXTRA_S)
    light_on = False
    last_directional_seq = None
    obstacle_candidate = obstacle.ObstacleCandidate()
    pending_ceiling = False
    pending_ceiling_opened = False
    last_obstacle = None
    # Use an explicit first-sample marker instead of subtracting an interval:
    # tests and callers may supply a different monotonic-clock origin.
    last_camera_check = None
    last_obstacle_check = None
    last_motion_sent_at = None
    home_entry_yaw = entry_yaw
    follower.begin(entry_yaw, recenter=recenter)
    jump.reset_side()
    ceiling_selector = ceiling_selector or nav.CeilingSourceSelector()

    def show_telemetry(*, phase=None, command=None, confirmation="", turn_result=None):
        distance = _odom_distance_mm(odom0, mcu.odom_mm) if use_odom else distance_estimate
        result_context = {} if turn_result is None else {"turn_result": turn_result}
        _emit_telemetry(
            c, phase=phase or ("路口核验" if junction_pending else "回退" if rollback else "道路直行"),
            segment_distance_mm=distance, reference_yaw_deg=entry_yaw,
            command=last_command if command is None else command,
            confirmation=confirmation, **result_context,
        )

    def show_stop():
        show_telemetry(phase="停车确认", command=(0, 0), confirmation="双轮停稳核验")

    def finish(
        reason: str,
        *,
        side_junction: bool = False,
        obstacle_result=None,
    ) -> mission_mod.DriveResult:
        final_yaw_error = update_projected_progress()
        final_directional = getattr(mcu, "directional", None)
        final_seq = final_directional.directional_cycle_seq if final_directional is not None else None
        if observer is not None and final_seq is not None and final_seq != last_directional_seq:
            final_sides = side_observer.update(final_seq, mcu.left_mm, mcu.left_status,
                                               mcu.right_mm, mcu.right_status,
                                               yaw_error_deg=final_yaw_error)
            observe_sample(final_sides, junction_candidates, junction_confirmed, 0.0)
        current_odom = mcu.odom_mm if use_odom else None
        odom_distance = _odom_distance_mm(odom0, current_odom)
        if odom_distance is not None:
            result_distance = max(distance_estimate or 0.0, odom_distance)
        else:
            result_distance = distance_estimate
        result = mission_mod.DriveResult(
            reason=reason,
            distance_mm=result_distance,
            odom_distance_mm=odom_distance,
            side_junction=side_junction,
            yaw_deg=mcu.yaw_deg,
            odom_reliable=odom_distance is not None,
            obstacle_result=obstacle_result,
            return_requested=return_requested,
        )
        show_telemetry(phase="道路结束", command=(0, 0), confirmation=reason)
        if road_progress is not None:
            road_progress.observe(result)
        c["_last_scan_elapsed_s"] = scan_elapsed
        runlog.emit("road_result", "道路行驶结束", road=getattr(step, "road_id", None),
                    entry=getattr(step, "from_node", None),
                    destination=getattr(step, "to_node", None), reason=reason, distance_mm=odom_distance,
                    side_junction=side_junction, rollback=rollback)
        observe("segment_end", result=result, progress_mm=sample_progress,
                recenter_complete=follower.recenter_complete,
                recenter_elapsed_s=follower.recenter_elapsed_s, recenter_timed_out=follower.timed_out)
        return result

    observe("segment_start", road_id=step.road_id, from_node=step.from_node,
            to_node=step.to_node, odom_mm=odom0, yaw_deg=entry_yaw)
    if terminal_home and (not use_odom or home_entry_yaw is None):
        _zero_motion(mcu)
        return finish("home_position_fault")
    if require_camera and cam is None:
        if ledger is not None:
            ledger.record_visual_unavailable(step, "摄像头组件不可用")
        _zero_motion(mcu)
        return finish("vision_fault")

    while True:
        if time.monotonic() >= deadline:
            _zero_motion(mcu)
            _light_off(mcu, light_on)
            return finish("deadline")
        try:
            mcu.pump()
        except mcu_mod.McuError:
            _zero_motion(mcu)
            _light_off(mcu, light_on)
            return finish("fault")
        poll_audio = getattr(c.get("audio"), "poll", None)
        if callable(poll_audio):
            try:
                poll_audio()
            except Exception as exc:
                runlog.exception("audio_poll_failed", "播报队列更新失败", exc)
        fault = _telemetry_fault(mcu, require_upper=False)
        if fault is not None:
            runlog.emit("telemetry_fault", "行驶遥测不可用", reason=fault, road=getattr(step, "road_id", None))
            _zero_motion(mcu)
            _light_off(mcu, light_on)
            return finish("fault")

        left, right = mcu.left_mm, mcu.right_mm
        ls, rs = mcu.left_status, mcu.right_status
        upper_available = use_upper_for_scans and _update_upper_scan_health(
            c, jump, ceiling_selector, ledger, step)
        if upper_available:
            ul, ur = mcu.upper_left_mm, mcu.upper_right_mm
            upper_statuses = list(mcu.upper_effective_statuses)
            for index, age in enumerate(mcu.upper_age_ms):
                if (not isinstance(age, int) or isinstance(age, bool)
                        or not 0 <= age <= config.CULVERT_UPPER_LASER_MAX_AGE_MS):
                    upper_statuses[index] = None
            uls, urs = upper_statuses
            selected_upper = "upper_right" if nav.classify(ur, urs) != "fault" else "upper_left"
            if ceiling_selector.source != selected_upper:
                # 切源也重建去抖基线，丢弃备用通道在切源前累计的候选。
                jump.reset_upper()
        else:
            ul = ur = uls = urs = None
            pending_ceiling = pending_ceiling_opened = False
        dir_seq = mcu.directional.directional_cycle_seq if mcu.directional is not None else None
        up_seq = mcu.culvert_upper.upper_cycle_seq if upper_available else None
        directional_new = dir_seq is not None and dir_seq != last_directional_seq
        if directional_new:
            last_directional_seq = dir_seq

        yaw_error = update_projected_progress()
        side_observation = side_observer.update(dir_seq, left, ls, right, rs,
                                               yaw_error_deg=yaw_error)
        if side_observation.dual_wall:
            side_armed = True
            c["_side_total_range"] = side_observer.total_range
        events = jump.update(dir_seq, up_seq, left, ls, right, rs, ul, uls, ur, urs,
                             side_observation=side_observation)
        if upper_available:
            upper_left = nav.classify(ul, uls)
            upper_right = nav.classify(ur, urs)
            upper_right_healthy = upper_right != "fault"
            upper_primary = upper_right if upper_right_healthy else upper_left
            # CH5 的 NO_TARGET/OUT_OF_RANGE 是健康观测，只有故障才切到 CH4。
        else:
            events = [event for event in events if event.channel in {"left", "right"}]
            upper_right_healthy = False
            upper_primary = "open"

        now = time.monotonic()
        actual_distance = _odom_distance_mm(odom0, mcu.odom_mm) if use_odom else distance_estimate
        runlog.status(road=getattr(step, "road_id", None), entry=getattr(step, "from_node", None),
                      destination=getattr(step, "to_node", None),
                      phase="rollback" if rollback else (action.kind if action else "drive"),
                      distance_mm=actual_distance, yaw=mcu.yaw_deg,
                      remaining_s=max(0.0, deadline - now),
                      deadline_monotonic=deadline,
                      audio_remaining_s=_audio_remaining(c),
                      return_mode=return_requested or bool(planner and planner.return_mode))
        if actual_distance is not None and actual_distance >= last_progress_mm + config.ROAD_PROGRESS_MIN_MM:
            last_progress_at, last_progress_mm = now, actual_distance
        if not junction_pending and now - last_progress_at > config.ROAD_NO_PROGRESS_TIMEOUT_S:
            _zero_motion(mcu)
            _light_off(mcu, light_on)
            return finish("no_progress")
        if road_len_mm and (now - entered_at - scan_elapsed > road_timeout or
                actual_distance is not None and actual_distance > max(
                    road_len_mm * config.ROAD_OVERRUN_RATIO, road_len_mm + config.ROAD_OVERRUN_MIN_MM)):
            _zero_motion(mcu)
            _light_off(mcu, light_on)
            return finish("road_overrun")
        if (not junction_pending and budget is not None and action is not None and not rollback
                and not planner.return_mode and not return_requested):
            needed = budget.required_from_road(planner, action, actual_distance or 0.0,
                                               audio_remaining_s=_audio_remaining(c))
            if budget.remaining_s <= needed:
                retreat = budget.required_to_retreat(planner, action, actual_distance or 0.0,
                                                     audio_remaining_s=_audio_remaining(c))
                if step.road_id in planner.completed_roads and needed <= retreat:
                    return_requested = True
                    allow_scans = False
                    runlog.emit("return_budget", "已走通道路继续到出口返航更快，停止新增任务",
                                decision="continue_to_exit", continue_s=needed, retreat_s=retreat,
                                remaining_s=budget.remaining_s)
                else:
                    _zero_motion(mcu)
                    _light_off(mcu, light_on)
                    runlog.emit("return_budget", "停止新增任务，掉头返回已确认入口",
                                decision="retreat_to_entry", continue_s=needed, retreat_s=retreat,
                                remaining_s=budget.remaining_s)
                    return finish("budget_return")
        frame = None
        frame_is_new = False
        obstacle_due = detect_obstacles and (
            obstacle_candidate.active
            or last_obstacle_check is None
            or now - last_obstacle_check >= config.OBSTACLE_CHECK_INTERVAL_S
        )
        should_grab = require_camera and (
            obstacle_due
            or last_camera_check is None
            or now - last_camera_check >= config.CAMERA_HEALTH_CHECK_INTERVAL_S
        )
        if should_grab:
            last_camera_check = now
            if obstacle_due:
                last_obstacle_check = now
            try:
                frame = _grab_camera_frame(cam, frame_state)
            except Exception as exc:
                frame = None
                runlog.exception("camera_failed", "摄像头取帧失败", exc)
            if frame is None:
                if ledger is not None:
                    ledger.record_visual_unavailable(step, "摄像头帧不可用或过期")
                _zero_motion(mcu)
                ramp.reset()
                _light_off(mcu, light_on)
                return finish("vision_fault")
            if ledger is not None:
                ledger.record_visual_available(step)
            frame_is_new = bool(frame_state.get("is_new", True))
            if detect_obstacles and frame_state.get("metadata_error"):
                reason = str(frame_state["metadata_error"])
                if ledger is not None:
                    ledger.record_visual_unavailable(step, reason)
                _zero_motion(mcu)
                ramp.reset()
                _light_off(mcu, light_on)
                return finish("vision_fault", obstacle_result=obstacle.ObstacleResult.unavailable(reason))

        obstacle_state = "idle"
        if obstacle_due and frame_is_new:
            current_obstacle = obstacle.detect_colored_obstacle(frame)
            obstacle_state = obstacle_candidate.update(
                current_obstacle, frame_state.get("sequence"), time.monotonic())
            if obstacle_state == "fault":
                _zero_motion(mcu)
                ramp.reset()
                _light_off(mcu, light_on)
                return finish("vision_fault", obstacle_result=current_obstacle)
            last_obstacle = obstacle_candidate.last
            if obstacle_state == "first":
                runlog.emit("obstacle_suspected", "色块障碍候选限速核验", road=step.road_id,
                            color=current_obstacle.color, area_px=current_obstacle.area_px,
                            bbox=current_obstacle.bbox)
        if obstacle_candidate.expired(time.monotonic()) or obstacle_state == "timeout":
            _zero_motion(mcu)
            ramp.reset()
            return finish("obstacle_unconfirmed", obstacle_result=last_obstacle)

        if upper_available:
            ceiling, ceiling_opened = ceiling_selector.update(
                events,
                upper_right_has_numeric_value=upper_right_healthy,
                selected_state=upper_primary,
            )
        else:
            ceiling = ceiling_opened = False
        ceiling = ceiling or pending_ceiling
        ceiling_opened = ceiling_opened or pending_ceiling_opened
        junction_ready = False
        scanned_this_tick = False

        candidates = set(jump.side_candidates)
        # 入口尚未见到侧墙时，允许有界低速离开路口；不能把初始开阔当新节点。
        side_suspect = bool(candidates)
        if side_suspect and not terminal_home and not junction_pending:
            if not _zero_motion(mcu):
                return finish("fault")
            ramp.reset()
            last_command = (0, 0)
            junction_pending = True
            junction_stop_confirmed = False
            junction_trigger_mm = actual_distance
            junction_candidates.update(candidates)
            junction_since = now
            observe("junction_detected", distance_mm=actual_distance, progress_mm=sample_progress,
                    channels=sorted(candidates), monotonic_s=now)
            observe("stop_trigger", cause="junction", distance_mm=actual_distance, monotonic_s=now)
        if junction_pending:
            junction_candidates.update(candidates)
            # 确认必须描述当前开口，不能把先后出现的左/右开口并成双开口。
            current_open = {channel for channel, state in (
                ("left", nav.classify(left, ls)), ("right", nav.classify(right, rs)))
                if state == "open"}
            junction_confirmed.intersection_update(current_open)
            junction_confirmed.update(e.channel for e in events if e.side_opened)

        if directional_new:
            observe_sample(side_observation, jump.side_candidates,
                           [e.channel for e in events if e.side_opened], v_sent_prev)

        if obstacle_state == "confirmed":
            if not _zero_motion(mcu):
                return finish("fault")
            if not _motion_is_stable(mcu, deadline, require_upper=False, on_poll=show_stop):
                return finish("stop_unconfirmed")
            ramp.reset()
            if junction_pending:
                runlog.emit("junction_obstacle_conflict", "路口与障碍同时出现，停车等待人工核验",
                            road=step.road_id)
                return finish("junction_unknown")
            if rollback or not detect_obstacles:
                return finish("obstacle_unconfirmed", obstacle_result=last_obstacle)
            return finish("obstacle", obstacle_result=last_obstacle)

        if junction_pending:
            show_telemetry(command=(0, 0), confirmation=(
                f"候选 {','.join(sorted(junction_candidates))}；已确认 {','.join(sorted(junction_confirmed))}"))
            pending_ceiling = pending_ceiling or ceiling
            pending_ceiling_opened = pending_ceiling_opened or ceiling_opened
            if (junction_confirmed and junction_confirmed == current_open
                    and not side_observation.ambiguous
                    and nav.classify(left, ls) != "fault" and nav.classify(right, rs) != "fault"):
                if not _junction_matches(c, step, junction_confirmed, junction_trigger_mm, road_len_mm):
                    expected = _expected_junction_sides(c, step)
                    if expected is None or not junction_confirmed < expected:
                        _zero_motion(mcu)
                        return finish("junction_unknown")
                elif not junction_stop_confirmed:
                    if not _motion_is_stable(mcu, deadline, require_upper=False, on_poll=show_stop):
                        return finish("stop_unconfirmed")
                    ramp.reset()
                    junction_stop_confirmed = True
                    continue
                else:
                    observe("junction_confirmed", distance_mm=actual_distance,
                            progress_mm=sample_progress, channels=sorted(junction_confirmed))
                    junction_ready = True
            if junction_ready:
                junction_since = None
            elif now - junction_since >= config.JUNCTION_VERIFY_TIMEOUT_S:
                _zero_motion(mcu)
                return finish("junction_unknown")
            else:
                _drain_rfid_events(c, rf, ledger, step, actual_distance)
                time.sleep(0.02)
                continue

        pending_ceiling = pending_ceiling_opened = False
        if ceiling and obstacle_candidate.active:
            _zero_motion(mcu)
            return finish("culvert_fault")
        if ceiling:
            if step.dark_tunnel:
                pass
            elif (allow_scans and culvert_flow is not None and
                  (ledger is None or step.road_id not in ledger.culvert_handled)):
                if budget is not None and not budget.can_scan(
                        planner, action, actual_distance or 0.0, scan_s=config.CULVERT_ATTEMPT_TIMEOUT_S,
                        audio_remaining_s=_audio_remaining(c)):
                    _zero_motion(mcu)
                    _light_off(mcu, light_on)
                    return finish("budget_return")
                try:
                    stop_sequence = _send_motion(mcu, 0, 0, require_upper=False)
                    _confirm_motion_applied(mcu, stop_sequence, deadline)
                except mcu_mod.McuError:
                    _zero_motion(mcu)
                    _light_off(mcu, light_on)
                    return finish("fault")
                ramp.reset()
                if not _motion_is_stable(mcu, deadline, require_upper=False, on_poll=show_stop):
                    _zero_motion(mcu)
                    _light_off(mcu, light_on)
                    return finish("culvert_fault")
                # 停车确认会接收新遥测；扫描开始前重新核验上层触发能力。
                if not _update_upper_scan_health(c, jump, ceiling_selector, ledger, step):
                    pending_ceiling = pending_ceiling_opened = False
                    continue
                follower.pause()
                started_scan = time.monotonic()
                if ledger is not None:
                    ledger.culvert_handled.add(step.road_id)
                for attempt in range(config.CULVERT_MAX_ATTEMPTS):
                    if attempt and budget is not None and not budget.can_scan(
                            planner, action, actual_distance or 0.0, scan_s=config.CULVERT_ATTEMPT_TIMEOUT_S,
                            audio_remaining_s=_audio_remaining(c)):
                        break
                    scan_deadline = min(deadline, time.monotonic() + config.CULVERT_ATTEMPT_TIMEOUT_S)
                    runlog.emit("culvert_scan_start", "开始涵洞左右扫描", road=step.road_id, attempt=attempt + 1)
                    show_telemetry(phase="涵洞扫描", command=(0, 0), confirmation="扫描前")
                    inspection = culvert_flow.inspect(scan_deadline)
                    show_telemetry(phase="涵洞扫描", command=(0, 0), confirmation="扫描返回")
                    if ledger is not None:
                        ledger.record_culvert(step, actual_distance, inspection)
                        for audio_error in getattr(inspection, "audio_errors", ()):
                            ledger.record_audio_failure(str(audio_error))
                    complete = bool(getattr(inspection, "complete", False))
                    runlog.emit("culvert_scan_result", "涵洞扫描结果", road=step.road_id,
                                attempt=attempt + 1, complete=complete,
                                targets=getattr(inspection, "class_ids", ()))
                    if not bool(getattr(inspection, "navigation_view_restored", complete)):
                        _zero_motion(mcu)
                        _light_off(mcu, light_on)
                        return finish("culvert_fault")
                    if complete:
                        break
                scan_elapsed += time.monotonic() - started_scan
                last_progress_at = time.monotonic()
                # 恢复水平导航姿态后必须抓到一张新鲜画面，才恢复行驶。
                if require_camera and _grab_camera_frame(cam) is None:
                    _zero_motion(mcu)
                    _light_off(mcu, light_on)
                    return finish("culvert_fault")
                follower.resume()
                last_tick = time.monotonic()
                scanned_this_tick = True
            elif not step.dark_tunnel:
                runlog.emit("culvert_scan_skipped", "本路涵洞已处理或处于返航，不新增扫描", road=step.road_id)
        if ceiling_opened and light_on:
            _light_off(mcu, light_on)
            light_on = False
        if junction_ready:
            _light_off(mcu, light_on)
            _drain_rfid_events(c, rf, ledger, step, actual_distance)
            return finish("junction", side_junction=True)
        if scanned_this_tick:
            # 扫描期间仍在收遥测；恢复前重新裁决侧向激光与画面。
            v_sent_prev = 0.0
            continue
        _drain_rfid_events(c, rf, ledger, step, distance_estimate)
        if use_odom:
            odom_distance = _forward_distance_mm(odom0, mcu.odom_mm)
            if odom_distance is not None:
                distance_estimate = odom_distance * config.BRAKE_ODOM_SCALE
            else:
                distance_estimate = None
        else:
            now = time.monotonic()
            dt = min(now - last_tick, config.BRAKE_DR_DT_MAX_S)
            last_tick = now
            distance_estimate = (distance_estimate or 0.0) + v_sent_prev * dt

        if terminal_home:
            target_distance = max(
                config.ROLLBACK_MIN_ODOM_DISTANCE_MM,
                float(road_len_mm or 0.0),
            )
            if distance_estimate is None:
                _zero_motion(mcu)
                _light_off(mcu, light_on)
                return finish("home_position_fault")
            if distance_estimate >= target_distance:
                _soft_stop(mcu, ramp, deadline)
                _light_off(mcu, light_on)
                if _home_stop_is_stable(mcu, home_entry_yaw, deadline, on_poll=show_stop):
                    _drain_rfid_events(c, rf, ledger, step, distance_estimate)
                    return finish("home")
                return finish("home_position_fault")

        control_sides = side_observation
        if not side_armed and side_observation.ambiguous and side_observation.reason == "side_reference_missing":
            control_sides = nav.SideObservation(side_observation.left_state, side_observation.right_state)
        v, w, stop_reason = follower.control(left, ls, right, rs, mcu.yaw_deg, dir_seq,
                                               side_observation=control_sides)
        if follower.timed_out and not recenter_timeout_logged:
            recenter_timeout_logged = True
            runlog.emit("recenter_timeout", "5秒未居中，继续限速差速修正", road=step.road_id,
                        elapsed_s=follower.recenter_elapsed_s, left_mm=left, right_mm=right)
        if not side_armed:
            v = min(v, config.BRAKE_MIN_MMPS)
        if terminal_home and stop_reason == "blind_timeout":
            v, w, stop_reason = config.BRAKE_MIN_MMPS, 0, None
        if stop_reason is not None:
            _zero_motion(mcu)
            ramp.reset()
            _light_off(mcu, light_on)
            return finish(stop_reason)
        if terminal_home:
            v = min(v, config.BRAKE_MIN_MMPS)
        if distance_estimate is not None and not terminal_home:
            v = brake.scale_v(v, sample_progress if use_odom else distance_estimate)
        if obstacle_candidate.active:
            v = min(v, 100)
        if v == 0 and w == 0:
            if not _zero_motion(mcu):
                return finish("fault")
            ramp.reset()
            v_sent_prev = 0.0
            last_command = (0, 0)
            show_telemetry(command=(0, 0))
            time.sleep(0.02)
            continue
        show_telemetry()
        speed_limit = None
        if obstacle_candidate.active:
            speed_limit = min(100, v)
        if not side_armed or terminal_home:
            speed_limit = min(speed_limit or config.BRAKE_MIN_MMPS, config.BRAKE_MIN_MMPS)
        if obstacle_state != "first" and last_motion_sent_at is not None and (
                time.monotonic() - last_motion_sent_at < config.MOTION_SEND_INTERVAL_S):
            time.sleep(0.02)
            continue
        if w and abs(w) * config.FW_WHEEL_BASE_MM < 2000:
            w = 0
        v, w = ramp.step(v, w, max_linear_mmps=speed_limit)
        if time.monotonic() >= deadline:
            _zero_motion(mcu)
            ramp.reset()
            _light_off(mcu, light_on)
            return finish("deadline")
        try:
            motion_sequence = _send_motion(mcu, v, w, require_upper=False)
        except mcu_mod.McuError:
            _zero_motion(mcu)
            _light_off(mcu, light_on)
            return finish("fault")
        v_sent_prev = float(v)
        last_command = (v, w)
        show_telemetry()
        last_motion_sent_at = time.monotonic()
        time.sleep(0.02)


def _drive_standalone(
    c: dict,
    deadline: float,
    *,
    max_distance_mm: float | None = None,
    detect_obstacles: bool = False,
    upper_entry: bool = False,
    on_forward_frame=None,
) -> mission_mod.DriveResult:
    """Drive a test course with shared centering and safety, without map events."""
    if max_distance_mm is not None and (
            not math.isfinite(max_distance_mm) or max_distance_mm <= 0):
        raise ValueError("max_distance_mm must be finite and positive")
    mcu, camera = c["mcu"], c.get("camera")
    follower, ramp = nav.CenterFollower(), nav.VelocityRamp()
    side_observer = nav.SideObserver(c.get("_side_total_range"))
    ceiling_trackers = {
        "upper_left": nav.ChannelTracker("upper_left"),
        "upper_right": nav.ChannelTracker("upper_right"),
    }
    candidate = obstacle.ObstacleCandidate()
    entry_odom, entry_yaw = mcu.odom_mm, mcu.yaw_deg
    if entry_odom is None or entry_yaw is None:
        _zero_motion(mcu)
        return mission_mod.DriveResult("fault", None, None, False, entry_yaw, False)
    follower.begin(entry_yaw)
    frame_state: dict = {}
    last_dir_seq = last_upper_seq = last_frame_check = last_sent_at = None
    last_progress_at = time.monotonic()
    last_progress_mm = 0.0
    ceiling_since = None
    side_armed = False
    preview = on_forward_frame

    def finish(reason, *, obstacle_result=None):
        distance = _forward_distance_mm(entry_odom, mcu.odom_mm)
        return mission_mod.DriveResult(reason, distance, distance, False, mcu.yaw_deg,
                                       distance is not None, obstacle_result=obstacle_result)

    if camera is None:
        _zero_motion(mcu)
        return finish("vision_fault")
    if upper_entry:
        has_capability = getattr(mcu, "has_negotiated_capability", None)
        if not callable(has_capability) or not has_capability(Capability.CULVERT_UPPER2):
            _zero_motion(mcu)
            return finish("fault")
    while True:
        if time.monotonic() >= deadline:
            _zero_motion(mcu)
            return finish("deadline")
        try:
            mcu.pump()
        except mcu_mod.McuError:
            _zero_motion(mcu)
            return finish("fault")
        if time.monotonic() >= deadline:
            _zero_motion(mcu)
            return finish("deadline")
        fault = _telemetry_fault(mcu, require_upper=upper_entry)
        if fault is not None:
            _zero_motion(mcu)
            return finish("fault")
        now = time.monotonic()
        distance = _forward_distance_mm(entry_odom, mcu.odom_mm)
        if distance is None:
            _zero_motion(mcu)
            return finish("fault")

        directional = getattr(mcu, "directional", None)
        dir_seq = getattr(directional, "directional_cycle_seq", None)
        new_dir = dir_seq is not None and dir_seq != last_dir_seq
        if new_dir:
            last_dir_seq = dir_seq
        sides = side_observer.update(dir_seq, mcu.left_mm, mcu.left_status,
                                     mcu.right_mm, mcu.right_status,
                                     yaw_error_deg=_yaw_delta_deg(mcu.yaw_deg, entry_yaw)
                                     if mcu.yaw_deg is not None else None)
        if sides.dual_wall:
            side_armed = True
            c["_side_total_range"] = side_observer.total_range

        if upper_entry:
            upper_ok, upper_reason = selfcheck._upper_tof_health(mcu, required=True)
            if not upper_ok:
                runlog.emit("stage3_upper_fault", "上层不可用，阶段三未完成", reason=upper_reason)
                _zero_motion(mcu)
                return finish("fault")
            upper = getattr(mcu, "culvert_upper", None)
            upper_seq = getattr(upper, "upper_cycle_seq", None)
            if upper_seq is not None and upper_seq != last_upper_seq:
                last_upper_seq = upper_seq
                for channel, mm, status in (
                    ("upper_left", mcu.upper_left_mm, mcu.upper_left_status),
                    ("upper_right", mcu.upper_right_mm, mcu.upper_right_status),
                ):
                    tracker = ceiling_trackers[channel]
                    state = nav.classify(mm, status)
                    if tracker.current == "open" and state == "near" and tracker._pending != "near":
                        if ceiling_since is None:
                            if not _zero_motion(mcu):
                                return finish("fault")
                            ramp.reset()
                            ceiling_since = now
                    change = tracker.update(state)
                    if change == ("open", "near") and ceiling_since is not None:
                        if not _motion_is_stable(mcu, deadline, require_upper=True):
                            return finish("stop_unconfirmed")
                        return finish("ceiling")
        if ceiling_since is not None:
            if now - ceiling_since >= config.JUNCTION_VERIFY_TIMEOUT_S:
                _zero_motion(mcu)
                return finish("ceiling_unconfirmed")
            time.sleep(0.02)
            continue

        obstacle_due = detect_obstacles and (
            candidate.active or last_frame_check is None
            or now - last_frame_check >= config.OBSTACLE_CHECK_INTERVAL_S)
        camera_due = (obstacle_due or last_frame_check is None or
                      now - last_frame_check >= config.CAMERA_HEALTH_CHECK_INTERVAL_S)
        obstacle_state = "idle"
        if camera_due:
            last_frame_check = now
            try:
                frame = _grab_camera_frame(camera, frame_state)
            except Exception as exc:
                runlog.exception("standalone_camera_failed", "测试行驶取帧失败", exc)
                frame = None
            if frame is None or frame_state.get("metadata_error"):
                _zero_motion(mcu)
                return finish("vision_fault")
            if frame_state.get("is_new"):
                if preview is not None:
                    try:
                        preview(frame_state["captured"])
                    except Exception as exc:
                        runlog.exception("standalone_preview_failed", "行驶预览更新失败", exc)
                        preview = None
                if obstacle_due:
                    detected = obstacle.detect_colored_obstacle(frame)
                    obstacle_state = candidate.update(detected, frame_state["sequence"],
                                                      time.monotonic())
                    if obstacle_state == "fault":
                        _zero_motion(mcu)
                        return finish("vision_fault", obstacle_result=detected)
        if candidate.expired(time.monotonic()) or obstacle_state == "timeout":
            _zero_motion(mcu)
            return finish("obstacle_unconfirmed", obstacle_result=candidate.last)
        if obstacle_state == "confirmed":
            if not _zero_motion(mcu):
                return finish("fault")
            if not _motion_is_stable(mcu, deadline, require_upper=False):
                return finish("stop_unconfirmed")
            return finish("obstacle", obstacle_result=candidate.last)

        if distance >= last_progress_mm + config.ROAD_PROGRESS_MIN_MM:
            last_progress_mm, last_progress_at = distance, now
        if now - last_progress_at > config.ROAD_NO_PROGRESS_TIMEOUT_S:
            _zero_motion(mcu)
            return finish("no_progress")
        if max_distance_mm is not None and distance >= max_distance_mm:
            if not _zero_motion(mcu):
                return finish("fault")
            if not _motion_is_stable(mcu, deadline, require_upper=upper_entry):
                return finish("stop_unconfirmed")
            return finish("distance_limit")

        v, w, stop_reason = follower.control(
            mcu.left_mm, mcu.left_status, mcu.right_mm, mcu.right_status,
            mcu.yaw_deg, dir_seq, side_observation=sides)
        if stop_reason is not None:
            _zero_motion(mcu)
            return finish(stop_reason)
        if not side_armed:
            v = min(v, config.BRAKE_MIN_MMPS)
        if candidate.active:
            v = min(v, 100)
        if v == 0 and w == 0:
            _zero_motion(mcu)
            ramp.reset()
            time.sleep(0.02)
            continue
        if (obstacle_state != "first" and last_sent_at is not None and
                time.monotonic() - last_sent_at < config.MOTION_SEND_INTERVAL_S):
            time.sleep(0.02)
            continue
        if w and abs(w) * config.FW_WHEEL_BASE_MM < 2000:
            w = 0
        v, w = ramp.step(v, w, max_linear_mmps=min(100, v) if candidate.active else None)
        if time.monotonic() >= deadline:
            _zero_motion(mcu)
            return finish("deadline")
        try:
            _send_motion(mcu, v, w, require_upper=upper_entry)
        except mcu_mod.McuError:
            _zero_motion(mcu)
            return finish("fault")
        last_sent_at = time.monotonic()
        time.sleep(0.02)


def _light_on(mcu) -> None:
    sequence = mcu.send_light(config.LIGHT_DUTY_PERMILLE)
    mcu.wait_for_light_applied(sequence, config.LIGHT_DUTY_PERMILLE)


def _light_off(mcu, light_on: bool) -> None:
    if light_on:
        try:
            sequence = mcu.send_light(0)
            mcu.wait_for_light_applied(sequence, 0)
        except Exception as exc:
            runlog.exception("light_off_failed", "关灯命令未确认", exc)


def _zero_motion(mcu) -> bool:
    """真正停车使用二进制 STOP；零速同步/应用确认仍使用 CONTROL_TARGET。"""
    try:
        mcu.send_stop(StopReason.MISSION)
        return True
    except Exception as exc:
        runlog.exception("stop_failed", "STOP 命令发送失败", exc)
        return False


def _soft_stop(mcu, ramp, deadline: float | None = None) -> None:
    _zero_motion(mcu)
    ramp.reset()


def _announce_patrol_confirmation(c: dict, confirmed_nos: tuple[int, ...]) -> None:
    clues = c.get("clues")
    for number in confirmed_nos:
        clue = clues.patrol(number) if clues is not None else None
        if clue is None:
            print(f"读取到巡逻点 {number:02d} 号，但没有对应播报素材")
            ledger = c.get("_ledger")
            if ledger is not None:
                ledger.record_audio_failure(f"巡逻点{number:02d}缺少播报映射")
            continue
        print(clue.text)
        if not _safe_audio(c.get("audio"), clues.patrol_audio_path(number)):
            ledger = c.get("_ledger")
            if ledger is not None:
                ledger.record_audio_failure(f"巡逻点{number:02d}播报入队失败")


def _mark_position_unknown(
    c: dict,
    ledger: mission_mod.MissionLedger,
    reason: str,
) -> None:
    ledger.mark_position_unknown(reason)
    print(f"当前位置无法确认，禁止重规划转弯：{reason}")
    _zero_motion(c["mcu"])


def _audio_remaining(c: dict) -> float:
    remaining = getattr(c.get("audio"), "remaining_duration_s", None)
    if not callable(remaining):
        return 0.0
    try:
        return max(0.0, float(remaining()))
    except Exception as exc:
        runlog.exception("audio_budget_failed", "无法读取剩余播报时间", exc)
        return config.AUDIO_UNKNOWN_DURATION_S


def _stop_and_drain(c: dict, deadline: float, ledger) -> None:
    mcu = c["mcu"]
    _zero_motion(mcu)
    try:
        mcu.send_stop(StopReason.CLOSE)
        runlog.emit("mission_stop", "已发送比赛停车命令")
    except Exception as exc:
        runlog.exception("mission_stop_failed", "比赛停车命令失败", exc)
        ledger.note_terminal("最终STOP命令失败")
    reader = c.get("rfid")
    stop_polling = getattr(reader, "stop_polling", None)
    if callable(stop_polling):
        try:
            stop_polling()
        except Exception as exc:
            runlog.exception("rfid_stop_failed", "结束读卡失败", exc)
    audio = c.get("audio")
    runlog.status(phase="stationary_audio", deadline_monotonic=deadline,
                  audio_remaining_s=_audio_remaining(c))
    finish = getattr(audio, "finish_after_stop", None)
    if callable(finish):
        try:
            if not finish(deadline_monotonic=deadline):
                ledger.record_audio_failure(getattr(audio, "last_error", None) or "播报未全部成功")
        except Exception as exc:
            ledger.record_audio_failure(str(exc))
            runlog.exception("audio_drain_failed", "停车后播报收尾失败", exc)
    if getattr(audio, "failed_count", 0):
        ledger.record_audio_failure(f"本场有{audio.failed_count}条播放失败")
    if audio is None:
        ledger.record_audio_failure("音频组件不可用")


def run_navigation(c: dict, *, deadline: float | None = None) -> bool:
    if deadline is None:
        deadline = time.monotonic() + config.ROUND_TIME_LIMIT_S
    mcu = c["mcu"]
    c["_telemetry_display"] = telemetry_display.TelemetryDisplay(mcu)
    c["_telemetry_context"] = {}

    def show_turn(**context):
        _emit_telemetry(c, **context)

    fm = coverage.load_map()
    c["_field_map"] = fm
    ledger = mission_mod.MissionLedger(fm)
    for key in ("_upper_scan_available", "_upper_scan_last_seq", "_upper_scan_lost_seq"):
        c.pop(key, None)
    planner = mission_planner.MissionPlanner(fm)
    budget = mission_planner.ReturnBudget(fm, deadline, clock=time.monotonic)
    c["_ledger"] = ledger
    c["_planner"] = planner
    c["_budget"] = budget
    c["_match_deadline"] = deadline
    set_motion_deadline = getattr(mcu, "set_motion_deadline", None)
    if callable(set_motion_deadline):
        set_motion_deadline(deadline)
    follower, jump = nav.CenterFollower(), nav.JumpDetector()
    ceiling_selector = nav.CeilingSourceSelector()
    ramp = nav.VelocityRamp()
    turner = nav.TurnExecutor(mcu, ramp)
    culvert_flow = culvert_mod.CulvertFlow(
        mcu, c.get("gimbal"), c.get("camera"), c.get("detector"), c.get("audio"), c.get("clues")
    )
    node = fm.start_node_id
    # Placement is aligned with the single physical departure lane, not a compass angle.
    entrances = fm.adjacency[node]
    if len(entrances) != 1:
        raise coverage.CoverageError("出发区必须有唯一入口以确定初始车头行驶方向")
    heading = coverage.direction_between(fm, node, entrances[0][1])
    road_yaw = mcu.yaw_deg
    seen_states = set()
    try:
        if time.monotonic() >= deadline:
            ledger.note_terminal("初始化已耗尽比赛时限")
            return False
        fault = _telemetry_fault(mcu, require_upper=False)
        if fault is not None:
            ledger.note_terminal(f"导航前链路故障：{fault}")
            return False
        _update_upper_scan_health(c, jump, ceiling_selector, ledger)
        reader = c.get("rfid")
        start_polling = getattr(reader, "start_polling", None)
        if callable(start_polling):
            try:
                start_polling()
            except Exception as exc:
                runlog.exception("rfid_start_failed", "后台读卡启动失败；继续其余任务", exc)
        runlog.emit("navigation_start", "开始本场任务；障碍与涵洞记录均为空",
                    calibrated=budget.calibrated, time_limit_s=max(0.0, deadline - time.monotonic()))
        if not budget.calibrated:
            runlog.emit("budget_uncalibrated", "返航预算为配置推演，尚未实车标定",
                        estimated_speed_mmps=budget.speed_mmps, margin_s=budget.margin_s)
        while time.monotonic() < deadline:
            if not ledger.can_execute_turn:
                raise mcu_mod.McuError("当前位置尚未确认，禁止生成新的道路动作")
            action = planner.next_action(node)
            if action is None:
                if node == fm.start_node_id and planner.status == "returned":
                    ledger.mark_route_plan_finished()
                else:
                    ledger.note_terminal(f"无可执行路线：{planner.status}")
                break
            if not planner.return_mode and not budget.can_start(
                    planner, action, heading=heading, audio_remaining_s=_audio_remaining(c)):
                planner.begin_return("剩余时间不足以完成下一动作并保留返航预算")
                runlog.emit("return_started", "停止新任务，沿已走通道路返航",
                            required_s=budget.required_for_action(planner, action, heading=heading),
                            remaining_s=budget.remaining_s, node=node)
                continue
            step = action.step
            marker = (node, action.kind, action.target, tuple(sorted(planner.blocked_roads)),
                      tuple(sorted(planner.completed_roads)),
                      tuple((k, task.status) for k, task in sorted(planner.probe_tasks.items())),
                      planner.return_mode)
            if marker in seen_states:
                ledger.note_terminal("任务状态重复且没有推进，禁止持续徘徊")
                runlog.emit("mission_no_progress", "检测到重复任务状态，结束当前探索", node=node)
                if planner.return_mode:
                    break
                planner.begin_return("任务没有推进")
                continue
            seen_states.add(marker)
            runlog.emit("mission_action", "执行道路动作", kind=action.kind, road=step.road_id,
                        entry=step.from_node, destination=step.to_node, pending=list(planner.stack),
                        blocked=sorted(planner.blocked_roads), remaining_s=budget.remaining_s)
            action_started = time.monotonic()
            previous_heading = heading
            next_heading = coverage.direction_between(fm, step.from_node, step.to_node)
            entry_turn = coverage.turn_kind(heading, next_heading)
            if not _execute_turn(turner, entry_turn, deadline, ramp, on_poll=show_turn):
                _mark_position_unknown(c, ledger, "转弯或停车确认失败")
                break
            heading = next_heading
            if entry_turn != coverage.TURN_STRAIGHT:
                road_yaw = turner.target_yaw_deg
            progress = ledger.begin_road(step, mcu.odom_mm, mcu.yaw_deg)
            length_mm = mission_planner.road_length_mm(fm.roads[step.road_id])
            terminal_home = step.to_node == fm.start_node_id
            result = _drive_until_junction(
                c, follower, jump, culvert_flow, reader, step, deadline, ramp, length_mm,
                detect_obstacles=not step.dark_tunnel, require_camera=not step.dark_tunnel,
                terminal_home=terminal_home, ledger=ledger, road_progress=progress,
                ceiling_selector=ceiling_selector, allow_scans=not planner.return_mode,
                planner=planner, action=action, budget=budget,
                expected_yaw_deg=road_yaw, recenter=entry_turn != coverage.TURN_STRAIGHT,
            )
            if result.reason in ("obstacle", "budget_return"):
                is_obstacle = result.reason == "obstacle"
                if not result.odom_reliable or result.odom_distance_mm is None:
                    _mark_position_unknown(c, ledger, "回退所需里程不可用")
                    break
                if (not is_obstacle and result.odom_distance_mm == 0.0
                        and _motion_is_stable(
                            mcu, deadline, require_upper=False,
                            on_poll=lambda: _emit_telemetry(
                                c, phase="停车确认", command=(0, 0), confirmation="双轮停稳核验"))):
                    ledger.confirm_rollback(step.from_node)
                    planner.begin_return("未离开当前入口，直接切换返航任务")
                    continue
                if result.odom_distance_mm < config.ROLLBACK_MIN_ODOM_DISTANCE_MM:
                    _mark_position_unknown(c, ledger, "离入口距离过短，无法可靠执行掉头回退")
                    break
                if is_obstacle:
                    planner.observe_obstacle(action)
                    ledger.record_blocked_road(step, result, result.obstacle_result)
                    runlog.emit("road_blocked", "道路禁止穿越，保留另一端补查",
                                road=step.road_id, entry=step.from_node,
                                blocked_count=len(planner.blocked_roads))
                if not _execute_turn(turner, coverage.TURN_UTURN, deadline, ramp, on_poll=show_turn):
                    _mark_position_unknown(c, ledger, "返回入口前掉头失败")
                    break
                heading = (-heading[0], -heading[1])
                road_yaw = turner.target_yaw_deg
                rollback_step = coverage.Step(
                    step.index, step.to_node, step.from_node, step.road_id, step.dark_tunnel)
                rollback_progress = ledger.begin_rollback(step, result, mcu.odom_mm, mcu.yaw_deg)
                rollback = _drive_until_junction(
                    c, follower, jump, culvert_flow, reader, rollback_step, deadline, ramp,
                    result.odom_distance_mm, detect_obstacles=False,
                    require_camera=not step.dark_tunnel,
                    terminal_home=step.from_node == fm.start_node_id, rollback=True,
                    ledger=ledger, road_progress=rollback_progress, ceiling_selector=ceiling_selector,
                    allow_scans=False,
                    expected_yaw_deg=road_yaw, recenter=True,
                )
                confirmed, reason = rollback_progress.can_confirm_rollback(
                    rollback, home_node_id=fm.start_node_id)
                if not confirmed:
                    _mark_position_unknown(c, ledger, reason)
                    break
                node = step.from_node
                ledger.confirm_rollback(node)
                if is_obstacle:
                    planner.obstacle_returned(action)
                if not is_obstacle or result.return_requested or len(planner.blocked_roads) > 3:
                    planner.begin_return(
                        "返航预算不足" if not is_obstacle or result.return_requested
                        else "阻塞道路超过官方三处，停止新增探查")
                runlog.emit("rollback_confirmed", "已确认回到原入口，重新规划",
                            node=node, probes=planner.summary()["probes"])
                continue
            if terminal_home and result.reason == "home":
                ledger.confirm_home_arrival(step, result)
            elif result.reason == "junction" and result.side_junction:
                if action.kind == "probe":
                    ledger.note_terminal("从封闭道路另一端走通，障碍证据或定位存在矛盾")
                    break
                ledger.confirm_arrival(step, result)
            else:
                _mark_position_unknown(c, ledger, f"道路行驶未确认到达：{result.reason}")
                break
            planner.arrived(action)
            node = step.to_node
            if result.return_requested:
                planner.begin_return("预算不足，已沿较短方向确认节点，直接返航")
            elapsed = time.monotonic() - action_started - float(c.get("_last_scan_elapsed_s", 0.0))
            budget.observe_elapsed(action, elapsed, heading=previous_heading)
            runlog.emit("node_arrived", "已确认到达节点", node=node, road=step.road_id,
                        completed_roads=len(planner.completed_roads))
        else:
            ledger.note_terminal("比赛时间截止，停止运动和探索")
    except Exception as exc:
        ledger.note_terminal(f"导航异常：{exc}")
        runlog.exception("navigation_failed", "导航异常，停车并保留未完成任务", exc)
    finally:
        ledger.record_planner(planner)
        _stop_and_drain(c, deadline, ledger)
        _emit_telemetry(c, phase="比赛结束", command=(0, 0), force=True)
        print(ledger.summary_text())
        runlog.emit("mission_summary", "比赛结果", complete=ledger.status().complete,
                    returned_to_start=ledger.status().returned_to_start, planner=planner.summary(),
                    terminal_reasons=ledger.terminal_reasons, audio_failures=ledger.audio_failures)
    return ledger.status().complete


def _positive_duration(value: str) -> float:
    try:
        duration = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("duration 必须是秒数") from exc
    if not (0.0 < duration <= 120.0):
        raise argparse.ArgumentTypeError("duration 必须在 0 到 120 秒之间")
    return duration


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="RDK 车辆程序")
    subcommands = parser.add_subparsers(dest="mode")
    check_parser = subcommands.add_parser("selfcheck", help="执行自检")
    check_parser.add_argument(
        "--passive",
        action="store_true",
        help="只读取 STM32 HELLO/遥测，不操作云台、音频或其他外设",
    )
    subcommands.add_parser("start", help="自动自检及精确规划后开始三阶段融合比赛，无全场时限")
    diagnose_parser = subcommands.add_parser("diagnose", help="无动作采集 STM32 诊断")
    diagnose_parser.add_argument(
        "--duration",
        type=_positive_duration,
        default=20.0,
        help="采集秒数（默认 20，最大 120）",
    )
    gimbal_parser = subcommands.add_parser("gimbal-test", help="交互调整上下 PWM，标记实测水平位置")
    gimbal_parser.add_argument("--port", default=None, help="STM32 串口（默认使用 config.MCU_PORT）")
    args = parser.parse_args(argv[1:] if len(argv) > 1 else ["selfcheck"])
    if args.mode == "start":
        return cmd_start()
    if args.mode == "diagnose":
        return cmd_diagnose(args.duration)
    if args.mode == "gimbal-test":
        return cmd_gimbal_test(port=args.port)
    return cmd_selfcheck(passive=bool(getattr(args, "passive", False)))


if __name__ == "__main__":
    with runlog.session(sys.argv[1] if len(sys.argv) > 1 else "selfcheck"):
        exit_code = main(sys.argv)
    sys.exit(exit_code)
