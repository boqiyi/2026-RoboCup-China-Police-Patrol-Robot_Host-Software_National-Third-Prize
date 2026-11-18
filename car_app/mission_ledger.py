"""比赛任务账本与道路位置状态。

本模块只记录上位机已经确认的事实：不会把计划步骤、未确认的色块候选或估计位置
误写成“已完成”。所有内容保留在内存中，比赛结束由主程序输出中文汇总。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import TYPE_CHECKING

import config
import coverage

if TYPE_CHECKING:
    from rfid import RfidEvent


POSITION_CONFIRMED_NODE = "已确认节点"
POSITION_ON_ROAD = "道路中"
POSITION_UNKNOWN = "未知"


@dataclass(frozen=True)
class DriveResult:
    """一次道路行驶的上位机观测结果。"""

    reason: str
    distance_mm: float | None
    odom_distance_mm: float | None
    side_junction: bool
    yaw_deg: float | None
    odom_reliable: bool
    obstacle_result: object | None = None
    return_requested: bool = False

    def __eq__(self, other: object) -> bool:
        """兼容旧的字符串原因比较，避免桌面旧脚本被无谓破坏。"""
        if isinstance(other, str):
            return self.reason == other
        if not isinstance(other, DriveResult):
            return NotImplemented
        return (
            self.reason,
            self.distance_mm,
            self.odom_distance_mm,
            self.side_junction,
            self.yaw_deg,
            self.odom_reliable,
            self.obstacle_result,
            self.return_requested,
        ) == (
            other.reason,
            other.distance_mm,
            other.odom_distance_mm,
            other.side_junction,
            other.yaw_deg,
            other.odom_reliable,
            other.obstacle_result,
            other.return_requested,
        )


@dataclass
class RoadProgress:
    """当前道路的进度与位置可信度。"""

    road_id: str
    from_node: str
    to_node: str
    entry_odom_mm: tuple[int, int] | None
    entry_yaw_deg: float | None
    last_confirmed_node: str
    step_index: int
    entered_monotonic_s: float = field(default_factory=time.monotonic)
    current_distance_mm: float | None = 0.0
    odom_distance_mm: float | None = None
    current_yaw_deg: float | None = None
    confidence: str = POSITION_ON_ROAD
    rollback: bool = False
    expected_rollback_odom_mm: float | None = None
    forward_yaw_deg: float | None = None

    def observe(self, result: DriveResult) -> None:
        self.current_distance_mm = result.distance_mm
        self.odom_distance_mm = result.odom_distance_mm
        self.current_yaw_deg = result.yaw_deg

    def can_confirm_rollback(self, result: DriveResult, *, home_node_id: str | None = None) -> tuple[bool, str]:
        """只在侧向路口、编码器和航向三项证据齐全时确认退回原节点。"""
        self.observe(result)
        home_return = (home_node_id is not None and self.to_node == home_node_id
                       and result.reason == "home")
        if not home_return and (result.reason != "junction" or not result.side_junction):
            return False, f"回退未收到侧向路口确认（原因={result.reason}）"
        if not result.odom_reliable or result.odom_distance_mm is None:
            return False, "回退编码器里程不可用"
        if self.expected_rollback_odom_mm is None:
            return False, "前进段没有可信编码器里程"
        if not all(math.isfinite(value) and value >= 0 for value in
                   (result.odom_distance_mm, self.expected_rollback_odom_mm)):
            return False, "回退里程包含无效数值"
        expected = max(
            config.ROLLBACK_MIN_ODOM_DISTANCE_MM,
            self.expected_rollback_odom_mm
            - min(
                config.ROLLBACK_ODOM_TOLERANCE_MM,
                self.expected_rollback_odom_mm * 0.20,
            ),
        )
        if result.odom_distance_mm < expected:
            return False, (
                f"回退编码器里程不足：{result.odom_distance_mm:.1f}mm < "
                f"{expected:.1f}mm"
            )
        maximum = self.expected_rollback_odom_mm + min(
            config.ROLLBACK_ODOM_TOLERANCE_MM, self.expected_rollback_odom_mm * 0.20
        )
        if result.odom_distance_mm > maximum:
            return False, f"回退编码器里程超过原入口范围：{result.odom_distance_mm:.1f}mm > {maximum:.1f}mm"
        if self.forward_yaw_deg is None or result.yaw_deg is None:
            return False, "回退航向不可用"
        if not math.isfinite(self.forward_yaw_deg) or not math.isfinite(result.yaw_deg):
            return False, "回退航向包含无效数值"
        delta = _angle_delta_deg(result.yaw_deg - self.forward_yaw_deg)
        if abs(abs(delta) - 180.0) > config.ROLLBACK_YAW_TOLERANCE_DEG:
            return False, (
                f"回退航向未确认掉头：航向差 {delta:.1f}°，"
                f"容差 ±{config.ROLLBACK_YAW_TOLERANCE_DEG:.1f}°"
            )
        return True, "回退位置已确认"


@dataclass(frozen=True)
class RfidRecord:
    uid_hex: str | None
    patrol_no: int | None
    monotonic_s: float
    road_id: str
    step_index: int
    distance_mm: float | None
    error: str | None = None
    block_no: int | None = None
    block_data_hex: str | None = None
    reader_status: int | None = None


@dataclass(frozen=True)
class CulvertRecord:
    scan_index: int
    road_id: str
    step_index: int
    distance_mm: float | None
    left_class_ids: tuple[int, ...]
    right_class_ids: tuple[int, ...]
    valid_class_ids: tuple[int, ...]
    complete: bool
    frame_failures: int
    inference_failures: int
    invalid_class_ids: tuple[int, ...]


@dataclass(frozen=True)
class ObstacleRecord:
    road_id: str
    step_index: int
    color: str | None
    area_px: float | None
    bbox: tuple[int, int, int, int] | None
    available: bool
    reason: str | None


@dataclass(frozen=True)
class ArrivalRfidResult:
    confirmed_patrol_nos: tuple[int, ...]
    mismatch_count: int
    missed_expected_patrol_no: int | None


@dataclass(frozen=True)
class MissionStatus:
    complete: bool
    status_text: str
    completed_roads: tuple[str, ...]
    missing_roads: tuple[str, ...]
    confirmed_patrol_nos: tuple[int, ...]
    missing_patrol_nos: tuple[int, ...]
    returned_to_start: bool
    rfid_evidence_observed: bool = False
    unfinished_probes: tuple[str, ...] = ()
    failed_culvert_roads: tuple[str, ...] = ()
    missing_culvert_count: int = 0
    audio_failure_count: int = 0


def _angle_delta_deg(value: float) -> float:
    return ((value + 180.0) % 360.0) - 180.0


class MissionLedger:
    """以地图和真实确认事件为准的轻量比赛账本。"""

    def __init__(self, field_map: coverage.FieldMap) -> None:
        self.field_map = field_map
        # 巡逻号以实际卡片0扇区1块的数据为准，不与UID或地图节点绑定。
        self.patrol_to_node: dict[int, str] = {}
        self.completed_roads: set[str] = set()
        self.blocked_roads: set[str] = set()
        self.confirmed_nodes: set[str] = {field_map.start_node_id}
        self.last_confirmed_node = field_map.start_node_id
        self.position_confidence = POSITION_CONFIRMED_NODE
        self.current_progress: RoadProgress | None = None
        self.rfid_records: list[RfidRecord] = []
        self.rfid_mismatches: list[str] = []
        self.rfid_missed: list[str] = []
        self.rfid_errors: list[str] = []
        self.confirmed_patrol_nos: set[int] = set()
        # UID仅关联实际读块卡片身份；各入口分别决定重复播报策略。
        self.confirmed_patrol_uids: dict[int, set[str]] = {}
        self.culvert_records: list[CulvertRecord] = []
        self.culvert_handled: set[str] = set()
        self.culvert_attempts: dict[str, int] = {}
        self.audio_failures: list[str] = []
        self.planner_summary: dict | None = None
        self.no_target_culvert_count = 0
        self.obstacle_records: list[ObstacleRecord] = []
        self.visual_unavailable_events: list[str] = []
        self.upper_availability_events: list[dict] = []
        self.terminal_reasons: list[str] = []
        self.replan_result = None
        self.route_plan_finished = False
        self._visual_unavailable_active: set[str] = set()

    @property
    def can_execute_turn(self) -> bool:
        return self.position_confidence == POSITION_CONFIRMED_NODE

    def begin_road(
        self,
        step: coverage.Step,
        entry_odom_mm: tuple[int, int] | None,
        entry_yaw_deg: float | None,
    ) -> RoadProgress:
        progress = RoadProgress(
            road_id=step.road_id,
            from_node=step.from_node,
            to_node=step.to_node,
            entry_odom_mm=entry_odom_mm,
            entry_yaw_deg=entry_yaw_deg,
            last_confirmed_node=self.last_confirmed_node,
            step_index=step.index,
        )
        self.current_progress = progress
        self.position_confidence = POSITION_ON_ROAD
        return progress

    def begin_rollback(
        self,
        step: coverage.Step,
        forward: DriveResult,
        entry_odom_mm: tuple[int, int] | None,
        entry_yaw_deg: float | None,
    ) -> RoadProgress:
        progress = RoadProgress(
            road_id=step.road_id,
            from_node=step.to_node,
            to_node=step.from_node,
            entry_odom_mm=entry_odom_mm,
            entry_yaw_deg=entry_yaw_deg,
            last_confirmed_node=self.last_confirmed_node,
            step_index=step.index,
            rollback=True,
            expected_rollback_odom_mm=forward.odom_distance_mm,
            forward_yaw_deg=forward.yaw_deg,
        )
        self.current_progress = progress
        self.position_confidence = POSITION_ON_ROAD
        return progress

    def record_rfid_event(
        self,
        event: RfidEvent,
        step: coverage.Step,
        distance_mm: float | None,
    ) -> RfidRecord:
        """凭成功读块证据确认巡逻号；UID仅记录卡片身份。"""
        from rfid import parse_patrol_no

        uid = getattr(event, "uid", None)
        error = getattr(event, "error", None)
        block_no = getattr(event, "block_no", None)
        block_data = getattr(event, "block_data", None)
        reader_status = getattr(event, "reader_status", None)
        patrol_no = None
        if not error:
            if not isinstance(uid, bytes) or len(uid) != 4:
                error = "巡逻读块缺少完整4字节UID身份"
            elif (type(block_no) is not int or block_no != config.RFID_PATROL_BLOCK
                    or type(reader_status) is not int or reader_status != 0):
                error = "缺少指定巡逻数据块的成功响应证据"
            elif not isinstance(block_data, bytes) or len(block_data) != 16:
                error = "巡逻数据块必须包含完整16字节，不能用UID替代"
            else:
                patrol_no = parse_patrol_no(block_data)
                if patrol_no is None:
                    error = "0扇区1块第0字节必须为10、20、…、90、A0、B0、C0"
        timestamp = float(getattr(event, "monotonic_s", time.monotonic()))
        record = RfidRecord(
            uid.hex() if isinstance(uid, bytes) else None,
            patrol_no if isinstance(patrol_no, int) else None,
            timestamp,
            step.road_id,
            step.index,
            distance_mm,
            str(error) if error else None,
            block_no if type(block_no) is int else None,
            block_data.hex() if isinstance(block_data, bytes) else None,
            reader_status if type(reader_status) is int else None,
        )
        self.rfid_records.append(record)
        if record.error:
            self.rfid_errors.append(
                f"道路 {record.road_id} 步骤 {record.step_index} RFID 异常：{record.error}"
            )
        elif record.patrol_no is None:
            self.rfid_errors.append(
                f"道路 {record.road_id} 步骤 {record.step_index} UID "
                f"{record.uid_hex or '未知'} 无有效巡逻号"
            )
        else:
            self.confirmed_patrol_nos.add(record.patrol_no)
            if record.uid_hex is not None:
                self.confirmed_patrol_uids.setdefault(record.patrol_no, set()).add(record.uid_hex)
        return record

    def confirm_arrival(
        self, step: coverage.Step, result: DriveResult
    ) -> ArrivalRfidResult:
        """确认侧向路口到达；RFID已凭实际块数据独立确认。"""
        if result.reason != "junction" or not result.side_junction:
            raise ValueError("未确认侧向路口，不得确认道路终点")
        self.completed_roads.add(step.road_id)
        self.confirmed_nodes.add(step.to_node)
        self.last_confirmed_node = step.to_node
        self.position_confidence = POSITION_CONFIRMED_NODE
        self.current_progress = None
        return ArrivalRfidResult((), 0, None)

    def confirm_home_arrival(self, step: coverage.Step, result: DriveResult) -> None:
        """以最终回程的里程和稳定航向确认到达出发区，不伪造侧向路口。"""
        if step.to_node != self.field_map.start_node_id or result.reason != "home":
            raise ValueError("未满足出发区终段确认条件")
        if not result.odom_reliable or result.odom_distance_mm is None or result.yaw_deg is None:
            raise ValueError("出发区终段缺少里程或航向确认")
        if (not math.isfinite(result.odom_distance_mm) or result.odom_distance_mm <= 0
                or not math.isfinite(result.yaw_deg)):
            raise ValueError("出发区终段里程或航向数值无效")
        self.completed_roads.add(step.road_id)
        self.confirmed_nodes.add(step.to_node)
        self.last_confirmed_node = step.to_node
        self.position_confidence = POSITION_CONFIRMED_NODE
        self.current_progress = None

    def confirm_rollback(self, node_id: str) -> None:
        self.confirmed_nodes.add(node_id)
        self.last_confirmed_node = node_id
        self.position_confidence = POSITION_CONFIRMED_NODE
        self.current_progress = None

    def mark_position_unknown(self, reason: str) -> None:
        self.position_confidence = POSITION_UNKNOWN
        self.current_progress = None
        self.terminal_reasons.append(reason)

    def note_terminal(self, reason: str) -> None:
        self.terminal_reasons.append(reason)

    def record_blocked_road(self, step: coverage.Step, result: DriveResult, obstacle_result) -> None:
        self.blocked_roads.add(step.road_id)
        self.obstacle_records.append(
            ObstacleRecord(
                road_id=step.road_id,
                step_index=step.index,
                color=getattr(obstacle_result, "color", None),
                area_px=getattr(obstacle_result, "area_px", None),
                bbox=getattr(obstacle_result, "bbox", None),
                available=bool(getattr(obstacle_result, "available", False)),
                reason=getattr(obstacle_result, "reason", None),
            )
        )

    def record_visual_unavailable(self, step: coverage.Step, reason: str) -> bool:
        """记录一次连续不可用状态；恢复后下一次不可用会重新记账。"""
        key = f"{step.road_id}:{step.index}"
        if key in self._visual_unavailable_active:
            return False
        self._visual_unavailable_active.add(key)
        self.visual_unavailable_events.append(
            f"道路 {step.road_id} 步骤 {step.index} 摄像头不可用：{reason}"
        )
        return True

    def record_visual_available(self, step: coverage.Step) -> None:
        self._visual_unavailable_active.discard(f"{step.road_id}:{step.index}")

    def record_culvert(self, step: coverage.Step, distance_mm: float | None, inspection) -> None:
        left = getattr(inspection, "left", None)
        right = getattr(inspection, "right", None)
        left_ids = tuple(getattr(left, "class_ids", ()))
        right_ids = tuple(getattr(right, "class_ids", ()))
        valid = tuple(getattr(inspection, "class_ids", ()))
        complete = bool(getattr(inspection, "complete", False))
        frame_failures = int(getattr(left, "frame_failures", 0)) + int(
            getattr(right, "frame_failures", 0)
        )
        inference_failures = int(getattr(left, "inference_failures", 0)) + int(
            getattr(right, "inference_failures", 0)
        )
        invalid_ids = tuple(getattr(inspection, "invalid_class_ids", ()))
        self.culvert_records.append(
            CulvertRecord(
                len(self.culvert_records) + 1,
                step.road_id,
                step.index,
                distance_mm,
                left_ids,
                right_ids,
                valid,
                complete,
                frame_failures,
                inference_failures,
                invalid_ids,
            )
        )
        self.culvert_attempts[step.road_id] = self.culvert_attempts.get(step.road_id, 0) + 1
        if complete or self.culvert_attempts[step.road_id] >= 2:
            self.culvert_handled.add(step.road_id)
        if complete and not valid:
            self.no_target_culvert_count += 1

    def record_audio_failure(self, reason: str) -> None:
        self.audio_failures.append(reason)

    def record_upper_availability(
        self,
        available: bool,
        reason: str,
        road_id: str | None = None,
        monotonic_s: float | None = None,
    ) -> None:
        """记录涵洞触发的降级和恢复，不把漏掉的扫描记为完成。"""
        self.upper_availability_events.append({
            "available": available,
            "reason": reason,
            "road_id": road_id,
            "monotonic_s": time.monotonic() if monotonic_s is None else monotonic_s,
        })

    def record_planner(self, planner) -> None:
        self.planner_summary = planner.summary()

    def record_replan(self, result) -> None:
        self.replan_result = result

    def mark_route_plan_finished(self) -> None:
        self.route_plan_finished = True

    def status(self) -> MissionStatus:
        all_roads = set(self.field_map.roads)
        missing_roads = sorted(all_roads - self.blocked_roads - self.completed_roads)
        active_nodes = {self.field_map.start_node_id}
        for road_id, road in self.field_map.roads.items():
            if road_id not in self.blocked_roads:
                active_nodes.add(road.a)
                active_nodes.add(road.b)
        missing_nodes = sorted(active_nodes - self.confirmed_nodes)
        missing_patrols = sorted(set(range(1, 13)) - self.confirmed_patrol_nos)
        last_culverts = {record.road_id: record for record in self.culvert_records}
        failed_culverts = sorted(road_id for road_id, record in last_culverts.items() if not record.complete)
        completed_culverts = sum(record.complete for record in last_culverts.values())
        missing_culverts = max(0, int(getattr(config, "EXPECTED_CULVERT_COUNT", 8)) - completed_culverts)
        unfinished_probes: list[str] = []
        if self.planner_summary is not None:
            unfinished_probes = [
                f"{task['road_id']}@{task['entry_node']}:{task['status']}"
                for task in self.planner_summary.get("probes", ()) if task["status"] != "complete"
            ]
        returned = (
            self.position_confidence == POSITION_CONFIRMED_NODE
            and self.last_confirmed_node == self.field_map.start_node_id
        )
        can_return = True
        unreachable_roads: set[str] = set()
        if self.replan_result is not None:
            can_return = bool(getattr(self.replan_result, "can_return_to_end", True))
            unreachable_roads = set(
                getattr(self.replan_result, "unreachable_remaining_roads", ())
            )
        route_complete = (
            self.route_plan_finished
            and not missing_roads
            and not missing_nodes
            and not unreachable_roads
            and returned
            and can_return
            and self.position_confidence != POSITION_UNKNOWN
        )
        rfid_evidence_observed = bool(self.confirmed_patrol_nos)
        complete = (route_complete and not missing_patrols and not unfinished_probes
                    and not failed_culverts and not missing_culverts
                    and not self.audio_failures and not self.rfid_errors)
        if complete:
            status_text = "比赛任务完整完成"
        elif route_complete and not rfid_evidence_observed:
            status_text = "路线与节点完成，但未读取有效巡逻数据块，不能确认比赛任务完整完成"
        else:
            status_text = "任务未完整完成"
        return MissionStatus(
            complete=complete,
            status_text=status_text,
            completed_roads=tuple(sorted(self.completed_roads)),
            missing_roads=tuple(missing_roads),
            confirmed_patrol_nos=tuple(sorted(self.confirmed_patrol_nos)),
            missing_patrol_nos=tuple(missing_patrols),
            returned_to_start=returned,
            rfid_evidence_observed=rfid_evidence_observed,
            unfinished_probes=tuple(unfinished_probes),
            failed_culvert_roads=tuple(failed_culverts),
            missing_culvert_count=missing_culverts,
            audio_failure_count=len(self.audio_failures),
        )

    def summary_text(self) -> str:
        status = self.status()
        obstacle_lines = [
            f"[{item.road_id}/步骤{item.step_index}] 颜色={item.color or '未知'} "
            f"面积={item.area_px if item.area_px is not None else '无'} "
            f"边界框={item.bbox or '无'} 视觉可用={'是' if item.available else '否'} "
            f"原因={item.reason or '无'}"
            for item in self.obstacle_records
        ]
        culvert_lines = []
        for item in self.culvert_records:
            valid = ",".join(str(cid) for cid in item.valid_class_ids) or "无有效目标"
            culvert_lines.append(
                f"第{item.scan_index}次[{item.road_id}/步骤{item.step_index}] "
                f"左={list(item.left_class_ids)} 右={list(item.right_class_ids)} "
                f"有效={valid} 完整={'是' if item.complete else '否'} "
                f"抓帧失败={item.frame_failures} 推理失败={item.inference_failures} "
                f"无效类别={list(item.invalid_class_ids) or '无'}"
            )
        replan_note = "无"
        if self.replan_result is not None:
            unreachable = sorted(
                getattr(self.replan_result, "unreachable_remaining_roads", ())
            )
            replan_note = (
                f"可达剩余道路={sorted(getattr(self.replan_result, 'reachable_remaining_roads', ()))}；"
                f"不可达道路={unreachable or '无'}；"
                f"可回起点={'是' if getattr(self.replan_result, 'can_return_to_end', False) else '否'}"
            )
        confirmed_patrols = [
            f"{number:02d}(UID={','.join(sorted(self.confirmed_patrol_uids.get(number, ()))) or '未知'})"
            for number in status.confirmed_patrol_nos
        ]
        return "\n".join(
            [
                "\n========== 比赛任务账本汇总 ==========" ,
                f"已完成道路：{list(status.completed_roads) or '无'}",
                f"遗漏道路：{list(status.missing_roads) or '无'}",
                f"被屏蔽道路：{sorted(self.blocked_roads) or '无'}",
                f"已确认节点：{sorted(self.confirmed_nodes)}",
                f"最后确认节点：{self.last_confirmed_node}",
                f"已确认巡逻号及卡片UID：{confirmed_patrols or '无'}",
                f"巡逻数据块证据：{'已读取' if status.rfid_evidence_observed else '未读取'}",
                f"遗漏巡逻点：{list(status.missing_patrol_nos) or '无'}",
                f"色块障碍证据：{obstacle_lines or ['无']}",
                f"未完成障碍补查：{list(status.unfinished_probes) or '无'}",
                f"涵洞最终失败道路：{list(status.failed_culvert_roads) or '无'}",
                f"尚缺完整涵洞检查：{status.missing_culvert_count}",
                f"播报失败：{self.audio_failures or '无'}",
                f"RFID 不匹配：{self.rfid_mismatches or '无'}",
                f"RFID 漏读：{self.rfid_missed or '无'}",
                f"RFID 异常：{self.rfid_errors or '无'}",
                f"涵洞识别结果：{culvert_lines or ['无涵洞扫描记录']}",
                f"未识别到嫌疑人或刀具次数：{self.no_target_culvert_count}",
                f"摄像头不可用：{self.visual_unavailable_events or '无'}",
                f"上层涵洞触发降级/恢复：{self.upper_availability_events or '无'}",
                f"安全停止/异常：{self.terminal_reasons or '无'}",
                f"重规划状态：{replan_note}",
                f"是否回到起点：{'是' if status.returned_to_start else '否'}",
                f"最终状态：{status.status_text}",
                "======================================",
            ]
        )

