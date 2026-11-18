"""阶段1：按实际下层路口执行最短闭合邮递员路线、RFID及每次经过播报。

控制复用test_motion_sequence；阶段1本地适配转弯等待、陀螺仪离口、地图动作、速度与账本。
普通道路按原始双侧差≤3mm居中，实际均速目标150±3%，差速不足才临时单轮停转；
最近可信道路方向仅辅助，实际超±45°才主动回正至±5°。
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
from dataclasses import dataclass, asdict, replace
import hashlib
import json
import math
from pathlib import Path
import queue
import shutil
import sys
import threading
import time
from typing import Callable
import wave

import audio
import clues
import config
import coverage
import mcu
import mission_ledger
import rfid
import runlog
import test_motion_sequence as sequence

START_SPEED_MMPS = 150
CRUISE_SPEED_MMPS = 150
INITIAL_SPEED_MMPS = 150
INITIAL_TRAVEL_MM = 0.0
POST_TURN_SPEED_MMPS = 91
HOME_SPEED_MMPS = 61
HOME_TRAVEL_MM = 160.0
HOME_INITIAL_BRAKE_S = 0.2
JUNCTION_WAIT_S = 1.0
POST_TURN_WAIT_S = 1.0
TURN_SPEED_MMPS = 91
DEPARTURE_STRATEGIES = {"10mm_turn": 10.0, "40mm_pivot": 40.0}
DEPARTURE_PIVOT_ANGULAR_MRADPS = 1456
DEFAULT_DURATION_S = None
PATROL_FIXED_HEADING_TOLERANCE_DEG = 1.0
PATROL_RECOVERY_EXIT_DEG = 5.0
PATROL_CORRECTION_HEADING_LIMIT_DEG = 45.0
PATROL_SIDE_DEADBAND_MM = 3.0


def turn_recovery_decision(goal_world_deg, world_yaw_deg, yaw_rate_degps, source_age_s,
                           *, brake_tail_deg=0.0, allowed_arc_deg=None, traveled_deg=0.0):
    """Shared arithmetic for a saved endpoint; it never authorizes a UART write."""
    error = goal_world_deg - world_yaw_deg
    margin = max(.5, abs(yaw_rate_degps) * (config.MCU_COMMAND_CONFIRM_TIMEOUT_S
        + max(0.0, source_age_s) + sequence.COMMAND_INTERVAL_S + .05) + brake_tail_deg)
    allowed = abs(error) if allowed_arc_deg is None else allowed_arc_deg
    return dict(error_deg=error, remaining_deg=abs(error), predicted_brake_deg=margin,
                done=abs(error) <= 5.0 + 1e-9, overshot=error < -5.0 - 1e-9,
                should_brake=abs(error) <= margin or traveled_deg >= allowed,
                allowed_arc_deg=allowed)


def cancel_motion_commands_after_brake(link, brake_sequence, sent_at):
    """A complete zero-motion write supersedes pending drive, without inventing ACKs."""
    identity = link.session_nonce, link.boot_id
    for key, record in list(link._pending_commands.items()):
        target = record[5]
        moving = (key[0] is sequence.MessageType.CONTROL_TARGET and any(target[1:3])
                  or key[0] is sequence.MessageType.MOTION_ACTION
                  and target[1] is not sequence.MotionActionCode.BRAKE)
        rejected = record[8] is not None and record[8].status is not sequence.AckStatus.ACCEPTED
        if not moving or rejected or tuple(record[6:8]) != identity or record[0] > sent_at:
            continue
        record[10].update(disposition="cancelled_by_brake", cancelled_by_brake=brake_sequence,
                          cancelled_at=sent_at,
                          original_deadline=record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S,
                          drive_invalidated_by_brake=True)
        link._pending_commands.pop(key, None)
        runlog.emit("motion_command_cancelled_by_brake", "完整BRAKE写入取消待确认非零目标，保留原确认历史",
                    message_type=key[0].name, sequence=key[1], brake_sequence=brake_sequence,
                    original_deadline=record[10]["original_deadline"], cancelled_at=sent_at)


def wait_brake_applied_in_fault(link, number, timeout_s=None):
    """A fault cannot block confirmation of the command that makes the wheels stop."""
    key = sequence.MessageType.MOTION_ACTION, number
    raw = link._action_sends.get(number)
    if raw is None or raw[0].action is not sequence.MotionActionCode.BRAKE:
        raise mcu.McuError("故障停车缺少本次BRAKE命令")
    deadline = raw[1] + link._confirm_timeout(timeout_s)
    identity = raw[3], raw[4]
    while True:
        if not link.connected or (link.session_nonce, link.boot_id) != identity:
            raise mcu.McuError("BRAKE确认期间MCU会话变化")
        record = link._command_ledger.get(key)
        if record is not None and record[8] is not None and record[8].status is not sequence.AckStatus.ACCEPTED:
            raise mcu.McuError(f"BRAKE seq={number}被拒：{record[8].status.name}")
        if record is not None and tuple(record[6:8]) == identity:
            state, state_at, ack, ack_at = record[2], record[3], record[8], record[9]
            if (state is not None and ack is not None and state_at is not None and ack_at is not None
                    and (state.session_nonce, state.boot_id) == identity
                    and (ack.session_nonce, ack.boot_id) == identity
                    and ack.acknowledged_type is key[0] and ack.acknowledged_sequence == number
                    and state.state_flags & sequence.StateFlag.APPLIED_CONTROL_PRESENT
                    and state.applied_control_sequence == number
                    and not state.target_linear_mms and not state.target_angular_mrads
                    and raw[1] - 1e-9 <= min(state_at, ack_at)
                    and max(state_at, ack_at) <= deadline + 1e-9):
                # The shared pump deliberately does not release drive under a
                # fault latch. Preserve this BRAKE's real ACK/STATE witnesses.
                link._confirmed_commands[key] = state
                link._confirmed_send_times[key] = raw[1]
                link._confirmed_evidence_times[key] = max(state_at, ack_at)
                link._pending_commands.pop(key, None)
        applied, evidence = link._confirmed_commands.get(key), link._confirmed_evidence_times.get(key)
        if (applied is not None and evidence is not None and evidence <= deadline + 1e-9
                and not applied.target_linear_mms and not applied.target_angular_mrads):
            return applied
        if time.monotonic() >= deadline:
            link._record_confirmation_timeout(key, record, deadline, time.monotonic())
            raise mcu.McuError(f"BRAKE seq={number}未在确认窗口内应用零目标")
        link.pump()
        if key not in link._confirmed_commands:
            time.sleep(.005)


class FaultStopTracker:
    """Fresh BRAKE/ODOM/gyro evidence may prove parking while a known stop is latched."""

    def __init__(self, link, brake_sequence):
        self.link, self.brake_sequence = link, brake_sequence
        self.tracker = sequence._StopTracker(link, source_barrier=_brake_evidence_at(link))

    def update(self, samples=None, trace_samples=None):
        link, state = self.link, self.link.state
        known_stops = (sequence.WireSafetyStopReason.NONE, sequence.WireSafetyStopReason.BOOT,
                       sequence.WireSafetyStopReason.HOST, sequence.WireSafetyStopReason.LEASE_EXPIRED,
                       sequence.WireSafetyStopReason.ENCODER_FAULT)
        if (state is None or not link._sample_is_fresh("state")
                or state.odom_status != 0 or not state.state_flags & sequence.StateFlag.ODOM_VALID
                or state.fault_bits & ~(sequence._ignored_runtime_fault_bits(link) | (1 << 20))
                or state.fault_bits & (1 << 20) and (link.diagnostics is None
                    or link.diagnostics.last_stop_reason not in known_stops)
                or sequence._gyro_health_reason(link) is not None
                or not state.state_flags & sequence.StateFlag.APPLIED_CONTROL_PRESENT
                or state.applied_control_sequence != self.brake_sequence
                or state.target_linear_mms or state.target_angular_mrads
                or state.state_flags & sequence.StateFlag.TURN_ACTIVE):
            self.tracker.quiet_since = None
            return None
        if samples is None:
            drain = getattr(link, "drain_state_samples", None)
            samples = drain() if callable(drain) else []
            if not samples:
                samples = [(state, link._sample_time("state"))]
        invalid = [at for sample, at in samples if at > self.tracker.source_barrier + 1e-9
            and ((sample.session_nonce, sample.boot_id) != (link.session_nonce, link.boot_id)
                 or sample.odom_status != 0 or not sample.state_flags & sequence.StateFlag.ODOM_VALID
                 or sample.fault_bits & ~(sequence._ignored_runtime_fault_bits(link) | (1 << 20))
                 or not sample.state_flags & sequence.StateFlag.APPLIED_CONTROL_PRESENT
                 or sample.applied_control_sequence != self.brake_sequence
                 or sample.target_linear_mms or sample.target_angular_mrads
                 or sample.state_flags & sequence.StateFlag.TURN_ACTIVE)]
        if invalid:
            self.tracker.source_barrier = max(invalid)
            self.tracker.quiet_since = None
            self.tracker.count = self.tracker.observed_ms = 0
        if not self.tracker.update(link, samples, trace_samples):
            return None
        at = link._sample_time("state")
        world = getattr(link, "patrol_world_yaw_deg", None)
        return dict(at=at, positions=(state.left_mm, state.right_mm),
                    raw_yaw=state.yaw_tenth_deg / 10.0,
                    world_yaw=state.yaw_tenth_deg / 10.0 if world is None else world,
                    boot_id=state.boot_id, session_nonce=state.session_nonce)


def _patrol_zero_transition(previous, proposed):
    return (sequence._wheel_transition_requires_stop(previous, proposed)
            or any(previous) and any(proposed)
            and tuple(wheel == 0 for wheel in previous) != tuple(wheel == 0 for wheel in proposed))



_PATROL_CONTEXT_UNSET = object()


def _patrol_turn_sign(action):
    if action in (sequence.MotionActionCode.TURN_LEFT, sequence.MotionActionCode.UTURN_LEFT):
        return 1
    if action is sequence.MotionActionCode.TURN_RIGHT:
        return -1
    raise mcu.McuError("阶段一动作不是合法转向")


def _patrol_turn_wheels(action, speed_mmps=91):
    if action is sequence.MotionActionCode.UTURN_LEFT:
        return -speed_mmps, speed_mmps
    return ((0, speed_mmps) if _patrol_turn_sign(action) > 0
            else (speed_mmps, 0))


class _PatrolLimit(Exception):
    """An explicitly supplied duration requests a final stop."""


def parse_stage1_patrol_no(block_data: bytes) -> int | None:
    """Use the shared patrol-card decoder."""
    return rfid.parse_patrol_no(block_data)


def _stage1_rfid_event(event: rfid.RfidEvent) -> rfid.RfidEvent:
    """Only replace the shared decoder's number error on complete success."""
    number = parse_stage1_patrol_no(event.block_data)
    if (number is None or not isinstance(event.uid, bytes) or len(event.uid) != 4
            or type(event.block_no) is not int or event.block_no != config.RFID_PATROL_BLOCK
            or type(event.reader_status) is not int or event.reader_status != 0):
        return event
    index = config.RFID_PATROL_BYTE_INDEX
    number_error = (f"RFID invalid patrol number: block={event.block_no} "
                    f"offset={index} value=0x{event.block_data[index]:02X}")
    if event.error is not None and event.error != number_error:
        return event
    return replace(event, patrol_no=number, error=None, transaction_kind="card")


class PatrolRfidReader(rfid.RfidReader):
    """Keep the shared wire protocol and polling; adapt stage-one card data."""

    def read_patrol_card(self) -> rfid.RfidEvent:
        event = _stage1_rfid_event(super().read_patrol_card())
        self._last_read_error = event.error
        return event

    def _log_transaction(self, command, request, raw, frame, event):
        super()._log_transaction(command, request, raw, frame, _stage1_rfid_event(event))


def _record_stage1_rfid_event(ledger, event, step, distance_mm):
    """Recheck raw block evidence without changing the shared ledger parser."""
    event = _stage1_rfid_event(event)
    number = parse_stage1_patrol_no(event.block_data)
    if (event.error is not None or number is None
            or not isinstance(event.uid, bytes) or len(event.uid) != 4
            or type(event.block_no) is not int or event.block_no != config.RFID_PATROL_BLOCK
            or type(event.reader_status) is not int or event.reader_status != 0
            or rfid.parse_patrol_no(event.block_data) is not None):
        return ledger.record_rfid_event(event, step, distance_mm)
    record = mission_ledger.RfidRecord(
        uid_hex=event.uid.hex(), patrol_no=number,
        monotonic_s=float(event.monotonic_s), road_id=step.road_id,
        step_index=step.index, distance_mm=distance_mm,
        block_no=event.block_no, block_data_hex=event.block_data.hex(),
        reader_status=event.reader_status,
    )
    ledger.rfid_records.append(record)
    ledger.confirmed_patrol_nos.add(number)
    ledger.confirmed_patrol_uids.setdefault(number, set()).add(record.uid_hex)
    return record


def _validate_route(
    field_map: coverage.FieldMap,
    steps: list[coverage.Step],
    *,
    expected_counts: Counter | None = None,
) -> None:
    """Check physical continuity and coverage without inferring any arrival."""
    if not steps:
        raise coverage.CoverageError("中国邮差路线为空")
    current = field_map.start_node_id
    visited = {current}
    for index, step in enumerate(steps):
        road = field_map.roads.get(step.road_id)
        if (step.index != index or step.from_node != current or road is None
                or {step.from_node, step.to_node} != {road.a, road.b}):
            raise coverage.CoverageError(f"中国邮差路线第 {index} 段不连续或道路端点不符")
        coverage.direction_between(field_map, step.from_node, step.to_node)
        current = step.to_node
        visited.add(current)
    actual_counts = Counter(step.road_id for step in steps)
    if current != field_map.start_node_id:
        raise coverage.CoverageError("中国邮差路线未回到出发区")
    if set(actual_counts) != set(field_map.roads) or visited != set(field_map.nodes):
        raise coverage.CoverageError("中国邮差路线未覆盖全部道路和节点")
    if expected_counts is not None and actual_counts != expected_counts:
        raise coverage.CoverageError("转弯优化改变了中国邮差最优边遍历次数")


def _minimum_turn_euler_steps(
    field_map: coverage.FieldMap,
    optimal_steps: list[coverage.Step],
) -> list[coverage.Step]:
    """Exactly minimize nonstraight actions for the supplied shortest multiset.

    Pairing the incident edge instances at every node decomposes an Eulerian
    multigraph into circuits. Rollback union-find rejects a closed subcircuit
    before it contains every edge. Independent node minima bound the remaining
    cost; sorted options make equal-cost choices deterministic. One pairing at
    the start is the tour boundary and incurs no arrival-to-departure action.
    """
    _validate_route(field_map, optimal_steps)
    start = field_map.start_node_id
    required_counts = Counter(step.road_id for step in optimal_steps)
    edges = [
        (field_map.roads[road_id].a, field_map.roads[road_id].b, road_id)
        for road_id in sorted(required_counts)
        for _ in range(required_counts[road_id])
    ]
    incident = {node: [] for node in sorted(field_map.nodes)}
    outward = {}
    for edge_id, (a, b, _road_id) in enumerate(edges):
        for node, other in ((a, b), (b, a)):
            incident[node].append(edge_id)
            outward[node, edge_id] = coverage.direction_between(field_map, node, other)

    options_by_node = []
    for node, edge_ids in incident.items():
        if not edge_ids or len(edge_ids) % 2:
            raise coverage.CoverageError("最短中国邮差多重图存在孤立或奇度节点")
        options = []

        def match(remaining, pairs, costs):
            if not remaining:
                pair_tuple = tuple(pairs)
                if node == start:
                    for boundary, omitted in zip(pair_tuple, costs):
                        options.append((sum(costs) - omitted, pair_tuple, boundary))
                else:
                    options.append((sum(costs), pair_tuple, (-1, -1)))
                return
            first = remaining[0]
            incoming = tuple(-value for value in outward[node, first])
            for offset in range(1, len(remaining)):
                second = remaining[offset]
                cost = int(coverage.turn_kind(incoming, outward[node, second])
                           != coverage.TURN_STRAIGHT)
                match(remaining[1:offset] + remaining[offset + 1:],
                      pairs + [(first, second)], costs + [cost])

        match(tuple(edge_ids), [], [])
        options_by_node.append((node, sorted(options)))

    edge_count = len(edges)
    parent, sizes, history = list(range(edge_count)), [1] * edge_count, []

    def root(edge_id):
        while parent[edge_id] != edge_id:
            edge_id = parent[edge_id]
        return edge_id

    def connect(first, second):
        first, second = root(first), root(second)
        if first == second:
            return False
        if sizes[first] < sizes[second]:
            first, second = second, first
        history.append((second, first, sizes[first]))
        parent[second] = first
        sizes[first] += sizes[second]
        return True

    def rollback(snapshot):
        while len(history) > snapshot:
            child, ancestor, old_size = history.pop()
            parent[child], sizes[ancestor] = child, old_size

    lower_bounds = [0] * (len(options_by_node) + 1)
    for index in range(len(options_by_node) - 1, -1, -1):
        lower_bounds[index] = lower_bounds[index + 1] + options_by_node[index][1][0][0]
    chosen, best_options, best_cost = [], None, math.inf

    def search(index, cost, pair_count):
        nonlocal best_options, best_cost
        if cost + lower_bounds[index] >= best_cost:
            return
        if index == len(options_by_node):
            best_cost, best_options = cost, tuple(chosen)
            return
        node, options = options_by_node[index]
        for option_cost, pairs, boundary in options:
            if cost + option_cost + lower_bounds[index + 1] >= best_cost:
                break
            snapshot, used_pairs, valid = len(history), pair_count, True
            for first, second in pairs:
                if not connect(first, second) and used_pairs != edge_count - 1:
                    valid = False
                    break
                used_pairs += 1
            if valid:
                chosen.append((node, pairs, boundary))
                search(index + 1, cost + option_cost, used_pairs)
                chosen.pop()
            rollback(snapshot)
            if best_cost == lower_bounds[0]:
                return

    search(0, 0, 0)
    if best_options is None:
        raise coverage.CoverageError("最短中国邮差多重图无法构成连续闭合路线")
    partners, first_edge = {}, None
    for node, pairs, boundary in best_options:
        for first, second in pairs:
            partners[node, first], partners[node, second] = second, first
        if node == start:
            first_edge = boundary[0]
    current, edge_id, used, result = start, first_edge, set(), []
    for index in range(edge_count):
        if edge_id in used or edge_id is None:
            raise coverage.CoverageError("最少转弯路线重建发生独立闭环")
        used.add(edge_id)
        a, b, road_id = edges[edge_id]
        other = b if current == a else a
        result.append(coverage.Step(index, current, other, road_id,
                                    field_map.roads[road_id].dark_tunnel))
        current, edge_id = other, partners[other, edge_id]
    _validate_route(field_map, result, expected_counts=required_counts)
    if route_statistics(field_map, result)["turn_actions"] != best_cost:
        raise coverage.CoverageError("最少转弯路线重建成本校验失败")
    return result


def route_statistics(field_map: coverage.FieldMap, steps: list[coverage.Step]) -> dict:
    """Describe the traversed route; no turn is counted after the final stop."""
    turns = Counter(
        coverage.turn_kind(
            coverage.direction_between(field_map, before.from_node, before.to_node),
            coverage.direction_between(field_map, after.from_node, after.to_node),
        )
        for before, after in zip(steps, steps[1:])
    )
    right_angles = turns[coverage.TURN_LEFT] + turns[coverage.TURN_RIGHT]
    return {
        "length_m": sum(field_map.roads[step.road_id].length_m for step in steps),
        "segments": len(steps), "roads": len({step.road_id for step in steps}),
        "nodes": len({node for step in steps for node in (step.from_node, step.to_node)}),
        "straight": turns[coverage.TURN_STRAIGHT], "turns_90": right_angles,
        "uturns": turns[coverage.TURN_UTURN],
        "turn_actions": right_angles + turns[coverage.TURN_UTURN],
    }


def plan_route(field_map: coverage.FieldMap | None = None) -> list[coverage.Step]:
    """Minimize turns on the shared planner's shortest closed edge multiset."""
    field_map = field_map or coverage.load_map()
    if field_map.start_node_id not in field_map.nodes:
        raise coverage.CoverageError("最短中国邮差规划的起点节点不存在")
    for road in field_map.roads.values():
        if (type(road.length_m) not in (int, float) or not math.isfinite(road.length_m)
                or road.length_m <= 0):
            raise coverage.CoverageError("最短中国邮差规划要求每条道路具有有效正长度")
        if road.a not in field_map.nodes or road.b not in field_map.nodes:
            raise coverage.CoverageError("最短中国邮差规划的道路端点不存在")
        coverage.direction_between(field_map, road.a, road.b)
    endpoint_pairs = [frozenset((road.a, road.b)) for road in field_map.roads.values()]
    if len(set(endpoint_pairs)) != len(endpoint_pairs):
        raise coverage.CoverageError("当前共享地图规划器不支持同端点的平行道路")
    # Shared shortest-path tie choices follow dictionary iteration. Normalize
    # the copy rather than changing the shared planner or the caller's map.
    field_map = coverage.FieldMap(
        {node: field_map.nodes[node] for node in sorted(field_map.nodes)},
        {road: field_map.roads[road] for road in sorted(field_map.roads)},
        field_map.start_node_id,
    )
    field_map.adjacency = coverage._build_adjacency(field_map)
    coverage._check_connected(field_map)
    odd = coverage._odd_nodes(field_map.adjacency)
    if len(odd) > 8:
        raise coverage.CoverageError("奇度节点超过共享规划器精确匹配范围，禁止把 DFS 回退称为最短路线")
    distances, _next = coverage._floyd_warshall(field_map, field_map.adjacency)
    pairs = coverage._min_perfect_matching(odd, distances)
    optimum = (sum(road.length_m for road in field_map.roads.values())
               + sum(distances[a][b] for a, b in pairs))
    if not math.isfinite(optimum):
        raise coverage.CoverageError("地图不连通，不能计算最短中国邮差路线")
    optimal_steps = coverage.plan_coverage_route(field_map)
    _validate_route(field_map, optimal_steps)
    actual = sum(field_map.roads[step.road_id].length_m for step in optimal_steps)
    if not math.isclose(actual, optimum, rel_tol=1e-9, abs_tol=1e-9):
        raise coverage.CoverageError("共享规划器返回了非最短回退路线，本阶段拒绝执行")
    return _minimum_turn_euler_steps(field_map, optimal_steps)


class PatrolAudio:
    """Stage-one worker owns filesystem/player calls; motion only submits/consumes."""

    def __init__(self, device=None, mode=None):
        # Each pass is a small path request; retain FIFO rather than drop a pass
        # when playback is slower than RFID. Results likewise must not overflow.
        self._requests = queue.Queue()
        self._events = queue.Queue()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread = None
        self._current = None
        self._pending = {}
        self._terminal = set()
        self._closed = False
        self._deadline = None
        self.last_request_id = 0
        self.last_error = None
        self.last_returncode = None
        self._current_stderr = ""
        self.completed_count = self.failed_count = self.started_count = 0
        owner = self

        class Backend(audio.Audio):
            def _close_stderr(inner):
                detail = super()._close_stderr()
                if owner._current is not None:
                    owner._current_stderr = detail
                return detail

            def _emit(inner, event, message, **fields):
                super()._emit(event, message, terminal=False, **fields)
                owner._backend_event(inner, event, message, fields)

        self._backend = Backend(device=device, mode=mode)
        self.device, self.mode = self._backend.device, self._backend.mode

    def _publish(self, event, request_id, path, message, returncode=None, stderr=None):
        with self._lock:
            if request_id in self._terminal:
                return
            if event in ("completed", "failed"):
                self._terminal.add(request_id)
                self._pending.pop(request_id, None)
            if event == "completed":
                self.completed_count += 1
                self.last_returncode = returncode
            elif event == "failed":
                self.failed_count += 1
                self.last_error = message
            elif event == "started":
                self.started_count += 1
        record = dict(event=event, request_id=request_id, path=str(path),
                      message=message, returncode=returncode, stderr=stderr,
                      device=self.device)
        self._events.put_nowait(record)

    def _backend_event(self, backend, event, message, fields):
        current = self._current
        kind = {"audio_started": "started", "audio_completed": "completed",
                "audio_failed": "failed"}.get(event)
        if current is None or kind is None:
            return
        request_id, path = current
        if fields.get("path") is not None and str(fields["path"]) != str(path):
            self._publish("failed", request_id, path, "播放器事件路径与当前请求不一致")
            return
        returncode = backend.last_returncode if kind in ("completed", "failed") else None
        if kind == "completed" and returncode != 0:
            kind, message = "failed", "播放器没有提供成功返回码，不能确认播报完成"
        self._publish(kind, request_id, path, message, returncode,
                      self._current_stderr if kind in ("completed", "failed") else None)

    def _ensure_worker(self):
        with self._lock:
            if self._closed or self._thread is not None:
                return
            self._thread = threading.Thread(target=self._run, name="stage1-audio", daemon=True)
            self._thread.start()

    def play(self, wav_path):
        path = Path(wav_path)
        with self._lock:
            self.last_request_id += 1
            request_id = self.last_request_id
            self._pending[request_id] = path
            closed = self._closed
        if closed or self.mode != "aplay":
            self._publish("failed", request_id, path,
                          "巡逻音频已关闭" if closed else f"巡逻音频模式不可用：{self.mode}")
            return False
        # Publish before handing the job to the worker: started cannot precede queued.
        self._publish("queued", request_id, path, "巡逻音频已进入后台请求队列")
        self._requests.put_nowait((request_id, path))
        self._ensure_worker()
        self._wake.set()
        return True

    def poll(self):
        with self._lock:
            return bool(self._pending) or self._current is not None

    def consume_event(self):
        try:
            return self._events.get_nowait()
        except queue.Empty:
            return None

    def set_deadline(self, deadline_monotonic):
        self._deadline = deadline_monotonic
        self._wake.set()

    def _run(self):
        try:
            while not self._stop.is_set():
                self._backend.set_deadline(self._deadline)
                if self._current is not None:
                    if not self._backend.poll():
                        request_id, path = self._current
                        if request_id not in self._terminal:
                            self._publish("failed", request_id, path, "播放器结束但没有逐条结果")
                        self._current = None
                else:
                    try:
                        self._current = self._requests.get_nowait()
                    except queue.Empty:
                        pass
                    else:
                        request_id, path = self._current
                        self._current_stderr = ""
                        self._backend.last_returncode = None
                        if not self._backend.play(path):
                            self._publish("failed", request_id, path,
                                          self._backend.last_error or "后台播放器拒绝请求")
                            self._current = None
                self._wake.wait(0.005)
                self._wake.clear()
        except Exception as exc:
            self.last_error = f"巡逻音频后台异常：{exc}"
            with self._lock:
                self._closed = True
            runlog.exception("stage1_audio_worker_failed", self.last_error, exc, terminal=False)
        finally:
            try:
                self._backend.close()
            except Exception as exc:
                self.last_error = f"巡逻音频后台关闭失败：{exc}"
                runlog.exception("stage1_audio_worker_close_failed", self.last_error, exc, terminal=False)
            with self._lock:
                pending = list(self._pending.items())
            for request_id, path in pending:
                self._publish("failed", request_id, path,
                              self.last_error or "巡逻音频在播放完成前关闭")
            self._current = None

    def finish_after_stop(self, timeout_s=None, *, deadline_monotonic=None, on_poll=None):
        if deadline_monotonic is not None:
            self.set_deadline(deadline_monotonic)
        end = None if timeout_s is None else time.monotonic() + max(0.0, timeout_s)
        while True:
            if on_poll is not None:
                on_poll()
            if not self.poll():
                break
            if end is not None and time.monotonic() >= end:
                self.last_error = "停车后巡逻音频收尾超时"
                self.close()
                if on_poll is not None:
                    on_poll()
                return False
            time.sleep(0.005)
        return self.failed_count == 0

    def close(self):
        with self._lock:
            self._closed = True
        self._stop.set()
        self._wake.set()
        thread = self._thread
        if thread is not None:
            thread.join(max(0.0, getattr(config, "IO_CLOSE_TIMEOUT_S", 0.2)))
        if thread is not None and thread.is_alive():
            self.last_error = "巡逻音频后台仍在系统调用中，关闭已请求"
        with self._lock:
            pending = list(self._pending.items())
        for request_id, path in pending:
            self._publish("failed", request_id, path, self.last_error or "巡逻音频已取消")


class PatrolServices:
    """Nonblocking RFID/audio service for every motion and stationary phase.

    The caller supplies existing components, a ledger, and the current route
    step.  Display is an optional short-message callback.  Motor stop and strict
    stationary confirmation remain the motion controller's responsibility.
    """

    def __init__(
        self,
        c: dict,
        ledger: mission_ledger.MissionLedger,
        *,
        active_step: coverage.Step | None = None,
        display: Callable[[str], None] | None = None,
        diagnostic: bool = False,
    ) -> None:
        self.c = c
        self.ledger = ledger
        self.active_step = active_step
        self.distance_mm: float | None = None
        self.phase = "启动准备"
        self.display = display
        self.diagnostic = diagnostic
        self.attempted_patrol_nos: set[int] = set()
        self.queued_patrol_nos: set[int] = set()
        self.failures: list[str] = []
        self._reported_failures: set[tuple[str, str]] = set()
        self._rfid_ready = False
        self._polling_started = False
        self._closed = False
        self._finishing = False
        self._polling_requested = False
        self._rfid_retry_at = 0.
        self._rfid_recovery_work = None
        self._rfid_recovery_thread = None
        self._rfid_start_stop_lock = threading.Lock()
        self._event_budget = None
        self._last_audio_failed_count = 0
        self._audio_requests: dict[int, int] = {}
        self._audio_backend_requests: dict[int, int] = {}
        self._audio_paths: dict[str, set[int]] = {}
        self._audio_terminal: set[int] = set()
        self._audio_completed: set[int] = set()
        self.audio_request_outcomes: dict[int, dict] = {}
        self.ledger.stage1_audio_request_outcomes = self.audio_request_outcomes
        if not hasattr(self.ledger, "stage1_announced_patrol_nos"):
            self.ledger.stage1_announced_patrol_nos = set()

    @property
    def all_audio_requests_completed(self) -> bool:
        """Coverage of a number never substitutes for completion of a later pass."""
        return len(self._audio_requests) == len(self._audio_completed)

    def _message(self, message: str) -> None:
        if self.display is None:
            print(message)
            return
        try:
            self.display(message)
        except Exception as exc:
            runlog.exception("stage1_display_failed", "巡逻服务消息显示失败", exc)

    def _failure(self, kind: str, reason: str) -> None:
        key = (kind, reason)
        if key in self._reported_failures:
            return
        self._reported_failures.add(key)
        self.failures.append(reason)
        if kind == "audio":
            self.ledger.record_audio_failure(reason)
        elif kind == "rfid":
            self.ledger.rfid_errors.append(reason)
        event_name = "stage1_audio_service_failed" if kind == "audio" else f"stage1_{kind}_failed"
        runlog.emit(event_name, reason, terminal=False,
                    phase=self.phase, road=getattr(self.active_step, "road_id", None))
        if kind != "rfid" or self.diagnostic:
            self._message(reason)

    def _diagnostic_event(self, event: str, **fields) -> None:
        if self.diagnostic:
            self._message(f"[{event}] " + json.dumps(fields, ensure_ascii=False, default=str))

    def prepare(self) -> bool:
        """Check only patrol audio and open RFID; failures do not drive the car."""
        ok = True
        player = self.c.get("audio")
        mapping = self.c.get("clues")
        if player is None or mapping is None:
            self._failure("audio", "巡逻音频组件或巡逻映射不可用")
            ok = False
        else:
            if (getattr(player, "mode", None) != "aplay"
                    or not getattr(player, "device", None) or shutil.which("aplay") is None):
                self._failure("audio", "巡逻播放器配置或 aplay 不可用")
                ok = False
            for number in range(1, 13):
                try:
                    clue = mapping.patrol(number)
                    path = mapping.patrol_audio_path(number)
                    if (clue is None or clue.number != number
                            or not isinstance(clue.text, str) or not clue.text.strip()
                            or path is None):
                        raise ValueError("巡逻播报映射无效")
                    with wave.open(str(path), "rb") as source:
                        if (source.getframerate() <= 0 or source.getnchannels() <= 0
                                or source.getnframes() <= 0 or not source.readframes(1)):
                            raise ValueError("WAV 没有有效音频样本")
                except Exception as exc:
                    self._failure("audio", f"巡逻点 {number:02d} 音频准备失败：{exc}")
                    ok = False
        reader = self.c.get("rfid")
        try:
            if reader is None:
                raise RuntimeError("RFID 组件不可用")
            reader.open()
            self._rfid_ready = True
        except Exception as exc:
            self._failure("rfid", f"RFID 初始化失败：{exc}")
            ok = False
        return ok

    def set_step(
        self,
        step: coverage.Step,
        *,
        distance_mm: float | None = None,
        phase: str | None = None,
    ) -> None:
        """Update route context after a real localization event, without confirming it."""
        self.active_step = step
        self.distance_mm = distance_mm
        if phase is not None:
            self.phase = phase

    def start_polling(self) -> bool:
        with self._rfid_start_stop_lock:
            if self._closed or self._finishing:
                return False
            self._polling_requested = True
            if not self._rfid_ready:
                return False
            if self.active_step is None:
                raise ValueError("启动 RFID 轮询前必须提供当前道路上下文")
            if self._polling_started:
                return True
            try:
                self.c["rfid"].start_polling()
                self._polling_started = True
                return True
            except Exception as exc:
                self._failure("rfid", f"RFID 后台轮询启动失败：{exc}")
                return False

    def _poll_rfid_recovery(self):
        work = self._rfid_recovery_work
        if work is not None:
            if not work["done"].is_set():
                return
            self._rfid_recovery_work = None
            if "error" in work:
                self._failure("rfid", "RFID 后台恢复失败：" + work["error"])
            else:
                self._rfid_ready = True
                self._polling_started = work["polling"]
        reader = self.c.get("rfid")
        owner = getattr(reader, "_thread", None)
        needs_restart = (reader is None or not self._rfid_ready
                         or owner is None or not owner.is_alive()
                         or getattr(reader, "transport_consecutive_errors", 0) >= 3)
        now = time.monotonic()
        if (not self._polling_requested or self._closed or self._finishing
                or not needs_restart or now < self._rfid_retry_at):
            return
        self._rfid_retry_at = now + 1.
        work = {"done": threading.Event(), "polling": False}
        self._rfid_recovery_work = work
        self._rfid_recovery_thread = threading.Thread(
            target=rfid.recover_polling_owner,
            args=(self.c, reader, work, lambda: self._closed or self._finishing,
                  self._rfid_start_stop_lock),
            name="stage1-rfid-recovery", daemon=True)
        try:
            self._rfid_recovery_thread.start()
        except Exception as exc:
            work["error"] = str(exc)
            work["done"].set()
            self._failure("rfid", "RFID 后台恢复线程启动失败：" + str(exc))

    def _event_available(self):
        budget = self._event_budget
        return budget is None or budget[0] < 16 and time.monotonic() < budget[1]

    def _event_consumed(self):
        if self._event_budget is not None:
            self._event_budget[0] += 1

    def _collect_audio_failures(self) -> None:
        player = self.c.get("audio")
        if player is None:
            return
        count = getattr(player, "failed_count", 0)
        if not isinstance(count, int) or count < 0:
            return
        if count > self._last_audio_failed_count:
            error = getattr(player, "last_error", None) or "播放器未提供失败详情"
            self._failure("audio", f"巡逻播报累计 {count} 次失败：{error}")
        self._last_audio_failed_count = count

    def _consume_audio_events(self) -> None:
        player = self.c.get("audio")
        consume = getattr(player, "consume_event", None)
        if not callable(consume):
            return
        while self._event_available():
            try:
                event = consume()
            except Exception as exc:
                self._failure("audio", f"巡逻音频事件消费失败：{exc}")
                return
            if event is None:
                return
            self._event_consumed()
            if not isinstance(event, dict):
                self._failure("audio", "巡逻音频事件不是逐条结果记录")
                continue
            backend_id, path = event.get("request_id"), event.get("path")
            kind = event.get("event")
            request_id = None
            if isinstance(backend_id, int) and not isinstance(backend_id, bool):
                request_id = self._audio_backend_requests.get(backend_id)
            elif backend_id is None and isinstance(path, (str, Path)):
                requests = self._audio_paths.get(str(path), set())
                if len(requests) == 1:
                    request_id = next(iter(requests))
            number = self._audio_requests.get(request_id)
            if number is None or kind not in ("queued", "started", "completed", "failed"):
                runlog.emit("stage1_audio_event_unbound", "音频事件无法绑定唯一一次巡逻播报，不授予完成", terminal=False,
                            audio_event=event)
                continue
            if isinstance(path, (str, Path)) and request_id not in self._audio_paths.get(str(path), set()):
                self._failure("audio", f"巡逻点 {number:02d} 播放事件路径不一致")
                continue
            if request_id in self._audio_terminal:
                runlog.emit("stage1_audio_late_event", "已结束播报的后续事件不改变结果", terminal=False,
                            patrol_no=number, request_id=request_id, audio_event=kind)
                continue
            returncode = event.get("returncode")
            if kind == "completed":
                if not isinstance(returncode, int) or isinstance(returncode, bool) or returncode != 0:
                    kind = "failed"
                    event["message"] = "播放器未提供成功返回码，不能确认完成"
                else:
                    self.ledger.stage1_announced_patrol_nos.add(number)
                    self._audio_terminal.add(request_id)
                    self._audio_completed.add(request_id)
            if kind == "failed":
                self._audio_terminal.add(request_id)
                self._failure("audio", f"巡逻点 {number:02d} 请求 {request_id} 播放失败："
                              f"{event.get('message') or '播放器未提供失败详情'}")
            self.audio_request_outcomes[request_id]["event"] = kind
            self.audio_request_outcomes[request_id]["returncode"] = returncode
            self.audio_request_outcomes[request_id]["detail"] = event.get("message")
            self.audio_request_outcomes[request_id]["stderr"] = event.get("stderr")
            self.audio_request_outcomes[request_id]["device"] = event.get("device")
            runlog.emit(f"stage1_audio_{kind}", "巡逻播报逐条结果", terminal=False,
                        patrol_no=number, request_id=request_id, path=str(path),
                        backend_request_id=backend_id,
                        returncode=returncode, stderr=event.get("stderr"),
                        device=event.get("device"), detail=event.get("message"), phase=self.phase,
                        road=getattr(self.active_step, "road_id", None))
            self._diagnostic_event(f"stage1_audio_{kind}", patrol_no=number,
                                   request_id=request_id, backend_request_id=backend_id,
                                   path=str(path), device=event.get("device") or getattr(player, "device", None),
                                   returncode=returncode, stderr=event.get("stderr"),
                                   detail=event.get("message"))

    def _consume_events(self, limit: int | None = 64) -> None:
        reader = self.c.get("rfid")
        if reader is None or self.active_step is None:
            return
        consume = getattr(reader, "consume_event", None)
        if not callable(consume):
            self._failure("rfid", "RFID 组件没有非阻塞事件接口")
            return
        consumed = 0
        while (limit is None or consumed < limit) and self._event_available():
            try:
                event = consume()
            except Exception as exc:
                self._failure("rfid", f"RFID 事件消费失败：{exc}")
                return
            if event is None:
                return
            self._event_consumed()
            consumed += 1
            try:
                record = _record_stage1_rfid_event(
                    self.ledger, event, self.active_step, self.distance_mm)
            except Exception as exc:
                self._failure("rfid", f"RFID 事件记录失败：{exc}")
                continue
            runlog.emit("rfid_event", "读卡结果", terminal=False, uid=record.uid_hex,
                        patrol_no=record.patrol_no, error=record.error,
                        block_no=record.block_no, block_data_hex=record.block_data_hex,
                        reader_status=record.reader_status,
                        monotonic_s=record.monotonic_s, road=record.road_id,
                        step=record.step_index, distance_mm=record.distance_mm,
                        phase=self.phase)
            raw_block = getattr(event, "block_data", None)
            self._diagnostic_event("stage1_rfid_block", uid=record.uid_hex,
                                   block_no=record.block_no, block_hex=record.block_data_hex,
                                   first_byte_hex=(f"{raw_block[config.RFID_PATROL_BYTE_INDEX]:02X}"
                                       if isinstance(raw_block, bytes)
                                       and 0 <= config.RFID_PATROL_BYTE_INDEX < len(raw_block) else None),
                                   patrol_no=record.patrol_no, reader_status=record.reader_status,
                                   error=record.error)
            if record.error is not None or record.patrol_no is None:
                self._failure("rfid", f"RFID 事件无效：{record.error or '没有合法巡逻号'}")
                continue
            number = record.patrol_no
            self.attempted_patrol_nos.add(number)
            request_id = len(self._audio_requests) + 1
            self._audio_requests[request_id] = number
            self.audio_request_outcomes[request_id] = dict(
                patrol_no=number, uid=record.uid_hex, event="submitted", path=None,
                backend_request_id=None)
            mapping = self.c.get("clues")
            player = self.c.get("audio")
            path = None
            try:
                clue = mapping.patrol(number) if mapping is not None else None
                path = mapping.patrol_audio_path(number) if mapping is not None else None
                if clue is None or path is None or player is None:
                    raise RuntimeError("巡逻播报映射、音频路径或播放器不可用")
                self._message(clue.text)
                self._audio_paths.setdefault(str(path), set()).add(request_id)
                self.audio_request_outcomes[request_id]["path"] = str(path)
                accepted = player.play(path)
                backend_id = getattr(player, "last_request_id", None)
                if isinstance(backend_id, int) and not isinstance(backend_id, bool):
                    if backend_id in self._audio_backend_requests:
                        raise RuntimeError("播放器重复使用请求编号，无法区分本次经过")
                    self._audio_backend_requests[backend_id] = request_id
                    self.audio_request_outcomes[request_id]["backend_request_id"] = backend_id
                if not accepted:
                    raise RuntimeError(getattr(player, "last_error", None) or "播报未能入队")
                self.queued_patrol_nos.add(number)
                runlog.emit("stage1_audio_submitted", "本次经过的完整读块编号已提交播报请求", terminal=False,
                            patrol_no=number, request_id=request_id, path=str(path),
                            device=getattr(player, "device", None), phase=self.phase)
                self._diagnostic_event("stage1_audio_submitted", patrol_no=number,
                                       uid=record.uid_hex, request_id=request_id,
                                       backend_request_id=backend_id, path=str(path),
                                       device=getattr(player, "device", None))
            except Exception as exc:
                self._audio_terminal.add(request_id)
                self.audio_request_outcomes[request_id]["event"] = "failed"
                self.audio_request_outcomes[request_id]["detail"] = str(exc)
                self._failure("audio", f"巡逻点 {number:02d} 请求 {request_id} 播报提交失败：{exc}")
                runlog.emit('stage1_audio_failed', '本次经过播报请求提交失败', terminal=False,
                            patrol_no=number, request_id=request_id,
                            path=str(path) if path is not None else None,
                            reason=str(exc), returncode=None, phase=self.phase)
                self._diagnostic_event("stage1_audio_failed", patrol_no=number,
                                       request_id=request_id, path=str(path) if path else None,
                                       device=getattr(player, "device", None),
                                       returncode=None, stderr=None, detail=str(exc))

    def tick(
        self,
        *,
        step: coverage.Step | None = None,
        distance_mm: float | None | object = _PATROL_CONTEXT_UNSET,
        phase: str | None = None,
    ) -> None:
        """Call from each control/ACK/turn/settle/sample wait iteration."""
        if self._closed:
            return
        next_distance = (self.distance_mm if distance_mm is _PATROL_CONTEXT_UNSET
                         else distance_mm)
        if step is not None:
            self.set_step(step, distance_mm=next_distance, phase=phase)
        else:
            self.distance_mm = next_distance
            if phase is not None:
                self.phase = phase
        player = self.c.get("audio")
        if player is not None:
            try:
                player.poll()
            except Exception as exc:
                self._failure("audio", f"巡逻播放器轮询失败：{exc}")
        self._event_budget = [0, time.monotonic() + .002]
        self._last_event_budget = self._event_budget
        try:
            self._consume_audio_events()
            self._consume_events()
        finally:
            self._event_budget = None
        self._collect_audio_failures()
        self._poll_rfid_recovery()

    def finish_after_stop(
        self,
        timeout_s: float | None = None,
        *,
        on_poll: Callable[[], object] | None = None,
    ) -> bool:
        """Call only after the motion layer has stopped and confirmed stationary."""
        with self._rfid_start_stop_lock:
            if self._closed:
                return False
            self._finishing = True
            self._polling_requested = False
        self.phase = "停车播报收尾"
        reader = self.c.get("rfid")
        if reader is not None:
            try:
                reader.stop_polling()
                self._polling_started = False
            except Exception as exc:
                self._failure("rfid", f"RFID 轮询停止失败：{exc}")
        player = self.c.get("audio")
        ok = player is not None
        callback_failed = False

        def poll_services():
            nonlocal callback_failed
            previous_budget = getattr(self, "_last_event_budget", None)
            if on_poll is not None:
                try:
                    on_poll()
                except BaseException:
                    callback_failed = True
                    raise
            if getattr(self, "_last_event_budget", None) is previous_budget:
                self.tick(phase="停车播报收尾")

        poll_services()
        end = None if timeout_s is None else time.monotonic() + max(0., timeout_s)
        while True:
            reader = self.c.get("rfid")
            pending_count = getattr(reader, "pending_event_count", None)
            pending = (pending_count() if callable(pending_count)
                       else len(getattr(reader, "events", ())))
            owner = getattr(reader, "_thread", None)
            work = self._rfid_recovery_work
            if (not pending and (owner is None or not owner.is_alive())
                    and (work is None or work["done"].is_set())):
                break
            if end is not None and time.monotonic() >= end:
                self._failure("rfid", "停车后 RFID 读取所有者或遗留事件尚未结束")
                return False
            time.sleep(.005)
            poll_services()
        if player is None:
            self._failure("audio", "停车后无法完成巡逻播报：音频组件不可用")
        else:
            try:
                if isinstance(player, PatrolAudio):
                    finished = player.finish_after_stop(timeout_s=timeout_s, on_poll=poll_services)
                else:
                    # Existing test/backends may implement only the original drain
                    # signature; the production worker always uses the live callback.
                    finished = player.finish_after_stop(timeout_s=timeout_s)
                    poll_services()
                if not finished:
                    self._failure("audio", getattr(player, "last_error", None) or "停车后播报未全部完成")
                    ok = False
            except Exception as exc:
                if callback_failed:
                    raise
                self._failure("audio", f"停车后播报收尾失败：{exc}")
                ok = False
        self._collect_audio_failures()
        while True:
            previous_budget = getattr(self, "_last_event_budget", None)
            if on_poll is not None:
                on_poll()
            consume_budget = (self._last_event_budget
                if getattr(self, "_last_event_budget", None) is not previous_budget
                else [0, time.monotonic() + .002])
            self._event_budget = consume_budget
            consumed_before = consume_budget[0]
            try:
                self._consume_audio_events()
            finally:
                self._event_budget = None
            if (consume_budget[0] == consumed_before and consume_budget[0] < 16
                    and time.monotonic() < consume_budget[1]):
                break
        missing = set(self._audio_requests) - self._audio_completed
        if missing:
            self._failure("audio", f"缺少逐请求成功播放证据，请求编号：{sorted(missing)}")
        return ok and not missing and not self.ledger.audio_failures

    def stop_and_drain(
        self,
        stop_callback: Callable[[], object],
        *,
        timeout_s: float | None = None,
    ) -> bool:
        """Delegate motor stop to the original controller, then finish services."""
        try:
            if stop_callback() is False:
                raise RuntimeError("运动控制器未确认停车")
        except Exception as exc:
            reason = f"巡逻结束停车失败：{exc}"
            self.ledger.note_terminal(reason)
            runlog.exception("stage1_stop_failed", reason, exc)
            self._message(reason)
            self.close()
            return False
        return self.finish_after_stop(timeout_s)

    def close(self) -> None:
        with self._rfid_start_stop_lock:
            if self._closed:
                return
            self._closed = True
        for name in ("rfid", "audio"):
            if (name == "rfid" and self._rfid_recovery_work is not None
                    and not self._rfid_recovery_work["done"].is_set()):
                continue
            component = self.c.get(name)
            if component is None:
                continue
            try:
                component.close()
            except Exception as exc:
                self._failure(name, f"巡逻 {name} 组件关闭失败：{exc}")
        self._consume_audio_events()
        self._collect_audio_failures()
        self._consume_audio_events()
        self._closed = True


class PatrolCenteringController(sequence.CenteringController):
    """Patrol-only startup heading control and moving reference adoption."""

    PATROL_ANGULAR_LIMIT_MRADPS = 1568
    PATROL_HEADING_LIMIT_DEG = 45.0
    PATROL_SPEED_TOLERANCE = .03
    # Ordinary roads keep the common speed; real laser trend anticipates
    # the correction and gyro damping unloads its angular response.
    PATROL_LASER_YAW_GAIN_PER_S = 2.1
    PATROL_LARGE_ERROR_LASER_YAW_GAIN_PER_S = 3.7
    PATROL_LASER_TREND_HORIZON_S = 1.0
    # Initial low-speed laser response; gyro recovery keeps the shared gain.
    PATROL_ALIGNMENT_LASER_GAIN_PER_S = .6
    # Angular anticipation only; no body-clearance model gates patrol ALIGN.
    PATROL_BRAKE_TAIL_S = .2

    def __init__(self, now=None, *, settings=None):
        self.settings = None
        self.apply_settings(settings)
        super().__init__(now)

    def apply_settings(self, settings):
        self.settings = settings
        if settings is None:
            self.__dict__.pop("PATROL_LASER_YAW_GAIN_PER_S", None)
            self.__dict__.pop("PATROL_LARGE_ERROR_LASER_YAW_GAIN_PER_S", None)
        else:
            self.PATROL_LASER_YAW_GAIN_PER_S = settings.small_error_gain_per_s
            self.PATROL_LARGE_ERROR_LASER_YAW_GAIN_PER_S = settings.large_error_gain_per_s

    def _uses_home_policy(self):
        if getattr(self, "settings", None) is not None:
            return self._patrol_road_policy == "HOME" or self._patrol_home_control
        return self._patrol_home_control or self._ramp_requested == HOME_SPEED_MMPS

    def reset(self, now, source_barrier=None):
        # Stage 2 also imports this adapter; its default control stays shared.
        self._patrol_feedback_generation = getattr(self, "_patrol_feedback_generation", 0) + 1
        self._patrol_cruise_policy = getattr(self, "_patrol_cruise_policy", False)
        self._patrol_pending_cruise_goal = None
        self._patrol_speed_stage_started_at = None
        self._patrol_speed_stage_waiting_applied = None
        self._patrol_correction_phase = "HOLD"
        self._patrol_alignment_targets = set()
        self._patrol_alignment_hold = None
        self._patrol_cruise_hold_goal = None
        self._patrol_cruise_hold_sources = None
        self._patrol_missing_hold_goal = None
        self._patrol_home_control = False
        self.obstacle_approach_speed_cap = getattr(self, "obstacle_approach_speed_cap", None)
        self.patrol_road_speed_cap = getattr(self, "patrol_road_speed_cap", None)
        self._patrol_formal_reference = None
        self._patrol_centered_cycles = 0
        self._patrol_heading_recovery = False
        self._patrol_road_policy = getattr(self, "_patrol_road_policy", "ORDINARY")
        self._patrol_reference_samples = deque()
        self._patrol_reference_key = None
        super().reset(now, source_barrier=source_barrier)

    def enable_patrol_cruise_policy(self):
        self._patrol_cruise_policy = True

    def set_patrol_road_policy(self, policy):
        if policy not in ("START", "SHORT", "HOME", "ORDINARY"):
            raise ValueError("阶段一道路控制策略非法")
        self._patrol_road_policy = policy
        self._patrol_reference_samples.clear()
        self._patrol_reference_key = None
        self._patrol_heading_recovery = False

    @property
    def patrol_heading_limit_deg(self):
        return 20.0 if self._patrol_road_policy == "SHORT" else self.PATROL_HEADING_LIMIT_DEG

    @property
    def patrol_speed_tolerance(self):
        return self.PATROL_SPEED_TOLERANCE if self._patrol_road_policy == "ORDINARY" else .05

    def update_recent_road_reference(self, now, *, allowed):
        """Only a full stable, fresh one-second window may move the auxiliary axis."""
        self.refresh_yaw_rate(now)
        psi = self._heading_error()
        if psi is not None and abs(psi) > self.patrol_heading_limit_deg + 1e-9:
            self._patrol_heading_recovery = True
        if (self._patrol_road_policy != "ORDINARY" or not allowed
                or self._patrol_heading_recovery or not self.centered
                or self._pair_at is None or self._last_pair_pose is None
                or self._applied_linear <= 0 or self._applied_angular != 0
                or self._last_sent_angular != 0 or self._last_sent_wheels != sequence._motion_wheels(
                    self._applied_linear, self._applied_angular)
                or abs(self._error_rate) > .5 + 1e-9
                or self._effective_yaw_rate(now) is None
                or abs(self._effective_yaw_rate(now)) > .5 + 1e-9):
            self._patrol_reference_samples.clear()
            return None
        key = self._state_session, tuple(self._paired_keys)
        if key == self._patrol_reference_key:
            return None
        self._patrol_reference_key = key
        at, yaw = self._pair_at, self._last_pair_pose.yaw
        samples = self._patrol_reference_samples
        if samples and at - samples[-1][0] > .5 + 1e-9:
            samples.clear()
        samples.append((at, yaw))
        # Keep one bracketing sample for the complete trailing one-second window.
        while len(samples) > 1 and samples[1][0] <= at - 1.0 + 1e-9:
            samples.popleft()
        if len(samples) < 3 or at - samples[0][0] < 1.0 - 1e-9:
            return None
        reference = self._road_reference
        mean = self._angle(reference + math.fsum(self._angle(value - reference)
                         for _, value in samples) / len(samples))
        # Raw laser qualification and clocks do not change when this auxiliary
        # direction changes. The runner's independent odometry axis stays fixed.
        self._road_reference = self._hold_heading = mean
        self._patrol_formal_reference = mean
        self._patrol_cruise_hold_goal = mean
        self._control_feedback = None
        return mean

    @property
    def formal_reference_deg(self):
        if self._patrol_cruise_policy and self._patrol_formal_reference is not None:
            return self._patrol_formal_reference
        return super().formal_reference_deg

    def record_road_reference(self, mean_deg):
        """Keep learned yaw for the next turn without changing this road's anchor."""
        if (not self._patrol_cruise_policy or isinstance(mean_deg, bool)
                or not isinstance(mean_deg, (int, float)) or not math.isfinite(mean_deg)):
            return False
        self._patrol_formal_reference = self._angle(mean_deg)
        return True

    def _consume_pending_means(self, now):
        if not self._patrol_cruise_policy or self._uses_home_policy():
            return super()._consume_pending_means(now)
        consumed = False
        while self._pending_means:
            batch = self._pending_means[0]
            keys = getattr(batch, "source_keys", None)
            sources = getattr(batch, "source_times", None)
            identity = getattr(batch, "identity", None)
            started = getattr(batch, "started_at", None)
            valid = (batch.complete and batch.left_count > 0 and batch.right_count > 0
                     and identity == self._state_session and keys is not None and sources is not None
                     and isinstance(started, (int, float)) and not isinstance(started, bool)
                     and math.isfinite(started)
                     and abs(batch.ended_at - started - sequence.AVERAGE_INTERVAL_S) <= 1e-6
                     and (self._mean_last_end is None or abs(started - self._mean_last_end) <= 1e-6)
                     and len(keys) == 2 and len(sources) == 2
                     and all(isinstance(key, (int, float)) and not isinstance(key, bool)
                             and math.isfinite(key) for key in keys)
                     and -1e-9 <= now - batch.ended_at <= sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                     and all(sequence._source_key_newer(key, old) for key, old in zip(keys, self._mean_source_keys))
                     and all(isinstance(s, (int, float)) and not isinstance(s, bool) and math.isfinite(s)
                             and 0 <= batch.ended_at - s <= sequence.AVERAGE_INTERVAL_S + sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                             for s in sources)
                     and (self.source_barrier is None or all(s > self.source_barrier + 1e-9 for s in sources))
                     and self._forward_applied_barrier is not None
                     and started >= self._forward_applied_barrier - 1e-9
                     and all(s > self._forward_applied_barrier + 1e-9 for s in sources)
                     and self._road_reference is not None)
            pose = self._pose_at(batch.ended_at)
            if valid and pose is None and self._poses and batch.ended_at > self._poses[-1].at + 1e-9:
                break
            self._pending_means.popleft()
            self._mean_last_end = batch.ended_at
            samples = ([item for item in self._compensated_sources
                        if all(started + 1e-9 < source <= batch.ended_at + 1e-9 for source in item[1])]
                       if isinstance(started, (int, float)) else [])
            # Averaging L/R first and compensating once would lose sensor
            # forward-offset changes while the car turns inside this window.
            self._last_mean_difference = (abs(2.0 * math.fsum(item[3] for item in samples) / len(samples))
                                          if samples else None)
            self._last_mean_heading = (self._angle(pose.yaw - self._road_reference)
                                      if pose is not None and self._road_reference is not None else None)
            if valid:
                self._mean_source_keys = tuple(keys)
            acceptable = (valid and pose is not None
                          and bool(samples)
                          and self._last_mean_difference <= PATROL_SIDE_DEADBAND_MM + 1e-9
                          and abs(self._last_mean_heading) <= self.PATROL_HEADING_LIMIT_DEG + 1e-9)
            self._good_windows = min(2, self._good_windows + 1) if acceptable else 0
            self._mean_reason = ("连续均值窗与固定参考偏角达标" if acceptable else
                                 "空窗、旧源、超龄、车轴补偿均值差或偏角未达标")
            consumed = True
        return consumed

    @property
    def centered(self):
        if not self._patrol_cruise_policy:
            return super().centered
        return bool(self._usable_pair and self._raw_error is not None
                    and 2. * abs(self._raw_error) <= PATROL_SIDE_DEADBAND_MM + 1e-9
                    and self._patrol_centered_cycles >= 2
                    and all(item is not None and -1e-9 <= self._rate_now - item[0] <= .250 + 1e-9
                            for item in self._control_cache))

    def stage_snapshot(self):
        result = super().stage_snapshot()
        if self._patrol_cruise_policy:
            result.update(heading_tolerance_deg=self.PATROL_HEADING_LIMIT_DEG,
                          correction_heading_limit_deg=self.PATROL_HEADING_LIMIT_DEG,
                          raw_difference_deadband_mm=PATROL_SIDE_DEADBAND_MM,
                          fresh_centered_cycles=self._patrol_centered_cycles)
        return result


    def _quantized_angular(self, requested):
        if not self._patrol_cruise_policy or self._patrol_home_control:
            return super()._quantized_angular(requested)
        quantum = sequence.CENTER_ANGULAR_QUANTUM_MRADPS
        magnitude = min(self.PATROL_ANGULAR_LIMIT_MRADPS,
                        math.floor(abs(requested) / quantum + .5) * quantum)
        return int(math.copysign(magnitude, requested)) if requested else 0

    def _positive_action(self, wanted, angular, floor, direction=1):
        if not self._patrol_cruise_policy or self._patrol_home_control:
            return super()._positive_action(wanted, angular, floor, direction)
        if direction != 1:
            raise ValueError("巡航居中仅允许前进")
        linear = max(floor, min(sequence.MAX_WHEEL_SPEED_MMPS, round(wanted)))
        budget = min(linear - floor, sequence.MAX_WHEEL_SPEED_MMPS - linear)
        quantum = sequence.CENTER_ANGULAR_QUANTUM_MRADPS
        limit = min(self.PATROL_ANGULAR_LIMIT_MRADPS,
                    math.floor(2000 * budget / sequence.WHEEL_BASE_MM / quantum) * quantum)
        selected = max(-limit, min(limit, int(angular)))
        reason = "逐轮61～310mm/s约束限制差速" if selected != angular else None
        return int(linear), selected, reason

    def retry_pending_pair(self, sample=None, now=None):
        if not self._patrol_cruise_policy:
            return super().retry_pending_pair(sample=sample, now=now)
        if now is not None:
            self.refresh_yaw_rate(now)
        if sample is not None:
            return self.observe_side(sample)
        consumed = False
        while self._pending_pair_queue:
            candidate, histories, keys = self._pending_pair_queue[0]
            sources = tuple(history[-1][0] for history in histories)
            at = min(sources)
            if (candidate.source_identity is not None and candidate.source_identity != self._state_session
                    or self.source_barrier is not None and any(s <= self.source_barrier + 1e-9 for s in sources)
                    or any(not 0 <= self._rate_now - s <= .250 + 1e-9 for s in sources)
                    or self._pair_at is not None and at <= self._pair_at + 1e-9
                    or not all(sequence._source_key_newer(key, old) for key, old in zip(keys, self._paired_keys))):
                self._pending_pair_queue.popleft()
                continue
            if not self._poses or at > self._poses[-1].at + 1e-9:
                break
            self._pending_pair_queue.popleft()
            pose = self._pose_at(at)
            aligned = [self._side_at(history, at) for history in histories]
            if pose is None or any(item is None for item in aligned):
                continue
            difference = aligned[0][0] - aligned[1][0]
            observed = difference / 2.0
            error = observed
            if self._pair_at is not None and self._error is not None:
                dt = at - self._pair_at
                alpha = -math.expm1(-dt / sequence.CENTER_FILTER_TAU_S)
                previous = self._error
                predicted = previous
                if self._last_pair_pose is not None and self._road_reference is not None:
                    sensed = self._last_pair_pose
                    axis = math.radians(self._road_reference)
                    lateral = -(pose.x - sensed.x) * math.sin(axis) + (pose.y - sensed.y) * math.cos(axis)
                    # Propagate the front sensor position, without replacing
                    # its measured raw difference with an axle correction.
                    lateral += sequence.CENTER_SENSOR_FORWARD_MM * (math.sin(math.radians(
                        self._angle(pose.yaw - self._road_reference))) - math.sin(math.radians(
                        self._angle(sensed.yaw - self._road_reference))))
                    predicted -= lateral
                error = predicted + alpha * (observed - predicted)
                self._error_rate += alpha * ((error - previous) / dt - self._error_rate)
            self._pair_at, self._latest_source_times = at, sources
            self._last_pair_pose = pose
            self._raw_error = self._observed_error = observed
            self._last_raw_difference = difference
            self._error, self._usable_pair = error, True
            acceptable = (abs(difference) <= PATROL_SIDE_DEADBAND_MM + 1e-9
                          and abs(self._error_rate) <= .5 + 1e-9)
            self._patrol_centered_cycles = min(2, self._patrol_centered_cycles + 1) if acceptable else 0
            if not acceptable:
                self.revoke_normal_motion()
            self._paired_keys = list(keys)
            self.observation_token += 1
            self._lateral_sources.append((at, pose.travel_mm, observed))
            self._compensated_sources.append((at, sources, tuple(keys), observed, pose.yaw))
            consumed = True
        self._pending_pair_sample = self._pending_pair_queue[0][0] if self._pending_pair_queue else None
        return consumed

    def predicted_error_mm(self, now):
        error = super().predicted_error_mm(now)
        if (not self._patrol_cruise_policy or error is None or not self._poses
                or self._last_pair_pose is None or self._road_reference is None):
            return error
        return error - sequence.CENTER_SENSOR_FORWARD_MM * (
            math.sin(math.radians(self._angle(self._poses[-1].yaw - self._road_reference)))
            - math.sin(math.radians(self._angle(self._last_pair_pose.yaw - self._road_reference))))

    def request_cruise_speed_transition(self, goal=CRUISE_SPEED_MMPS):
        """Queue a speed-stage jump; only a successful write consumes it."""
        if (isinstance(goal, bool) or not isinstance(goal, (int, float))
                or not math.isfinite(goal) or not sequence.CENTER_MIN_WHEEL_SPEED_MMPS <= goal
                <= sequence.MAX_WHEEL_SPEED_MMPS):
            raise ValueError("巡航速度阶段目标不合法")
        self._patrol_pending_cruise_goal = float(goal)

    def _estimate_speed(self, now):
        if self._patrol_cruise_policy and self._patrol_speed_stage_waiting_applied is not None:
            return None
        barrier = self._patrol_speed_stage_started_at
        if self._patrol_cruise_policy and barrier is not None:
            samples = [item for item in self._speed_samples if item[2] > barrier + 1e-9]
            if (len(samples) < 2
                    or samples[-1][0] - samples[0][0] < sequence.CENTER_SPEED_WINDOW_S - 1e-9):
                return None
        return super()._estimate_speed(now)

    def _speed_proposal(self, goal, now, angular, *, cruise, frozen, departure=False):
        cap = self.obstacle_approach_speed_cap
        if self.patrol_road_speed_cap is not None:
            cap = self.patrol_road_speed_cap if cap is None else min(cap, self.patrol_road_speed_cap)
        if self._patrol_cruise_policy and cap is not None:
            # Formal obstacle approach keeps its baseline even on HOME61;
            # steering yields first when the minimum wheel speed uses all
            # available differential. Trim/ramp must never exceed the cap.
            wanted = min(goal, cap)
            budget = min(wanted - sequence.CENTER_MIN_WHEEL_SPEED_MMPS,
                         sequence.MAX_WHEEL_SPEED_MMPS - wanted)
            limit = math.floor(2000 * budget / sequence.WHEEL_BASE_MM
                               / sequence.CENTER_ANGULAR_QUANTUM_MRADPS) * sequence.CENTER_ANGULAR_QUANTUM_MRADPS
            angular = max(-limit, min(limit, angular))
            linear, angular, reason = self._positive_action(
                wanted, angular, sequence.CENTER_MIN_WHEEL_SPEED_MMPS)
            state = dict(_ramp_speed=float(linear), _ramp_requested=wanted,
                         _ramp_complete_at=now, _speed_trim=0.0,
                         _speed_integral_frozen=True, _speed_qualified=False,
                         _patrol_pending_cruise_goal=None)
            return linear, angular, reason, state, None
        if (self._patrol_cruise_policy and self._patrol_road_policy == "START"
                and goal == (self.settings.cruise_speed_mmps if self.settings is not None else INITIAL_SPEED_MMPS)
                and not self._patrol_home_control):
            # Until the first completed physical turn, 150 is the common UART
            # request, not an actual-speed setpoint with an accumulating trim.
            # Preserve heading steering, reducing differential before allowing
            # either wheel to leave the legal nonzero range.
            budget = min(goal - sequence.CENTER_MIN_WHEEL_SPEED_MMPS,
                         sequence.MAX_WHEEL_SPEED_MMPS - goal)
            limit = math.floor(2000 * budget / sequence.WHEEL_BASE_MM
                               / sequence.CENTER_ANGULAR_QUANTUM_MRADPS) * sequence.CENTER_ANGULAR_QUANTUM_MRADPS
            angular = max(-limit, min(limit, angular))
            linear, angular, reason = self._positive_action(
                goal, angular, sequence.CENTER_MIN_WHEEL_SPEED_MMPS)
            complete_at = (self._ramp_complete_at if self._ramp_requested == goal
                           and self._ramp_complete_at is not None else now)
            estimate = self._estimate_speed(now)
            qualified = bool(self._state_at is not None
                             and self._state_at - complete_at >= sequence.CENTER_SPEED_WINDOW_S - 1e-9
                             and estimate is not None
                             and abs(estimate - goal) <= self.patrol_speed_tolerance * goal + 1e-9)
            state = dict(_ramp_speed=float(goal), _ramp_requested=goal,
                         _ramp_complete_at=complete_at, _speed_trim=0.0,
                         _speed_last_update_at=now, _speed_last_integral_at=now,
                         _speed_last_state_key=self._state_uptime,
                         _speed_integral_frozen=True, _speed_qualified=qualified,
                         _patrol_pending_cruise_goal=None)
            return linear, angular, reason, state, estimate
        if (self._patrol_cruise_policy and self.settings is None
                and goal == HOME_SPEED_MMPS and not self._patrol_home_control):
            self._patrol_home_control = True
            try:
                return self._speed_proposal(goal, now, angular, cruise=cruise,
                                            frozen=frozen, departure=departure)
            finally:
                self._patrol_home_control = False
        if self._patrol_cruise_policy and self.settings is None and goal == POST_TURN_SPEED_MMPS:
            # Preserve the 91 common target by reducing differential first.
            # Minimum-wheel enforcement must not silently raise ALIGN speed.
            budget = min(goal - sequence.CENTER_MIN_WHEEL_SPEED_MMPS,
                         sequence.MAX_WHEEL_SPEED_MMPS - goal)
            limit = math.floor(2000 * budget / sequence.WHEEL_BASE_MM
                               / sequence.CENTER_ANGULAR_QUANTUM_MRADPS) * sequence.CENTER_ANGULAR_QUANTUM_MRADPS
            angular = max(-limit, min(limit, angular))
            linear, angular, reason, state, estimate = super()._speed_proposal(
                goal, now, angular, cruise=cruise, frozen=frozen, departure=departure)
            state.update(_ramp_speed=float(goal), _ramp_requested=goal, _speed_trim=0.0,
                         _speed_integral_frozen=True, _speed_qualified=False,
                         _patrol_pending_cruise_goal=None)
            return int(goal), int(angular), reason, state, estimate
        if (not self._patrol_cruise_policy
                or self._patrol_pending_cruise_goal != goal):
            if not self._patrol_cruise_policy or self._patrol_home_control:
                return super()._speed_proposal(goal, now, angular, cruise=cruise,
                                               frozen=frozen, departure=departure)
            return self._patrol_speed_proposal(goal, now, angular, cruise=cruise,
                                               frozen=frozen, departure=departure)
        linear, angular, reason = self._positive_action(
            goal, angular, sequence.CENTER_MIN_WHEEL_SPEED_MMPS)
        state = dict(_ramp_speed=float(goal), _ramp_complete_at=now, _ramp_requested=goal,
                     _speed_trim=0.0, _speed_last_update_at=now, _speed_last_integral_at=now,
                     _speed_last_state_key=None, _speed_qualified=False,
                     _speed_integral_frozen=True, _patrol_pending_cruise_goal=None,
                     _patrol_speed_stage_started_at=None,
                     _patrol_speed_stage_waiting_applied=(self._state_session, linear, angular, now))
        return linear, angular, reason, state, None

    def _patrol_speed_proposal(self, goal, now, angular, *, cruise, frozen, departure=False):
        """Stage-one feedback uses the real 500ms window and its road policy band."""
        elapsed = min(sequence.COMMAND_INTERVAL_S, max(0.0, now - self._last_target_at))
        ramp = float(goal) if self._ramp_speed is None else self._ramp_speed
        if self._ramp_speed is not None:
            delta = goal - ramp
            ramp += math.copysign(min(abs(delta), sequence.CENTER_ACCEL_MMPS2 * elapsed), delta) if delta else 0.0
        complete_at = self._ramp_complete_at if self._ramp_requested == goal else None
        if abs(ramp - goal) <= .5 and complete_at is None:
            complete_at = now
        estimate = self._estimate_speed(now)
        trim, updated = self._speed_trim, self._speed_last_update_at
        integral_at, key = self._speed_last_integral_at, self._speed_last_state_key
        settled = (complete_at is not None and self._state_at is not None
                   and self._state_at - complete_at >= sequence.CENTER_SPEED_WINDOW_S - 1e-9)
        confirmed = (self._applied_linear > 0 and sequence._motion_wheels(
            self._applied_linear, self._applied_angular) == self._last_sent_wheels)
        ready = not frozen and confirmed and settled and estimate is not None
        if ready and self._state_uptime != key and now - updated >= sequence.CENTER_SPEED_UPDATE_S - 1e-9:
            tolerance = self.patrol_speed_tolerance * goal if cruise else 1.0
            residual = math.copysign(max(0.0, abs(goal - estimate) - tolerance), goal - estimate)
            dt = min(sequence.CENTER_SPEED_UPDATE_S, max(0.0, now - integral_at))
            wanted = ramp + trim
            if not (wanted >= sequence.MAX_WHEEL_SPEED_MMPS and residual > 0
                    or wanted <= sequence.CENTER_MIN_WHEEL_SPEED_MMPS and residual < 0):
                trim = max(-sequence.MAX_WHEEL_SPEED_MMPS,
                           min(sequence.MAX_WHEEL_SPEED_MMPS, trim + .2 * residual * dt))
            updated, integral_at, key = now, now, self._state_uptime
        elif not ready:
            integral_at = now
        wanted = ramp + trim
        if not departure and self._ramp_speed is not None and all(wheel > 0 for wheel in self._last_sent_wheels):
            common = sum(self._last_sent_wheels) / 2.0
            allowance = sequence.CENTER_ACCEL_MMPS2 * elapsed
            wanted = max(common - allowance, min(common + allowance, wanted))
        linear, angular, reason = self._positive_action(wanted, angular, sequence.CENTER_MIN_WHEEL_SPEED_MMPS)
        qualified = bool(settled and estimate is not None and abs(estimate - goal)
                         <= self.patrol_speed_tolerance * goal + 1e-9)
        if settled and estimate is not None and not qualified:
            reason = reason or f"500ms实测均速超出目标±{self.patrol_speed_tolerance * 100:g}%，反馈补偿中"
        if ramp + trim >= sequence.MAX_WHEEL_SPEED_MMPS - .5 and estimate is not None and estimate < goal * (1 - self.patrol_speed_tolerance):
            reason = "逐轮速度已达可行上限，实速尚未达标"
        state = dict(_ramp_speed=ramp, _ramp_complete_at=complete_at, _ramp_requested=goal,
                     _speed_trim=trim, _speed_last_update_at=updated, _speed_last_integral_at=integral_at,
                     _speed_last_state_key=key, _speed_qualified=qualified,
                     _speed_integral_frozen=not ready)
        return linear, angular, reason, state, estimate

    def speed_snapshot(self, now=None):
        result = super().speed_snapshot(now)
        is_home = (self._uses_home_policy() if self.settings is not None
                   else self._ramp_requested == HOME_SPEED_MMPS)
        if self._patrol_cruise_policy and not is_home:
            goal = result["target_mean_mmps"]
            ratio = self.patrol_speed_tolerance
            result.update(tolerance_ratio=ratio,
                          acceptable_actual_mmps=(goal * (1 - ratio), goal * (1 + ratio)) if goal is not None else None,
                          basis=f"真实连续STATE的500ms双轮平均净轮程；目标±{ratio * 100:g}%，缺窗不虚报达标")
        return result

    def confirm_target_sent(self, target):
        proposal = target._proposal or {}
        if self._patrol_cruise_policy and (proposal.get("patrol_alignment") or proposal.get("patrol_short_correction")):
            if (target.linear_mmps <= 0
                    or any(wheel != 0 and not 61 <= wheel <= sequence.MAX_WHEEL_SPEED_MMPS for wheel in target.wheel_targets)
                    or abs(target.angular_mradps) > self.PATROL_ANGULAR_LIMIT_MRADPS):
                raise ValueError("阶段一矫正轮速须为0或61～310正向轮速")
            if not self.target_send_ready(target, self._rate_now):
                return False
            for name, value in proposal.get("state", {}).items():
                setattr(self, name, value)
            self._last_sent_wheels, self._last_sent_mode = target.wheel_targets, target.mode
            self._speed_limit_reason = target.limited_reason
            self._last_target_at = proposal.get("at", self._rate_now)
            self._patrol_alignment_targets.add((target.linear_mmps, target.angular_mradps))
            if not proposal.get("patrol_heading_recovery") and not proposal.get("patrol_alignment_cached"):
                self._patrol_alignment_hold = dict(
                    epoch=self._motion_epoch, identity=self._state_session,
                    reference=self._road_reference, sources=self.latest_source_times,
                    linear=target.linear_mmps, angular=target.angular_mradps)
            elif proposal.get("patrol_heading_recovery"):
                self._patrol_alignment_hold = None
            self.revoke_normal_motion()
            self.notify_sent(target.angular_mradps, sent_at=max(self._last_sent_at or -math.inf,
                                                              self._last_target_at))
            return True
        jump = (target._proposal or {}).get("state", {}).get("_patrol_speed_stage_waiting_applied")
        committed = super().confirm_target_sent(target)
        if (committed and self._patrol_cruise_policy
                and not proposal.get("departure") and not proposal.get("cached")):
            # The shared adapter marks any successfully sent fresh target as
            # normal. Patrol's laser qualification needs its two real stable
            # cycles; independent150-speed control must not fabricate that.
            self._normal_motion_qualified = bool(proposal.get("qualifies"))
        if committed and self._patrol_cruise_policy and jump is not None:
            # Match an application witness after the actual successful write,
            # never after an earlier target calculation.
            identity, linear, angular, calculated_at = jump
            sent_at = max(calculated_at, self._last_sent_at or calculated_at)
            self._patrol_speed_stage_waiting_applied = identity, linear, angular, sent_at
            self._ramp_complete_at = sent_at
            self._speed_last_update_at = self._speed_last_integral_at = sent_at
        return committed

    def notify_target_applied(self, linear_mmps, angular_mradps, applied_at=None, *, source_at=None):
        if not linear_mmps and abs(angular_mradps) == DEPARTURE_PIVOT_ANGULAR_MRADPS:
            # The saved first-junction pivot is supervised by its own gyro and
            # wheel evidence; a reverse wheel is not a centering trajectory.
            return
        super().notify_target_applied(linear_mmps, angular_mradps, applied_at, source_at=source_at)
        waiting = self._patrol_speed_stage_waiting_applied
        if not self._patrol_cruise_policy or waiting is None:
            return
        identity, linear, angular, sent_at = waiting
        if (self._state_session != identity or (linear_mmps, angular_mradps) != (linear, angular)
                or isinstance(source_at, bool) or not isinstance(source_at, (int, float))
                or not math.isfinite(source_at) or source_at < sent_at - 1e-9):
            return
        # The display calls this only for timely, identity-valid confirmed
        # records. Renewals must never slide the first new-stage source anchor.
        self._patrol_speed_stage_started_at = self._ramp_complete_at = source_at
        self._speed_last_update_at = self._speed_last_integral_at = source_at
        self._patrol_speed_stage_waiting_applied = None

    def notify_stopped(self, now, *, stable=True):
        self._patrol_centered_cycles = 0
        self._patrol_alignment_hold = None
        self._patrol_cruise_hold_goal = None
        self._patrol_cruise_hold_sources = None
        self._patrol_speed_stage_started_at = None
        self._patrol_speed_stage_waiting_applied = None
        super().notify_stopped(now, stable=stable)

    def _short_correction_action(self, angular):
        """Firmware maps each 16 mrad/s quantum to one mm/s half difference."""
        angular = max(-self.PATROL_ANGULAR_LIMIT_MRADPS,
                      min(self.PATROL_ANGULAR_LIMIT_MRADPS, int(angular)))
        half = abs(math.trunc(angular * sequence.WHEEL_BASE_MM / 2000))
        if half >= 31:
            return half, angular, "单轮停转低里程矫正；交接须严格停稳"
        return 61 + half, angular, "矫正期间临时降低均速，非零轮速至少61mm/s"

    def _patrol_speed_and_differential(self, goal, now, angular, *, frozen=False,
                                      departure=False, allow_single_zero=False):
        """Keep the requested actual mean until its legal differential is exhausted."""
        requested = angular
        linear, selected, reason, state, estimate = self._speed_proposal(
            goal, now, angular, cruise=True, frozen=frozen, departure=departure)
        requested_half = abs(math.trunc(requested * sequence.WHEEL_BASE_MM / 2000))
        goal_budget = min(goal - sequence.CENTER_MIN_WHEEL_SPEED_MMPS,
                          sequence.MAX_WHEEL_SPEED_MMPS - goal)
        single_zero = (allow_single_zero and self.obstacle_approach_speed_cap is None
                       and self.patrol_road_speed_cap is None
                       and self._patrol_road_policy == "ORDINARY" and not frozen
                       and requested_half > goal_budget
                       and requested != selected and abs(requested) > abs(selected))
        if single_zero:
            half = abs(math.trunc(requested * sequence.WHEEL_BASE_MM / 2000))
            if half < 31:
                single_zero = False
            else:
                linear, selected = half, requested
                reason = "150均速下合法差速不足，临时单轮停转；停稳交接后重新核验"
                state.update(_ramp_speed=float(linear), _ramp_requested=goal,
                             _speed_qualified=False, _speed_integral_frozen=True,
                             _speed_trim=0., _patrol_pending_cruise_goal=float(goal))
        return linear, selected, reason, state, estimate, single_zero

    def _limit_patrol_heading(self, angular, now, psi, rate, limit):
        """Reserve the angle needed to unload successful command steps.

        Actual gyro rate and the executing/sent command both contribute. A
        failed write cannot reduce this reserve or create another slew step.
        """
        if not angular:
            return angular, False
        sign = 1 if angular > 0 else -1
        outward_rate = max(0., sign * (rate or 0.),
                           math.degrees(max(0., sign * angular, sign * self._last_sent_angular,
                                            sign * self._applied_angular) / 1000.))
        # Candidate remains applied through its confirmation window, then
        # decreases by 32 mrad/s at every successful 100ms command.
        pending = abs(angular)
        unload_angle = 0.
        while pending > 0:
            unload_angle += math.degrees(pending / 1000.) * sequence.COMMAND_INTERVAL_S
            pending = max(0, pending - 2 * sequence.CENTER_ANGULAR_QUANTUM_MRADPS)
        reserve = outward_rate * config.MCU_COMMAND_CONFIRM_TIMEOUT_S + unload_angle
        if sign * psi + reserve < limit - 1e-9:
            return angular, False
        steps = (1 if self._last_sent_at is None else max(0, math.floor(
            (now - self._last_sent_at + 1e-9) / sequence.COMMAND_INTERVAL_S)))
        allowance = 2 * sequence.CENTER_ANGULAR_QUANTUM_MRADPS * steps
        unloading = self._last_sent_angular - sign * allowance
        # Request RETURN, including countersteer if it fits this same step.
        return (max(-allowance, unloading) if sign > 0
                else min(allowance, unloading)), True

    def _target_patrol_home(self, xx, now, *, cached_feedback=False, departure_only=False,
               correction_heading_limit_deg=PATROL_CORRECTION_HEADING_LIMIT_DEG):
        if not self._patrol_cruise_policy:
            return super().target(xx, now, cached_feedback=cached_feedback,
                                  departure_only=departure_only)
        self.refresh_yaw_rate(now)
        self._consume_pending_means(now)
        pose = self._poses[-1] if self._poses else None
        if (pose is None or self._road_reference is None or self._state_at is None
                or not 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                or not departure_only and not self.control_available(now)):
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        controls_fresh = all(item is not None
                             and 0 <= now - item[0] <= sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                             for item in self._control_cache)
        cached = not departure_only and (cached_feedback or not controls_fresh)
        if cached and not self._normal_motion_qualified:
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        psi, rate = self._angle(pose.yaw - self._road_reference), self._effective_yaw_rate(now)
        error = self.predicted_error_mm(now)
        position = math.copysign(max(0.0, abs(error or 0.0) - PATROL_SIDE_DEADBAND_MM / 2.0),
                                 error or 0.0)
        hold_offset = self._angle((self._hold_heading if self._hold_heading is not None
                                   else self._road_reference) - self._road_reference)
        desired, capture, state, position_feedback = hold_offset, False, {}, None
        correction_phase = self._patrol_correction_phase
        if departure_only:
            desired = 0.0
        elif cached:
            desired = hold_offset
        elif abs(error or 0.0) <= PATROL_SIDE_DEADBAND_MM / 2.0 and abs(psi) <= sequence.NORMAL_HEADING_TOLERANCE_DEG + 1e-9:
            # HOLD tracks the fixed reference, never the tilted end of SHIFT.
            desired, capture, correction_phase = 0.0, abs(psi) <= .2 and abs(rate or 0.) <= .5, "HOLD"
        else:
            trim, position_state, position_feedback = self._position_proposal(
                now, error, frozen=not controls_fresh)
            state.update(position_state)
            desired = math.degrees(math.atan2(position, sequence.CENTER_LOOKAHEAD_MM)) + trim
            if position == 0 and abs(psi) > sequence.NORMAL_HEADING_TOLERANCE_DEG:
                desired = 0.0
            correction_phase = "SHIFT"
            if correction_heading_limit_deg is None:
                # Predict the lateral distance spent unloading the actual
                # angular command and returning the heading. Wheel travel is
                # measured independently; stale laser prediction gives no
                # new qualification evidence.
                speed = max(float(xx), self.actual_speed_mmps or 0.)
                return_rate = max(.032, min(.768, 4.0 * abs(math.radians(psi))))
                unload_s = abs(self._last_sent_angular) / 320.0
                return_s = abs(math.radians(psi)) / return_rate
                reserve = speed * abs(math.sin(math.radians(psi))) * (
                    config.MCU_COMMAND_CONFIRM_TIMEOUT_S + unload_s + .5 * return_s)
                trend = max(0., -math.copysign(1., error or 1.) * self._error_rate)
                reserve = max(reserve, trend * sequence.COMMAND_INTERVAL_S)
                toward = psi * (error or 0.) > 0 or (rate or 0.) * (error or 0.) > 0
                if (self._patrol_correction_phase == "RETURN"
                        and abs(psi) > .2 or toward and abs(error or 0.) <= reserve + 2.5):
                    desired, correction_phase = 0.0, "RETURN"
                desired = max(-30.0, min(30.0, desired))
        if correction_heading_limit_deg is not None:
            desired = max(-correction_heading_limit_deg,
                          min(correction_heading_limit_deg, desired))
        requested = (0.0 if capture else
                     (sequence.CENTER_FORWARD_SHIFT_YAW_GAIN_PER_S * math.radians(desired - psi)
                      - sequence.CENTER_YAW_RATE_DAMPING * math.radians(rate or 0.0)) * 1000.0)
        if cached:
            old = self._last_sent_angular
            requested = math.copysign(min(abs(old), abs(requested)), old) if old * requested > 0 else 0.0
        confirmed = (self._applied_linear > 0 and sequence._motion_wheels(
            self._applied_linear, self._applied_angular) == self._last_sent_wheels)
        angular, residual, residual_sign = self._fine_angular_proposal(
            requested, now, enabled=not departure_only and not cached and controls_fresh
            and confirmed and not capture)
        if not cached:
            # Smooth ordinary-road steering without clipping its position goal.
            # Recalculation and failed writes cannot advance this allowance.
            steps = (1 if self._last_sent_at is None else max(0, math.floor(
                (now - self._last_sent_at + 1e-9) / sequence.COMMAND_INTERVAL_S)))
            allowance = 2 * sequence.CENTER_ANGULAR_QUANTUM_MRADPS * steps
            angular = max(self._last_sent_angular - allowance,
                          min(self._last_sent_angular + allowance, angular))
        if not cached and not departure_only:
            limit = 30. if correction_heading_limit_deg is None else correction_heading_limit_deg
            angular, angle_guard = self._limit_patrol_heading(angular, now, psi, rate, limit)
            if angle_guard:
                correction_phase = "RETURN"
                desired = 0.
        if cached:
            old = self._last_sent_angular
            angular = int(math.copysign(min(abs(old), abs(angular)), old)) if old * angular > 0 else 0
        linear, angular, reason, speed_state, estimate = self._speed_proposal(
            xx, now, angular, cruise=True, frozen=cached, departure=departure_only)
        state.update(speed_state, _angular_residual_mradps=residual,
                     _angular_residual_request_sign=residual_sign,
                     _patrol_correction_phase=correction_phase)
        normal_allowed = bool(not departure_only and (cached and self._normal_motion_qualified
                              or not cached and self._usable_pair))
        state["_control_feedback"] = dict(
            raw_difference_mm=self._last_raw_difference,
            axle_equivalent_difference_mm=self.signed_difference_mm, filtered_half_difference_mm=error,
            fixed_reference_deg=self._road_reference, hold_heading_deg=self._hold_heading,
            desired_heading_deg=desired, actual_heading_offset_deg=psi, yaw_rate_degps=rate,
            selected_linear_mmps=linear, selected_angular_mradps=angular,
            position_control=position_feedback, cached_feedback=cached,
            heading_tolerance_deg=sequence.NORMAL_HEADING_TOLERANCE_DEG,
            correction_heading_limit_deg=correction_heading_limit_deg,
            position_integral_limit_deg=sequence.POSITION_INTEGRAL_LIMIT_DEG,
            correction_phase=correction_phase,
            basis=("原始左右激光差与固定参考；侧移±30°、提前收头、固定参考保持"
                   if correction_heading_limit_deg is None else
                   f"原始左右激光差与固定参考；纠偏目标限±{correction_heading_limit_deg:g}°"))
        proposal = dict(epoch=self._motion_epoch, token=self.observation_token, at=now, state=state,
                        identity=self._state_session, departure=departure_only, cached=cached,
                        normal=normal_allowed, qualifies=normal_allowed,
                        capture_hold=capture and angular == 0 and not self._hold_locked,
                        adjustment=angular != 0)
        return sequence.CenteringTarget(
            int(linear), int(angular), "DEPARTURE" if departure_only else "CACHE" if cached
            else "CRUISE_CORRECT", error, psi, rate, self.axis_trusted,
            self.centered and not cached and not departure_only, proposal, reason, estimate,
            speed_state["_speed_qualified"], normal_allowed)

    def target(self, xx, now, *, cached_feedback=False, departure_only=False,
               correction_heading_limit_deg=45.0):
        if self._patrol_cruise_policy and self._patrol_road_policy == "HOME":
            self._patrol_home_control = True
            try:
                return self._target_patrol_home(xx, now, cached_feedback=cached_feedback,
                    departure_only=departure_only, correction_heading_limit_deg=5.0)
            finally:
                self._patrol_home_control = False
        if not self._patrol_cruise_policy:
            return super().target(xx, now, cached_feedback=cached_feedback,
                                  departure_only=departure_only)
        self.refresh_yaw_rate(now)
        self._consume_pending_means(now)
        pose = self._poses[-1] if self._poses else None
        if (pose is None or self._road_reference is None or self._state_at is None
                or not 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                or not departure_only and not self.control_available(now)):
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        controls_fresh = all(item is not None
                             and 0 <= now - item[0] <= sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                             for item in self._control_cache)
        cached = not departure_only and (cached_feedback or not controls_fresh)
        if cached and not self._normal_motion_qualified:
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        psi, rate = self._angle(pose.yaw - self._road_reference), self._effective_yaw_rate(now)
        error = self.predicted_error_mm(now)
        position = math.copysign(max(0.0, abs(error or 0.0) - PATROL_SIDE_DEADBAND_MM / 2.0),
                                 error or 0.0)
        limit = self.patrol_heading_limit_deg
        desired, capture, state, position_feedback = 0.0, False, {}, None
        correction_phase = self._patrol_correction_phase
        if departure_only:
            desired = 0.0
        elif cached:
            desired = 0.0
        elif (abs(psi) > limit + 1e-9
              or self._patrol_heading_recovery and abs(psi) > PATROL_RECOVERY_EXIT_DEG + 1e-9):
            desired, correction_phase = 0.0, "RETURN"
        else:
            # Steering follows the laser displacement and its real-source
            # trend. A sampled, slightly tilted reference must not create a
            # permanent lateral equilibrium while its angle is still legal.
            if abs(self._raw_error or 0.) <= PATROL_SIDE_DEADBAND_MM / 2.:
                position = 0.0
            elif position == 0.0 or position * (self._raw_error or 0.0) <= 0:
                position = math.copysign(max(0., abs(self._raw_error) - PATROL_SIDE_DEADBAND_MM / 2.),
                                         self._raw_error)
            trend = self._error_rate if position else 0.0
            horizon = .5 if self._patrol_road_policy == "SHORT" else self.PATROL_LASER_TREND_HORIZON_S
            laser_increment = math.degrees(math.atan2(position + horizon * trend,
                                                      sequence.CENTER_LOOKAHEAD_MM))
            desired = psi + laser_increment
            if self._patrol_road_policy == "SHORT":
                desired = max(-limit, min(limit, desired))
            capture = (abs(self._raw_error or 0.) <= PATROL_SIDE_DEADBAND_MM / 2.
                       and abs(trend) <= .5 and abs(rate or 0.) <= .5)
            correction_phase = "HOLD" if abs(self._raw_error or 0.) <= PATROL_SIDE_DEADBAND_MM / 2. else "SHIFT"
            state.update(_position_integral_deg=0.0, _position_integral_key=tuple(self._paired_keys),
                         _position_integral_source_at=self._pair_at)
            position_feedback = dict(laser_error_mm=error, laser_error_rate_mmps=trend,
                                     trend_horizon_s=horizon, source_at=self._pair_at,
                                     source_keys=tuple(self._paired_keys),
                                     basis="新鲜激光误差与趋势优先；固定参考只限定允许偏角")
        # The frozen direction is auxiliary: healthy laser control is not
        # clipped or changed into active RETURN at an actual legal angle.
        gain = 1.5 if self._patrol_road_policy == "SHORT" else self.PATROL_LASER_YAW_GAIN_PER_S
        if (position_feedback is not None and self._patrol_road_policy == "ORDINARY"
                and self.patrol_road_speed_cap is None and abs(self._last_raw_difference) >= 25.0):
            gain = self.PATROL_LARGE_ERROR_LASER_YAW_GAIN_PER_S
        requested = (0.0 if capture else
                     (gain * math.radians(desired - psi)
                      - sequence.CENTER_YAW_RATE_DAMPING * math.radians(rate or 0.0)) * 1000.0)
        if cached:
            old = self._last_sent_angular
            requested = math.copysign(min(abs(old), abs(requested)), old) if old * requested > 0 else 0.0
        confirmed = (self._applied_linear > 0 and sequence._motion_wheels(
            self._applied_linear, self._applied_angular) == self._last_sent_wheels)
        angular, residual, residual_sign = self._fine_angular_proposal(
            requested, now, enabled=not departure_only and not cached and controls_fresh
            and confirmed and not capture)
        if not cached and not departure_only:
            state['_patrol_cruise_hold_goal'] = self._angle(self._road_reference + desired)
            state['_patrol_cruise_hold_sources'] = self.latest_source_times
            state['_patrol_missing_hold_goal'] = None
        if cached:
            old = self._last_sent_angular
            angular = int(math.copysign(min(abs(old), abs(angular)), old)) if old * angular > 0 else 0
        linear, angular, reason, speed_state, estimate, short_correction = self._patrol_speed_and_differential(
            xx, now, angular, frozen=cached, departure=departure_only,
            allow_single_zero=not cached and not departure_only and correction_phase != "RETURN")
        state.update(speed_state, _angular_residual_mradps=residual,
                     _angular_residual_request_sign=residual_sign,
                     _patrol_correction_phase=correction_phase,
                     _patrol_heading_recovery=correction_phase == "RETURN")
        normal_allowed = bool(not departure_only and (cached and self._normal_motion_qualified
                              or not cached and self.centered and correction_phase != "RETURN"))
        state["_control_feedback"] = dict(
            raw_difference_mm=self._last_raw_difference,
            axle_equivalent_difference_mm=self.signed_difference_mm, filtered_half_difference_mm=error,
            fixed_reference_deg=self._road_reference, hold_heading_deg=self._hold_heading,
            desired_heading_deg=desired, actual_heading_offset_deg=psi, yaw_rate_degps=rate,
            selected_linear_mmps=linear, selected_angular_mradps=angular,
            position_control=position_feedback, cached_feedback=cached,
            laser_yaw_gain_per_s=gain if position_feedback is not None else None,
            heading_tolerance_deg=sequence.NORMAL_HEADING_TOLERANCE_DEG,
            correction_heading_limit_deg=limit,
            position_integral_limit_deg=sequence.POSITION_INTEGRAL_LIMIT_DEG,
            correction_phase=correction_phase,
            basis=f"原始左右激光误差与趋势优先；可信参考限±{limit:g}°，实速目标±{self.patrol_speed_tolerance * 100:g}%")
        state['_control_feedback']['heading_tolerance_deg'] = self.PATROL_HEADING_LIMIT_DEG
        proposal = dict(epoch=self._motion_epoch, token=self.observation_token, at=now, state=state,
                        identity=self._state_session, departure=departure_only, cached=cached,
                        normal=normal_allowed, qualifies=normal_allowed,
                        patrol_short_correction=short_correction,
                        capture_hold=False,
                        adjustment=angular != 0)
        return sequence.CenteringTarget(
            int(linear), int(angular), "DEPARTURE" if departure_only else "CACHE" if cached
            else "CRUISE_CORRECT", error, psi, rate, self.axis_trusted,
            self.centered and not cached and not departure_only, proposal, reason, estimate,
            speed_state["_speed_qualified"], normal_allowed)

    def target_heading_only(self, xx, now, *, cached_feedback=False, departure_only=False,
                             fixed_reference_deg=None, road_adjustment=False):
        """Hold the last sent laser goal through missing measurements."""
        if (self._patrol_cruise_policy and self._patrol_road_policy == "HOME"
                and not self._patrol_home_control):
            self._patrol_home_control = True
            try:
                return self.target_heading_only(xx, now, cached_feedback=cached_feedback,
                    departure_only=departure_only, fixed_reference_deg=fixed_reference_deg,
                    road_adjustment=road_adjustment)
            finally:
                self._patrol_home_control = False
        if (self._patrol_cruise_policy and self.settings is None
                and xx == HOME_SPEED_MMPS and not self._patrol_home_control):
            self._patrol_home_control = True
            try:
                return self.target_heading_only(xx, now, cached_feedback=cached_feedback,
                    departure_only=departure_only, fixed_reference_deg=fixed_reference_deg,
                    road_adjustment=road_adjustment)
            finally:
                self._patrol_home_control = False
        fixed = fixed_reference_deg is not None
        if fixed:
            if (isinstance(fixed_reference_deg, bool)
                    or not isinstance(fixed_reference_deg, (int, float))
                    or not math.isfinite(fixed_reference_deg)):
                raise ValueError("固定航向参考必须为有限角度")
            # Fixed-reference tracking depends on STATE/gyro, never side age.
            departure_only = True
        # Missing measurements do not add laser evidence. Within the legal
        # ordinary-road angle, keep the last successfully sent laser goal;
        # a first heading-only proposal captures the current fresh STATE yaw.
        laser_hold = bool(not fixed and road_adjustment
                          and self._patrol_road_policy == "ORDINARY"
                          and not self._patrol_heading_recovery
                          and abs(self._heading_error() or 0.) <= self.patrol_heading_limit_deg + 1e-9)
        held = (self._patrol_cruise_hold_goal if self._patrol_cruise_hold_goal is not None
                else self._patrol_missing_hold_goal)
        if laser_hold and held is None and self._poses:
            held = self._poses[-1].yaw
        reference = (self._angle(fixed_reference_deg) if fixed else
                     held if laser_hold and held is not None else self._road_reference)
        tolerance = (.2 if laser_hold
                     else PATROL_FIXED_HEADING_TOLERANCE_DEG if fixed or self._patrol_cruise_policy and not self._patrol_home_control
                     else sequence.NORMAL_HEADING_TOLERANCE_DEG)
        self.refresh_yaw_rate(now)
        pose = self._poses[-1] if self._poses else None
        if (pose is None or self._road_reference is None
                or self._state_at is None
                or not 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                or not departure_only and not self.control_available(now)):
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        controls_fresh = all(item is not None
                             and 0 <= now - item[0] <= sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                             for item in self._control_cache)
        cached = not departure_only and (cached_feedback or not controls_fresh)
        if cached and not self._normal_motion_qualified:
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        psi = self._angle(pose.yaw - reference)
        rate = self._effective_yaw_rate(now)
        capture = abs(psi) <= tolerance + 1e-9
        gain = ((self.PATROL_LASER_YAW_GAIN_PER_S if self._patrol_road_policy == "ORDINARY" else 1.5)
                if self._patrol_cruise_policy and not self._patrol_home_control
                else sequence.CENTER_FORWARD_SHIFT_YAW_GAIN_PER_S)
        requested = (0.0 if capture else
                     (-gain * math.radians(psi)
                      - sequence.CENTER_YAW_RATE_DAMPING * math.radians(rate or 0.0)) * 1000.0)
        if cached:
            old = self._last_sent_angular
            requested = math.copysign(min(abs(old), abs(requested)), old) if old * requested > 0 else 0.0
        confirmed_driving = (self._applied_linear > 0
                             and sequence._motion_wheels(self._applied_linear, self._applied_angular)
                             == self._last_sent_wheels)
        angular, residual, residual_sign = self._fine_angular_proposal(
            requested, now, enabled=confirmed_driving and not capture
            and (fixed or not departure_only and not cached and controls_fresh))
        if self._patrol_home_control and not cached:
            steps = (1 if self._last_sent_at is None else max(0, math.floor(
                (now - self._last_sent_at + 1e-9) / sequence.COMMAND_INTERVAL_S)))
            allowance = 2 * sequence.CENTER_ANGULAR_QUANTUM_MRADPS * steps
            angular = max(self._last_sent_angular - allowance,
                          min(self._last_sent_angular + allowance, angular))
        if cached:
            old = self._last_sent_angular
            angular = int(math.copysign(min(abs(old), abs(angular)), old)) if old * angular > 0 else 0
        heading_recovery = (self._patrol_cruise_policy and self._patrol_road_policy != "HOME" and road_adjustment
                            and (abs(self._heading_error() or 0.) > self.patrol_heading_limit_deg + 1e-9
                                 or self._patrol_heading_recovery
                                 and abs(self._heading_error() or 0.) > PATROL_RECOVERY_EXIT_DEG + 1e-9))
        linear, angular, reason, state, estimate, short_correction = self._patrol_speed_and_differential(
            xx, now, angular, frozen=cached, departure=departure_only)
        if fixed:
            # Only a successful write commits this reference/trim state. Do
            # not capture the instantaneous tilted heading as a new hold goal.
            state.update(_hold_heading=reference, _hold_locked=capture and angular == 0,
                         _position_integral_deg=0.0, _position_integral_key=None,
                         _position_integral_source_at=None)
        if laser_hold:
            state['_patrol_missing_hold_goal'] = reference
        error = self.predicted_error_mm(now)
        if self._patrol_cruise_policy and road_adjustment:
            state['_patrol_heading_recovery'] = heading_recovery
        state.update(_angular_residual_mradps=residual,
                     _angular_residual_request_sign=residual_sign,
                     _control_feedback=dict(
                         raw_difference_mm=self._last_raw_difference,
                         axle_equivalent_difference_mm=self.signed_difference_mm,
                         filtered_half_difference_mm=error,
                         fixed_reference_deg=self._road_reference,
                         hold_heading_deg=reference if fixed or laser_hold else self._hold_heading,
                         desired_heading_deg=(self._angle(reference - self._road_reference) if laser_hold
                                              else 0.0 if fixed else psi if capture else 0.0),
                         actual_heading_offset_deg=self._angle(pose.yaw - self._road_reference),
                         yaw_rate_degps=rate, selected_linear_mmps=linear, selected_angular_mradps=angular,
                         held_laser_source_times=self._patrol_cruise_hold_sources if laser_hold else None,
                         position_control=None, cached_feedback=cached,
                         correction_phase='RETURN' if heading_recovery else 'HOLD',
                         heading_tolerance_deg=tolerance,
                         basis="短缺测保持最近成功激光纠偏目标；固定道路参考不变" if laser_hold else
                         "固定参考±1°航向回正；真实横偏仅显示" if fixed
                         else "仅航向控制；真实横偏仅显示，不参与转向或居中资格"))
        normal_allowed = bool(not short_correction and not departure_only
                              and (cached and self._normal_motion_qualified or not cached and self._usable_pair))
        proposal = dict(epoch=self._motion_epoch, token=self.observation_token, at=now, state=state,
                        identity=self._state_session, departure=departure_only, cached=cached,
                        patrol_laser_hold=laser_hold,
                        patrol_road_adjustment=road_adjustment, patrol_short_correction=short_correction,
                        normal=normal_allowed, qualifies=normal_allowed,
                        capture_hold=(not self._patrol_cruise_policy or self._patrol_home_control)
                        and not fixed and capture and not cached and angular == 0 and not self._hold_locked,
                        adjustment=angular != 0)
        return sequence.CenteringTarget(
            int(linear), int(angular), "HEADING_FIXED" if fixed else "HEADING_LASER_HOLD" if laser_hold
            else "HEADING_DEPARTURE" if departure_only
            else "HEADING_CACHE" if cached else "HEADING_ONLY",
            error, psi, rate, self.axis_trusted, False, proposal, reason, estimate,
            state["_speed_qualified"], normal_allowed)

    def _patrol_laser_hold_ready(self, now):
        sources = self._patrol_cruise_hold_sources
        return bool(self._patrol_cruise_hold_goal is not None and sources is not None
                    and all(0 <= now - source <= .500 + 1e-9 for source in sources)
                    and self.control_available(now) and all(item is not None
                        and 0 <= now - item[0] <= .500 + 1e-9 for item in self._control_cache))

    def target_patrol_heading_recovery(self, xx, now):
        if not self._patrol_cruise_policy:
            raise ValueError("阶段一航向恢复须显式启用巡逻策略")
        # This entry is always the low-speed stage, irrespective of the
        # caller's preceding cruise goal. It must not inherit the fixed-91
        # differential clamp used by the legacy departure adapter.
        return self._patrol_forward_alignment_target(
            91 if self._patrol_road_policy == "SHORT" else xx, now, recovery=True)

    def _patrol_alignment_ready(self, now):
        self.refresh_yaw_rate(now)
        sources = self.latest_source_times
        return bool(self._poses and self._road_reference is not None and self._state_at is not None
                    and 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                    and self._usable_pair and self.control_available(now) and sources is not None
                    and all(0 <= now - source <= .250 + 1e-9 for source in sources)
                    and self._raw_error is not None)

    def target_patrol_alignment_cached(self, now):
        """Renew only the last successfully sent forward target for <=500ms."""
        hold = self._patrol_alignment_hold
        if not self._patrol_alignment_hold_ready(now):
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        proposal = dict(epoch=hold["epoch"], identity=hold["identity"], at=now,
                        token=self.observation_token, patrol_alignment=True,
                        patrol_alignment_cached=True, alignment_hold=hold,
                        # The dedicated fixed-source cache is revalidated below;
                        # it cannot acquire shared normal-motion qualification.
                        departure=True, cached=True, normal=False, qualifies=False,
                        state=dict(_speed_qualified=False, _speed_integral_frozen=True,
                            _control_feedback=dict(raw_difference_mm=self._last_raw_difference,
                                fixed_reference_deg=hold["reference"], actual_heading_offset_deg=self._heading_error(),
                                selected_linear_mmps=hold["linear"], selected_angular_mradps=hold["angular"],
                                cached_source_times=hold["sources"], correction_phase="FORWARD_CACHE",
                                basis="仅续发最后成功前进矫正目标；原始双侧源龄≤500ms，不增加居中证据")))
        return sequence.CenteringTarget(hold["linear"], hold["angular"], "PATROL_ALIGN_FORWARD_CACHE",
                                        self._error, self._heading_error(), self._effective_yaw_rate(),
                                        self.axis_trusted, False, proposal, normal_motion_allowed=False)

    def _patrol_alignment_hold_ready(self, now):
        hold = self._patrol_alignment_hold
        return bool(self._patrol_cruise_policy and hold is not None
                    and hold["epoch"] == self._motion_epoch and hold["identity"] == self._state_session
                    and hold["reference"] == self._road_reference
                    and hold["sources"] is not None
                    and all(0 <= now - source <= .500 + 1e-9 for source in hold["sources"])
                    and self._state_at is not None
                    and 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                    and self.control_available(now)
                    and self._patrol_alignment_angle_safe(hold["angular"], now))

    def _patrol_alignment_angle_safe(self, angular, now):
        pose = self._poses[-1] if self._poses else None
        if pose is None or self._road_reference is None:
            return False
        psi = self._angle(pose.yaw - self._road_reference)
        return abs(psi) <= self.patrol_heading_limit_deg + 1e-9 or angular * psi <= 0

    def target_patrol_alignment(self, mode, sign, now, *, goal=91):
        """Continuous forward correction from current signed laser feedback."""
        if not self._patrol_cruise_policy:
            raise ValueError("阶段一前进矫正须显式启用巡逻策略")
        if mode != "FORWARD" or isinstance(sign, bool) or sign not in (-1, 1):
            raise ValueError("阶段一矫正仅支持FORWARD及兼容±1方向参数")
        if isinstance(goal, bool) or not isinstance(goal, (int, float)) or not math.isfinite(goal) or not 91 <= goal <= 150:
            raise ValueError("阶段一前进矫正目标须在91～150mm/s")
        # The compatible sign argument no longer freezes a correction round.
        return self._patrol_forward_alignment_target(goal, now, recovery=False)

    def _patrol_forward_alignment_target(self, goal, now, *, recovery):
        self.refresh_yaw_rate(now)
        state_ready = (self._poses and self._road_reference is not None and self._state_at is not None
                       and 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9)
        if not state_ready or not recovery and not self._patrol_alignment_ready(now):
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        psi, rate = self._heading_error(), self._effective_yaw_rate(now)
        error = self.predicted_error_mm(now)
        position_feedback = None
        phase = "RETURN" if recovery else "FORWARD"
        if recovery:
            desired = 0.
        else:
            position = math.copysign(max(0., abs(error or 0.) - PATROL_SIDE_DEADBAND_MM / 2.), error or 0.)
            if position == 0. or position * self._raw_error <= 0:
                position = math.copysign(max(0., abs(self._raw_error) - PATROL_SIDE_DEADBAND_MM / 2.), self._raw_error)
            trend = self._error_rate
            desired = psi + math.degrees(math.atan2(position + .5 * trend, sequence.CENTER_LOOKAHEAD_MM))
            position_feedback = dict(laser_error_mm=error, laser_error_rate_mmps=trend,
                                     trend_horizon_s=.5, source_at=self._pair_at,
                                     source_keys=tuple(self._paired_keys))
        requested = ((sequence.CENTER_FORWARD_SHIFT_YAW_GAIN_PER_S if recovery
                      else self.PATROL_ALIGNMENT_LASER_GAIN_PER_S) * math.radians(desired - psi)
                     - sequence.CENTER_YAW_RATE_DAMPING * math.radians(rate or 0.)) * 1000.
        if not recovery and abs(psi) > self.patrol_heading_limit_deg + 1e-9:
            desired, phase = 0., "RETURN"
            requested = (-sequence.CENTER_FORWARD_SHIFT_YAW_GAIN_PER_S * math.radians(psi)
                         - sequence.CENTER_YAW_RATE_DAMPING * math.radians(rate or 0.)) * 1000.
        confirmed = (self._applied_linear > 0 and sequence._motion_wheels(
            self._applied_linear, self._applied_angular) == self._last_sent_wheels)
        angular, residual, residual_sign = self._fine_angular_proposal(requested, now, enabled=confirmed)
        if not self._patrol_alignment_angle_safe(angular, now):
            # Recompute an inward command instead of issuing BRAKE at the
            # angle boundary. Only freshness and real faults stop this stage.
            desired, phase = 0., "RETURN"
            requested = (-sequence.CENTER_FORWARD_SHIFT_YAW_GAIN_PER_S * math.radians(psi)
                         - sequence.CENTER_YAW_RATE_DAMPING * math.radians(rate or 0.)) * 1000.
            if requested * psi >= 0 and psi:
                requested = -sequence.CENTER_FORWARD_SHIFT_YAW_GAIN_PER_S * math.radians(psi) * 1000.
            angular, residual, residual_sign = self._fine_angular_proposal(requested, now, enabled=confirmed)
        linear, angular, reason, speed_state, estimate, single_zero = self._patrol_speed_and_differential(
            goal, now, angular, allow_single_zero=not recovery and phase != "RETURN")
        state = dict(speed_state, _patrol_correction_phase=phase, _hold_locked=False,
                     _angular_residual_mradps=residual, _angular_residual_request_sign=residual_sign,
                     _control_feedback=dict(raw_difference_mm=self._last_raw_difference,
                         fixed_reference_deg=self._road_reference, actual_heading_offset_deg=psi,
                         desired_heading_deg=desired, yaw_rate_degps=rate, position_control=position_feedback,
                         selected_linear_mmps=linear, selected_angular_mradps=angular,
                         correction_heading_limit_deg=self.patrol_heading_limit_deg,
                         correction_phase=phase,
                         basis=("超过道路参考允许偏角，差速优先回正至±5°"
                                if recovery else "目标均速保持；原始激光误差与趋势优先，方向辅助")))
        proposal = dict(epoch=self._motion_epoch, token=self.observation_token, at=now,
                        identity=self._state_session, state=state, patrol_alignment=True,
                        patrol_heading_recovery=recovery, patrol_short_correction=single_zero, departure=recovery,
                        cached=False, normal=False, qualifies=False)
        return sequence.CenteringTarget(int(linear), int(angular),
                                        "PATROL_HEADING_RECOVERY" if recovery else "PATROL_ALIGN_FORWARD",
                                        self._error, psi, rate, self.axis_trusted, False, proposal,
                                        limited_reason=reason, estimated_speed_mmps=estimate,
                                        normal_motion_allowed=False)

    def target_send_ready(self, target, now):
        if not super().target_send_ready(target, now):
            return False
        proposal = target._proposal or {}
        if (self._patrol_cruise_policy and self._patrol_road_policy in ("ORDINARY", "SHORT")
                and proposal.get('patrol_road_adjustment')
                and not proposal.get('patrol_heading_recovery')
                and abs(self._heading_error() or 0.) > self.patrol_heading_limit_deg + 1e-9):
            return False
        if (self._patrol_cruise_policy and self._patrol_road_policy in ("ORDINARY", "SHORT")
                and not proposal.get('cached') and not proposal.get('departure')):
            feedback = proposal.get('state', {}).get('_control_feedback', {})
            if (feedback.get('correction_phase') != 'RETURN'
                    and abs(self._heading_error() or 0.) > self.patrol_heading_limit_deg + 1e-9):
                return False
        reference = proposal.get('state', {}).get('_control_feedback', {}).get('fixed_reference_deg')
        if reference is not None and abs(self._angle(reference - self._road_reference)) > 1e-9:
            return False
        if self._patrol_cruise_policy and (target._proposal or {}).get('patrol_laser_hold'):
            return (not self._patrol_heading_recovery
                    and abs(self._heading_error() or 0.) <= self.patrol_heading_limit_deg + 1e-9)
        if self._patrol_cruise_policy and (target._proposal or {}).get("patrol_alignment"):
            if target._proposal.get("patrol_alignment_cached"):
                return (target._proposal.get("alignment_hold") == self._patrol_alignment_hold
                        and self._patrol_alignment_hold_ready(now))
            return ((target._proposal.get("patrol_heading_recovery") or self._patrol_alignment_ready(now))
                    and self._patrol_alignment_angle_safe(target.angular_mradps, now))
        return True

    def adopt_moving_reference(self, mean_deg, now):
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in (mean_deg, now)):
            return False
        self.refresh_yaw_rate(now)
        if (not self.centered or self._state_session is None
                or None in self._state_session or self._pair_at is None
                or self._last_pair_pose is None or self._road_reference is None):
            return False

        # Re-express the already accepted raw source pair.  Its source times,
        # keys, pose and observation token stay unchanged: changing a frame
        # must never manufacture another laser measurement or drive witness.
        raw_sides = [self._side_at(history, self._pair_at)
                     for history in self._side_history]
        if any(side is None for side in raw_sides):
            return False
        raw_difference = raw_sides[0][0] - raw_sides[1][0]
        reference = self._angle(float(mean_deg))
        psi = math.radians(self._angle(self._last_pair_pose.yaw - reference))
        observed = (raw_difference / 2.0 if self._patrol_cruise_policy else
                    raw_difference * math.cos(psi) / 2.0
                    + sequence.CENTER_SENSOR_FORWARD_MM * math.sin(psi))
        if not math.isfinite(observed):
            return False

        self._road_reference = self._hold_heading = reference
        self._initial_reference_fixed = True
        self._hold_locked = False
        self._motion_epoch += 1
        self._position_integral_deg = 0.0
        self._position_integral_key = self._position_integral_source_at = None
        self._angular_residual_mradps = 0.0
        self._angular_residual_request_sign = 0
        self._lateral_sources.clear()
        self._good_windows = 0
        self._mean_last_end = None
        self._mean_source_keys = (None, None)
        self._pending_means.clear()
        self._compensated_sources.clear()
        self._last_mean_difference = self._last_mean_heading = None
        self._mean_reason = "运动1秒航向均值已采纳；旧参考下的居中均值证据清除"
        self._last_raw_difference = raw_difference
        self._raw_error = self._observed_error = self._error = observed
        self._error_rate = 0.0
        self._control_feedback = None
        return True

    def adopt_post_turn_reference(self, mean_deg, now):
        """Adopt a completed fixed window without another centering gate.

        Reframing retained real measurements does not renew their clocks or
        authorize motion during a stopped/fault recovery.
        """
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in (mean_deg, now)):
            return False
        if (self._state_session is None or None in self._state_session
                or self._road_reference is None):
            return False
        self.refresh_yaw_rate(now)
        reference = self._angle(float(mean_deg))
        observed = raw_difference = None
        if self._pair_at is not None and self._last_pair_pose is not None:
            raw_sides = [self._side_at(history, self._pair_at)
                         for history in self._side_history]
            if all(side is not None for side in raw_sides):
                raw_difference = raw_sides[0][0] - raw_sides[1][0]
                psi = math.radians(self._angle(self._last_pair_pose.yaw - reference))
                observed = (raw_difference / 2.0 if self._patrol_cruise_policy else
                            raw_difference * math.cos(psi) / 2.0
                            + sequence.CENTER_SENSOR_FORWARD_MM * math.sin(psi))
        self._road_reference = self._hold_heading = reference
        self._initial_reference_fixed = True
        self._hold_locked = False
        self._motion_epoch += 1
        self._position_integral_deg = 0.0
        self._position_integral_key = self._position_integral_source_at = None
        self._angular_residual_mradps = 0.0
        self._angular_residual_request_sign = 0
        self._lateral_sources.clear()
        self._good_windows = 0
        self._mean_last_end = None
        self._mean_source_keys = (None, None)
        self._pending_means.clear()
        self._compensated_sources.clear()
        self._last_mean_difference = self._last_mean_heading = None
        self._mean_reason = "转后固定1秒航向均值已采纳；旧参考均值证据清除"
        self._last_raw_difference = raw_difference
        self._raw_error = self._observed_error = self._error = observed
        self._error_rate = 0.0
        if observed is None:
            self._usable_pair = False
        self._control_feedback = None
        return True


def _send_patrol_feedback_target(link, display, linear, angular, deadline=math.inf, *, target=None):
    """Stage-one CONTROL_TARGET adapter; BRAKE and stable transitions stay explicit."""
    controller = display.controller
    if not getattr(controller, "_patrol_cruise_policy", False):
        return sequence._send_feedback_target(link, display, linear, angular, deadline, target=target)
    if getattr(link, "_patrol_turn_brake_required", False) and (linear or angular):
        raise sequence._CenteringProposalExpired("转弯路口已抢占控制，拒绝旧非零目标")
    if (isinstance(linear, bool) or isinstance(angular, bool)
            or not isinstance(linear, int) or not isinstance(angular, int)
            or angular % sequence.CENTER_ANGULAR_QUANTUM_MRADPS or abs(angular) > 1568):
        raise mcu.McuError("阶段一纠偏需要合法整数v/w、16mrad/s量化及1568mrad/s限幅")
    wheels = sequence._motion_wheels(linear, angular)
    alignment = target is not None and (target._proposal or {}).get("patrol_alignment", False)
    if (linear <= 0 or any(wheel != 0 and not 61 <= wheel <= sequence.MAX_WHEEL_SPEED_MMPS for wheel in wheels)):
        raise mcu.McuError("阶段一纠偏轮速须为0或61～310正向轮速")
    if _patrol_zero_transition(display.wheel_targets, wheels):
        raise mcu.McuError("阶段一已驱动轮归零或换向前须BRAKE并严格停稳")
    sequence._remaining(deadline)
    if target is not None and not controller.target_send_ready(target, time.monotonic()):
        raise sequence._CenteringProposalExpired("阶段一发令前新鲜反馈、身份或偏角已改变")
    if isinstance(link, PatrolLink) and target is not None:
        previous_guard = getattr(link, "_patrol_feedback_write_guard", None)
        link._patrol_feedback_write_guard = controller, target, deadline
        try:
            number = link.send_motion(linear, angular, require_ranges=True, require_upper=False)
        finally:
            link._patrol_feedback_write_guard = previous_guard
    else:
        number = link.send_motion(linear, angular, require_ranges=True, require_upper=False)
    link._stop_tracker = link._stable_barrier = link._brake_evidence_barrier = None
    link._ready_side_pairs = []
    display.command, display.wheel_targets = (linear, angular), wheels
    controller.notify_sent(angular, sent_at=time.monotonic())
    display.wheel_delta = wheels[1] - wheels[0]
    display.raw_delta = float(display.wheel_delta)
    display.command_kind = sequence.MessageType.CONTROL_TARGET
    runlog.emit("motion_sequence_command", "阶段一激光优先纠偏目标发送",
                stage=("HEADING_RECOVERY" if target is not None and (target._proposal or {}).get("patrol_heading_recovery")
                       else "ALIGN" if alignment else "NORMAL"), sequence=number,
                linear_mmps=linear, angular_mradps=angular, wheels_mmps=wheels)
    return number


class MovingHeadingSampler:
    """Collect a fresh one-second moving yaw mean after original centering.

    STATE source poses already include the frozen stopped calibration.  A
    moving window updates direction only; it never estimates gyro zero bias.
    Call cancel() before a turn or a junction action.  A completed sampler
    produces one mean until the outer road state starts a new sampling run.
    """

    def __init__(self):
        self.cancel()

    def cancel(self):
        self._started_at = self._now = None
        self._identity = None
        self._reference = None
        self._last_key = self._last_source_at = None
        self._samples = []
        self._completed = False

    @property
    def active(self):
        return self._started_at is not None and not self._completed

    @property
    def elapsed_s(self):
        return (max(0.0, self._now - self._started_at)
                if self._started_at is not None and self._now is not None else 0.0)

    @property
    def sample_count(self):
        return len(self._samples)

    def update(self, controller, state_samples, now, *, soft_missing=False):
        if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
            self.cancel()
            return None
        controller.refresh_yaw_rate(now)
        if self._completed:
            return None
        identity = getattr(controller, "_state_session", None)
        if (soft_missing or not controller.centered or identity is None
                or None in identity or controller._stopped
                or controller._applied_linear <= 0):
            self.cancel()
            return None
        if self._identity is not None and self._identity != identity:
            self.cancel()
        if (self._reference is not None and abs(controller._angle(
                controller.heading_reference_deg - self._reference)) > 1e-9):
            self.cancel()
            return None
        if self._started_at is None:
            self._started_at = now
            self._identity = identity
            self._reference = controller.heading_reference_deg
        self._now = now
        max_age_s = sequence.config.TELEMETRY_STALE_MS / 1000.0
        if (self._last_source_at is not None
                and now - self._last_source_at > max_age_s + 1e-9):
            self.cancel()
            return None

        for state, source_at in state_samples:
            key = getattr(state, "uptime_ms", None)
            sample_identity = (getattr(state, "session_nonce", None),
                               getattr(state, "boot_id", None))
            if (sample_identity != identity or isinstance(key, bool) or not isinstance(key, int)
                    or isinstance(source_at, bool) or not isinstance(source_at, (int, float))
                    or not math.isfinite(source_at) or source_at <= self._started_at + 1e-9
                    or not -1e-9 <= now - source_at <= max_age_s + 1e-9
                    or getattr(state, "odom_status", None) != 0
                    or not getattr(state, "state_flags", 0) & sequence.StateFlag.ODOM_VALID
                    or not sequence._source_key_newer(key, self._last_key)
                    or self._last_source_at is not None and source_at <= self._last_source_at):
                continue
            if (self._last_source_at is not None
                    and source_at - self._last_source_at > max_age_s + 1e-9):
                self.cancel()
                return None
            pose = controller.pose_at(source_at)
            if pose is None or not math.isfinite(pose.yaw):
                continue
            if abs(controller._angle(pose.yaw - self._reference)) > sequence.NORMAL_HEADING_TOLERANCE_DEG + 1e-9:
                self.cancel()
                return None
            self._samples.append((source_at, pose.yaw))
            self._last_key, self._last_source_at = key, source_at

        if now - self._started_at + 1e-9 < sequence.START_SAMPLE_S:
            return None
        window_start = now - sequence.START_SAMPLE_S
        self._samples = [sample for sample in self._samples if sample[0] > window_start + 1e-9]
        if (len(self._samples) < 15
                or self._samples[-1][0] - self._samples[0][0] < .8 - 1e-9):
            return None
        self._completed = True
        mean = math.fsum(yaw for _, yaw in self._samples) / len(self._samples)
        return controller._angle(mean)


class PostTurnHeadingSampler:
    """One fixed moving window; invalid sources are discarded, never retried."""

    def __init__(self):
        self.cancel()

    def cancel(self):
        self._started_at = self._now = self._deadline = None
        self._identity = self._reference = None
        self._samples = []
        self._seen = set()
        self._last_key = self._last_source_at = None
        self._events = deque(maxlen=256)
        self._completed = False
        self._mean = None
        self._rejected = 0
        self._reference_invalidated = False

    @property
    def active(self):
        return self._started_at is not None and not self._completed

    @property
    def completed(self):
        return self._completed

    @property
    def started_at(self):
        return self._started_at

    @property
    def deadline(self):
        return self._deadline

    @property
    def elapsed_s(self):
        return (min(sequence.START_SAMPLE_S, max(0.0, self._now - self._started_at))
                if self._started_at is not None and self._now is not None else 0.0)

    @property
    def sample_count(self):
        return len(self._samples)

    @property
    def rejected_count(self):
        return self._rejected

    @property
    def mean_deg(self):
        return self._mean

    def reference_matches(self, controller):
        reference = controller.heading_reference_deg
        return (not self._reference_invalidated and controller._state_session == self._identity
                and reference is not None and self._reference is not None
                and abs(controller._angle(reference - self._reference)) <= 1e-9)

    def start(self, controller, now):
        if self._started_at is not None or self._completed:
            return False
        if (isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now)
                or not controller.centered or controller._stopped
                or controller._applied_linear <= 0
                or controller._state_session is None or None in controller._state_session):
            return False
        self._started_at = self._now = float(now)
        self._deadline = self._started_at + sequence.START_SAMPLE_S
        self._identity = controller._state_session
        self._reference = controller.heading_reference_deg
        return True

    def observe_side(self, controller, sample):
        """Record each snapshot's source-time condition after its consumption."""
        if not self.active:
            return
        ages = sample.raw_ages_ms if sample.raw_ages_ms is not None else sample.ages_ms
        values = (*sample.source_times, *ages)
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in values):
            return
        at = max(source + age / 1000.0 for source, age in zip(sample.source_times, ages))
        sources = tuple(sample.source_times)
        good = (sample.source_identity == self._identity and sample.dual_wall
                and all(0 <= at - source <= .250 + 1e-9 for source in sources)
                and all(isinstance(value, (int, float)) and not isinstance(value, bool)
                        and math.isfinite(value) and 0 <= value < 65535
                        for value in (sample.left_mm, sample.right_mm)))
        pair_at = min(sources)
        pose = controller.pose_at(pair_at) if good else None
        aligned = ([controller._side_at(history, pair_at) for history in controller._side_history]
                   if pose is not None else [])
        if good and pose is not None and len(aligned) == 2 and all(item is not None for item in aligned):
            difference = aligned[0][0] - aligned[1][0]
            psi = math.radians(controller._angle(pose.yaw - self._reference))
            error = difference * math.cos(psi) / 2.0 + sequence.CENTER_SENSOR_FORWARD_MM * math.sin(psi)
        else:
            error = None
        self._events.append((at, good and error is not None, sources, pose, error))

    def _source_error(self, controller, source_at, pose):
        events = [event for event in self._events if event[0] <= source_at + 1e-9]
        if events:
            _, good, sources, sensed, error = max(events, key=lambda event: event[0])
            if not good or any(not -1e-9 <= source_at - at <= .250 + 1e-9 for at in sources):
                return None
        else:
            # Standalone callers and the interval before the next snapshot use
            # only already accepted, source-aligned real laser observations.
            pairs = [item for item in controller._compensated_sources
                     if all(at <= source_at + 1e-9 for at in item[1])]
            if not pairs:
                return None
            pair_at, sources, _keys, error, _yaw = max(pairs, key=lambda item: item[0])
            if any(not -1e-9 <= source_at - at <= .250 + 1e-9 for at in sources):
                return None
            sensed = controller.pose_at(pair_at)
            if sensed is None:
                return None
        axis = math.radians(self._reference)
        lateral = (-(pose.x - sensed.x) * math.sin(axis)
                   + (pose.y - sensed.y) * math.cos(axis))
        return error - lateral

    def update(self, controller, state_samples, now, *, soft_missing=False,
               side_samples=(), faulted=False):
        if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
            return None
        controller.refresh_yaw_rate(now)
        if self._completed:
            return None
        if not self.active and (soft_missing or faulted or not self.start(controller, now)):
            return None
        self._now = float(now)
        if not self.reference_matches(controller):
            self._reference_invalidated = True
            self._samples.clear()
            self._mean = None
        for sample in side_samples:
            self.observe_side(controller, sample)
        for observation in state_samples:
            state, source_at = observation[:2]
            identity = (getattr(state, "session_nonce", None), getattr(state, "boot_id", None))
            key = getattr(state, "uptime_ms", None)
            if isinstance(key, bool) or not isinstance(key, int):
                self._rejected += 1
                continue
            witness = identity, key
            if witness in self._seen:
                continue
            self._seen.add(witness)
            values = (getattr(state, "left_mm", None), getattr(state, "right_mm", None),
                      getattr(state, "yaw_tenth_deg", None), source_at)
            flags = getattr(state, "state_flags", None)
            raw_status = getattr(state, "odom_status", None)
            faults = getattr(state, "fault_bits", 0)
            source_valid = len(observation) < 3 or observation[2]
            finite_source = (isinstance(source_at, (int, float)) and not isinstance(source_at, bool)
                             and math.isfinite(source_at))
            newer = (sequence._source_key_newer(key, self._last_key)
                     and finite_source and (self._last_source_at is None
                                            or source_at > self._last_source_at + 1e-9))
            if identity == self._identity and newer:
                # An unusable observation still consumes its source progress.
                self._last_key, self._last_source_at = key, source_at
            good = (identity == self._identity and source_valid and not faulted
                    and not soft_missing and newer and not self._reference_invalidated
                    and all(isinstance(value, (int, float)) and not isinstance(value, bool)
                            and math.isfinite(value) for value in values)
                    and isinstance(flags, int) and not isinstance(flags, bool)
                    and isinstance(raw_status, int) and not isinstance(raw_status, bool)
                    and isinstance(faults, int) and not isinstance(faults, bool) and faults == 0
                    and raw_status == 0 and flags & sequence.StateFlag.ODOM_VALID
                    and self._started_at + 1e-9 < source_at <= self._deadline + 1e-9
                    and -1e-9 <= now - source_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9)
            if good:
                if flags & sequence.StateFlag.APPLIED_CONTROL_PRESENT:
                    applied = getattr(state, "target_linear_mms", None)
                    good = (isinstance(applied, (int, float)) and not isinstance(applied, bool)
                            and math.isfinite(applied) and applied > 0)
                else:
                    good = controller._applied_linear > 0 and not controller._stopped
            pose = controller.pose_at(source_at) if good else None
            error = self._source_error(controller, source_at, pose) if pose is not None else None
            if (pose is None or not math.isfinite(pose.yaw) or error is None
                    or 2.0 * abs(error) > (PATROL_SIDE_DEADBAND_MM if getattr(controller, "_patrol_cruise_policy", False)
                                           else sequence.SIDE_DEADBAND_MM) + 1e-9
                    or abs(controller._angle(pose.yaw - self._reference))
                    > sequence.NORMAL_HEADING_TOLERANCE_DEG + 1e-9):
                self._rejected += 1
                continue
            self._samples.append((source_at, pose.yaw))
        if now + 1e-9 < self._deadline:
            return None
        self._completed = True
        if self._samples:
            self._mean = controller._angle(math.fsum(yaw for _, yaw in self._samples) / len(self._samples))
        return self._mean


class PatrolRoadHeadingSampler(PostTurnHeadingSampler):
    """Stage 1 only: at most eleven fresh aligned laser-cycle yaw samples."""

    def reference_matches(self, controller):
        reference = controller.heading_reference_deg
        return (not self._reference_invalidated and controller._state_session == self._identity
                and reference is not None and self._started_at is not None
                and abs(controller._angle(reference - self._expected_reference)) <= 1e-9)

    def start(self, controller, now):
        if (self._started_at is not None or self._completed
                or not isinstance(now, (int, float)) or isinstance(now, bool)
                or not math.isfinite(now) or controller.heading_reference_deg is None
                or controller._state_session is None or None in controller._state_session):
            return False
        self._started_at = self._now = float(now)
        self._identity = controller._state_session
        self._reference = controller.heading_reference_deg
        self._expected_reference = self._reference
        self._deadline = math.inf
        self._invalid_state_sources = set()
        return True

    @property
    def elapsed_s(self):
        return (max(0., self._now - self._started_at)
                if self._started_at is not None and self._now is not None else 0.)

    def _source_error(self, controller, source_at, pose):
        events = [event for event in self._events if event[0] <= source_at + 1e-9]
        if events:
            _, good, sources, _sensed, error = max(events, key=lambda item: item[0])
            return error if good and all(-1e-9 <= source_at - at <= .250 + 1e-9 for at in sources) else None
        pairs = [item for item in controller._compensated_sources
                 if all(at <= source_at + 1e-9 for at in item[1])]
        if not pairs:
            return None
        pair = max(pairs, key=lambda item: item[0])
        return pair[3] if all(-1e-9 <= source_at - at <= .250 + 1e-9 for at in pair[1]) else None

    def observe_side(self, controller, sample):
        if not self.active:
            return
        ages = sample.raw_ages_ms if sample.raw_ages_ms is not None else sample.ages_ms
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in (*sample.source_times, *ages)):
            return
        at = max(source + age / 1000. for source, age in zip(sample.source_times, ages))
        sources = tuple(sample.source_times)
        good = (sample.source_identity == self._identity and sample.dual_wall
                and all(0 <= at - source <= .250 + 1e-9 for source in sources)
                and all(isinstance(value, (int, float)) and not isinstance(value, bool)
                        and math.isfinite(value) and 0 <= value < 65535
                        for value in (sample.left_mm, sample.right_mm)))
        pair_at = min(sources)
        pose = controller.pose_at(pair_at) if good else None
        aligned = [controller._side_at(history, pair_at) for history in controller._side_history] if pose else []
        error = (aligned[0][0] - aligned[1][0]) / 2. if len(aligned) == 2 and all(item is not None for item in aligned) else None
        self._events.append((at, good and error is not None, sources, pose, error))

    def update(self, controller, state_samples, now, **kwargs):
        if (not isinstance(now, (int, float)) or isinstance(now, bool) or not math.isfinite(now)
                or self._completed):
            return None
        if not self.active and not self.start(controller, now):
            return None
        self._now = float(now)
        if not self.reference_matches(controller):
            self._reference_invalidated = True
            return None
        invalid_sources = self._invalid_state_sources
        for observation in state_samples:
            state, source_at = observation[:2]
            flags = getattr(state, "state_flags", 0)
            if (len(observation) >= 3 and not observation[2]
                    or getattr(state, "odom_status", None) != 0
                    or not isinstance(flags, int) or not flags & sequence.StateFlag.ODOM_VALID
                    or getattr(state, "fault_bits", 0)):
                invalid_sources.add(source_at)
        changed = False
        for at, sources, keys, error, yaw in controller._compensated_sources:
            witness = self._identity, tuple(keys)
            if witness in self._seen or at <= self._started_at + 1e-9:
                continue
            self._seen.add(witness)
            fresh = (all(isinstance(value, (int, float)) and not isinstance(value, bool)
                         and math.isfinite(value) for value in (*sources, error, yaw))
                     and all(-1e-9 <= now - source <= .250 + 1e-9 for source in sources)
                     and (controller.source_barrier is None
                          or all(source > controller.source_barrier + 1e-9 for source in sources))
                     and (self._last_key is None or all(sequence._source_key_newer(key, previous)
                          for key, previous in zip(keys, self._last_key))))
            self._last_key = tuple(keys)
            if (not fresh or kwargs.get("soft_missing", False) or kwargs.get("faulted", False)
                    or at in invalid_sources or controller.pose_at(at) is None
                    or 2. * abs(error) > PATROL_SIDE_DEADBAND_MM + 1e-9
                    or abs(controller._angle(yaw - self._reference))
                       > controller.PATROL_HEADING_LIMIT_DEG + 1e-9):
                self._rejected += 1
                continue
            self._samples.append((at, yaw))
            changed = True
            if len(self._samples) >= 2:
                # Equal weighting, with angles unwrapped around the fixed entry anchor.
                count = len(self._samples)
                previous = (controller._angle(self._samples[0][1] - self._reference)
                            if self._mean is None else controller._angle(self._mean - self._reference))
                offset = ((count - 1) * previous + controller._angle(yaw - self._reference)) / count
                self._mean = controller._angle(self._reference + offset)
            if len(self._samples) == 11:
                self._completed = True
                break
        return self._mean if changed and len(self._samples) >= 2 else None


class StoppedHeadingSampler:
    """One fixed source-time window anchored to a successful BRAKE write."""

    def __init__(self):
        self.cancel()

    def cancel(self):
        self._started_at = self._deadline = self._identity = self._reference = None
        self._last_key = self._last_source = None
        self._samples = []
        self._completed = False
        self._mean = None

    @property
    def active(self):
        return self._started_at is not None and not self._completed

    @property
    def completed(self):
        return self._completed

    @property
    def started_at(self):
        return self._started_at

    @property
    def deadline(self):
        return self._deadline

    @property
    def sample_count(self):
        return len(self._samples)

    @property
    def mean_deg(self):
        return self._mean

    def reference_matches(self, controller):
        return (self._identity == controller._state_session
                and self._reference is not None and controller.heading_reference_deg is not None
                and abs(controller._angle(controller.heading_reference_deg - self._reference)) <= 1e-9)

    def start(self, controller, started_at):
        self.cancel()
        identity = controller._state_session
        if (identity is None or None in identity or controller.heading_reference_deg is None
                or isinstance(started_at, bool) or not isinstance(started_at, (int, float))
                or not math.isfinite(started_at)):
            return False
        self._started_at, self._deadline = started_at, started_at + sequence.START_SAMPLE_S
        self._identity, self._reference = identity, controller.heading_reference_deg
        return True

    def update(self, controller, states, now, *, faulted=False):
        if self._started_at is None:
            return None
        if faulted or not self.reference_matches(controller):
            self.cancel()
            return None
        if self._completed:
            return self._mean
        for observation in states:
            state, source_at = observation[:2]
            identity = getattr(state, 'session_nonce', None), getattr(state, 'boot_id', None)
            if identity != self._identity:
                continue
            if (getattr(state, 'fault_bits', 0) & ~(config.MCU_IGNORED_RUNTIME_FAULT_BITS | 0x00000F3F)
                    or len(observation) > 2 and not observation[2]):
                self.cancel()
                return None
            key = getattr(state, 'uptime_ms', None)
            yaw_raw = getattr(state, 'yaw_tenth_deg', None)
            wheels = getattr(state, 'left_mm', None), getattr(state, 'right_mm', None)
            if (isinstance(key, bool) or not isinstance(key, int)
                    or type(yaw_raw) is not int or not -1800 <= yaw_raw <= 1799
                    or any(type(wheel) is not int or not -2147483648 <= wheel <= 2147483647
                           for wheel in wheels)
                    or isinstance(source_at, bool) or not isinstance(source_at, (int, float))
                    or not math.isfinite(source_at)
                    or not self._started_at + 1e-9 < source_at <= self._deadline + 1e-9
                    or not -1e-9 <= now - source_at <= .250 + 1e-9
                    or not sequence._source_key_newer(key, self._last_key)
                    or self._last_source is not None and source_at <= self._last_source
                    or getattr(state, 'odom_status', None) != 0
                    or not getattr(state, 'state_flags', 0) & sequence.StateFlag.ODOM_VALID):
                continue
            pose = controller.pose_at(source_at)
            if pose is None or not math.isfinite(pose.yaw):
                continue
            self._samples.append((source_at, pose.yaw))
            self._last_key, self._last_source = key, source_at
        if now + 1e-9 < self._deadline:
            return None
        self._completed = True
        if (len(self._samples) >= 15
                and self._samples[-1][0] - self._samples[0][0] >= .8 - 1e-9):
            sine = math.fsum(math.sin(math.radians(yaw)) for _, yaw in self._samples)
            cosine = math.fsum(math.cos(math.radians(yaw)) for _, yaw in self._samples)
            if math.hypot(sine, cosine) > 1e-9:
                self._mean = controller._angle(math.degrees(math.atan2(sine, cosine)))
        return self._mean


def _check_patrol_sides(sample, now=None, **_kwargs):
    """Validate wire fields without treating legal missing states as terminal."""
    for distance, status, age in zip((sample.left_mm, sample.right_mm),
                                     (sample.left_status, sample.right_status), sample.ages_ms):
        if not isinstance(status, sequence.WireRangeStatus):
            raise mcu.McuError("下层激光非法状态字段")
        if isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
            raise mcu.McuError("下层激光源龄非法")
        if status == sequence.WireRangeStatus.VALID and (
                isinstance(distance, bool) or not isinstance(distance, (int, float))
                or not math.isfinite(distance) or not 0 <= distance < 65535):
            raise mcu.McuError("下层激光VALID距离字段非法")


@dataclass(frozen=True)
class PatrolJunctionEvidence:
    kind: str
    identity: tuple
    confirmed_at: float
    snapshot_uptime: int | None
    openings: tuple[str, ...]
    start_distance_mm: float | None
    confirmed_distance_mm: float | None

    @property
    def source_at(self):
        return self.confirmed_at


class PatrolJunctionDetector:
    """Independent raw opening and advancing missing-snapshot evidence."""
    def __init__(self):
        self.reset(600, 1000)

    def reset(self, window_min_mm, window_max_mm, identity=None, require_window=True):
        self.minimum, self.maximum = window_min_mm, window_max_mm
        self.identity, self.require_window = identity, require_window
        self._snap = None
        self._explicit_snap = None
        self._open_keys = [None, None]
        self._episodes = [None, None]
        self._valid_seen = False

    def observe_explicit(self, sample, source_at, distance_mm=None, *, now=None,
                         source_barrier=None):
        _check_patrol_sides(sample)
        check_at = source_at if now is None else now
        if not 0 <= check_at - source_at <= .250 + 1e-9:
            return None
        identity = sample.source_identity
        if identity is None or None in identity:
            return None
        if self.identity is None:
            self.identity = identity
        elif self.identity != identity:
            return None
        if not sequence._snapshot_newer(sample, self._explicit_snap):
            return None
        self._explicit_snap = sample
        explicit = []
        for i, status in enumerate((sample.left_status, sample.right_status)):
            name = ("left", "right")[i]
            key = sample.source_keys[i] if sample.source_keys is not None else sample.seq
            fresh = 0 <= check_at - sample.source_times[i] <= .250 + 1e-9
            advanced = sequence._source_key_newer(key, self._open_keys[i])
            if status in (sequence.WireRangeStatus.NO_TARGET, sequence.WireRangeStatus.OUT_OF_RANGE):
                if advanced:
                    self._open_keys[i] = key
                if (fresh and advanced and (source_barrier is None
                        or sample.source_times[i] > source_barrier + 1e-9)):
                    explicit.append(name)
                continue
            if status == sequence.WireRangeStatus.VALID:
                if advanced:
                    self._open_keys[i] = key
        if explicit:
            return PatrolJunctionEvidence('explicit_opening', identity, source_at,
                sample.snapshot_uptime_ms, tuple(explicit), distance_mm, distance_mm)
        return None

    def observe(self, sample, source_at, distance_mm, has_dual_valid, *, now=None):
        explicit = self.observe_explicit(sample, source_at, distance_mm, now=now)
        if explicit is not None:
            return explicit
        check_at = source_at if now is None else now
        if (not 0 <= check_at - source_at <= .250 + 1e-9
                or sample.source_identity != self.identity
                or sample.source_identity is None or None in sample.source_identity
                or not sequence._snapshot_newer(sample, self._snap)):
            return None
        self._snap = sample
        self._valid_seen |= bool(has_dual_valid)
        eligible = (not self.require_window or distance_mm is not None
                    and self.minimum <= distance_mm <= self.maximum)
        snapshot_at = source_at
        for i, status in enumerate((sample.left_status, sample.right_status)):
            if status in sequence.HEALTHY_RANGE_STATUSES:
                self._episodes[i] = None
                continue
            # 3/4/5/6 use snapshot progression; their measurement key is stale.
            episode = self._episodes[i]
            if episode is not None and (snapshot_at - episode['last'] > .250 + 1e-9
                                        or snapshot_at <= episode['last']):
                # A gap restarts duration/count; it cannot move the onset of
                # an already observed continuous missing episode into a window.
                self._episodes[i] = dict(start=snapshot_at, last=snapshot_at, count=1,
                    eligible=episode['eligible'], distance=episode['distance'])
                continue
            if episode is None:
                self._episodes[i] = dict(start=snapshot_at, last=snapshot_at, count=1,
                    eligible=eligible and self._valid_seen, distance=distance_mm)
            else:
                episode['last'] = snapshot_at
                episode['count'] += 1
        for i, episode in enumerate(self._episodes):
            if (episode is not None and episode['eligible'] and episode['count'] >= 3
                    and snapshot_at - episode['start'] >= .300 - 1e-9):
                return PatrolJunctionEvidence('missing_inference', self.identity, snapshot_at,
                    sample.snapshot_uptime_ms, (("left", "right")[i],), episode['distance'], distance_mm)
        return None


def _brake_evidence_at(link) -> float | None:
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    if getattr(link, "_last_brake_identity", None) != identity:
        return None
    key = sequence.MessageType.MOTION_ACTION, getattr(link, "_last_brake_sequence", None)
    at = getattr(link, "_confirmed_evidence_times", {}).get(key)
    if at is None or not math.isfinite(at):
        return None
    # The real UART ledger records full-frame ACK and applied STATE receive times.
    if isinstance(link, PatrolLink):
        record = link._command_ledger.get(key)
        witness_key = link._superseded_commands.get(key, key)
        witness = link._command_ledger.get(witness_key)
        if (record is None or witness is None or tuple(record[6:8]) != identity
                or witness[4:8] != record[4:8] or witness[0] < record[0]
                or witness[2] is None or witness[8] is None
                or witness[8].status is not sequence.AckStatus.ACCEPTED
                or witness[2].target_linear_mms or witness[2].target_angular_mrads
                or witness[3] is None or witness[9] is None
                or at != max(witness[3], witness[9])
                or not record[0] <= at <= record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S + 1e-9):
            return None
    return at


def _latch_patrol_junction(link, sample, *, initial=False):
    """Save real openings without rejecting a missing reading on the other side."""
    now = time.monotonic()
    _check_patrol_sides(sample, now, check_age=False, allow_stale=True, allow_timeout=True)
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    if sample.source_identity != identity:
        raise mcu.McuError("路口样本身份不属于当前会话")
    openings = (set(getattr(link, "_junction_openings", ()))
                if getattr(link, "_junction_identity", None) == identity else set())
    keys = sample.source_keys or sample.source_times
    previous = (list(getattr(link, "_junction_side_sources", (None, None)))
                if openings else [None, None])
    for index, (name, status) in enumerate((("left", sample.left_status), ("right", sample.right_status))):
        fresh = (status in sequence.HEALTHY_RANGE_STATUSES
                 and 0 <= now - sample.source_times[index] <= .250 + 1e-9
                 and sequence._source_key_newer(keys[index], previous[index]))
        if fresh:
            previous[index] = keys[index]
            if (status in (sequence.WireRangeStatus.NO_TARGET, sequence.WireRangeStatus.OUT_OF_RANGE)
                    and (openings or initial and name in sample.candidates)):
                openings.add(name)
    if not openings:
        if not initial:
            return
        raise mcu.McuError("路口候选缺少250ms内真正新鲜开口")
    link._junction_identity = identity
    link._junction_openings = frozenset(openings)
    link._junction_side_sources = previous
    sequence._clear_side_recovery(link)
    link._side_brake_pending = True


def _junction_wait_remaining(link, now=None) -> float:
    at = getattr(link, "_stable_barrier", None)
    if at is None or _brake_evidence_at(link) is None:
        return math.inf
    return max(0.0, at + JUNCTION_WAIT_S - (time.monotonic() if now is None else now))


def _wait_stable(link, display, deadline=math.inf, *, input_source=None) -> float:
    tracker = getattr(link, "_stop_tracker", None)
    if tracker is None:
        barrier = _brake_evidence_at(link)
        if barrier is None:
            raise mcu.McuError("缺少本次BRAKE及时接受与应用的接收证据")
        tracker = sequence._StopTracker(link, source_barrier=barrier)
    link._stop_tracker = tracker
    while True:
        sequence._check_input_exit(input_source)
        display.tick()
        sequence._remaining(deadline)
        display.consume_samples(collect_means=False)
        if tracker.update(link, display.state_samples if display.state_samples else None,
                          getattr(display, "trace_samples", [])):
            link._stable_barrier = time.monotonic()
            link._ready_side_pairs = []
            display.controller.notify_stopped(link._stable_barrier)
            runlog.emit("motion_sequence_stable", "双轮连续500ms整数里程不变，已停稳",
                        brake_evidence_barrier=tracker.source_barrier,
                        quiet_since=tracker.quiet_since, state_source=tracker.last_state_source,
                        stable_barrier=link._stable_barrier, wheel_positions_mm=link.odom_mm,
                        observed_ms=tracker.observed_ms, state_count=tracker.count,
                        **getattr(link, "patrol_route_context", {}))
            return link._stable_barrier
        link._stable_barrier = None
        remaining = _junction_wait_remaining(link)
        display.confirmation = (f"等待双轮连续500ms静止；左右制动尾程="
                                f"{tracker.left_sum}/{tracker.right_sum} mm；"
                                f"静止窗口={tracker.observed_ms} ms；{tracker.reset_reason}"
                                + (f"；停稳后额外1秒剩余={remaining:.2f}s"
                                   if math.isfinite(remaining) and sequence._pending_junction_openings(link)
                                   else ""))
        display.maybe_print()
        time.sleep(sequence.CONTROL_INTERVAL_S)


def _ensure_stable(link, display, deadline=math.inf, *, input_source=None) -> float:
    if sequence._assert_still_stable(link, display):
        return link._stable_barrier
    return _wait_stable(link, display, deadline, input_source=input_source)


def _brake(link, display, deadline=math.inf, *, input_source=None) -> None:
    display.action = "刹车与停稳确认"
    sequence._apply_brake(link, display, deadline)
    if _brake_evidence_at(link) is None:
        raise mcu.McuError("缺少本次BRAKE及时接受与应用的接收证据")
    _wait_stable(link, display, deadline, input_source=input_source)


def _wait_junction_turn_ready(link, display, deadline=math.inf, *, input_source=None) -> None:
    display.action = "路口停车：严格停稳后额外等待1秒"
    while True:
        sequence._check_input_exit(input_source)
        sequence._remaining(deadline)
        display.tick()
        for sample in display.consume_samples(collect_means=False):
            if sequence._pending_junction_openings(link):
                _latch_patrol_junction(link, sample)
        remaining = _junction_wait_remaining(link)
        if math.isinf(remaining):
            raise mcu.McuError("路口转弯缺少本会话BRAKE接受与应用的接收证据")
        stable = sequence._assert_still_stable(link, display)
        display.confirmation = (f"BRAKE序号={link._last_brake_sequence}；"
                                f"确认时刻={_brake_evidence_at(link):.6f}s；停稳后1秒剩余={remaining:.2f}s；"
                                f"双轮连续500ms静止={'已确认' if stable else '等待'}")
        display.maybe_print()
        if remaining <= 1e-9 and stable:
            tracker = link._stop_tracker
            runlog.emit("motion_sequence_junction_turn_ready", "双轮连续500ms静止后额外等待1秒并复检通过，允许转弯",
                        brake_sequence=link._last_brake_sequence,
                        brake_sent_at=link._last_brake_sent_at,
                        brake_evidence_at=_brake_evidence_at(link),
                        elapsed_s=time.monotonic() - _brake_evidence_at(link),
                        stable_barrier=link._stable_barrier,
                        wheel_tail_mm=(tracker.left_sum, tracker.right_sum),
                        openings=sorted(sequence._pending_junction_openings(link)),
                        **getattr(link, "patrol_route_context", {}))
            return
        if not stable:
            _wait_stable(link, display, deadline, input_source=input_source)
        else:
            time.sleep(sequence.CONTROL_INTERVAL_S)


def _confirm_junction(link, display, first, deadline=math.inf, *, input_source=None):
    checkpoint = getattr(link, "patrol_checkpoint", {})
    evidence = checkpoint.get("junction_evidence") or {}
    evidence_identity = tuple(evidence.get("identity", ()))
    identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
    migrated = evidence_identity in getattr(link, "_patrol_coordinate_identities", ())
    saved_verified = (evidence.get("kind") in ("explicit_opening", "missing_inference")
                      and len(evidence_identity) == 2 and evidence_identity[1] == identity[1]
                      and (evidence_identity == identity or migrated)
                      and not getattr(link, "patrol_manual_required", None))
    saved = frozenset(checkpoint.get("saved_openings", ())) if saved_verified else frozenset()
    if saved:
        # A saved decision has already passed the route/window/source gates.
        # HELLO may change its session nonce; never reparse the old candidate
        # as a new event or require the opening to reappear after reconnect.
        runlog.emit("stage1_saved_junction_reused", "经同boot坐标核验后复用原路口决定，新会话重新BRAKE与严格停稳",
                    junction_evidence=evidence, original_identity=evidence_identity,
                    current_identity=identity, saved_openings=sorted(saved),
                    coordinates_migrated=evidence_identity != identity)
    elif first is not None and first.candidates:
        _latch_patrol_junction(link, first, initial=True)
        saved |= sequence._pending_junction_openings(link)
    if saved and callable(getattr(link, "store_checkpoint", None)):
        link.store_checkpoint(saved_openings=tuple(sorted(saved)))
    tracker = getattr(link, "_stop_tracker", None)
    if (tracker is not None and getattr(link, "_stable_barrier", None) is not None
            and not any(display.command)):
        display.tick()
        _ensure_stable(link, display, deadline, input_source=input_source)
    elif getattr(link, "_patrol_early_junction_brake", None) == identity and _brake_evidence_at(link) is not None:
        _wait_stable(link, display, deadline, input_source=input_source)
    else:
        _brake(link, display, deadline, input_source=input_source)
    link._patrol_early_junction_brake = None
    sequence._remaining(deadline)
    for sample in display.consume_samples(collect_means=False):
        _latch_patrol_junction(link, sample)
    _ensure_stable(link, display, deadline, input_source=input_source)
    advance = checkpoint.get("departure_advance")
    if (advance is not None and advance.get("state") == "REACHED"
            and advance.get("step") == getattr(link, "patrol_route_context", {}).get("step")):
        advance["stopped_distance_mm"] = math.fsum(current - origin for current, origin in zip(
            link.odom_mm, advance["origin_mm"])) / 2.0
        advance["stopped_positions_mm"] = tuple(link.odom_mm)
    openings = sequence._pending_junction_openings(link) or saved
    if not openings:
        raise mcu.McuError("路口停车期间会话变化，已保存开口失效")
    if _brake_evidence_at(link) is None:
        raise mcu.McuError("路口停车缺少本次BRAKE及时接受与应用的接收证据")
    sequence._clear_side_recovery(link)
    display.action = "地图转弯路口已停稳：额外等待1秒再转弯"
    display.confirmation = (f"已保存开口 {','.join(sorted(openings))}；"
                            f"BRAKE序号={link._last_brake_sequence}；确认时刻={_brake_evidence_at(link):.6f}s；"
                            f"停稳后1秒剩余={_junction_wait_remaining(link):.2f}s")
    runlog.emit("motion_sequence_junction_confirmed", "路口证据已保存，严格停稳后额外等待1秒才转弯",
                openings=sorted(openings), stable_barrier=link._stable_barrier,
                source_keys=link._junction_side_sources, brake_sequence=link._last_brake_sequence,
                brake_sent_at=link._last_brake_sent_at, brake_evidence_at=_brake_evidence_at(link),
                wait_remaining_s=_junction_wait_remaining(link),
                **getattr(link, "patrol_route_context", {}))
    display.maybe_print()
    return openings


def _turn_planned(link: mcu.McuLink, display: sequence.MotionDisplay, openings: frozenset[str],
                  deadline: float = math.inf, *, side: str, turn_speed_mmps: int = 91,
                  departure_speed_mmps: int = CRUISE_SPEED_MMPS, input_source=None,
                  angle_deg: int = 90, midroad_authorization=None,
                  goal_reference_deg=None, pivot=False, absolute_uturn=False) -> None:
    """One saved route decision, with a persistent world-heading endpoint."""
    if side not in ("left", "right"):
        raise mcu.McuError("地图转向必须为left或right")
    if type(angle_deg) is not int or angle_deg not in (90, 180) or angle_deg == 180 and side != "left":
        raise mcu.McuError("阶段一只支持左右90度或原地左180度")
    authorized_speed = getattr(link, "authorized_turn_speed_mmps", TURN_SPEED_MMPS)
    if type(turn_speed_mmps) is not int or turn_speed_mmps != authorized_speed:
        raise mcu.McuError(f"阶段一固定转弯轮速请求必须为{authorized_speed} mm/s")
    if pivot and (angle_deg != 90 or midroad_authorization is not None):
        raise mcu.McuError("出发区原地转仅支持正常左右90度")
    if absolute_uturn and angle_deg != 180:
        raise mcu.McuError("道路反向绝对终点补角仅支持原地180度")
    midroad = midroad_authorization is not None
    if midroad and (angle_deg != 180 or side != "left" or openings
                   or midroad_authorization is not getattr(link, "_patrol_midroad_authorization", None)
                   or midroad_authorization.get("kind") != "midroad_uturn"
                   or tuple(midroad_authorization.get("identity", ())) != (link.session_nonce, link.boot_id)
                   or midroad_authorization.get("road") != getattr(link, "patrol_route_context", {}).get("road")):
        raise mcu.McuError("道路中途掉头缺少当前道路及身份的本地授权")
    if not midroad and (not openings or not openings <= {"left", "right"}):
        raise mcu.McuError("转弯缺少已保存的路口证据")
    display.patrol_deadline = min(getattr(display, "patrol_deadline", math.inf), deadline)
    sequence._check_input_exit(input_source)
    context = getattr(link, "patrol_route_context", {})
    entry_key = (context.get("step"), context.get("road"), context.get("node"), side)
    if angle_deg == 180:
        entry_key += (angle_deg,)
    if midroad:
        entry_key += ("midroad",)
    checkpoint = getattr(link, "patrol_checkpoint", {})
    turn = checkpoint.get("turn")
    if turn is None or turn.get("entry_key") != entry_key:
        previous = getattr(display.controller, "formal_reference_deg", None)
        if previous is None:
            previous = display.controller.heading_reference_deg
        if previous is None or link.yaw_deg is None:
            raise mcu.McuError("转弯前缺少固定道路参考或新鲜航向")
        world = getattr(link, "patrol_world_yaw_deg", None)
        if world is None:
            world = link.yaw_deg
        delta = angle_deg if side == "left" else -angle_deg
        reference_world = world + sequence._normalize_yaw(previous - link.yaw_deg)
        goal_yaw = sequence._normalize_yaw(previous + delta)
        goal_world = reference_world + delta
        advance = checkpoint.get("departure_advance") or {}
        departure_goal = (angle_deg == 90 and advance.get("step") == context.get("step")
                          and advance.get("side") == side and advance.get("goal_world") is not None)
        if departure_goal:
            previous = advance["previous_reference"]
            goal_yaw, goal_world = advance["goal_yaw"], advance["goal_world"]
        if goal_reference_deg is not None and not departure_goal:
            if (isinstance(goal_reference_deg, bool) or not isinstance(goal_reference_deg, (int, float))
                    or not math.isfinite(goal_reference_deg)):
                raise mcu.McuError("短道路绝对转后参考不合法")
            fixed_goal = sequence._normalize_yaw(goal_reference_deg)
            goal_world += sequence._normalize_yaw(fixed_goal - goal_yaw)
            goal_yaw = fixed_goal
        turn = dict(entry_key=entry_key, side=side, state="NEW", previous_reference=previous,
                    angle_deg=angle_deg, kind="uturn" if angle_deg == 180 else "turn",
                    speed_mmps=turn_speed_mmps,
                    goal_yaw=goal_yaw,
                    goal_world=goal_world, issued=[], active_seen=False,
                    started_at=None, identity=(link.session_nonce, link.boot_id),
                    correction=False, cutoff=False, last_brake_sequence=None,
                    overshoot_arc_limit=None, pivot=bool(pivot),
                    absolute_uturn=bool(absolute_uturn), uturn_action_completed=False)
        checkpoint["turn"] = turn
        if angle_deg == 180:
            link._patrol_uturn_response = None
            link._patrol_uturn_terminal_fault = False
        checkpoint["saved_openings"] = tuple(sorted(openings))
        turn["authorization_kind"] = "midroad" if midroad else "junction"
        if callable(getattr(link, "store_checkpoint", None)):
            link.store_checkpoint(**checkpoint)
    if turn.get("speed_mmps", TURN_SPEED_MMPS) != turn_speed_mmps:
        raise mcu.McuError("保存转弯的轮速与当前实例授权不一致，禁止改变未完成动作")
    revoke = getattr(display.controller, "revoke_normal_motion", None)
    if callable(revoke):
        revoke()
    finish = getattr(link, "finish_departure", None)
    if callable(finish):
        finish()
    link.side_age_required = False
    name = f"{'原地左' if angle_deg == 180 else '左' if side == 'left' else '右'}{angle_deg}°"
    display.origin = None
    display.turn_target_yaw = turn["goal_yaw"]
    display.reset_averages()
    pivot = bool(turn.get("pivot", False))

    def uturn_active_confirmed():
        current = getattr(link, "state", None)
        if (current is None or turn.get("manual_required")
                or getattr(link, "_patrol_uturn_terminal_fault", False)
                or (current.session_nonce, current.boot_id) != tuple(turn["identity"])
                or current.applied_control_sequence not in turn.get("issued", ())
                or not current.state_flags & sequence.StateFlag.TURN_ACTIVE
                or not current.state_flags & sequence.StateFlag.APPLIED_CONTROL_PRESENT
                or not link._sample_is_fresh("state")):
            return False
        key = sequence.MessageType.MOTION_ACTION, current.applied_control_sequence
        raw = getattr(link, "_action_sends", {}).get(current.applied_control_sequence)
        health = getattr(link, "_live_health_reason", None)
        return (key in getattr(link, "_confirmed_commands", {})
                and raw is not None and raw[0].action is sequence.MotionActionCode.UTURN_LEFT
                and callable(health) and health() is None)

    def recover(exc):
        sequence._remaining(deadline)
        runlog.emit("motion_sequence_turn_aborted", "转弯进入恢复等待，保留原绝对终点",
                    reason=str(exc), target_yaw_deg=turn["goal_yaw"],
                    checkpoint_state=turn["state"], **context)
        callback = getattr(link, "recover_fault", None)
        if not callable(callback):
            raise exc
        raw = getattr(link, "state", None)
        diagnostics = getattr(link, "diagnostics", None)
        pure_angle_abort = sequence._recoverable_turn_abort(link)
        pending_abort = getattr(link, "_turn_abort_evidence", None)
        if (not pure_angle_abort and pending_abort is not None
                and not sequence._active_runtime_fault_bits(link)
                and pending_abort["state"].applied_control_sequence in turn.get("issued", ())):
            # The event STATE may precede its DIAG in another UART pass. Park
            # before waiting; preserve the original command/endpoint witness.
            number = link.send_motion_action(sequence.MotionActionCode.BRAKE, 0,
                                            require_ranges=False, require_upper=False)
            display.set_stopped()
            wait_brake_applied_in_fault(link, number)
            classification_until = min(deadline, time.monotonic() + config.TELEMETRY_STALE_MS / 1000.0)
            while time.monotonic() < classification_until:
                sequence._check_input_exit(input_source)
                if sequence._active_runtime_fault_bits(link):
                    break
                pure_angle_abort = sequence._recoverable_turn_abort(link)
                if pure_angle_abort:
                    break
                link.pump()
                time.sleep(sequence.CONTROL_INTERVAL_S)
            raw, diagnostics = getattr(link, "state", None), getattr(link, "diagnostics", None)
        known_latched_stop = (diagnostics is not None and diagnostics.last_stop_reason in (
            sequence.WireSafetyStopReason.ENCODER_FAULT, sequence.WireSafetyStopReason.LEASE_EXPIRED))
        unplanned_abort = (raw is not None and raw.state_flags & sequence.StateFlag.TURN_ABORTED
                           and not raw.fault_bits & (1 << 20)
                           and not known_latched_stop and not turn.get("cutoff")
                           and not pure_angle_abort)
        if (pending_abort is not None and not pure_angle_abort
                and pending_abort["state"].applied_control_sequence in turn.get("issued", ())):
            unplanned_abort = True
        if angle_deg == 180 and (getattr(link, "_patrol_uturn_terminal_fault", False) or unplanned_abort):
            try:
                require_manual_uturn()
            except mcu.McuError as stop:
                runlog.emit("stage1_uturn_manual_stop", "原地掉头无法可靠继续，先请求制动再等待人工",
                            reason=str(stop), original_reason=str(exc), target_world=turn["goal_world"])
        link._latch_hard_fault(str(exc))
        callback(display, input_source=input_source)
        saved_identity = tuple(turn["identity"])
        current_identity = link.session_nonce, link.boot_id
        if (angle_deg == 180 and saved_identity != current_identity
                and (saved_identity not in getattr(link, "_patrol_coordinate_identities", ())
                     or saved_identity[1] != current_identity[1])):
            require_manual_uturn()
        sequence._check_input_exit(input_source)

    def settle():
        current_cutoff = (turn.get("cutoff") and turn.get("last_brake_sequence")
                          == getattr(link, "_last_brake_sequence", None))
        if (_brake_evidence_at(link) is None
                or getattr(link, "_stop_tracker", None) is None and not current_cutoff):
            _brake(link, display, deadline, input_source=input_source)
        elif getattr(link, "_stop_tracker", None) is None:
            _wait_stable(link, display, deadline, input_source=input_source)
        else:
            display.tick()
            display.consume_samples(collect_means=False, allow_stale=True)
            _ensure_stable(link, display, deadline, input_source=input_source)
        _wait_junction_turn_ready(link, display, deadline, input_source=input_source)

    def poll():
        sequence._check_input_exit(input_source)
        sequence._remaining(deadline)
        display.tick(pump=False)
        display.consume_samples(collect_means=False, allow_stale=True)
        display.maybe_print()

    def require_manual_uturn():
        if turn["state"] not in ("COMPLETED", "CUTOFF", "SETTLED"):
            turn["state"] = "PARTIAL"
        turn["manual_required"] = True
        link.patrol_manual_required = "原地180度原终点或坐标关系不可验证，等待人工"
        try:
            if (_brake_evidence_at(link) is not None
                    and getattr(link, "_stop_tracker", None) is not None
                    and getattr(link, "_stable_barrier", None) is not None
                    and not any(display.command)):
                _ensure_stable(link, display, deadline, input_source=input_source)
            else:
                _brake(link, display, deadline, input_source=input_source)
        except mcu.McuError as exc:
            link.patrol_manual_required += f"；BRAKE或停稳确认失败：{exc}"
            raise mcu.McuError(link.patrol_manual_required) from exc
        link.patrol_manual_required += "；已取得严格停稳证据"
        raise mcu.McuError(link.patrol_manual_required)

    def absolute_uturn_correction_ready():
        current = getattr(link, "state", None)
        identity = link.session_nonce, link.boot_id
        saved = tuple(turn["identity"])
        world = getattr(link, "patrol_world_yaw_deg", None)
        return bool(turn.get("absolute_uturn") and turn.get("uturn_action_completed")
            and not turn.get("manual_required")
            and not getattr(link, "_patrol_uturn_terminal_fault", False)
            and (saved == identity or saved in getattr(link, "_patrol_coordinate_identities", ())
                 and saved[1] == identity[1])
            and current is not None and (current.session_nonce, current.boot_id) == identity
            and current.odom_status == 0 and current.state_flags & sequence.StateFlag.ODOM_VALID
            and not sequence._active_runtime_fault_bits(link)
            and link._sample_is_fresh("state")
            and isinstance(world, (int, float)) and math.isfinite(world)
            and math.isfinite(turn["goal_world"]))

    while turn["state"] != "SETTLED":
        sequence._check_input_exit(input_source)
        sequence._remaining(deadline)
        try:
            if angle_deg == 180 and turn["state"] == "PARTIAL" and uturn_active_confirmed():
                turn["state"] = "ACTIVE"
                turn["correction"] = False
            # COMPLETED is recorded before the post-turn gate, so a fault there
            # cannot replay a completed ninety-degree action.
            if turn["state"] in ("COMPLETED", "CUTOFF"):
                completed_at = turn.get("completed_at", time.monotonic())
                if turn.get("cutoff") and turn.get("last_brake_sequence") == link._last_brake_sequence:
                    settle()
                else:
                    _brake(link, display, deadline, input_source=input_source)
                    _wait_junction_turn_ready(link, display, deadline, input_source=input_source)
                actual = link.yaw_deg
                if actual is None:
                    raise mcu.McuError("转后缺少新鲜航向")
                error = sequence._normalize_yaw(turn["goal_yaw"] - actual)
                if angle_deg == 180:
                    world = getattr(link, "patrol_world_yaw_deg", None)
                    if world is None:
                        raise mcu.McuError("原地180度转后缺少连续世界航向")
                    error = turn["goal_world"] - world
                cutoff = getattr(link, "_patrol_expected_cutoff", None)
                world = getattr(link, "patrol_world_yaw_deg", None)
                if turn.get("cutoff") and cutoff is not None and world is not None:
                    sign = 1 if cutoff["side"] == "left" else -1
                    tail = max(0.0, sign * (world - cutoff["world_yaw"]))
                    link._patrol_brake_tail_deg[cutoff["side"]] = max(
                        link._patrol_brake_tail_deg[cutoff["side"]], tail)
                    if sign * (turn["goal_world"] - world) < 0:
                        turn["overshoot_arc_limit"] = max(.5, abs(world - turn["start_world"]) / 2.0)
                if abs(error) <= 5.0 + 1e-9:
                    turn["state"] = "SETTLED"
                    turn["completed_at"] = completed_at
                    break
                if angle_deg == 180 and turn.get("absolute_uturn"):
                    if not absolute_uturn_correction_ready():
                        require_manual_uturn()
                    # The full firmware UTURN has finished. All remaining
                    # motion uses a saved-endpoint differential pivot instead
                    # of replaying another relative 180-degree action.
                    pivot = turn["pivot"] = True
                elif angle_deg == 180 and error < -5.0 - 1e-9:
                    require_manual_uturn()
                turn["state"] = "PARTIAL"
                turn["correction"] = True
                link._turn_abort_evidence = None

            if turn["state"] != "ACTIVE":
                settle()
                actual = link.yaw_deg
                if actual is None:
                    raise mcu.McuError("补角前缺少新鲜航向")
                world = getattr(link, "patrol_world_yaw_deg", None)
                if world is None:
                    world = actual
                error = turn["goal_world"] - world
                if turn["state"] != "NEW" and abs(error) <= 5.0 + 1e-9:
                    turn["state"] = "SETTLED"
                    break
                absolute_correction = (angle_deg == 180 and turn.get("absolute_uturn")
                                       and turn.get("uturn_action_completed"))
                if absolute_correction:
                    if not absolute_uturn_correction_ready():
                        require_manual_uturn()
                    pivot = turn["pivot"] = True
                if (angle_deg == 180 and (turn.get("manual_required")
                        or error < -5.0 - 1e-9 and not absolute_correction
                        or turn.get("absolute_uturn") and turn["state"] != "NEW"
                        and not turn.get("uturn_action_completed"))):
                    require_manual_uturn()
                chosen = side if turn["state"] == "NEW" else ("left" if error > 0 else "right")
                turn["correction"] = turn["state"] != "NEW"
                if turn["correction"] and callable(getattr(link, "correction_arc_available", None)):
                    if not link.correction_arc_available(turn, world):
                        link.patrol_manual_required = "剩余补角没有可验证的制动余量，保留原终点等待人工"
                        raise mcu.McuError(link.patrol_manual_required)
                action = (sequence.MotionActionCode.TURN_LEFT if chosen == "left"
                          else sequence.MotionActionCode.TURN_RIGHT)
                if angle_deg == 180:
                    action = sequence.MotionActionCode.UTURN_LEFT
                first_issue = turn["state"] == "NEW"
                turn.update(current_side=chosen, start_world=world, active_seen=False, issued=[],
                            cutoff=False, started_at=time.monotonic(),
                            identity=(link.session_nonce, link.boot_id), state="ISSUING")
                before_brake = link._last_brake_sequence
                stable = link._stable_barrier
                sequence_before = getattr(link, "_seq", None)
                action_keys_before = frozenset(getattr(link, "_action_sends", {}))
                command_keys_before = frozenset(getattr(link, "_command_ledger", {}))
                try:
                    if pivot:
                        angular = DEPARTURE_PIVOT_ANGULAR_MRADPS * (1 if chosen == "left" else -1)
                        link._patrol_exclusive_motion = dict(kind="departure_pivot",
                            identity=(link.session_nonce, link.boot_id), target=(0, angular), checkpoint=turn)
                        number = link.send_motion(0, angular, require_ranges=False, require_upper=False)
                    else:
                        number = link.send_motion_action(action, turn_speed_mmps, brake_after_turn=True,
                                                          require_ranges=False, require_upper=False)
                except Exception as exc:
                    # A coordinator veto before UART is not a partial pivot.
                    # Only a never-issued original action may return to NEW;
                    # actual writes, active renewals and corrections retain
                    # the existing non-replay protection and fixed endpoint.
                    if (getattr(exc, "motion_not_written", False) and first_issue
                            and turn["state"] == "ISSUING" and not turn.get("issued")
                            and not turn.get("active_seen") and sequence_before is not None
                            and getattr(link, "_seq", None) == sequence_before
                            and frozenset(getattr(link, "_action_sends", {})) == action_keys_before
                            and frozenset(getattr(link, "_command_ledger", {})) == command_keys_before):
                        turn.update(state="NEW", started_at=None)
                    raise
                turn.update(state="ACTIVE", sequence=number, action=action,
                            issued=[number], started_at=time.monotonic())
                link._active_turn_details = dict(action=action.name, sequence=number,
                                                origin_mm=link.odom_mm)
                runlog.emit("motion_sequence_turn_sent", "停稳500ms后额外1秒，发送地图转弯或原终点补角",
                            action=action.name, sequence=number, speed_mmps=turn_speed_mmps,
                            brake_sequence=before_brake, stable_barrier=stable,
                            brake_evidence_at=_brake_evidence_at(link),
                            target_yaw_deg=turn["goal_yaw"], previous_road_reference_deg=turn["previous_reference"],
                            next_road_reference_deg=turn["goal_yaw"], correction=turn["correction"],
                            saved_openings=sorted(openings), **context)
            action = turn["action"]
            display.action = "原绝对终点补角" if turn["correction"] else name
            display.control_mode = ("道路反向终点原地补角，双轮±91；100ms同参数续约" if pivot and angle_deg == 180 else
                                    "出发区原地90°，双轮±91；100ms同参数续约" if pivot else
                                    "原地180°，双轮-91/+91；100ms同参数续约" if angle_deg == 180
                                    else f"{turn_speed_mmps} mm/s外轮、内轮0；100ms同参数续约")
            wheels = ((-91, 91) if turn["current_side"] == "left" else (91, -91)) if pivot else _patrol_turn_wheels(action, turn_speed_mmps)
            display.command = (0, round((wheels[1] - wheels[0]) * 1000 / sequence.WHEEL_BASE_MM))
            display.command_kind = (sequence.MessageType.CONTROL_TARGET if pivot
                                    else sequence.MessageType.MOTION_ACTION)
            display.wheel_targets = wheels
            display.turn_result = f"原目标 {turn['goal_yaw']:.1f}°；动作序号={turn['sequence']}"
            display.maybe_print(force=True)
            if pivot:
                link.wait_for_departure_pivot(turn, display, deadline=deadline, on_poll=poll)
            else:
                link.wait_for_motion_action_complete(turn["sequence"], timeout_s=config.TURN_TIMEOUT_S,
                                                      require_ranges=False, require_upper=False, on_poll=poll)
                if angle_deg == 180 and turn.get("absolute_uturn") and not turn.get("cutoff"):
                    turn["uturn_action_completed"] = True
            turn["state"] = "CUTOFF" if turn.get("cutoff") else "COMPLETED"
            turn["completed_at"] = time.monotonic()
            display.set_stopped()
            runlog.emit("motion_sequence_turn_completion_received", "收到真实完成或计划BRAKE截断证据，继续严格停稳与1秒复检",
                        sequence=turn["sequence"], cutoff=turn.get("cutoff", False),
                        turn_completed_at=turn["completed_at"], post_turn_wait_s=POST_TURN_WAIT_S)
        except mcu.McuError as exc:
            recover(exc)
            current = getattr(link, "state", None)
            issued = turn.get("issued", ())
            if (turn["state"] not in ("COMPLETED", "SETTLED", "CUTOFF")
                    and current is not None and (current.session_nonce, current.boot_id) == tuple(turn["identity"])
                    and current.applied_control_sequence in issued
                    and current.state_flags & sequence.StateFlag.TURN_ACTIVE
                    and (angle_deg != 180 or uturn_active_confirmed())):
                turn["state"] = "ACTIVE"
            elif turn["state"] not in ("COMPLETED", "SETTLED", "CUTOFF"):
                turn["state"] = "PARTIAL"
                turn["correction"] = True

    barrier = getattr(link, "_stable_barrier", None)
    if barrier is None or not sequence._assert_still_stable(link, display):
        raise mcu.McuError("转后缺少当前严格停稳证据，禁止离口")
    actual = link.yaw_deg
    link.post_turn_barrier = barrier
    display.reset_averages(source_barrier=barrier)
    display.controller.set_initial_reference(turn["goal_yaw"])
    runlog.emit("motion_sequence_initial_reference", "上一道路固定参考加转向角建立下一道路临时参考",
                yaw_deg=turn["goal_yaw"], previous_road_reference_deg=turn["previous_reference"],
                actual_yaw_deg=actual, angle_deg=angle_deg, after_turn=True)
    link.side_age_required = True
    begin = getattr(link, "begin_gyro_departure", None)
    if callable(begin):
        begin(barrier, speed_mmps=departure_speed_mmps)
    elif callable(getattr(link, "begin_departure", None)):
        link.begin_departure(barrier)
    display.turn_target_yaw = None
    display.origin = None
    display.after_turn = True
    display.input_speed_mmps = departure_speed_mmps
    display.segment_distance_mm = None
    display.tick(pump=False)
    display.confirmation = (f"严格停稳500ms后额外1秒完成；{departure_speed_mmps} mm/s沿继承参考离口，"
                            "等待真正新双墙")
    display.turn_result = f"{name}完成并停稳，目标 {turn['goal_yaw']:.1f}°，实际 {actual:.1f}°"
    runlog.emit("motion_sequence_turn_completed", f"原绝对终点进入5度容限，严格停稳后额外1秒，允许{departure_speed_mmps}离口",
                action=turn["action"].name if "action" in turn else name, sequence=turn.get("sequence"),
                departure_speed_mmps=departure_speed_mmps,
                target_yaw_deg=turn["goal_yaw"], actual_yaw_deg=actual, post_turn_barrier=barrier,
                turn_completed_at=turn.get("completed_at"), brake_sequence=link._last_brake_sequence,
                brake_evidence_at=_brake_evidence_at(link), post_turn_wait_s=POST_TURN_WAIT_S)
    link._active_turn_details = None
    link._patrol_exclusive_motion = None
    display.maybe_print(force=True)


class _PatrolGuardedSerial:
    """One CONTROL_TARGET write guard; nested safety writes pass through."""

    def __init__(self, serial_port, guard):
        self._serial_port, self._guard, self._armed = serial_port, guard, True
        self._write_attempted = False
        self._host_write_started_at = None

    def __getattr__(self, name):
        return getattr(self._serial_port, name)

    def write(self, data):
        if self._armed:
            # Disarm before checks: an expired lease may itself send STOP.
            self._armed = False
            self._guard()
        self._write_attempted = True
        self._host_write_started_at = time.monotonic()
        written = self._serial_port.write(data)
        inner = getattr(self._serial_port, "_host_write_started_at", None)
        if inner is not None:
            self._host_write_started_at = max(self._host_write_started_at, inner)
        return written


class PatrolLink(sequence.MotionSequenceLink):
    """Patrol-only turn timing, stopped departure and authorized moving passage."""

    def __init__(self, *args, settings=None, **kwargs):
        self.settings = settings
        self._recoverable_link_diagnostic_mask = 0x00000F3F
        self._patrol_uart_sends = {}
        self.patrol_next_action = coverage.TURN_LEFT
        self.patrol_checkpoint = {}
        self.patrol_fault_history = sequence.deque(maxlen=512)
        self.patrol_manual_required = None
        self.patrol_yaw_offset_deg = 0.0
        self.patrol_world_yaw_deg = None
        self._patrol_last_raw_yaw = None
        self._patrol_last_yaw_source = None
        self._patrol_yaw_rate_degps = 0.0
        self._patrol_static_window = None
        self._patrol_stationary_anchor = None
        self._patrol_fault_at = None
        self._patrol_retry_at = 0.0
        self._patrol_expected_cutoff = None
        self._patrol_session_history = sequence.deque(maxlen=16)
        self._patrol_brake_tail_deg = {"left": 0.0, "right": 0.0}
        self._patrol_recovery_hello_active = False
        self._patrol_coordinate_identities = set()
        self._patrol_gyro_departure_identity = None
        self._patrol_gyro_departure_side_age_required = None
        self._patrol_gyro_mode = None
        self._patrol_uturn_response = None
        self._patrol_uturn_terminal_fault = False
        self._patrol_feedback_write_guard = None
        self._patrol_turn_brake_required = False
        self._patrol_exclusive_motion = None
        super().__init__(*args, **kwargs)

    @property
    def authorized_turn_speed_mmps(self):
        settings = getattr(self, "settings", None)
        return settings.turn_speed_mmps if settings is not None else TURN_SPEED_MMPS

    def patrol_exclusive_motion_valid(self, linear, angular):
        authorization = self._patrol_exclusive_motion
        if (authorization is None or not self.connected
                or tuple(authorization.get("identity", ())) != (self.session_nonce, self.boot_id)
                or authorization.get("target") != (linear, angular)):
            return False
        saved = authorization.get("checkpoint")
        identity = self.session_nonce, self.boot_id
        state = self.state
        source = getattr(self, "_sample_time", None)
        display = getattr(self, "_patrol_display", None)
        at = (source("state") if callable(source) else
              getattr(getattr(display, "controller", None), "_state_at", None))
        if (saved is None or tuple(saved.get("identity", ())) != identity
                or state is None or (state.session_nonce, state.boot_id) != identity
                or state.odom_status != 0 or not state.state_flags & sequence.StateFlag.ODOM_VALID
                or at is None or not 0 <= time.monotonic() - at
                <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9):
            return False
        if authorization.get("kind") == "departure_advance":
            current_distance = math.fsum(current - origin for current, origin in zip(
                (state.left_mm, state.right_mm), saved["origin_mm"])) / 2.0
            return (saved is self.patrol_checkpoint.get("departure_advance")
                    and saved.get("state") == "ADVANCING"
                    and (linear, angular) == (saved.get("speed_mmps", INITIAL_SPEED_MMPS), 0)
                    and saved.get("step") == getattr(self, "patrol_route_context", {}).get("step")
                    and saved.get("threshold_mm") == DEPARTURE_STRATEGIES.get(saved.get("strategy"))
                    and saved.get("distance_mm", math.inf) < saved["threshold_mm"]
                    and current_distance + 1e-9 < saved["threshold_mm"])
        if authorization.get("kind") == "departure_pivot":
            if (saved.get("angle_deg") == 180 and not
                    (saved.get("absolute_uturn") and saved.get("uturn_action_completed"))):
                return False
            expected = {"left": DEPARTURE_PIVOT_ANGULAR_MRADPS,
                        "right": -DEPARTURE_PIVOT_ANGULAR_MRADPS}.get(saved.get("current_side"))
            if saved.get("state") == "ISSUING":
                anchor = getattr(self, "_patrol_stationary_anchor", None)
                if (anchor is None or anchor.get("positions") != (state.left_mm, state.right_mm)
                        or (anchor.get("session_nonce"), anchor.get("boot_id")) != identity
                        or not 0 <= time.monotonic() - anchor["at"]
                        <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                        or _junction_wait_remaining(self) > 1e-9
                        or state.target_linear_mms or state.target_angular_mrads
                        or state.state_flags & sequence.StateFlag.TURN_ACTIVE):
                    return False
            return (saved is self.patrol_checkpoint.get("turn") and saved.get("pivot")
                    and saved.get("state") in ("ISSUING", "ACTIVE") and not linear
                    and angular == expected)
        return False

    class _WriteClock:
        def __init__(self, serial_port):
            self.serial_port, self.started_at = serial_port, None

        def __getattr__(self, name):
            return getattr(self.serial_port, name)

        def write(self, data):
            self.started_at = time.monotonic()
            written = self.serial_port.write(data)
            inner = getattr(self.serial_port, "_host_write_started_at", None)
            if inner is not None:
                self.started_at = max(self.started_at, inner)
            return written

    def _patrol_transport_send(self, message_type, payload):
        original = self._ser
        if original is None:
            return super()._send(message_type, payload)
        clock = self._WriteClock(original)
        self._ser = clock
        try:
            number = super()._send(message_type, payload)
        finally:
            self._ser = original
        self._patrol_uart_sends[(message_type, number)] = clock.started_at
        while len(self._patrol_uart_sends) > 256:
            self._patrol_uart_sends.pop(next(iter(self._patrol_uart_sends)))
        return number

    def _track_command(self, kind, number, sent_at, baseline, target):
        sent_at = self._patrol_uart_sends.get((kind, number), sent_at)
        raw = self._action_sends.get(number) if kind is sequence.MessageType.MOTION_ACTION else None
        if raw is not None:
            self._action_sends[number] = (raw[0], sent_at, *raw[2:])
            if raw[0].action is sequence.MotionActionCode.BRAKE:
                self._last_brake_sent_at = sent_at
            else:
                self._active_motion_sent_at = sent_at
        elif kind is sequence.MessageType.CONTROL_TARGET and len(target) == 3 and any(target[1:3]):
            self._active_motion_sent_at = sent_at
        return super()._track_command(kind, number, sent_at, baseline, target)

    def _send(self, message_type, payload):
        if message_type is not sequence.MessageType.CONTROL_TARGET:
            return self._patrol_transport_send(message_type, payload)
        command = mcu.decode_control_target(payload)
        moving = bool(command.linear_mms or command.angular_mrads)
        if (moving and getattr(self, "_patrol_turn_brake_required", False)
                and not self.patrol_exclusive_motion_valid(command.linear_mms, command.angular_mrads)):
            raise sequence._CenteringProposalExpired("转弯路口已保存，禁止非零CONTROL_TARGET写入")
        guard = getattr(self, "_patrol_feedback_write_guard", None)
        if not moving and guard is None:
            return self._patrol_transport_send(message_type, payload)
        original_serial = self._ser
        if original_serial is None:
            return self._patrol_transport_send(message_type, payload)

        def before_write():
            cap = getattr(self, "_patrol_road_speed_cap", None)
            if cap is not None and command.linear_mms > cap:
                raise sequence._CenteringProposalExpired("n06穿口及前50mm限速100，重算旧高速目标")
            exclusive = self.patrol_exclusive_motion_valid(command.linear_mms, command.angular_mrads)
            if (moving and getattr(self, "_patrol_turn_brake_required", False) and not exclusive):
                raise sequence._CenteringProposalExpired("转弯路口已抢占，禁止旧非零目标写入")
            if moving and exclusive:
                self._check_motion_deadline()
                self._check_motion_lease()
                reason = self.motion_health_reason() or sequence._gyro_health_reason(self)
                if reason is not None:
                    raise mcu.McuError(reason)
                if not self.patrol_exclusive_motion_valid(command.linear_mms, command.angular_mrads):
                    raise sequence._CenteringProposalExpired("出发动作实际写入前身份、目标或阶段已变化")
            if guard is None:
                return
            controller, target, deadline = guard
            self._check_motion_deadline()
            self._check_motion_lease()
            reason = self.motion_health_reason()
            if reason is not None:
                raise mcu.McuError(reason)
            reason = self._laser_stale_reason(require_upper=False)
            if reason is not None:
                raise mcu.McuError(reason)
            sequence._remaining(deadline)
            if (not self.connected or getattr(self, "_hard_fault_reason", None) is not None
                    or (self.session_nonce, self.boot_id) != controller._state_session):
                raise mcu.McuError(getattr(self, "_hard_fault_reason", None)
                                   or "阶段一实际写入前会话身份已改变")
            if not controller.target_send_ready(target, time.monotonic()):
                raise sequence._CenteringProposalExpired("阶段一实际写入前反馈、身份或偏角已改变")
            initial_guard = getattr(self, "_patrol_initial_launch_write_guard", None)
            if callable(initial_guard):
                initial_guard()

        original_sequence = self._seq
        guarded_serial = _PatrolGuardedSerial(original_serial, before_write)
        self._ser = guarded_serial
        try:
            return self._patrol_transport_send(message_type, payload)
        except Exception:
            if not guarded_serial._write_attempted:
                self._seq = original_sequence
            raise
        finally:
            self._ser = original_serial

    @property
    def patrol_gyro_departure_active(self):
        return self.patrol_gyro_only_active and self._patrol_gyro_mode == "DEPARTURE"

    @property
    def patrol_gyro_passage_active(self):
        return self.patrol_gyro_only_active and self._patrol_gyro_mode == "STRAIGHT"

    @property
    def patrol_gyro_only_active(self):
        if self._patrol_gyro_departure_identity is None:
            return False
        if (not self.connected or not self.side_departure_active
                or self._patrol_gyro_departure_identity != (self.session_nonce, self.boot_id)):
            self.finish_departure()
            return False
        return True

    def _reset_link_state(self):
        super()._reset_link_state()
        self._patrol_uart_sends.clear()
        self._patrol_last_raw_yaw = None
        self._patrol_last_yaw_source = None
        self._patrol_yaw_rate_degps = 0.0
        self._patrol_static_window = None
        self._patrol_uturn_response = None
        self._patrol_stationary_anchor = None
        self._patrol_gyro_departure_identity = None
        self._patrol_gyro_departure_side_age_required = None
        self._patrol_gyro_mode = None

    def _start_capture(self):
        super()._start_capture()
        if self._capture is None:
            return
        try:
            with self._capture._lock:
                settings = dict(self._capture._metadata.get("motion_sequence", {}))
                sources = list(self._capture._metadata.get("source_paths", []))
            settings.update(junction_wait_s=JUNCTION_WAIT_S,
                            junction_wait_anchor="strict_500ms_stationary_barrier_plus_1s",
                            post_turn_wait_s=POST_TURN_WAIT_S,
                            post_turn_wait_anchor="strict_500ms_stationary_barrier_plus_1s",
                            departure_side_timeout_s=None,
                            gyro_departure_side_timeout_s=None,
                            straight_passage_side_timeout_s=None,
                            normal_passage_side_timeout_s=None,
                            turn_speed_mmps=91,
                            turn_inner_wheel_speed_mmps=0,
                            uturn_wheel_speed_mmps=(-TURN_SPEED_MMPS, TURN_SPEED_MMPS),
                            turn_goal="previous_fixed_road_reference_plus_or_minus_90deg_or_plus_180deg",
                            directional_frame_required_during_gyro_departure=False,
                            directional_frame_required_during_straight_passage=False,
                            initial_target_mmps=INITIAL_SPEED_MMPS,
                            initial_travel_mm=INITIAL_TRAVEL_MM,
                            initial_speed_transition="direct_150_until_first_completed_turn",
                            low_speed_target_mmps=START_SPEED_MMPS,
                            alignment_target_mmps=START_SPEED_MMPS,
                            post_turn_low_speed_target_mmps=None,
                            post_turn_alignment_target_mmps=CRUISE_SPEED_MMPS,
                            heading_recovery_target_mmps=CRUISE_SPEED_MMPS,
                            short_road_heading_recovery_target_mmps=CRUISE_SPEED_MMPS,
                            heading_recovery_entry_deg=45., heading_recovery_exit_deg=5.,
                            short_road_heading_recovery_entry_deg=45.,
                            post_turn_heading_sample_policy="trailing_1s_fresh_aligned_raw_le3_trend_le0_5_gyro_le0_5_applied_w0",
                            post_turn_heading_sample_window_s=1.0,
                            post_turn_heading_sample_min_count=3,
                            post_turn_heading_sample_max_count=None,
                            post_turn_heading_mean_max_updates=None,
                            post_turn_alignment_time_limit_s=None,
                            post_turn_alignment_mode="CONTINUOUS_FORWARD",
                            first_exit_heading_sample_policy="ordinary_road_fresh_reference_learning",
                            post_turn_departure_target_mmps=CRUISE_SPEED_MMPS,
                            correction_heading_limit_deg=45,
                            side_difference_deadband_mm=PATROL_SIDE_DEADBAND_MM,
                            correction_nonzero_wheel_min_mmps=61,
                            correction_single_zero_allowed=True,
                            correction_single_zero_rule="fresh_laser_quantized_differential_exceeds_goal_mean_budget",
                            home_correction_heading_limit_deg=5.,
                            actual_cruise_speed_window_s=sequence.CENTER_SPEED_WINDOW_S,
                            actual_cruise_speed_tolerance_fraction=.03,
                            home_target_mmps=HOME_SPEED_MMPS,
                            home_travel_mm=HOME_TRAVEL_MM,
                            home_origin="first_fresh_dual_valid_later_measurement_source",
                            moving_heading_sample_s=None,
                            initial_stopped_heading_sample_s=sequence.START_SAMPLE_S,
                            cruise_target_mmps=CRUISE_SPEED_MMPS,
                            end_condition="returned_n01_and_strictly_stopped")
            if self.settings is not None:
                settings.update(
                    route_test_settings=asdict(self.settings),
                    cruise_target_mmps=self.settings.cruise_speed_mmps,
                    initial_target_mmps=self.settings.cruise_speed_mmps,
                    initial_speed_transition="configured_cruise_until_first_completed_turn",
                    low_speed_target_mmps=self.settings.cruise_speed_mmps,
                    alignment_target_mmps=self.settings.cruise_speed_mmps,
                    post_turn_alignment_target_mmps=self.settings.cruise_speed_mmps,
                    heading_recovery_target_mmps=self.settings.cruise_speed_mmps,
                    short_road_heading_recovery_target_mmps=self.settings.cruise_speed_mmps,
                    post_turn_departure_target_mmps=self.settings.cruise_speed_mmps,
                    turn_speed_mmps=self.settings.turn_speed_mmps,
                    uturn_wheel_speed_mmps=(-self.settings.turn_speed_mmps, self.settings.turn_speed_mmps),
                    home_target_mmps=self.settings.home_speed_mmps,
                    special_target_cap_mmps=self.settings.special_speed_mmps,
                )
            source = str(Path(__file__).resolve())
            if source not in sources:
                sources.append(source)
            self._capture_call("update_metadata", motion_sequence=settings, source_paths=sources)
        except Exception as exc:
            if not self._capture_error_reported:
                self._capture_error_reported = True
                runlog.exception("wire_capture_error", "阶段1采集元数据适配失败，控制继续", exc)

    def begin_departure(self, stable_barrier):
        self.finish_departure()
        super().begin_departure(stable_barrier)

    def begin_gyro_departure(self, stable_barrier, *, speed_mmps=START_SPEED_MMPS):
        if (self.patrol_gyro_departure_active
                and self._departure_barrier == stable_barrier):
            return
        self.finish_departure()
        if (not self.connected or self.session_nonce is None or self.boot_id is None
                or not self.side_age_required
                or getattr(self, "_stable_barrier", None) != stable_barrier):
            raise mcu.McuError("转后陀螺仪离口缺少本会话合法停稳证据")
        super().begin_departure(stable_barrier)
        self._patrol_gyro_departure_identity = self.session_nonce, self.boot_id
        self._patrol_gyro_departure_side_age_required = self.side_age_required
        self._patrol_gyro_mode = "DEPARTURE"
        self.side_age_required = False
        sequence._clear_side_recovery(self)
        runlog.emit("stage1_gyro_departure_started", f"转后{speed_mmps}陀螺仪离口，等待首组新鲜且STATE对齐的双墙；无激光等待期限",
                    source_barrier=stable_barrier, session_nonce=self.session_nonce,
                    boot_id=self.boot_id, target_mmps=speed_mmps)

    def _fresh_gyro_passage_barrier(self, opening):
        now = time.monotonic()
        identity = self.session_nonce, self.boot_id
        evidence = self.patrol_checkpoint.get("junction_evidence") or {}
        saved_identity = tuple(evidence.get("identity", ()))
        coordinate_verified = (saved_identity == identity
                               or saved_identity in self._patrol_coordinate_identities)
        if (evidence.get("kind") in ("explicit_opening", "missing_inference")
                and len(saved_identity) == 2 and saved_identity[1] == self.boot_id
                and coordinate_verified and evidence.get("openings")
                and self.connected and None not in identity and self.side_motion_active
                and not self.patrol_manual_required and self._hard_fault_reason is None
                and self.odom_mm is not None and self.yaw_deg is not None
                and self._live_health_reason() is None):
            runlog.emit("stage1_saved_passage_authorized", "保存的路口决定在健康进度恢复后继续直行，保留原始证据",
                        junction_evidence=evidence, original_identity=saved_identity,
                        current_identity=identity, passage_action_barrier=now,
                        coordinates_migrated=saved_identity != identity)
            return now
        if (evidence.get("kind") == "missing_inference"
                and tuple(evidence.get("identity", ())) == identity
                and isinstance(evidence.get("confirmed_at"), (int, float))
                and 0 <= now - evidence["confirmed_at"] <= .250 + 1e-9
                and self.connected and None not in identity and self.side_motion_active):
            return evidence["confirmed_at"]
        if opening is None:
            raise mcu.McuError("直行穿口缺少已核验坐标的保存路口决定")
        self.check_patrol_sides(opening)
        fresh_sides = [index for index, (name, status) in enumerate((
            ("left", opening.left_status), ("right", opening.right_status)))
            if name in opening.candidates
            and status in (sequence.WireRangeStatus.NO_TARGET, sequence.WireRangeStatus.OUT_OF_RANGE)
            and 0 <= now - opening.source_times[index]
            <= sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9]
        if (not self.connected or None in identity or opening.source_identity != identity
                or not fresh_sides or not self.side_motion_active):
            raise mcu.McuError("直行穿口缺少本会话250ms内真正新鲜的合法开口")
        return max(opening.source_times[index] for index in fresh_sides)

    def begin_gyro_passage(self, opening, *, speed_mmps):
        """Only a true fresh opening authorizes moving gyro-only straight passage."""
        barrier = self._fresh_gyro_passage_barrier(opening)
        if (self._active_motion_sent_at is None and not self.moving
                or getattr(self, "_stable_barrier", None) is not None):
            raise mcu.McuError("移动中直行穿口缺少实际行驶证据")
        self.finish_departure()
        super().begin_departure(barrier)
        self._patrol_gyro_departure_identity = self.session_nonce, self.boot_id
        self._patrol_gyro_departure_side_age_required = self.side_age_required
        self._patrol_gyro_mode = "STRAIGHT"
        self.side_age_required = False
        sequence._clear_side_recovery(self)
        runlog.emit("stage1_gyro_passage_started", "新鲜开口授权移动中按原道路航向直行，等待首组新双墙；无侧激光等待期限",
                    source_barrier=barrier, session_nonce=self.session_nonce,
                    boot_id=self.boot_id, target_mmps=speed_mmps,
                    authorization="moving_fresh_opening",
                    entry_sources=None if opening is None else opening.source_keys,
                    openings=sorted(self.patrol_checkpoint.get("saved_openings", ()))
                        if opening is None else sorted(opening.candidates),
                    **getattr(self, "patrol_route_context", {}))
        return barrier

    def begin_stopped_gyro_passage(self, opening, *, speed_mmps):
        """Stopped recovery needs fresh opening plus current strict BRAKE evidence."""
        barrier = self._fresh_gyro_passage_barrier(opening)
        stable_barrier = getattr(self, "_stable_barrier", None)
        tracker = getattr(self, "_stop_tracker", None)
        if (not self.side_age_required or stable_barrier is None or tracker is None
                or self._active_motion_sent_at is not None
                or _brake_evidence_at(self) is None or not tracker.update(self, [])):
            raise mcu.McuError("停稳后直行穿口缺少本会话BRAKE及双轮连续500ms停稳证据")
        self.finish_departure()
        super().begin_departure(barrier)
        self._patrol_gyro_departure_identity = self.session_nonce, self.boot_id
        self._patrol_gyro_departure_side_age_required = self.side_age_required
        self._patrol_gyro_mode = "STRAIGHT"
        self.side_age_required = False
        sequence._clear_side_recovery(self)
        runlog.emit("stage1_gyro_passage_started", "严格停稳与新鲜开口授权按原道路航向直行，等待开口源时间后的首组新双墙；无侧激光等待期限",
                    source_barrier=barrier, stable_barrier=stable_barrier,
                    brake_sequence=self._last_brake_sequence,
                    brake_evidence_at=_brake_evidence_at(self),
                    session_nonce=self.session_nonce, boot_id=self.boot_id,
                    target_mmps=speed_mmps, authorization="strict_stop_and_fresh_opening",
                    entry_sources=None if opening is None else opening.source_keys,
                    openings=sorted(self.patrol_checkpoint.get("saved_openings", ()))
                        if opening is None else sorted(opening.candidates),
                    **getattr(self, "patrol_route_context", {}))
        return barrier

    def finish_departure(self):
        restore = self._patrol_gyro_departure_side_age_required
        super().finish_departure()
        self._patrol_gyro_departure_identity = None
        self._patrol_gyro_departure_side_age_required = None
        self._patrol_gyro_mode = None
        if restore is not None:
            self.side_age_required = restore

    def store_checkpoint(self, **updates):
        self.patrol_checkpoint.update(updates)
        return self.patrol_checkpoint

    def _latch_hard_fault(self, reason):
        # The shared base retains terminal faults. This application retains the
        # same raw evidence, but waits for recovery without sending a host STOP.
        reported_abort = reason == "转向被下位机中止"
        if reported_abort and self.state is not None:
            # The shared parser reports TURN_ABORTED before fault/odom fields.
            # A deliberate BRAKE may explain that flag, never a real fault in
            # the same STATE, even if a later healthy frame shares the batch.
            state = self.state
            if state.fault_bits & (1 << 20):
                reason = "下位机本地停机锁存"
            elif self.motion_checks_enabled and self._active_fault_bits():
                reason = "MCU固件故障"
            elif self.motion_checks_enabled and (state.odom_status != 0
                    or not state.state_flags & sequence.StateFlag.ODOM_VALID):
                reason = "ODOM无效"
            elif self.is_expected_turn_abort(state):
                return
        now = time.monotonic()
        if (not self.patrol_fault_history or self.patrol_fault_history[-1]["reason"] != reason
                or now - self.patrol_fault_history[-1]["at"] >= 1.0):
            entry = dict(reason=reason, at=now, session_nonce=self.session_nonce,
                         boot_id=self.boot_id, route=dict(getattr(self, "patrol_route_context", {})))
            self.patrol_fault_history.append(entry)
            runlog.emit("stage1_recovery_fault", "故障证据保留，停止无依据的续约并等待恢复",
                        reason=reason, fault_at_s=now, session_nonce=self.session_nonce,
                        boot_id=self.boot_id, route=entry["route"])
        if (self._hard_fault_reason is None or reason != "转向被下位机中止"
                and self._hard_fault_reason == "转向被下位机中止"):
            self._hard_fault_reason = reason
            self._patrol_fault_at = now

    def _stop_failed_confirmation(self, reason):
        self._latch_hard_fault(reason)

    def check_patrol_sides(self, sample, now=None, **_kwargs):
        validator = globals().get("_check_patrol_sides")
        if callable(validator):
            validator(sample, time.monotonic() if now is None else now,
                      check_age=False, allow_stale=True, allow_timeout=True)
        if sample.source_identity != (self.session_nonce, self.boot_id):
            raise mcu.McuError("下层激光样本身份不属于当前会话")
        for status, distance in ((sample.left_status, sample.left_mm),
                                 (sample.right_status, sample.right_mm)):
            if not isinstance(status, sequence.WireRangeStatus):
                raise mcu.McuError("下层激光非法状态字段")
            if status == sequence.WireRangeStatus.VALID and (
                    isinstance(distance, bool) or not isinstance(distance, (int, float))
                    or not math.isfinite(distance) or not 0 <= distance < 65535):
                raise mcu.McuError("下层激光VALID距离字段非法")

    def _laser_stale_reason(self, *, require_upper=True):
        # 3/4/5/6 remain visible as missing readings; no fabricated VALID cache.
        sample = self._latest_side_sample
        if sample is not None:
            try:
                self.check_patrol_sides(sample)
            except mcu.McuError as exc:
                return str(exc)
        return None

    def _accept_directional(self, directional):
        if not self._session_matches(directional.session_nonce, directional.boot_id):
            if self.connected and directional.boot_id != self.boot_id:
                self.patrol_manual_required = "MCU boot变化，原道路轮程及航向连续关系不可验证"
                self._latch_hard_fault(self.patrol_manual_required)
            return
        if (self._last_directional_seq is not None
                and directional.directional_cycle_seq != self._last_directional_seq
                and sequence.classify_sequence(directional.directional_cycle_seq, self._last_directional_seq)
                is not sequence.SequenceDisposition.NEWER):
            return
        if not self._accept_uptime("directional", directional.uptime_ms):
            return
        self._last_directional_seq = directional.directional_cycle_seq
        self.directional = directional
        sample = sequence._annotate_side_sample(self, sequence._side_sample(self))
        self._latest_side_sample = sample
        try:
            self.check_patrol_sides(sample)
            if sequence._pending_junction_openings(self) and sample.candidates:
                _latch_patrol_junction(self, sample)
        except mcu.McuError as exc:
            self._latch_hard_fault(str(exc))
        if len(self._side_events) >= 256:
            self._latch_hard_fault("下层激光快照队列溢出，测量证据不完整")
            return
        self._side_events.append(sample)

    @property
    def yaw_deg(self):
        raw = mcu.McuLink.yaw_deg.fget(self)
        return None if raw is None else sequence._normalize_yaw(raw + self.patrol_yaw_offset_deg)

    def drain_state_samples(self):
        samples = super().drain_state_samples()
        if not self.patrol_yaw_offset_deg:
            return samples
        # These are explicitly derived controller observations. self.state,
        # UART capture, command ledger and turn completion witnesses stay raw.
        return [(sequence.replace(state, yaw_tenth_deg=round(sequence._normalize_yaw(
            state.yaw_tenth_deg / 10.0 + self.patrol_yaw_offset_deg) * 10)), at)
                for state, at in samples]

    def _accept_state(self, state):
        if self.connected and state.boot_id != self.boot_id:
            self.patrol_manual_required = "MCU重启，无法证明原道路与航向连续关系，等待人工"
        previous = self.state
        super()._accept_state(state)
        if self.state is not state or previous is state:
            return
        at = self._sample_time("state")
        raw = state.yaw_tenth_deg / 10.0
        if self._patrol_last_raw_yaw is None:
            self.patrol_world_yaw_deg = raw + self.patrol_yaw_offset_deg
        else:
            delta = sequence._normalize_yaw(raw - self._patrol_last_raw_yaw)
            self.patrol_world_yaw_deg += delta
            elapsed = at - self._patrol_last_yaw_source
            if elapsed > 1e-9 and elapsed <= config.TELEMETRY_STALE_MS / 1000.0:
                self._patrol_yaw_rate_degps = delta / elapsed
            else:
                self._patrol_yaw_rate_degps = 0.0
        self._patrol_last_raw_yaw, self._patrol_last_yaw_source = raw, at
        if (state.state_flags & sequence.StateFlag.ODOM_VALID and state.odom_status == 0
                and not state.target_linear_mms and not state.target_angular_mrads
                and not state.state_flags & sequence.StateFlag.TURN_ACTIVE):
            self._active_motion_sent_at = None
        positions = state.left_mm, state.right_mm
        window = self._patrol_static_window
        if (window is None or window[1] != positions or at - window[3] > config.TELEMETRY_STALE_MS / 1000.0
                or abs(sequence._normalize_yaw(raw - window[2])) > .5
                or not state.state_flags & sequence.StateFlag.ODOM_VALID or state.odom_status != 0):
            self._patrol_static_window = at, positions, raw, at
            self._patrol_stationary_anchor = None
        else:
            self._patrol_static_window = window[:3] + (at,)
            if at - window[0] >= .5 - 1e-9:
                self._patrol_stationary_anchor = dict(at=at, positions=positions,
                    raw_yaw=raw, world_yaw=self.patrol_world_yaw_deg,
                    boot_id=state.boot_id, session_nonce=state.session_nonce)
        cutoff = self._patrol_expected_cutoff
        if cutoff is not None and self.is_expected_turn_abort(state):
            cutoff["observed_abort"] = True
            if self._hard_fault_reason == "转向被下位机中止":
                self._hard_fault_reason = None

    def _accept_wheel_trace(self, sample, frame_sequence=None):
        previous = self.wheel_trace
        super()._accept_wheel_trace(sample, frame_sequence)
        if self.wheel_trace is not sample or previous is sample:
            return
        window = self._patrol_static_window
        source = self._trace_history[-1][1] if self._trace_history else self._last_trace_recv
        if (window is not None and source > window[0] + 1e-9
                and (sample.left_encoder_delta or sample.right_encoder_delta)):
            self._patrol_static_window = None
            self._patrol_stationary_anchor = None
            runlog.emit("stage1_hello_anchor_reset", "停后真实轮迹仍有编码器增量，HELLO静止锚重新累计500ms",
                        source_at=source, left_encoder_delta=sample.left_encoder_delta,
                        right_encoder_delta=sample.right_encoder_delta)

    def is_expected_turn_abort(self, state):
        cutoff = self._patrol_expected_cutoff
        if cutoff is None or state is None or not state.state_flags & sequence.StateFlag.TURN_ABORTED:
            return False
        key = sequence.MessageType.MOTION_ACTION, cutoff.get("sequence")
        record = self._command_ledger.get(key)
        if (record is None or record[8] is None or record[8].status is not sequence.AckStatus.ACCEPTED
                or record[9] is None or record[9] > record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S + 1e-9
                or (state.session_nonce, state.boot_id) != tuple(cutoff["identity"])
                or state.applied_control_sequence != cutoff.get("sequence")
                or state.target_linear_mms or state.target_angular_mrads
                or not state.state_flags & sequence.StateFlag.APPLIED_CONTROL_PRESENT
                or not 0 < ((state.uptime_ms - cutoff["baseline"]) & 0xFFFFFFFF) < 0x80000000):
            return False
        return self._sample_is_fresh("state")

    def send_motion(self, linear_mmps, angular_mradps, **kwargs):
        if (getattr(self, "_patrol_turn_brake_required", False) and (linear_mmps or angular_mradps)
                and not self.patrol_exclusive_motion_valid(int(round(linear_mmps)), int(round(angular_mradps)))):
            raise sequence._CenteringProposalExpired("转弯路口等待BRAKE与转向，禁止非零续约")
        if getattr(self, "_turn_waiting", False) and (linear_mmps or angular_mradps):
            raise mcu.McuError("固定转向执行中不能发送CONTROL_TARGET")
        linear, angular = int(round(linear_mmps)), int(round(angular_mradps))
        sent_at, baseline = time.monotonic(), getattr(self.state, "uptime_ms", None)
        if linear or angular:
            self._check_motion_deadline()
            self._check_motion_lease()
            reason = self.motion_health_reason()
            if reason is not None:
                raise mcu.McuError(reason)
            reason = self._laser_stale_reason(require_upper=False)
            if reason is not None:
                raise mcu.McuError(reason)
        if not self.connected or self._ser is None:
            raise mcu.McuError("未建立STM32会话")
        command = mcu.ControlTarget(session_nonce=self.session_nonce, flags=mcu.ControlFlag.MOTION_VALID,
            linear_mms=linear, angular_mrads=angular, aim_id=0, pose_id=mcu.WireGimbalPose.UNKNOWN,
            pan_pwm_us=0, tilt_pwm_us=0)
        number = self._send(sequence.MessageType.CONTROL_TARGET, mcu.encode_control_target(command))
        if linear or angular:
            self._active_motion_sent_at = time.monotonic()
            self._stable_barrier = self._stop_tracker = self._brake_evidence_barrier = None
            self._ready_side_pairs = []
            self._track_command(sequence.MessageType.CONTROL_TARGET, number, sent_at, baseline,
                                (sequence.MessageType.CONTROL_TARGET, linear, angular))
        else:
            self._active_motion_sent_at = None
            self._clear_command_evidence()
            self.finish_departure()
        return number

    def send_motion_action(self, action, speed_mms=0, **kwargs):
        turn = action in (sequence.MotionActionCode.TURN_LEFT, sequence.MotionActionCode.TURN_RIGHT,
                          sequence.MotionActionCode.UTURN_LEFT)
        if turn and speed_mms != self.authorized_turn_speed_mmps:
            raise mcu.McuError(f"阶段一转弯轮速请求必须为{self.authorized_turn_speed_mmps} mm/s")
        flags = int(sequence.MotionActionFlag.BRAKE_AFTER_TURN) if kwargs.get("brake_after_turn", True) else 0
        if turn and getattr(self, "_turn_waiting", False) and self._current_target is not None:
            angle = 1800 if action is sequence.MotionActionCode.UTURN_LEFT else 900
            if self._current_target != (sequence.MessageType.MOTION_ACTION, action, speed_mms, flags, angle):
                raise mcu.McuError("固定转向执行中只允许同参数MOTION_ACTION续约")
        if turn and not getattr(self, "_turn_waiting", False):
            tracker = getattr(self, "_stop_tracker", None)
            if (tracker is None or getattr(self, "_stable_barrier", None) is None
                    or not tracker.update(self, []) or _junction_wait_remaining(self) > 1e-9):
                raise mcu.McuError("转弯前须BRAKE接受与应用、双轮连续500ms停稳后额外等待1秒")
        sent_at, baseline = time.monotonic(), getattr(self.state, "uptime_ms", None)
        number = mcu.McuLink.send_motion_action(self, action, speed_mms, **kwargs)
        sent_at = self._patrol_uart_sends[(sequence.MessageType.MOTION_ACTION, number)]
        if action is sequence.MotionActionCode.BRAKE:
            cancel_motion_commands_after_brake(self, number, sent_at)
            saved_turn = self.patrol_checkpoint.get("turn", {})
            raw = self.state
            existing = self._patrol_expected_cutoff
            if (raw is not None and saved_turn.get("active_seen")
                    and tuple(saved_turn.get("identity", ())) == (self.session_nonce, self.boot_id)
                    and raw.applied_control_sequence in saved_turn.get("issued", ())
                    and raw.state_flags & (sequence.StateFlag.TURN_ACTIVE | sequence.StateFlag.TURN_ABORTED)):
                if existing is None or existing.get("sequence") is not None:
                    existing = dict(identity=(self.session_nonce, self.boot_id), baseline=baseline,
                        issued=tuple(saved_turn["issued"]), action=saved_turn.get("action"),
                        active_proven=True, world_yaw=self.patrol_world_yaw_deg,
                        side=saved_turn.get("current_side", saved_turn.get("side")))
                    self._patrol_expected_cutoff = existing
                existing["sequence"] = number
            elif (existing is not None and raw is not None
                    and tuple(existing.get("identity", ())) == (self.session_nonce, self.boot_id)
                    and raw.applied_control_sequence == existing.get("sequence")
                    and raw.state_flags & sequence.StateFlag.TURN_ABORTED
                    and not raw.target_linear_mms and not raw.target_angular_mrads):
                # Repeating a BRAKE does not make the already explained abort
                # a new turn failure. Raw fault and odometry gates still apply.
                existing.update(sequence=number, baseline=baseline)
            self._last_brake_sent_at = sent_at
            self._last_brake_identity = self.session_nonce, self.boot_id
            self._last_brake_sequence, self._last_brake_origin = number, self.odom_mm
            self.side_cached_active = False
            self._clear_command_evidence(preserve_pending=True)
            self.finish_departure()
        elif turn:
            self._patrol_turn_brake_required = False
            self.finish_departure()
            self._stable_barrier = self._stop_tracker = self._brake_evidence_barrier = None
            self._ready_side_pairs = []
            sequence._clear_junction(self)
            sequence._clear_side_recovery(self)
        command = self._action_sends[number][0]
        target = (sequence.MessageType.MOTION_ACTION, command.action, command.speed_mms,
                  int(command.flags), command.turn_angle_tenth_deg)
        self._track_command(sequence.MessageType.MOTION_ACTION, number, sent_at, baseline, target)
        return number

    def send_gimbal(self, pose, pan_us, tilt_us):
        baseline = getattr(self.state, "uptime_ms", None)
        number = super().send_gimbal(pose, pan_us, tilt_us)
        self._track_command(sequence.MessageType.CONTROL_TARGET, number,
                            self._patrol_uart_sends[(sequence.MessageType.CONTROL_TARGET, number)], baseline,
                            (sequence.MessageType.CONTROL_TARGET, 0, 0, "gimbal", int(pose),
                             int(pan_us), int(tilt_us)))
        return number

    def _wait_applied(self, kind, number, timeout_s=None):
        raw = self._action_sends.get(number)
        if (kind is sequence.MessageType.MOTION_ACTION and raw is not None
                and raw[0].action is sequence.MotionActionCode.BRAKE):
            return wait_brake_applied_in_fault(self, number, timeout_s)
        return super()._wait_applied(kind, number, timeout_s)

    def _retire_brake_interruptions(self):
        brakes = [(key, record) for key, record in self._command_ledger.items()
                  if key[0] is sequence.MessageType.MOTION_ACTION
                  and record[5][1] is sequence.MotionActionCode.BRAKE
                  and record[2] is not None and record[8] is not None
                  and not record[2].target_linear_mms and not record[2].target_angular_mrads
                  and record[8].status is sequence.AckStatus.ACCEPTED]
        for key, record in list(self._pending_commands.items()):
            expiry = record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S
            if (record[2] is not None or record[8] is None
                    or record[8].status is not sequence.AckStatus.ACCEPTED
                    or record[9] is None or record[9] > expiry + 1e-9):
                continue
            authorized = key[0] is sequence.MessageType.CONTROL_TARGET
            cutoff = self._patrol_expected_cutoff
            if (not authorized and cutoff is not None and key[0] is sequence.MessageType.MOTION_ACTION
                    and key[1] in cutoff.get("issued", ()) and cutoff.get("active_proven")):
                authorized = record[5][1] == cutoff.get("action")
            if not authorized:
                continue
            proof = next(((brake_key, brake) for brake_key, brake in brakes
                if brake[0] >= record[0] and tuple(brake[6:8]) == tuple(record[6:8])
                and max(brake[3], brake[9]) <= min(expiry, brake[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S) + 1e-9), None)
            if proof is None:
                continue
            brake_key, brake = proof
            record[10].update(interrupted_by_brake=brake_key[1], interrupted_at=max(brake[3], brake[9]),
                              original_deadline=expiry, disposition="interrupted")
            self._pending_commands.pop(key)
            runlog.emit("stage1_command_interrupted", "及时BRAKE打断已接受旧命令；不授予应用或完成资格",
                        message_type=key[0].name, sequence=key[1], brake_sequence=brake_key[1],
                        original_deadline=expiry, evidence_received_at=max(brake[3], brake[9]))

    def pump(self):
        input_source = getattr(self, "_patrol_input", None)
        display = getattr(self, "_patrol_display", None)
        if self._patrol_recovery_hello_active:
            if input_source is None:
                input_source = getattr(self, "_patrol_recovery_input", None)
            if display is None:
                display = getattr(self, "_patrol_recovery_display", None)
        sequence._check_input_exit(input_source)
        serial_port = self._ser
        self._frame_received_at = None
        if serial_port is not None:
            self._ser = self._ReceiveClock(serial_port, self)
        try:
            frames = mcu.McuLink.pump(self)
        finally:
            self._ser, self._frame_received_at = serial_port, None
        now = time.monotonic()
        if self.last_parse_error and self.last_parse_error.startswith("READ_ERROR:"):
            self._latch_hard_fault(self.last_parse_error)
        self._retire_brake_interruptions()
        if (self._hard_fault_reason == "转向被下位机中止"
                and self.is_expected_turn_abort(self.state)):
            self._hard_fault_reason = None
        proven = [(key, record) for key, record in self._command_ledger.items()
                  if record[2] is not None and record[8] is not None
                  and record[8].status is sequence.AckStatus.ACCEPTED
                  and tuple(record[6:8]) == (self.session_nonce, self.boot_id)]
        for key, record in list(self._pending_commands.items()):
            expiry = record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S
            if record[8] is not None and record[8].status is not sequence.AckStatus.ACCEPTED:
                record[10]["disposition"] = "rejected"
                self._pending_commands.pop(key)
                continue
            proof = next(((witness_key, witness) for witness_key, witness in proven
                if witness[4:8] == record[4:8] and witness[0] >= record[0]
                and max(witness[3], witness[9]) <= expiry + 1e-9), None)
            if proof is not None:
                witness_key, witness = proof
                self._confirmed_commands[key], self._confirmed_send_times[key] = witness[2], record[0]
                self._confirmed_evidence_times[key] = max(witness[3], witness[9])
                if witness_key != key:
                    self._superseded_commands[key] = witness_key
                self._pending_commands.pop(key)
            elif now >= expiry:
                self._record_confirmation_timeout(key, record, expiry, now)
                record[10]["disposition"] = "confirmation_timeout"
                self._pending_commands.pop(key)
                self._latch_hard_fault(f"{key[0].name} seq={key[1]}未在确认窗口内应用")
        services = getattr(display, "services", None)
        if services is not None and self._hard_fault_reason is None:
            try:
                self._check_motion_lease()
            except mcu.McuError:
                return frames
            if self.motion_checks_enabled and self.motion_health_reason() is not None:
                return frames
            services.tick(distance_mm=getattr(display, "segment_distance_mm", None),
                          phase=getattr(display, "action", "确认与接收"))
        return frames

    def _live_health_reason(self):
        latched = self._hard_fault_reason
        self._hard_fault_reason = None
        try:
            return super().motion_health_reason()
        finally:
            self._hard_fault_reason = latched

    def recovery_anchor(self):
        anchor = self._patrol_stationary_anchor
        window = self._patrol_static_window
        if anchor is not None and window is not None:
            for trace, source in self._trace_history:
                if ((trace.session_nonce, trace.boot_id) == (self.session_nonce, self.boot_id)
                        and source > window[0] + 1e-9
                        and (trace.left_encoder_delta or trace.right_encoder_delta)):
                    self._patrol_static_window = None
                    self._patrol_stationary_anchor = None
                    return None
        if (anchor is None or time.monotonic() - anchor["at"] > config.TELEMETRY_STALE_MS / 1000.0
                or sequence._gyro_health_reason(self) is not None
                or self.state is None or self.state.odom_status != 0):
            return None
        return dict(anchor)

    def migrate_hello_anchor(self, before, after):
        if (before is None or after is None or before["boot_id"] != after["boot_id"]
                or before["positions"] != after["positions"]):
            self.patrol_manual_required = "HELLO前后静止航向或双轮里程连续关系不可验证，等待人工"
            return False
        shift = before["world_yaw"] - after["raw_yaw"]
        self.patrol_yaw_offset_deg = shift
        self.patrol_world_yaw_deg = before["world_yaw"]
        self._patrol_last_raw_yaw = after["raw_yaw"]
        self._patrol_last_yaw_source = after["at"]
        self._patrol_yaw_rate_degps = 0.0
        for anchor in (before, after):
            if anchor.get("session_nonce") is not None:
                self._patrol_coordinate_identities.add((anchor["session_nonce"], anchor["boot_id"]))
        runlog.emit("stage1_hello_coordinate_migrated", "同boot静止与轮程连续，保留原道路参考并转换HELLO后的航向坐标",
                    before=before, after=after, yaw_offset_deg=shift)
        return True

    def _observe_uturn_manual_stop(self, display):
        turn = self.patrol_checkpoint.get("turn", {})
        if turn.get("angle_deg") != 180 or not turn.get("manual_required"):
            return
        state = self.state
        healthy_stop = (self._live_health_reason() is None and _brake_evidence_at(self) is not None
                        and self._sample_is_fresh("state") and state is not None
                        and (state.session_nonce, state.boot_id) == (self.session_nonce, self.boot_id)
                        and state.applied_control_sequence == self._last_brake_sequence
                        and state.state_flags & sequence.StateFlag.APPLIED_CONTROL_PRESENT
                        and not state.target_linear_mms and not state.target_angular_mrads)
        tracker = self._stop_tracker
        if not healthy_stop:
            self._stable_barrier = None
            if tracker is not None:
                tracker.quiet_since = None
                tracker.count = tracker.observed_ms = 0
                tracker.reset_reason = "人工等待期间当前健康或BRAKE零目标证据中断"
            return
        if tracker is None:
            tracker = self._stop_tracker = sequence._StopTracker(self, source_barrier=_brake_evidence_at(self))
        samples = display.state_samples or [(state, self._sample_time("state"))]
        # A healthy latest frame cannot conceal an invalid intermediate STATE.
        # Begin the quiet window after the last such frame, preserving all
        # later wheel evidence and the manual/fault latch.
        invalid_sources = [source for sample, source in samples
            if source > tracker.source_barrier + 1e-9 and (
                (sample.session_nonce, sample.boot_id) != (self.session_nonce, self.boot_id)
                or sequence._active_runtime_fault_bits(self, sample)
                or sample.odom_status != 0
                or not sample.state_flags & sequence.StateFlag.ODOM_VALID
                or not sample.state_flags & sequence.StateFlag.APPLIED_CONTROL_PRESENT
                or sample.applied_control_sequence != self._last_brake_sequence
                or sample.target_linear_mms or sample.target_angular_mrads)]
        if invalid_sources:
            tracker.source_barrier = max(invalid_sources)
            tracker.quiet_since = None
            tracker.count = tracker.observed_ms = 0
            tracker.reset_reason = "人工等待期间STATE健康或BRAKE零目标证据中断"
            self._stable_barrier = None
        stable = tracker.update(self, samples, getattr(display, "trace_samples", []))
        if not stable:
            self._stable_barrier = None
        elif self._stable_barrier is None:
            self._stable_barrier = time.monotonic()
            runlog.emit("stage1_uturn_manual_stable", "当前健康反馈补齐严格停稳证据，仍保留人工锁及原故障历史",
                        stable_barrier=self._stable_barrier, observed_ms=tracker.observed_ms,
                        brake_sequence=self._last_brake_sequence, wheel_positions_mm=self.odom_mm)

    def recover_fault(self, display, *, input_source=None):
        """Actively BRAKE, prove parking, and keep the saved coordinates across HELLO."""
        self._patrol_display = display
        if input_source is not None:
            self._patrol_input = input_source
        stop = None
        brake_retry_at = 0.0
        while True:
            sequence._check_input_exit(input_source)
            sequence._remaining(getattr(display, "patrol_deadline", math.inf))
            now = time.monotonic()
            if stop is None and now >= brake_retry_at and self.connected and self._ser is not None:
                brake_retry_at = now + 1.0
                try:
                    number = self.send_motion_action(sequence.MotionActionCode.BRAKE, 0,
                                                    require_ranges=False, require_upper=False)
                    display.set_stopped()
                    wait_brake_applied_in_fault(self, number)
                    self._brake_evidence_barrier = _brake_evidence_at(self)
                    stop = FaultStopTracker(self, number)
                    self._stop_tracker = stop.tracker
                    self._stable_barrier = None
                    runlog.emit("stage1_recovery_brake", "故障立即BRAKE，不等待运动租约到期；独立收集500ms停稳证据",
                                sequence=number, checkpoint=self.patrol_checkpoint)
                except mcu.McuError as exc:
                    self._latch_hard_fault(str(exc))
            self.pump()
            try:
                display.tick(pump=False, check_health=False)
                display.consume_samples(require_healthy=False, collect_means=False, allow_stale=True)
            except mcu.McuError:
                pass
            now = time.monotonic()
            display.action = "故障恢复等待：保留路线及原转弯终点"
            display.confirmation = self.patrol_manual_required or self._hard_fault_reason or "检查新鲜遥测"
            display.maybe_print()
            anchor = stop.update(getattr(display, "state_samples", None),
                                 getattr(display, "trace_samples", None)) if stop is not None else None
            if anchor is not None and not self.patrol_manual_required:
                self._stable_barrier = now if self._stable_barrier is None else self._stable_barrier
                self._patrol_stationary_anchor = anchor
            if self.patrol_manual_required:
                self._observe_uturn_manual_stop(display)
                time.sleep(sequence.CONTROL_INTERVAL_S)
                continue
            if now < self._patrol_retry_at:
                time.sleep(sequence.CONTROL_INTERVAL_S)
                continue
            self._patrol_retry_at = now + 1.0
            live = self._live_health_reason()
            fault_at = self._patrol_fault_at or -math.inf
            fresh_after = (self.state is not None and self.diagnostics is not None
                           and self._sample_time("state") > fault_at
                           and self._sample_time("diagnostics") > fault_at)
            if live is None and fresh_after and anchor is not None and not self._pending_commands:
                self._hard_fault_reason = None
                self._patrol_fault_at = None
                self.last_parse_error = None
                callback = getattr(self, "patrol_recovery_callback", None)
                if callable(callback):
                    callback()
                runlog.emit("stage1_recovery_resumed", "新鲜健康反馈恢复，接续保存动作", checkpoint=self.patrol_checkpoint)
                return
            # HELLO resets gyro coordinates. Never use it without a physical
            # stationary anchor, even if a new session would clear a latch.
            if anchor is not None and self._ser is not None and self._hard_fault_reason is not None:
                self._patrol_session_history.append(dict(identity=(self.session_nonce, self.boot_id),
                    ledger=dict(self._command_ledger), faults=list(self.patrol_fault_history)))
                self._patrol_recovery_hello_active = True
                self._patrol_recovery_input = input_source
                self._patrol_recovery_display = display
                try:
                    self.hello(timeout_s=1.0)
                    # HELLO clears the source-clock anchor. Rebuild it with
                    # an applied zero-output probe before trusting freshness
                    # or collecting the new session's stationary evidence.
                    self.synchronize_clock()
                    stop = None
                    until = time.monotonic() + 1.5
                    while time.monotonic() < until:
                        sequence._check_input_exit(input_source)
                        self.pump()
                        if (stop is None and self.state is not None and self._sample_is_fresh("state")
                                and self.state.odom_status == 0
                                and self.state.state_flags & sequence.StateFlag.ODOM_VALID):
                            number = self.send_motion_action(sequence.MotionActionCode.BRAKE, 0,
                                                            require_ranges=False, require_upper=False)
                            wait_brake_applied_in_fault(self, number)
                            self._brake_evidence_barrier = _brake_evidence_at(self)
                            stop = FaultStopTracker(self, number)
                            self._stop_tracker = stop.tracker
                            self._stable_barrier = None
                        after = stop.update() if stop is not None else None
                        if after is not None:
                            if not self.migrate_hello_anchor(anchor, after):
                                break
                            self._hard_fault_reason = None
                            self._patrol_fault_at = None
                            self._stable_barrier = time.monotonic()
                            self._patrol_stationary_anchor = after
                            if self._live_health_reason() is None:
                                callback = getattr(self, "patrol_recovery_callback", None)
                                if callable(callback):
                                    callback()
                                runlog.emit("stage1_recovery_resumed", "HELLO后同boot轮程与停稳核验通过，接续保存动作",
                                            checkpoint=self.patrol_checkpoint)
                                return
                            break
                        time.sleep(sequence.CONTROL_INTERVAL_S)
                    else:
                        self.patrol_manual_required = "HELLO后未取得可靠500ms静止航向，等待人工"
                except mcu.McuError as exc:
                    self._latch_hard_fault(str(exc))
                finally:
                    self._patrol_recovery_hello_active = False
            time.sleep(sequence.CONTROL_INTERVAL_S)

    def correction_arc_available(self, turn, world):
        remaining = abs(turn["goal_world"] - world)
        rate = abs(self._patrol_yaw_rate_degps)
        side = "left" if turn["goal_world"] > world else "right"
        margin = self._predicted_brake_angle(side, rate)
        cap = turn.get("overshoot_arc_limit")
        turn["allowed_arc"] = max(0.0, min(remaining, remaining if cap is None else cap))
        return turn["allowed_arc"] > max(.5, margin) and remaining > 5.0

    def _predicted_brake_angle(self, side, rate):
        source = self._patrol_last_yaw_source
        age = 0.0 if source is None else max(0.0, time.monotonic() - source)
        return max(.5, rate * (config.MCU_COMMAND_CONFIRM_TIMEOUT_S + age
            + sequence.COMMAND_INTERVAL_S + .05) + self._patrol_brake_tail_deg[side])

    def wait_for_departure_pivot(self, turn, display, *, deadline=math.inf, on_poll=None):
        """A CONTROL_TARGET pivot retains the saved absolute endpoint on every renewal."""
        identity = self.session_nonce, self.boot_id
        sign = 1 if turn["current_side"] == "left" else -1
        angular = sign * DEPARTURE_PIVOT_ANGULAR_MRADPS
        wheels = -91 * sign, 91 * sign
        response = sequence._WheelResponseTracker(self, time.monotonic())
        renew_at = time.monotonic() + sequence.COMMAND_INTERVAL_S
        completion_deadline = min(deadline, turn["started_at"] + config.TURN_TIMEOUT_S)
        sent_at = turn["started_at"]
        while True:
            self.pump()
            now = time.monotonic()
            sequence._remaining(deadline)
            if identity != (self.session_nonce, self.boot_id) or not self.connected:
                raise mcu.McuError("出发区原地转期间MCU身份变化")
            reason = self.motion_health_reason() or sequence._gyro_health_reason(self)
            if reason:
                raise mcu.McuError(reason)
            if on_poll is not None:
                on_poll()
            state = self.state
            key = sequence.MessageType.CONTROL_TARGET, state.applied_control_sequence
            confirmed = (state.applied_control_sequence in turn["issued"]
                         and key in self._confirmed_commands
                         and state.target_linear_mms == 0 and state.target_angular_mrads == angular)
            if confirmed:
                turn["active_seen"] = True
                if sign * (self.patrol_world_yaw_deg - turn["start_world"]) <= -1.0 + 1e-9:
                    if turn.get("absolute_uturn"):
                        self._patrol_uturn_terminal_fault = True
                    raise mcu.McuError("出发区原地转已应用后反向航向达到1度，停车保留原终点")
            executing, accepted_at, applied = sequence._confirmed_wheel_response(self, sent_at, wheels)
            response.update(self, executing, now, accepted_at, display.state_samples, applied_state=applied)
            if confirmed:
                world = self.patrol_world_yaw_deg
                remaining = sign * (turn["goal_world"] - world)
                traveled = abs(world - turn["start_world"])
                margin = self._predicted_brake_angle(turn["current_side"], abs(self._patrol_yaw_rate_degps))
                arc_limit = turn.get("allowed_arc", abs(turn["goal_world"] - turn["start_world"]))
                if remaining <= max(5.0, margin) or traveled >= arc_limit:
                    cutoff_world = world
                    _brake(self, display, deadline, input_source=getattr(self, "_patrol_input", None))
                    tail = max(0.0, sign * (self.patrol_world_yaw_deg - cutoff_world))
                    self._patrol_brake_tail_deg[turn["current_side"]] = max(
                        self._patrol_brake_tail_deg[turn["current_side"]], tail)
                    if sign * (turn["goal_world"] - self.patrol_world_yaw_deg) < 0:
                        turn["overshoot_arc_limit"] = max(.5, traveled / 2.0)
                    turn.update(cutoff=True, last_brake_sequence=self._last_brake_sequence)
                    return
            if now >= completion_deadline:
                raise mcu.McuError("出发区原地转未在原转弯时间内到达可制动终点")
            if now >= renew_at:
                number = self.send_motion(0, angular, require_ranges=False, require_upper=False)
                turn["sequence"] = number
                turn["issued"].append(number)
                sent_at = time.monotonic()
                renew_at = sent_at + sequence.COMMAND_INTERVAL_S
            time.sleep(sequence.CONTROL_INTERVAL_S)

    def wait_for_motion_action_complete(self, number, *, timeout_s=13.0,
                                       require_ranges=True, require_upper=True, on_poll=None):
        record = self._action_sends.get(number)
        if record is None:
            raise mcu.McuError(f"未知MOTION_ACTION seq={number}")
        command, sent_at, baseline, nonce, boot = record
        sign = _patrol_turn_sign(command.action)
        is_uturn = command.action is sequence.MotionActionCode.UTURN_LEFT
        turn = self.patrol_checkpoint.get("turn")
        issued = set(turn.get("issued", ()) if turn else ()) | {number}
        active_seen = bool(turn and turn.get("active_seen"))
        self._turn_events = sequence.deque()
        self._turn_waiting = True
        next_renew = max(time.monotonic(), sent_at + sequence.COMMAND_INTERVAL_S)
        completion_deadline = time.monotonic() + timeout_s
        active_source = None
        guard = sequence._TurnDirectionGuard(
            sign,
            self.state.yaw_tenth_deg / 10.0, (nonce, boot))
        response = None
        if is_uturn:
            response = self._patrol_uturn_response
            if response is None:
                response = self._patrol_uturn_response = sequence._WheelResponseTracker(self, time.monotonic())
            else:
                # Preserve per-wheel accumulated drive/travel, excluding the
                # unobserved recovery wait from this new confirmation window.
                response.pause(self, time.monotonic())
                response.active_since = time.monotonic()
                response.applied_token = self.state.uptime_ms
                response.applied_position = self.odom_mm
        self.side_age_required = False
        try:
            while True:
                self.pump()
                now = time.monotonic()
                if (self.session_nonce, self.boot_id) != (nonce, boot) or not self.connected:
                    raise mcu.McuError("转向期间MCU会话变化")
                reason = self.motion_health_reason()
                if reason is not None:
                    raise mcu.McuError(reason)
                if on_poll is not None:
                    on_poll()
                if now >= completion_deadline:
                    if is_uturn:
                        self._patrol_uturn_terminal_fault = True
                    raise mcu.McuError("固定转弯13秒局部确认超时，保留绝对终点等待恢复")
                turn_samples = []
                completed = None
                completed_at = None
                while self._turn_events:
                    state, source, uptime = self._turn_events.popleft()
                    if (state.applied_control_sequence not in issued
                            or not state.state_flags & sequence.StateFlag.APPLIED_CONTROL_PRESENT
                            or now - source > config.TELEMETRY_STALE_MS / 1000.0
                            or uptime is None or baseline is not None and uptime <= baseline):
                        continue
                    key = sequence.MessageType.MOTION_ACTION, state.applied_control_sequence
                    if key not in self._confirmed_commands:
                        continue
                    wrong_direction = guard.observe(state, source, now, applied=True)
                    if wrong_direction is not None:
                        if is_uturn:
                            self._patrol_uturn_terminal_fault = True
                        raise mcu.McuError(wrong_direction)
                    if state.state_flags & sequence.StateFlag.TURN_ABORTED:
                        raise mcu.McuError("固件转向中止，保留原绝对终点等待恢复")
                    if state.state_flags & sequence.StateFlag.TURN_ACTIVE:
                        turn_samples.append((state, source))
                        active_seen = True
                        active_source = source
                        if turn:
                            turn["active_seen"] = True
                    elif (active_seen and state.state_flags & sequence.StateFlag.TURN_COMPLETED
                          and (active_source is None or source > active_source)):
                        completed, completed_at = state, source
                        break
                if response is not None:
                    proofs = [(self._confirmed_evidence_times[key], self._confirmed_commands[key])
                              for number in issued
                              for key in [(sequence.MessageType.MOTION_ACTION, number)]
                              if key in self._confirmed_commands and key in self._confirmed_evidence_times
                              and self._confirmed_commands[key].state_flags & sequence.StateFlag.TURN_ACTIVE]
                    confirmed_at, applied = min(proofs, key=lambda item: item[0]) if proofs else (None, None)
                    drive_until = (now if completed_at is None else max(response.at, min(now, completed_at)))
                    try:
                        response.update(self, _patrol_turn_wheels(command.action, command.speed_mms), drive_until,
                                        confirmed_at, samples=turn_samples, applied_state=applied)
                    except mcu.McuError:
                        self._patrol_uturn_terminal_fault = True
                        raise
                if completed is not None:
                    self._active_motion_sent_at = None
                    return completed
                if (turn and turn.get("correction") and (active_seen or is_uturn)
                        and self.patrol_world_yaw_deg is not None and self._sample_is_fresh("state")
                        and tuple(turn.get("identity", ())) == (nonce, boot)
                        and self._patrol_last_yaw_source is not None
                        and self._patrol_last_yaw_source >= sent_at - 1e-9):
                    world = self.patrol_world_yaw_deg
                    remaining = sign * (turn["goal_world"] - world)
                    traveled = sign * (world - turn["start_world"])
                    side = "left" if sign > 0 else "right"
                    decision = turn_recovery_decision(sign * turn["goal_world"], sign * world,
                        self._patrol_yaw_rate_degps, max(0., now - self._patrol_last_yaw_source),
                        brake_tail_deg=self._patrol_brake_tail_deg[side],
                        allowed_arc_deg=turn.get("allowed_arc", math.inf), traveled_deg=traveled)
                    margin = decision["predicted_brake_deg"]
                    if decision["should_brake"]:
                        cutoff = dict(identity=(nonce, boot), baseline=self.state.uptime_ms,
                            issued=tuple(issued), action=command.action, active_proven=active_seen,
                            world_yaw=world, side=side)
                        self._patrol_expected_cutoff = cutoff
                        brake = self.send_motion_action(sequence.MotionActionCode.BRAKE, 0,
                                                        require_ranges=False, require_upper=False)
                        cutoff["sequence"] = brake
                        turn.update(cutoff=True, last_brake_sequence=brake)
                        raw = self.wait_for_motion_action_applied(brake)
                        tail = max(0.0, sign * (self.patrol_world_yaw_deg - world))
                        self._patrol_brake_tail_deg[side] = max(self._patrol_brake_tail_deg[side], tail)
                        if sign * (turn["goal_world"] - self.patrol_world_yaw_deg) < 0:
                            turn["overshoot_arc_limit"] = max(.5, traveled / 2.0)
                        runlog.emit("stage1_turn_correction_cutoff", "根据真实剩余航向与制动预测主动BRAKE，原始STATE保持不变",
                                    sequence=number, brake_sequence=brake, remaining_deg=remaining,
                                    predicted_brake_deg=margin, observed_tail_deg=tail,
                                    raw_turn_aborted=bool(raw.state_flags & sequence.StateFlag.TURN_ABORTED))
                        return raw
                if now >= next_renew:
                    renewed = self.send_motion_action(command.action, command.speed_mms,
                        brake_after_turn=bool(command.flags & sequence.MotionActionFlag.BRAKE_AFTER_TURN),
                        require_ranges=False, require_upper=False)
                    issued.add(renewed)
                    if turn:
                        turn["issued"] = list(issued)
                    next_renew = time.monotonic() + sequence.COMMAND_INTERVAL_S
                time.sleep(min(sequence.CONTROL_INTERVAL_S, max(.001, next_renew - time.monotonic())))
        finally:
            self._turn_events = None
            self._turn_waiting = False
            self.side_age_required = False

    def send_stop(self, reason=sequence.StopReason.MISSION):
        result = super().send_stop(reason)
        self.finish_departure()
        return result

    def close(self):
        try:
            super().close()
        finally:
            self.finish_departure()


class PatrolDisplay(sequence.MotionDisplay):
    def __init__(self, link, emitter=None, *, settings=None):
        super().__init__(link, emitter=emitter)
        link._patrol_display = self
        self.controller = PatrolCenteringController(time.monotonic(), settings=settings)
        self.services = None
        self.patrol_deadline = math.inf
        self._post_turn_sampling_observer = None

    def tick(self, **kwargs):
        completed = False
        try:
            if kwargs.get("check_health", True):
                sequence._remaining(self.patrol_deadline)
            super().tick(**kwargs)
            completed = True
        finally:
            if self._post_turn_sampling_observer is not None:
                self._post_turn_sampling_observer("tick", None, time.monotonic(), completed)

    def consume_samples(self, *, require_healthy=True, collect_means=True,
                        allow_stale=None, pair_observer=None, update_controller=None):
        samples, self._samples = self._samples, []
        now = time.monotonic()
        if update_controller is None:
            update_controller = collect_means
        if require_healthy:
            if self.link.directional is not None:
                _check_patrol_sides(sequence._side_sample(self.link), now)
            for sample in samples:
                _check_patrol_sides(sample, now)

        def publish_means(batches):
            for batch in batches:
                if collect_means and update_controller:
                    self.controller.observe_mean_batch(batch, now=now)
                self._ready_batches.append(batch)

        for sample in samples:
            sequence._annotate_side_sample(self.link, sample)
            observation = (f"开口:{','.join(sorted(sample.openings))}" if sample.openings else "双墙")
            if any(status not in sequence.HEALTHY_RANGE_STATUSES
                   for status in (sample.left_status, sample.right_status)):
                observation = "陀螺仪穿口／离口缺测；不设激光等待期限，等待首组真正新双墙"
            try:
                _check_patrol_sides(sample, now)
            except mcu.McuError:
                observation = "故障"
            changes = [None, None]
            for index, (distance, status) in enumerate((
                    (sample.left_mm, sample.left_status), (sample.right_mm, sample.right_status))):
                previous = self._previous_valid_sides[index]
                key = sample.source_keys[index] if sample.source_keys is not None else sample.seq
                if (observation != "故障" and status == sequence.WireRangeStatus.VALID
                        and sample.ages_ms[index] + max(0.0, (now - sample.received_at) * 1000.0)
                        <= sequence.LASER_MEASUREMENT_MAX_AGE_MS + 1e-6
                        and (previous is None or sequence._source_key_newer(key, previous[0]))):
                    if previous is not None:
                        changes[index] = distance - previous[1]
                    self._previous_valid_sides[index] = key, distance
            self.side_changes = tuple(changes)
            if self.action != "停车等待xx":
                runlog.emit("motion_sequence_laser", "下层激光观测",
                            changes_mm=self.side_changes, observation=observation,
                            candidate_sides=sorted(sample.candidates),
                            saved_openings=sorted(sequence._pending_junction_openings(self.link)),
                            candidate_rule="同会话源键推进且真实源龄<=250ms的1/2开口",
                            departure_mode=getattr(self.link, "_patrol_gyro_mode", "GYRO"),
                            **sample.diagnostics(now))
            publish_means(self.averages.advance(sample.received_at))
            if collect_means or update_controller:
                elapsed_ms = max(0.0, (now - sample.received_at) * 1000.0)
                evidence = sequence.replace(sample, received_at=now,
                                            ages_ms=tuple(age + elapsed_ms for age in sample.ages_ms))
                if collect_means:
                    self.averages.add(evidence)
            if update_controller and sample.dual_wall and all(
                    0 <= now - source <= .250 + 1e-9 for source in sample.source_times):
                old_token = self.controller.observation_token
                self.controller.observe_side(evidence)
                if pair_observer is not None and self.controller.observation_token != old_token:
                    pair_observer(sample)
            elif (update_controller and self.controller._patrol_cruise_policy
                  and pair_observer is not None):
                pair_observer(sample)
            if self._post_turn_sampling_observer is not None:
                self._post_turn_sampling_observer("side", sample, now, True)
        publish_means(self.averages.advance(now))
        if self._post_turn_sampling_observer is not None:
            self._post_turn_sampling_observer("consume", None, now, True)
        return samples

    def junction_snapshot(self, now=None):
        snapshot = super().junction_snapshot(now)
        if snapshot is not None:
            remaining = _junction_wait_remaining(self.link, now)
            snapshot["wait_remaining_s"] = None if math.isinf(remaining) else remaining
            snapshot["brake_evidence_at"] = _brake_evidence_at(self.link)
        return snapshot

    def _format_snapshot(self, batch):
        snapshot = super()._format_snapshot(batch).replace("转前3秒剩余=", "严格停稳后额外1秒剩余=")
        feedback = self.controller._control_feedback or {}
        limit = feedback.get("correction_heading_limit_deg", PATROL_CORRECTION_HEADING_LIMIT_DEG)
        if self.controller._patrol_cruise_policy and limit is not None:
            snapshot = snapshot.replace("纠偏目标不设固定偏角上限", f"纠偏目标固定参考±{limit:g}°")
        return snapshot


class PatrolRunner:
    """Continuous road following; distances never manufacture a normal junction."""

    def __init__(self, link, display, field_map, steps, ledger, services=None,
                 *, deadline=math.inf, input_source=None, departure_strategy="10mm_turn", settings=None):
        if departure_strategy not in DEPARTURE_STRATEGIES:
            raise ValueError("departure_strategy必须为10mm_turn或40mm_pivot")
        self.departure_strategy = departure_strategy
        self.link, self.display = link, display
        self.settings = settings
        if settings is not None:
            self.link.settings = settings
            self.display.controller.apply_settings(settings)
        if not hasattr(link, "patrol_checkpoint"):
            link.patrol_checkpoint = {}
        enable_policy = getattr(self.display.controller, "enable_patrol_cruise_policy", None)
        if callable(enable_policy):
            enable_policy()
        self.field_map, self.steps, self.ledger = field_map, steps, ledger
        self.services, self.deadline, self.input_source = services, deadline, input_source
        link._patrol_input = input_source
        self.index = 0
        self.phase = "START"
        self.goal = self.cruise_speed_mmps if settings is not None else INITIAL_SPEED_MMPS
        self._n06_slow_active = False
        self.display.controller.patrol_road_speed_cap = None
        self.link._patrol_road_speed_cap = None
        self.armed = True
        self.passage_barrier = None
        self.pending_wall = None
        self.first_wall_source = None
        self.first_wall_sources = None
        self.sampler = MovingHeadingSampler()
        self.post_turn_sampler = PatrolRoadHeadingSampler()
        self.stopped_sampler = StoppedHeadingSampler()
        self._alignment_mode_index = 0
        self.post_turn_low_speed = False
        self.first_exit_no_sampling = False
        self.cruise_transition_pending = False
        self._post_turn_pending_states = []
        self.response = sequence._WheelResponseTracker(link, time.monotonic())
        self.progress = sequence._RoadProgress(link.odom_mm)
        self.next_send = time.monotonic()
        self.sent_at = None
        self.brake_delay_s = HOME_INITIAL_BRAKE_S
        self.home_result = None
        self._previous_observer = display._distance_observer
        self._odom_warning = False
        self.detector = PatrolJunctionDetector()
        self.junction_evidence = None
        self.pending_candidate = None
        self.pending_junction_decision = None
        self.first_actual_turn_completed = False
        self.departure_normal_turn_completed = False
        self.reference_fixed = True
        self.alignment_goal = self.cruise_speed_mmps if settings is not None else START_SPEED_MMPS
        self._initialized = False
        self._localization_samples = deque(maxlen=256)
        self._laser_resume_token = None
        self._control_selection = None
        self._midroad_uturn_reason = None
        self._midroad_returning = False
        self._midroad_return_wait = False
        self._retreat_contract = None
        self._coordinate_mode = "ROAD"
        self.transit_distance_mm = 0.0
        self.road_axis_deg = self.display.controller.heading_reference_deg
        self.startup_reference_deg = self.road_axis_deg
        self.road_position_mm = 0.0
        self.wall_offset_mm = None
        self._coordinate_baseline = None
        self._coordinate_history = deque(maxlen=512)
        self._coordinate_identity = None
        self._initial_origin_mm = None
        self._initial_identity = None
        self._initial_positions_mm = None
        self._initial_state_key = self._initial_state_source = None
        self._initial_command = None
        self._initial_confirmed = False
        self._initial_transition_queued = False
        self._initial_complete = False

    @property
    def cruise_speed_mmps(self):
        settings = getattr(self, "settings", None)
        return settings.cruise_speed_mmps if settings is not None else CRUISE_SPEED_MMPS

    @property
    def special_speed_mmps(self):
        settings = getattr(self, "settings", None)
        return settings.special_speed_mmps if settings is not None else 100

    @property
    def turn_speed_mmps(self):
        settings = getattr(self, "settings", None)
        return settings.turn_speed_mmps if settings is not None else TURN_SPEED_MMPS

    @property
    def home_speed_mmps(self):
        settings = getattr(self, "settings", None)
        return settings.home_speed_mmps if settings is not None else HOME_SPEED_MMPS

    @property
    def initial_speed_mmps(self):
        settings = getattr(self, "settings", None)
        return settings.cruise_speed_mmps if settings is not None else INITIAL_SPEED_MMPS

    @property
    def start_speed_mmps(self):
        settings = getattr(self, "settings", None)
        return settings.cruise_speed_mmps if settings is not None else START_SPEED_MMPS

    def _departure_turn_pending(self):
        return (not self.departure_normal_turn_completed
                and self.next_action in (coverage.TURN_LEFT, coverage.TURN_RIGHT))

    def _departure_state_source(self):
        source = getattr(self.link, "_sample_time", None)
        return source("state") if callable(source) else self.display.controller._state_at

    def _departure_state_fresh(self):
        source = self._departure_state_source()
        return (source is not None and 0 <= time.monotonic() - source
                <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9)

    def _departure_advance_state(self, state, source_at):
        saved = getattr(self.link, "patrol_checkpoint", {}).get("departure_advance")
        if (saved is None or saved.get("state") != "ADVANCING"
                or saved["step"] != self.step.index):
            return False
        identity = state.session_nonce, state.boot_id
        if identity != (self.link.session_nonce, self.link.boot_id):
            raise mcu.McuError("首路口追加前进STATE不是当前会话身份")
        original = tuple(saved["identity"])
        if identity != original:
            if (identity[1] != original[1] or original not in
                    getattr(self.link, "_patrol_coordinate_identities", ())):
                raise mcu.McuError("首路口追加前进的原编码器坐标不可验证")
            saved["identity"] = identity
        if (state.odom_status != 0 or not state.state_flags & sequence.StateFlag.ODOM_VALID
                or not self._departure_state_fresh()
                or source_at <= saved["source_at"] + 1e-9
                or not sequence._source_key_newer(state.uptime_ms, saved["source_key"])):
            return False
        saved.update(source_at=source_at, source_key=state.uptime_ms,
                     positions_mm=(state.left_mm, state.right_mm))
        saved["distance_mm"] = math.fsum(current - origin for current, origin in zip(
            saved["positions_mm"], saved["origin_mm"])) / 2.0
        if saved["distance_mm"] + 1e-9 >= saved["threshold_mm"]:
            saved["state"] = "REACHED"
            sequence._apply_brake(self.link, self.display, self.deadline)
            self.link._patrol_early_junction_brake = self.link.session_nonce, self.link.boot_id
            self.link._patrol_exclusive_motion = None
            runlog.emit("stage1_departure_advance_reached", "首路口追加前進跨过编码器阈值，立即BRAKE",
                        **saved)
            return True
        return False

    def _advance_departure_junction(self):
        if not self._departure_turn_pending():
            return
        link, display = self.link, self.display
        saved = link.patrol_checkpoint.get("departure_advance")
        if saved is None or saved["step"] != self.step.index:
            raise mcu.McuError("首路口追加前进缺少确认时的新鲜STATE轮程基线")
        if saved["state"] != "ADVANCING":
            return
        speed = saved.get("speed_mmps", self.initial_speed_mmps)
        previous_observer = display._distance_observer
        previous_side_age = link.side_age_required
        display._distance_observer = self._observe_distance
        link.side_age_required = False
        next_send, sent_at = time.monotonic(), None
        response = sequence._WheelResponseTracker(link, next_send)
        try:
            while saved["state"] == "ADVANCING":
                sequence._check_input_exit(self.input_source)
                sequence._remaining(self.deadline)
                display.tick()
                display.consume_samples(collect_means=False, update_controller=False, allow_stale=True)
                now = time.monotonic()
                if saved["state"] != "ADVANCING":
                    break
                self._departure_advance_state(link.state, self._departure_state_source())
                if saved["state"] != "ADVANCING":
                    break
                executing, accepted_at, applied = sequence._confirmed_wheel_response(link, sent_at, (speed, speed))
                response.update(link, executing, now, accepted_at, display.state_samples, applied_state=applied)
                if now >= next_send:
                    link._patrol_exclusive_motion = dict(kind="departure_advance",
                        identity=(link.session_nonce, link.boot_id), target=(speed, 0), checkpoint=saved)
                    try:
                        link.send_motion(speed, 0, require_ranges=False, require_upper=False)
                    except Exception as exc:
                        if (isinstance(exc, sequence._CenteringProposalExpired)
                                or getattr(exc, "motion_not_written", False)):
                            self._departure_advance_state(link.state, self._departure_state_source())
                            if saved["state"] != "ADVANCING":
                                break
                        raise
                    display.command, display.wheel_targets = (speed, 0), (speed, speed)
                    display.command_kind = sequence.MessageType.CONTROL_TARGET
                    display.action = "首路口保存决定后追加前进"
                    display.control_mode = f"{self.departure_strategy}；编码器净行程{saved['distance_mm']:.1f}/{saved['threshold_mm']:g}mm"
                    sent_at = time.monotonic()
                    next_send = sent_at + sequence.COMMAND_INTERVAL_S
                display.maybe_print()
                time.sleep(sequence.CONTROL_INTERVAL_S)
        finally:
            link._patrol_exclusive_motion = None
            link.side_age_required = previous_side_age
            display._distance_observer = previous_observer

    def _target_speed_goal(self):
        """Internal speed cap hook; the road's requested baseline stays intact."""
        return min(self.goal, self.special_speed_mmps) if self._n06_slow_active else self.goal

    def _straight_passage_speed_goal(self, default_goal):
        if self.step.to_node == "n06" and self.next_action == coverage.TURN_STRAIGHT:
            self._n06_slow_active = True
            self.display.controller.patrol_road_speed_cap = self.special_speed_mmps
            self.link._patrol_road_speed_cap = self.special_speed_mmps
            self.post_turn_sampler.cancel()
            self.next_send = time.monotonic()
        return min(default_goal, self.special_speed_mmps) if self._n06_slow_active else default_goal

    def _update_n06_passage(self, now):
        if (not self._n06_slow_active or not self.armed or self.step.from_node != "n06"
                or self.progress.position_mm is None or self.progress.position_mm < 50.0):
            return
        self._n06_slow_active = False
        self.display.controller.patrol_road_speed_cap = None
        self.link._patrol_road_speed_cap = None
        self.goal = self.cruise_speed_mmps
        self.cruise_transition_pending = True
        self.next_send = now
        self.post_turn_sampler.cancel()
        self.post_turn_sampler.start(self.display.controller, now)
        self._checkpoint()
        runlog.emit("stage1_n06_alignment_finished", "n06后编码器满50mm，恢复150纠偏及新道路采样",
                    step=self.step.index, distance_mm=self.progress.position_mm)

    def _use_start_heading_control(self, now):
        """Formal obstacle approach may opt into fresh aligned laser control."""
        return (self.phase != "HOME" and not getattr(self, "first_actual_turn_completed", False))

    @property
    def initial_distance_mm(self):
        if self._initial_origin_mm is None or self._initial_positions_mm is None:
            return None
        return math.fsum(current - origin for current, origin in zip(
            self._initial_positions_mm, self._initial_origin_mm)) / 2.0

    def _prepare_initial_launch(self):
        if self.index != 0 or self.phase != "START" or self._initial_command is not None:
            return
        if self._initial_origin_mm is not None:
            return
        # Take the origin immediately before the first drive, after the
        # existing BRAKE/stationary and heading-sampling authorization.
        if not sequence._assert_still_stable(self.link, self.display):
            raise mcu.McuError("首发150目标缺少严格停车轮程基线")
        self._initial_origin_mm = tuple(self.link.odom_mm)
        self._initial_positions_mm = self._initial_origin_mm
        self._initial_identity = self.link.session_nonce, self.link.boot_id
        self._initial_state_key = self.link.state.uptime_ms
        self._initial_state_source = self.display.controller._state_at
        self._checkpoint()

    def _initial_launch_write_guard(self):
        """Pure-read final UART check; no pump or extra sample window."""
        if self._initial_command is not None or self.index != 0 or self.phase != "START":
            return
        if (not sequence._assert_still_stable(self.link, self.display)
                or self._initial_identity != (self.link.session_nonce, self.link.boot_id)
                or self._initial_origin_mm != tuple(self.link.odom_mm)):
            raise mcu.McuError("首次150实际写入前严格停稳或原始轮程基线已改变")

    def _observe_initial_launch_state(self, state, at):
        if self._initial_origin_mm is None or self._initial_complete:
            return False
        identity = getattr(state, "session_nonce", None), getattr(state, "boot_id", None)
        positions = getattr(state, "left_mm", None), getattr(state, "right_mm", None)
        key = getattr(state, "uptime_ms", None)
        now = time.monotonic()
        migrated = (identity != self._initial_identity
                    and self._initial_identity in getattr(self.link, "_patrol_coordinate_identities", ())
                    and identity in getattr(self.link, "_patrol_coordinate_identities", ())
                    and identity[1] == self._initial_identity[1])
        if (identity != (self.link.session_nonce, self.link.boot_id)
                or identity != self._initial_identity and not migrated
                or isinstance(key, bool) or not isinstance(key, int)
                or getattr(state, "odom_status", None) != 0
                or not getattr(state, "state_flags", 0) & sequence.StateFlag.ODOM_VALID
                or sequence._active_runtime_fault_bits(self.link, state)
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in (*positions, at))
                or not -1e-9 <= now - at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                or self._initial_state_source is not None and at <= self._initial_state_source + 1e-9
                or not migrated and self._initial_state_key is not None
                and not sequence._source_key_newer(key, self._initial_state_key)):
            return False
        self._initial_identity = identity
        self._initial_state_key, self._initial_state_source = key, at
        self._initial_positions_mm = positions
        return True

    def _update_initial_launch(self, now):
        if self._initial_complete or self.index != 0 or self.phase != "START":
            return
        command = self._initial_command
        if not self._initial_confirmed and command is not None:
            key, target, identity, sent_at = command
            record = getattr(self.link, "_command_ledger", {}).get(key)
            applied = getattr(self.link, "_confirmed_commands", {}).get(key)
            evidence_at = getattr(self.link, "_confirmed_evidence_times", {}).get(key)
            if (record is not None and applied is not None and evidence_at is not None
                    and tuple(record[6:8]) == identity
                    and tuple(record[5]) == target and record[0] == sent_at
                    and (applied.session_nonce, applied.boot_id) == identity
                    and sent_at - 1e-9 <= evidence_at
                    <= sent_at + config.MCU_COMMAND_CONFIRM_TIMEOUT_S + 1e-9
                    and not (len(record) > 10 and record[10].get("drive_invalidated_by_brake"))):
                self._initial_confirmed = True
        if self._initial_confirmed:
            self._initial_complete = True
            self._checkpoint()
            runlog.emit("stage1_initial_speed_ready", "首发150已及时接受并应用，保持启动航向直到第一次实际转弯完成",
                        origin_mm=self._initial_origin_mm, target_mmps=self.goal)

    @property
    def step(self):
        return self.steps[self.index]

    @property
    def next_action(self):
        if self.index + 1 == len(self.steps):
            return None
        following = self.steps[self.index + 1]
        before = coverage.direction_between(self.field_map, self.step.from_node, self.step.to_node)
        after = coverage.direction_between(self.field_map, following.from_node, following.to_node)
        return coverage.turn_kind(before, after)

    def _set_step(self, *, after_turn=False):
        self._retreat_contract = None
        if after_turn:
            self.first_actual_turn_completed = True
        if after_turn or self.step.from_node != "n06" or self.phase == "HOME":
            self._n06_slow_active = False
        cap = self.special_speed_mmps if self._n06_slow_active else None
        self.display.controller.patrol_road_speed_cap = cap
        self.link._patrol_road_speed_cap = cap
        self._odom_warning = False
        self._laser_resume_token = None
        self._control_selection = None
        self.post_turn_sampler.cancel()
        self._post_turn_pending_states.clear()
        legacy_sampling = self.phase in ("ALIGN", "SAMPLE")
        self.first_exit_no_sampling = False
        self.display.controller.set_patrol_road_policy(
            "HOME" if self.phase == "HOME" else "START" if not self.first_actual_turn_completed else "ORDINARY")
        self.link._patrol_turn_brake_required = False
        self.link._patrol_exclusive_motion = None
        self.stopped_sampler.cancel()
        self._alignment_mode_index = 0
        self.cruise_transition_pending = self.index == 0 and not self._initial_complete
        self.display.controller._patrol_pending_cruise_goal = None
        # Legacy isolated alignment helpers remain callable for their safety
        # tests; the navigation loop never enters either of these phases.
        self.post_turn_low_speed = bool(legacy_sampling and after_turn and self.index > 0)
        self.display.controller._patrol_formal_reference = None
        special = self.index == 0 or self.phase == "HOME"
        origin = self.link.odom_mm if self.index == 0 else None
        self.ledger.begin_road(self.step, origin, self.link.yaw_deg)
        self.display.origin = tuple(origin) if origin is not None else None
        self.display.segment_distance_mm = 0.0 if origin is not None else None
        self.progress = sequence._RoadProgress(origin)
        self.progress.update(self.link.state, self.display.controller._state_at or time.monotonic())
        self.reference_fixed = not legacy_sampling or self.index == 0 or self.phase == "HOME" or not after_turn
        self.road_axis_deg = self.display.controller.heading_reference_deg
        self.road_position_mm = 0.0
        self.wall_offset_mm = None
        self._coordinate_mode = "ROAD"
        self._coordinate_baseline = None
        self._coordinate_history.clear()
        self._coordinate_identity = None
        self._observe_coordinate(self.link.state, self.display.controller._state_at or time.monotonic())
        if (not special and self.first_actual_turn_completed and not legacy_sampling
                and not self._n06_slow_active):
            self.post_turn_sampler.start(self.display.controller, time.monotonic())
        self.junction_evidence = self.pending_candidate = self.pending_junction_decision = None
        self._localization_samples.clear()
        minimum, maximum = (200., 500.) if self.step.road_id in ("r02", "r03") else (600., 1000.)
        self.detector.reset(minimum, maximum,
            identity=(getattr(self.link, 'session_nonce', None), getattr(self.link, 'boot_id', None)),
            require_window=not special)
        self.link.patrol_next_action = self.next_action or coverage.TURN_STRAIGHT
        self.link.patrol_route_context = dict(step=self.step.index, road=self.step.road_id,
                                             node=self.step.to_node, planned_action=self.next_action)
        if self.services is not None:
            self.services.set_step(self.step)
        self._checkpoint()
        runlog.emit("stage1_road_started", "按实际路口进入下一道路",
                    step=self.step.index, road=self.step.road_id, from_node=self.step.from_node,
                    to_node=self.step.to_node, next_action=self.next_action,
                    after_turn=after_turn, phase=self.phase, wheel_origin_mm=self.progress.origin_mm,
                    wheel_positions_mm=self.link.odom_mm)

    def _short_road_reference(self, step):
        return None

    def _learn_road_heading(self, now, *, soft_missing=False):
        if (not self.armed or self.phase == "HOME"
                or not getattr(self, "first_actual_turn_completed", False)
                or self._n06_slow_active or (self.phase != "CRUISE" and not (
                    self.phase == "RETURN_ROAD" and getattr(self, "_retreat_contract", None)))
                or self.pending_candidate is not None or self.pending_junction_decision is not None
                or getattr(self.link, "_patrol_turn_brake_required", False)
                or getattr(self.link, "_hard_fault_reason", None)):
            return
        controller = self.display.controller
        sampler = self.post_turn_sampler
        mean = sampler.update(controller, self.display.state_samples, now, soft_missing=soft_missing)
        if mean is not None:
            controller._road_reference = controller._hold_heading = mean
            controller._patrol_formal_reference = controller._patrol_cruise_hold_goal = mean
            controller._control_feedback = None
            sampler._expected_reference = mean
            runlog.emit("stage1_reference_updated", "本道路合格航向累计均值更新，最多10次；计程轴保持",
                        step=self.step.index, road=self.step.road_id, reference_deg=mean,
                        sample_count=sampler.sample_count, completed=sampler.completed,
                        fixed_entry_reference_deg=sampler._reference)

    def _observe_distance(self, state):
        at = self.display.controller._state_at
        self._departure_advance_state(state, at if at is not None else time.monotonic())
        self._observe_initial_launch_state(state, at if at is not None else time.monotonic())
        self.progress.update(state, at if at is not None else time.monotonic())
        self._observe_coordinate(state, at if at is not None else time.monotonic())
        self.display.segment_distance_mm = (self.progress.position_mm if self.phase == "HOME"
                                            else self.road_position_mm)
        if self._previous_observer is not None:
            self._previous_observer(state)

    def _observe_coordinate(self, state, at):
        identity = getattr(state, "session_nonce", None), getattr(state, "boot_id", None)
        key = getattr(state, "uptime_ms", None)
        wheels = getattr(state, "left_mm", None), getattr(state, "right_mm", None)
        lookup = getattr(self.display.controller, "pose_at", None)
        pose = lookup(at) if callable(lookup) else None
        yaw = pose.yaw if pose is not None else self.link.yaw_deg
        if (None in identity or isinstance(key, bool) or not isinstance(key, int)
                or getattr(state, "odom_status", None) != 0
                or not getattr(state, "state_flags", 0) & sequence.StateFlag.ODOM_VALID
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in (*wheels, yaw, at))):
            return False
        previous = self._coordinate_baseline
        if previous is not None:
            old_at, old_key, old_wheels, old_yaw = previous
            if at <= old_at + 1e-9:
                return False
            if identity == self._coordinate_identity and not sequence._source_key_newer(key, old_key):
                return False
            if identity != self._coordinate_identity and self._coordinate_identity not in getattr(
                    self.link, "_patrol_coordinate_identities", ()):
                # Unverified boot/session changes cannot keep a trusted road.
                self.road_axis_deg = None
            distance = sum(a - b for a, b in zip(wheels, old_wheels)) / 2.
            if self._coordinate_mode == "TRANSIT":
                self.transit_distance_mm += distance
            elif self._coordinate_mode == "ROAD" and self.road_axis_deg is not None:
                middle = old_yaw + sequence._normalize_yaw(yaw - old_yaw) / 2.
                self.road_position_mm += distance * math.cos(math.radians(middle - self.road_axis_deg))
        self._coordinate_baseline = at, key, wheels, yaw
        self._coordinate_identity = identity
        self._coordinate_history.append((at, self.road_position_mm))
        return True

    def request_midroad_uturn(self, reason) -> bool:
        """Local request only; callers cannot forge a route-junction decision."""
        now = time.monotonic()
        controller = self.display.controller
        if (not isinstance(reason, str) or not reason.strip() or self.index == 0
                or self.phase != "CRUISE" or not self.armed or not self.reference_fixed
                or self._midroad_uturn_reason is not None or self._midroad_returning
                or self._midroad_return_wait or self.road_axis_deg is None
                or self._coordinate_baseline is None or self.first_wall_source is None
                or not 0 <= now - self._coordinate_baseline[0] <= config.TELEMETRY_STALE_MS / 1000. + 1e-9
                or controller.heading_reference_deg is None
                or getattr(self.link, "_hard_fault_reason", None)
                or getattr(self.link, "patrol_manual_required", None)
                or self._coordinate_identity != (self.link.session_nonce, self.link.boot_id)):
            return False
        self._midroad_uturn_reason = reason.strip()
        return True

    def _execute_midroad_uturn(self):
        self.response.pause(self.link, time.monotonic())
        _brake(self.link, self.display, self.deadline, input_source=self.input_source)
        # BRAKE tail belongs to this road; freeze only the actual pivot.
        self._coordinate_mode = "PIVOT"
        reason = self._midroad_uturn_reason
        authorization = dict(kind="midroad_uturn", road=self.step.road_id,
                             identity=(self.link.session_nonce, self.link.boot_id),
                             reason=reason, position_mm=self.road_position_mm)
        self.link._patrol_midroad_authorization = authorization
        _turn_planned(self.link, self.display, frozenset(), self.deadline, side="left",
                      turn_speed_mmps=self.turn_speed_mmps,
                      angle_deg=180, departure_speed_mmps=self.cruise_speed_mmps,
                      input_source=self.input_source, midroad_authorization=authorization)
        self._coordinate_mode = "ROAD"
        self._midroad_uturn_reason = None
        self._midroad_returning = True
        self.phase, self.goal = "RETURN_ROAD", self.cruise_speed_mmps
        self.reference_fixed, self.post_turn_low_speed = True, False
        self._begin_passage(self.link.post_turn_barrier, after_turn=True)
        self.sent_at, self.next_send = None, time.monotonic()
        self.response.pause(self.link, self.next_send)
        self._checkpoint()
        runlog.emit("stage1_midroad_return_started", "原地掉头保留道路坐标轴，150沿同一道路返回入口",
                    **authorization, fixed_reference_deg=self.display.controller.heading_reference_deg,
                    road_axis_deg=self.road_axis_deg)

    def _finish_midroad_return(self):
        self.response.pause(self.link, time.monotonic())
        _brake(self.link, self.display, self.deadline, input_source=self.input_source)
        self.ledger.confirm_rollback(self.step.from_node)
        self.road_position_mm = 0.0
        self._coordinate_history.clear()
        self._midroad_return_wait = True
        self.phase = "RETURN_WAIT"
        self.pending_candidate = None
        self.armed = False
        self.link.side_candidates_enabled = False
        self._checkpoint()
        runlog.emit("stage1_midroad_return_confirmed", "真实开口确认返回原入口，严格停稳等待；路线未推进",
                    road=self.step.road_id, node=self.step.from_node,
                    evidence=asdict(self.junction_evidence), route_index=self.index)

    def _checkpoint(self):
        callback = getattr(self.link, 'store_checkpoint', None)
        if callable(callback):
            callback(route_index=self.index, phase=self.phase,
                     road_origin_mm=self.progress.origin_mm,
                     road_origin_source_at=self.progress.origin_source_at,
                     fixed_reference=(self.display.controller.heading_reference_deg
                         if self.display.controller.heading_reference_deg is not None
                         else getattr(self.link, 'patrol_checkpoint', {}).get('fixed_reference')),
                     reference_fixed=self.reference_fixed,
                     road_axis_deg=self.road_axis_deg, road_position_mm=self.road_position_mm,
                     wall_offset_mm=self.wall_offset_mm, coordinate_mode=self._coordinate_mode,
                     transit_distance_mm=self.transit_distance_mm,
                     midroad_returning=self._midroad_returning,
                     midroad_return_wait=self._midroad_return_wait,
                     first_exit_no_sampling=self.first_exit_no_sampling,
                     first_actual_turn_completed=self.first_actual_turn_completed,
                     departure_normal_turn_completed=self.departure_normal_turn_completed,
                     departure_strategy=self.departure_strategy,
                     pending_junction_decision=self.pending_junction_decision,
                     goal_mmps=self.goal, alignment_goal_mmps=self.alignment_goal,
                     cruise_transition_pending=self.cruise_transition_pending,
                     initial_launch=dict(origin_mm=self._initial_origin_mm,
                         identity=self._initial_identity, positions_mm=self._initial_positions_mm,
                         source_at=self._initial_state_source, source_key=self._initial_state_key,
                         distance_mm=self.initial_distance_mm, command=self._initial_command,
                         confirmed=self._initial_confirmed,
                         transition_queued=self._initial_transition_queued,
                         completed=self._initial_complete),
                     post_turn_low_speed=self.post_turn_low_speed,
                     post_turn_sampling_started_at=self.stopped_sampler.started_at,
                     post_turn_sampling_deadline=self.stopped_sampler.deadline,
                     post_turn_sampling_completed=self.stopped_sampler.completed,
                     alignment_policy="CONTINUOUS_FORWARD",
                     junction_evidence=asdict(self.junction_evidence) if self.junction_evidence else None)

    def _observe_post_turn_sampling(self, kind, sample, now, healthy):
        """Sample accepted STATE throughout BRAKE acknowledgement and strict settling."""
        if healthy and self.pending_candidate is None and self.armed and self.phase != "HOME":
            candidates = getattr(self.display, "_samples", ()) if kind == "tick" else (sample,) if sample is not None else ()
            candidate = self._detect_junction(candidates, now) if candidates else None
            if candidate is not None:
                self._capture_junction(candidate)
        if kind == "tick" and self.stopped_sampler.started_at is not None:
            self.stopped_sampler.update(self.display.controller,
                getattr(self.display, "state_samples", ()) if healthy else (), now,
                faulted=not healthy or bool(getattr(self.link, "_hard_fault_reason", None)
                    or getattr(self.link, "patrol_manual_required", None)))

    def _capture_junction(self, candidate):
        """Preserve the route decision, then write BRAKE before checkpoint/log work."""
        if self.next_action == coverage.TURN_STRAIGHT and self.step.to_node == "n06":
            self.goal = self._straight_passage_speed_goal(self.goal)
        self.pending_candidate = candidate
        if self.junction_evidence is not None:
            self.pending_junction_decision = dict(step=self.step.index, action=self.next_action,
                                                 evidence=asdict(self.junction_evidence))
            checkpoint = getattr(self.link, "patrol_checkpoint", None)
            if checkpoint is not None:
                checkpoint.update(junction_evidence=asdict(self.junction_evidence),
                                  saved_openings=self.junction_evidence.openings)
        if self.next_action != coverage.TURN_STRAIGHT:
            self.link._patrol_turn_brake_required = True
            if candidate is not None and candidate.candidates:
                _latch_patrol_junction(self.link, candidate, initial=True)
            if self._departure_turn_pending():
                saved = self.link.patrol_checkpoint.get("departure_advance")
                if saved is None or saved["step"] != self.step.index:
                    state = self.link.state
                    if (state is None or not self._departure_state_fresh()
                            or (state.session_nonce, state.boot_id) != (self.link.session_nonce, self.link.boot_id)
                            or state.odom_status != 0 or not state.state_flags & sequence.StateFlag.ODOM_VALID):
                        raise mcu.McuError("首路口确认时缺少新鲜合法STATE编码器基线")
                    advance = dict(
                        step=self.step.index, strategy=self.departure_strategy,
                        speed_mmps=self.initial_speed_mmps,
                        state="ADVANCING", origin_mm=(state.left_mm, state.right_mm),
                        positions_mm=(state.left_mm, state.right_mm), identity=(state.session_nonce, state.boot_id),
                        source_at=self._departure_state_source(), source_key=state.uptime_ms,
                        distance_mm=0.0, threshold_mm=DEPARTURE_STRATEGIES[self.departure_strategy])
                    reference = getattr(self.display.controller, "formal_reference_deg", None)
                    if reference is None:
                        reference = self.display.controller.heading_reference_deg
                    if reference is None or self.link.yaw_deg is None:
                        raise mcu.McuError("首路口确认缺少冻结道路航向")
                    world = getattr(self.link, "patrol_world_yaw_deg", None)
                    if world is None:
                        world = self.link.yaw_deg
                    delta = 90 if self.next_action == coverage.TURN_LEFT else -90
                    advance.update(
                        side="left" if delta > 0 else "right", previous_reference=reference,
                        goal_yaw=sequence._normalize_yaw(reference + delta),
                        goal_world=world + sequence._normalize_yaw(reference - self.link.yaw_deg) + delta)
                    self.link.patrol_checkpoint["departure_advance"] = advance
            elif any(self.display.wheel_targets) or _brake_evidence_at(self.link) is None:
                sequence._apply_brake(self.link, self.display, self.deadline)
                self.link._patrol_early_junction_brake = (self.link.session_nonce, self.link.boot_id)
        self.sampler.cancel()
        self.post_turn_sampler.cancel()
        self.stopped_sampler.cancel()
        revoke = getattr(self.display.controller, "revoke_normal_motion", None)
        if callable(revoke) and self.next_action != coverage.TURN_STRAIGHT:
            revoke()
        self._checkpoint()

    def _alignment_tick(self):
        sequence._check_input_exit(self.input_source)
        sequence._remaining(self.deadline)
        self.display.tick()
        samples = self.display.consume_samples(collect_means=False, update_controller=True,
                                               pair_observer=self._sample_pair_guard)
        now = time.monotonic()
        if self.pending_candidate is None:
            candidate = self._detect_junction(samples, now)
            if candidate is not None:
                self._capture_junction(candidate)
        return now, samples

    @staticmethod
    def _laser_pair_centered(sample, now):
        return (sample is not None and sample.dual_wall
                and all(0 <= now - source <= .250 + 1e-9 for source in sample.source_times)
                and abs(sample.left_mm - sample.right_mm) <= PATROL_SIDE_DEADBAND_MM)

    @staticmethod
    def _stopped_pairs_centered(pairs):
        # Each retained pair was independently fresh when verified. Requiring
        # both historical sources to remain fresh at the second acquisition
        # would reject a healthy sensor whose period exceeds 250ms.
        return (len(pairs) >= sequence.POST_STOP_PAIRS and all(pair.dual_wall
                and abs(pair.left_mm - pair.right_mm) <= PATROL_SIDE_DEADBAND_MM
                for pair in pairs))

    def _alignment_stop(self, *, sample_heading=False):
        """BRAKE, strict stop, then two new dual-wall sources; pump services throughout."""
        link, display = self.link, self.display
        self.response.pause(link, time.monotonic())
        sequence._apply_brake(link, display, self.deadline)
        if sample_heading:
            if not self.stopped_sampler.start(display.controller, link._last_brake_sent_at):
                raise mcu.McuError("停车航向采样缺少当前身份或本次BRAKE发送时刻")
            runlog.emit("stage1_stopped_sampling_started", "BRAKE写入即开始固定1秒航向窗，包含制动过程",
                        step=self.step.index, started_at=self.stopped_sampler.started_at,
                        deadline=self.stopped_sampler.deadline,
                        temporary_reference_deg=display.controller.heading_reference_deg)
        evidence = _brake_evidence_at(link)
        if evidence is None:
            raise mcu.McuError("短时矫正停车缺少BRAKE接受与应用证据")
        tracker = sequence._StopTracker(link, source_barrier=evidence)
        link._stop_tracker = tracker
        link._stable_barrier = None
        self._alignment_stop_deadline = min(self.deadline, time.monotonic() + 5.0)
        return self._alignment_stopped_pairs()

    def _alignment_stopped_pairs(self):
        """Keep the same BRAKE window and rebuild stop evidence after any tail motion."""
        link, display = self.link, self.display
        tracker = link._stop_tracker
        previous, verified = None, []
        while True:
            now, samples = self._alignment_tick()
            stable = tracker.update(link, display.state_samples or None,
                                    getattr(display, "trace_samples", []))
            if not stable:
                sequence._remaining(self._alignment_stop_deadline, "短时矫正双轮停稳超过5秒")
                link._stable_barrier = None
                link._ready_side_pairs = []
                display.controller.notify_stopped(now, stable=False)
                previous, verified = None, []
                display.maybe_print()
                time.sleep(sequence.CONTROL_INTERVAL_S)
                continue
            if link._stable_barrier is None:
                link._stable_barrier = now
                display.controller.notify_stopped(now)
                runlog.emit("motion_sequence_stable", "双轮连续500ms整数里程不变，已停稳",
                            stable_barrier=now, brake_evidence_barrier=tracker.source_barrier,
                            wheel_positions_mm=link.odom_mm, step=self.step.index)
            if self.pending_candidate is not None or self._midroad_uturn_reason is not None:
                return []
            barrier = link._stable_barrier
            for sample in samples:
                if (sample.dual_wall and sequence._fresh_post_stop_pair(sample, now, barrier, previous)
                        and sample.source_identity == display.controller._state_session):
                    previous = sample
                    verified.append(sample)
                    verified = verified[-sequence.POST_STOP_PAIRS:]
            if len(verified) >= sequence.POST_STOP_PAIRS:
                controller = display.controller
                controller.retry_pending_pair(now=now)
                poses, latest = controller._poses, controller.latest_source_times
                covered = bool(poses and latest is not None and all(
                    poses[0].at - 1e-9 <= source <= poses[-1].at + 1e-9
                    and latest[index] >= source - 1e-9
                    for pair in verified for index, source in enumerate(pair.source_times)))
                current = sequence._side_sample(link) if link.directional is not None else None
                fresh_current = (current is not None and current.dual_wall
                    and current.source_identity == controller._state_session
                    and all(0 <= now - source <= .250 + 1e-9 for source in current.source_times))
                if covered and fresh_current:
                    link._verified_side_pairs = verified
                    sequence._set_ready_feedback(link, verified, barrier)
                    sequence._clear_side_recovery(link)
                    self.sent_at, self.next_send = None, now
                    return verified
                display.confirmation = "两组停后激光已保存，等待实际STATE覆盖后复用"
            display.maybe_print()
            time.sleep(sequence.CONTROL_INTERVAL_S)

    def _alignment_application_source(self, kind, number, sent_at):
        link = self.link
        key = kind, number
        record = getattr(link, "_command_ledger", {}).get(key)
        state = getattr(link, "_confirmed_commands", {}).get(key)
        if record is None or state is None:
            return None
        witness_key = getattr(link, "_superseded_commands", {}).get(key, key)
        witness = link._command_ledger.get(witness_key, record)
        source = witness[10].get("state_source_at") if len(witness) > 10 else None
        if source is None:
            sample_time = getattr(link, "_sample_time", None)
            if callable(sample_time) and link.state is not None:
                delta = ((link.state.uptime_ms - state.uptime_ms) & 0xFFFFFFFF) / 1000.0
                source = sample_time("state") - delta
        identity = getattr(link, "session_nonce", None), getattr(link, "boot_id", None)
        if (source is None or source < sent_at - 1e-9
                or tuple(record[6:8]) != identity
                or (state.session_nonce, state.boot_id) != identity):
            return None
        return source

    def _sample_stopped_reference(self):
        self.phase = "SAMPLE"
        self.goal = 0
        self._checkpoint()
        while True:
            self.stopped_sampler.cancel()
            pairs = self._alignment_stop(sample_heading=True)
            if self.pending_candidate is not None or self._midroad_uturn_reason is not None:
                return False
            if not self._stopped_pairs_centered(pairs):
                self.stopped_sampler.cancel()
                return False
            while not self.stopped_sampler.completed:
                if not self.stopped_sampler.active:
                    raise mcu.McuError("停车航向采样期间身份、参考或健康状态改变")
                now, samples = self._alignment_tick()
                if self.pending_candidate is not None or self._midroad_uturn_reason is not None:
                    return False
                if not sequence._assert_still_stable(self.link, self.display):
                    pairs = self._alignment_stopped_pairs()
                    if (self.pending_candidate is not None or self._midroad_uturn_reason is not None
                            or not self._stopped_pairs_centered(pairs)):
                        self.stopped_sampler.cancel()
                        return False
                if not self.stopped_sampler.completed:
                    time.sleep(sequence.CONTROL_INTERVAL_S)
            # Drain newly arrived sources and recheck the mechanical stop at
            # the handoff, including movement received at the window deadline.
            while True:
                now, _samples = self._alignment_tick()
                if self.pending_candidate is not None or self._midroad_uturn_reason is not None:
                    return False
                if not sequence._assert_still_stable(self.link, self.display):
                    pairs = self._alignment_stopped_pairs()
                    if (self.pending_candidate is not None or self._midroad_uturn_reason is not None
                            or not self._stopped_pairs_centered(pairs)):
                        self.stopped_sampler.cancel()
                        return False
                    now = time.monotonic()
                identity = getattr(self.link, "session_nonce", None), getattr(self.link, "boot_id", None)
                controller = self.display.controller
                if (controller._state_session != identity
                        or not self.stopped_sampler.reference_matches(controller)):
                    raise mcu.McuError("停车航向样本身份或固定参考已经改变")
                current = sequence._side_sample(self.link) if self.link.directional is not None else None
                fresh = (current is not None and current.dual_wall
                    and current.source_identity == identity
                    and all(0 <= now - source <= .250 + 1e-9 for source in current.source_times))
                if not fresh:
                    self.display.confirmation = "固定1秒采样窗已结束，保持停稳等待当前新鲜双墙核验"
                    self.display.maybe_print()
                    time.sleep(sequence.CONTROL_INTERVAL_S)
                    continue
                if not self._laser_pair_centered(current, now):
                    self.stopped_sampler.cancel()
                    return False
                break
            mean = self.stopped_sampler.mean_deg
            if mean is None:
                runlog.emit("stage1_stopped_sampling_retry", "固定1秒航向窗不足15点或跨度0.8秒，保持停车重采",
                            sample_count=self.stopped_sampler.sample_count,
                            started_at=self.stopped_sampler.started_at,
                            deadline=self.stopped_sampler.deadline, step=self.step.index)
                continue
            controller = self.display.controller
            if not self.stopped_sampler.reference_matches(controller):
                raise mcu.McuError("停车航向样本身份或固定参考已经改变")
            if not controller.adopt_post_turn_reference(mean, now):
                raise mcu.McuError("停车航向均值采纳失败")
            self.display.averages.reset(now, source_barrier=controller.source_barrier)
            self.display._ready_batches.clear()
            self.phase, self.goal = "CRUISE", self.cruise_speed_mmps
            self.reference_fixed, self.post_turn_low_speed = True, False
            self.cruise_transition_pending = True
            controller.request_cruise_speed_transition(self.goal)
            self._checkpoint()
            runlog.emit("stage1_reference_adopted", "停车固定1秒新鲜航向均值已采纳，下一成功目标直接150",
                        step=self.step.index, reference_deg=mean,
                        state_count=self.stopped_sampler.sample_count,
                        started_at=self.stopped_sampler.started_at,
                        deadline=self.stopped_sampler.deadline,
                        elapsed_s=self.stopped_sampler.deadline-self.stopped_sampler.started_at,
                        goal_mmps=self.goal)
            return True

    def _run_short_alignment(self):
        """Continue forward laser correction until two fresh centered sources authorize sampling."""
        self.sampler.cancel()
        self.post_turn_sampler.cancel()
        self.stopped_sampler.cancel()
        original_index = self.index
        controller = self.display.controller
        self.cruise_transition_pending = False
        controller._patrol_pending_cruise_goal = None
        self.phase, self.goal = "ALIGN", POST_TURN_SPEED_MMPS
        first_number = first_sent_at = applied_at = None
        previous, centered_count = None, 0
        # An already stationary entry may prove centering without a gratuitous
        # motion. Moving entries require fresh sources after the first applied
        # low-speed command, retaining the normal application evidence gate.
        stationary_entry = not any(self.display.wheel_targets)
        source_barrier = controller.source_barrier
        source_barrier = -math.inf if source_barrier is None else source_barrier
        self._checkpoint()
        runlog.emit("stage1_short_alignment_started", "临时低速连续前进激光矫正，无单次矫正计时限制",
                    step=self.step.index, mode="FORWARD", goal_mmps=self.goal,
                    temporary_reference_deg=controller.heading_reference_deg)
        while self.index == original_index and self.post_turn_low_speed:
            now, samples = self._alignment_tick()
            if self.pending_candidate is not None or self._midroad_uturn_reason is not None:
                return False
            if getattr(self.link, "side_recovery_active", False):
                self._recover()
                if self.pending_candidate is not None or self.index != original_index:
                    return False
                self.phase, self.goal = "ALIGN", POST_TURN_SPEED_MMPS
                first_number = first_sent_at = applied_at = None
                previous, centered_count = None, 0
                stationary_entry = True
                source_barrier = self.link._stable_barrier
                continue
            if self._alignment_heading_recovery():
                return False
            if first_number is not None and applied_at is None:
                applied_at = self._alignment_application_source(sequence.MessageType.CONTROL_TARGET,
                                                               first_number, first_sent_at)
                if applied_at is not None:
                    runlog.emit("stage1_short_alignment_applied", "首个低速矫正目标已接受并应用",
                                step=self.step.index, mode="FORWARD", started_at=applied_at)
            barrier = applied_at if applied_at is not None else source_barrier
            if applied_at is not None or stationary_entry:
                poses, latest = controller._poses, controller.latest_source_times
                for sample in samples:
                    covered = bool(poses and latest is not None and all(
                        poses[0].at - 1e-9 <= source <= poses[-1].at + 1e-9
                        and latest[index] >= source - 1e-9
                        for index, source in enumerate(sample.source_times)))
                    if (covered and sample.source_identity == controller._state_session
                            and sequence._fresh_post_stop_pair(sample, now, barrier, previous)):
                        previous = sample
                        centered_count = centered_count + 1 if (self._laser_pair_centered(sample, now)
                            and abs(controller._error_rate) <= .5 + 1e-9) else 0
            if centered_count >= sequence.POST_STOP_PAIRS:
                if self._sample_stopped_reference():
                    return True
                if self.pending_candidate is not None or self._midroad_uturn_reason is not None:
                    return False
                # The one formal BRAKE already proved a strict stop; resume
                # forward directly when post-stop lasers invalidate centering.
                self.phase, self.goal = "ALIGN", POST_TURN_SPEED_MMPS
                first_number = first_sent_at = applied_at = None
                previous, centered_count = None, 0
                stationary_entry = True
                source_barrier = self.link._stable_barrier
                self.sent_at, self.next_send = None, time.monotonic()
                self._checkpoint()
                runlog.emit("stage1_short_alignment_retry", "停车后激光失标，恢复临时低速前进矫正",
                            step=self.step.index, mode="FORWARD")
                continue
            executing, confirmed_at, applied = sequence._confirmed_wheel_response(
                self.link, self.sent_at, self.display.wheel_targets)
            self.response.update(self.link, executing, now, confirmed_at,
                                 self.display.state_samples, applied_state=applied)
            pending = getattr(self.link, "motion_target_pending", False)
            pending = pending() if callable(pending) else pending
            current = sequence._side_sample(self.link) if self.link.directional is not None else None
            ready = (current is not None and current.dual_wall
                     and current.source_identity == controller._state_session
                     and all(0 <= now - source <= .250 + 1e-9 for source in current.source_times)
                     and controller._patrol_alignment_ready(now))
            hold = (not ready and current is not None
                    and current.source_identity == controller._state_session
                    and all(status in (sequence.WireRangeStatus.VALID,
                                       sequence.WireRangeStatus.STALE,
                                       sequence.WireRangeStatus.TIMEOUT)
                            for status in (current.left_status, current.right_status))
                    and controller._patrol_alignment_hold_ready(now))
            if any(self.display.wheel_targets) and (not ready and not hold or pending and
                    not controller._patrol_alignment_angle_safe(self.display.command[1], now)):
                self._alignment_stop()
                if self.pending_candidate is not None or self._midroad_uturn_reason is not None:
                    return False
                first_number = first_sent_at = applied_at = None
                previous, centered_count = None, 0
                stationary_entry = True
                source_barrier = self.link._stable_barrier
                self.sent_at, self.next_send = None, time.monotonic()
                continue
            # A centered stationary entry waits only for its second independent
            # laser source; an uncentered entry starts forward immediately.
            observe_stationary = (stationary_entry and self._laser_pair_centered(current, now))
            if (ready or hold) and not observe_stationary and not pending and now + 1e-9 >= self.next_send:
                sign = 1 if not ready or current.left_mm >= current.right_mm else -1
                target = (controller.target_patrol_alignment("FORWARD", sign, now) if ready
                          else controller.target_patrol_alignment_cached(now))
                if any(target.wheel_targets):
                    if _patrol_zero_transition(self.display.wheel_targets, target.wheel_targets):
                        self._alignment_stop()
                        first_number = first_sent_at = applied_at = None
                        previous, centered_count = None, 0
                        stationary_entry = True
                        source_barrier = self.link._stable_barrier
                        continue
                    try:
                        self.display.centering_target = target
                        number = _send_patrol_feedback_target(self.link, self.display,
                            target.linear_mmps, target.angular_mradps, self.deadline, target=target)
                    except sequence._CenteringProposalExpired:
                        time.sleep(sequence.CONTROL_INTERVAL_S)
                        continue
                    except sequence._SideRecoveryRequired:
                        self._recover()
                        first_number = first_sent_at = applied_at = None
                        previous, centered_count = None, 0
                        stationary_entry = True
                        source_barrier = self.link._stable_barrier
                        continue
                    self.sent_at = time.monotonic()
                    controller.confirm_target_sent(target)
                    stationary_entry = False
                    if first_number is None:
                        first_number = number
                        first_sent_at = self.link._command_ledger[(sequence.MessageType.CONTROL_TARGET, number)][0]
                    self.next_send = self.sent_at + sequence.COMMAND_INTERVAL_S
            self.display.input_speed_mmps = POST_TURN_SPEED_MMPS
            self.display.control_mode = ("合法短缺测保持最近成功前进目标；原始激光源有效期500ms"
                                         if hold else "临时低速前进居中；两组3mm达标后BRAKE并固定1秒采样")
            self.display.action = (f"激光连续前进矫正；命令均速{self.display.command[0]}mm/s；"
                                   f"激光两新周期{centered_count}/2")
            self.display.maybe_print()
            time.sleep(sequence.CONTROL_INTERVAL_S)
        return False

    def _alignment_heading_recovery(self):
        error = self.display.controller._heading_error()
        if error is None or abs(error) <= 45.0 + 1e-9:
            return False
        self.stopped_sampler.cancel()
        if self.pending_candidate is None and self._midroad_uturn_reason is None:
            self.post_turn_low_speed, self.reference_fixed = False, False
            self.phase, self.goal = "HEADING_RECOVERY", POST_TURN_SPEED_MMPS
            self._checkpoint()
        return True

    def _position_at(self, at):
        if self.phase != "HOME" and self._coordinate_history:
            history = self._coordinate_history
            if not history[0][0] - 1e-9 <= at <= history[-1][0] + 1e-9:
                return None
            previous = history[0]
            for current in history:
                if current[0] + 1e-9 >= at:
                    fraction = (at - previous[0]) / (current[0] - previous[0]) if current[0] > previous[0] else 0.
                    return previous[1] + fraction * (current[1] - previous[1])
                previous = current
            return None
        history = self.progress._history
        origin = self.progress.origin_mm
        if origin is None or not history or not history[0][0] - 1e-9 <= at <= history[-1][0] + 1e-9:
            return None
        previous = history[0]
        for current in history:
            if current[0] + 1e-9 >= at:
                fraction = ((at - previous[0]) / (current[0] - previous[0])
                            if current[0] > previous[0] else 0.)
                positions = tuple(a + fraction * (b - a) for a, b in zip(previous[1], current[1]))
                return sum(position - start for position, start in zip(positions, origin)) / 2.
            previous = current
        return None

    def _junction_distance_at(self, at):
        position = self._position_at(at)
        contract = getattr(self, "_retreat_contract", None) if self._midroad_returning else None
        return (contract["distance_mm"] - position
                if contract is not None and position is not None else position)

    def _detect_junction(self, samples, now):
        if not self.armed:
            return None
        contract = getattr(self, "_retreat_contract", None) if self._midroad_returning else None
        if contract is not None and contract["short_node_return"]:
            return None
        identity = getattr(self.link, 'session_nonce', None), getattr(self.link, 'boot_id', None)
        if self.detector.identity != identity:
            self.detector.reset(self.detector.minimum, self.detector.maximum, identity,
                                self.detector.require_window)
        # Real openings need no odometry. Process the entire batch before the
        # STATE-aligned missing queue, so a delayed pose cannot hide an opening.
        for sample in samples:
            if sample.source_identity != identity:
                continue
            at = sample.received_at - sample.receive_delay_ms / 1000.
            evidence = self.detector.observe_explicit(sample, at, self._junction_distance_at(at), now=now,
                                                     source_barrier=self.first_wall_source)
            if evidence is not None:
                self.junction_evidence = evidence
                return sample
        if self._midroad_returning and contract is None:
            # Encoder zero and missing readings cannot confirm the entry node.
            return None
        for sample in samples:
            if sample.source_identity != identity:
                continue
            if len(self._localization_samples) == self._localization_samples.maxlen:
                raise mcu.McuError('路口定位快照队列溢出，停止无依据的非零续约')
            self._localization_samples.append(sample)
        while self._localization_samples:
            sample = self._localization_samples[0]
            if sample.source_identity != identity:
                self._localization_samples.popleft()
                continue
            at = sample.received_at - sample.receive_delay_ms / 1000.
            if now - at > .250 + 1e-9:
                self._localization_samples.popleft()
                continue
            if not self.progress._history or at > self.progress._history[-1][0] + 1e-9:
                break
            self._localization_samples.popleft()
            if contract is not None and at <= self.first_wall_source + 1e-9:
                continue
            distance = self._junction_distance_at(at)
            if distance is None:
                continue
            fresh_walls = (sample.dual_wall and all(0 <= now - source <= .250 + 1e-9
                and self._position_at(source) is not None for source in sample.source_times))
            valid_seen = self.detector.identity == identity and self.detector._valid_seen
            evidence = self.detector.observe(sample, at, distance,
                                             valid_seen or fresh_walls, now=now)
            if evidence is not None:
                self.junction_evidence = evidence
                return sample
        return None

    def _begin_passage(self, barrier, *, after_turn=False, moving_opening=None,
                       stopped_opening=None, saved_passage=False):
        if self.phase == "SAMPLE":
            self.sampler.cancel()
            self.phase = "ALIGN"
        self.armed = False
        self.passage_barrier = barrier
        self.pending_wall = self.first_wall_source = None
        self.first_wall_sources = None
        self.link.side_candidates_enabled = False
        stopped = (stopped_opening is not None or saved_passage
                   and getattr(self.link, "_stable_barrier", None) is not None
                   and not any(self.display.command))
        begin = getattr(self.link, "begin_gyro_passage", None) if (moving_opening is not None
                     or saved_passage and not stopped) else None
        if callable(begin):
            self.passage_barrier = barrier = begin(moving_opening, speed_mmps=self.goal)
        elif stopped and callable(getattr(self.link, "begin_stopped_gyro_passage", None)):
            self.passage_barrier = barrier = self.link.begin_stopped_gyro_passage(
                stopped_opening, speed_mmps=self.goal)
        elif after_turn and callable(getattr(self.link, "begin_gyro_departure", None)):
            begin = self.link.begin_gyro_departure
            begin(barrier, speed_mmps=self.goal)
        else:
            begin = getattr(self.link, "begin_departure", None)
            if callable(begin):
                begin(barrier)
        self.display.reset_averages(source_barrier=barrier, reset_controller=False)
        self._clear_window_evidence()

    def _clear_window_evidence(self):
        controller = self.display.controller
        controller._good_windows = 0
        controller._pending_means.clear()
        controller._mean_last_end = None
        controller._mean_source_keys = (None, None)

    def _sample_pair_guard(self, sample):
        now = time.monotonic()
        if (self.armed and self.phase not in ("START", "HOME")
                and (not sample.dual_wall or any(
                    not 0 <= now - source <= sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                    for source in sample.source_times)) and self._laser_resume_token is None):
            controller = self.display.controller
            self._laser_resume_token = (controller._state_session, controller._patrol_feedback_generation,
                                        controller.observation_token)
        if self.sampler.active and not self.display.controller.centered:
            self.sampler.cancel()

    def _arm_on_walls(self, samples, now):
        controller = self.display.controller
        identity = getattr(self.link, "session_nonce", None), getattr(self.link, "boot_id", None)
        contract = getattr(self, "_retreat_contract", None) if self._midroad_returning else None
        if contract is not None and contract["short_node_return"]:
            return
        def fresh(sample):
            return (sample.dual_wall
                    and None not in identity and sample.source_identity == identity
                    and all(self.passage_barrier is None or source > self.passage_barrier + 1e-9
                            for source in sample.source_times)
                    and all(0 <= now - source <= sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                            for source in sample.source_times))
        if self.pending_wall is not None and not fresh(self.pending_wall):
            self.pending_wall = None
        for sample in samples:
            if not fresh(sample):
                continue
            controller.observe_side(sample)
            if self.pending_wall is None:
                self.pending_wall = sample
        if self.pending_wall is None:
            return
        controller.retry_pending_pair(now=now)
        source = max(self.pending_wall.source_times)
        poses = controller._poses
        latest = controller.latest_source_times
        if (not poses or latest is None or not all(
                poses[0].at - 1e-9 <= at <= poses[-1].at + 1e-9 and latest[index] >= at - 1e-9
                for index, at in enumerate(self.pending_wall.source_times))
                or not self.progress.establish(source)):
            return
        self.armed, self.first_wall_source = True, source
        if self.wall_offset_mm is None:
            self.wall_offset_mm = self._position_at(source)
        self.first_wall_sources = self.pending_wall.source_times
        self.display.origin = self.progress.origin_mm
        self.display.segment_distance_mm = self.progress.position_mm
        if self.ledger.current_progress is not None:
            self.ledger.current_progress.entry_odom_mm = self.progress.origin_mm
        self.detector._valid_seen = True
        finish = getattr(self.link, "finish_departure", None)
        if callable(finish):
            finish()
        self.link.side_candidates_enabled = True
        self.display.reset_averages(source_barrier=self.passage_barrier, reset_controller=False)
        self._clear_window_evidence()
        if self.phase == "DEPARTURE":
            self.phase, self.goal = "CRUISE", (min(self.cruise_speed_mmps, self.special_speed_mmps) if self._n06_slow_active else self.cruise_speed_mmps)
            self.display.controller.request_cruise_speed_transition(self.goal)
        self.pending_wall = None
        self._checkpoint()
        runlog.emit("stage1_walls_armed", "真正新双墙恢复首段航向控制" if self.phase == "START"
                    else "首组新双墙建立返家计程，启用61目标激光居中" if self.phase == "HOME"
                                        else "真正新双墙恢复原激光纠偏",
                    step=self.step.index, phase=self.phase, source_time=source,
                    source_barrier=self.passage_barrier, goal_mmps=self.goal,
                    wheel_origin_mm=self.progress.origin_mm,
                    fixed_reference_deg=controller.heading_reference_deg)

    def _arrival(self, reason="junction"):
        distance = (self.progress.position_mm if self.phase == "HOME" else
                    self.road_position_mm if self.road_axis_deg is not None
                    and self._coordinate_baseline is not None else None)
        return mission_ledger.DriveResult(reason, distance, distance, reason == "junction",
                                          self.link.yaw_deg, distance is not None)

    def _confirm_road(self):
        distance = self._commit_road_arrival()
        if distance is not None:
            self._finish_road_arrival(distance)

    def _commit_road_arrival(self):
        """Pure semantic commit; competition serializes this with stop facts."""
        records = getattr(self.ledger, 'stage1_junction_records', None)
        if records is None:
            records = self.ledger.stage1_junction_records = []
        if any(record['step'] == self.step.index for record in records):
            return None
        if self.junction_evidence is None:
            raise mcu.McuError('缺少本次道路窗口内的路口证据')
        if self.junction_evidence.kind == 'explicit_opening':
            self.ledger.confirm_arrival(self.step, self._arrival())
        else:
            # Shared arrival contract requires a raw side opening.  The local
            # inferred event is recorded separately, retaining its real kind.
            self.ledger.completed_roads.add(self.step.road_id)
            self.ledger.confirmed_nodes.add(self.step.to_node)
            self.ledger.last_confirmed_node = self.step.to_node
            self.ledger.position_confidence = mission_ledger.POSITION_CONFIRMED_NODE
            if self.ledger.current_progress is not None:
                self.ledger.current_progress.observe(self._arrival('missing_inference'))
        records.append(dict(step=self.step.index, road=self.step.road_id,
                            **asdict(self.junction_evidence)))
        distance = self.road_position_mm
        # A missing odometry value is valid for explicit opening evidence. The
        # tuple separates that value from the already-committed sentinel.
        return (distance,)

    def _finish_road_arrival(self, committed):
        distance, = committed
        expected = (300.0 if self.step.road_id in ('r02', 'r03') else
                    self.field_map.roads[self.step.road_id].length_m * 1000.0
                    if self.index == 0 else 800.0)
        if distance is not None and abs(distance - expected) > max(150.0, .2 * expected):
            runlog.emit("stage1_odom_discrepancy", "编码器仅告警；按真实开口顺序确认路口",
                        step=self.step.index, road=self.step.road_id, distance_mm=distance,
                        nominal_mm=expected)
        runlog.emit("stage1_step_complete", "路口证据确认道路及节点",
                    step=self.step.index, road=self.step.road_id, node=self.step.to_node,
                    odom_distance_mm=distance, next_action=self.next_action,
                    evidence_kind=self.junction_evidence.kind,
                    evidence=asdict(self.junction_evidence))
        self.progress = sequence._RoadProgress()
        self.display.origin = self.display.segment_distance_mm = None
        self._localization_samples.clear()
        self.detector._episodes = [None, None]
        self._checkpoint()
        runlog.emit("stage1_road_origin_cleared", "道路退出证据已保存，清空活动计程；保留已保存路口动作",
                    step=self.step.index, road=self.step.road_id, exit_distance_mm=distance)

    def _measure_brake(self, origin, speed):
        tail = max(0.0, sum(a - b for a, b in zip(self.link.odom_mm, origin)) / 2.0)
        self.display.braking_tail_mm = tail
        if speed > 0:
            self.brake_delay_s = max(self.brake_delay_s, tail / speed)
        return tail

    def _junction(self, candidate):
        if self._midroad_returning:
            self._finish_midroad_return()
            return
        if self.next_action != coverage.TURN_STRAIGHT and any(self.display.wheel_targets):
            self._capture_junction(candidate)
        self._midroad_uturn_reason = None
        self.sampler.cancel()
        self.post_turn_sampler.cancel()
        self._post_turn_pending_states.clear()
        self.post_turn_low_speed = False
        self.cruise_transition_pending = False
        self.display.controller._patrol_pending_cruise_goal = None
        action = self.next_action
        side = ("left" if action in (coverage.TURN_LEFT, coverage.TURN_UTURN)
                else "right" if action == coverage.TURN_RIGHT else None)
        checkpoint = getattr(self.link, 'store_checkpoint', None)
        if callable(checkpoint):
            checkpoint(junction_evidence=asdict(self.junction_evidence),
                       saved_openings=self.junction_evidence.openings, planned_side=side,
                       planned_angle_deg=180 if action == coverage.TURN_UTURN else 90 if side else 0)
        runlog.emit("stage1_route_decision", "激光识别路口，闭合路线决定动作",
                    seen_openings=sorted(self.junction_evidence.openings) if candidate is None
                        else sorted(candidate.candidates),
                    saved_openings=sorted(sequence._pending_junction_openings(self.link)),
                    phase=self.phase, goal_mmps=self.goal,
                    source_keys=None if candidate is None else candidate.source_keys,
                    fixed_reference_deg=self.display.controller.heading_reference_deg,
                    **getattr(self.link, "patrol_route_context", {}))
        if action == coverage.TURN_STRAIGHT:
            passage_evidence = asdict(self.junction_evidence)
            inherited_reference = self.display.controller.formal_reference_deg
            self.goal = self._straight_passage_speed_goal(
                self.home_speed_mmps if self.index + 1 == len(self.steps) - 1 else self.cruise_speed_mmps)
            self.alignment_goal = self.start_speed_mmps
            stable_barrier = getattr(self.link, "_stable_barrier", None)
            if stable_barrier is not None and not any(self.display.command):
                # Recovery already established a strict stop.  It uses that
                # stopped authorization, never the moving-passage exemption.
                self._begin_passage(passage_evidence['confirmed_at'], stopped_opening=candidate,
                                    saved_passage=candidate is None)
            else:
                self._begin_passage(passage_evidence['confirmed_at'], moving_opening=candidate,
                                    saved_passage=candidate is None)
            # Commit the route transition only after passage authorization;
            # a transport fault can retry this saved decision on the same step.
            self._confirm_road()
            if inherited_reference is not None:
                controller = self.display.controller
                controller._road_reference = controller._hold_heading = inherited_reference
                controller._patrol_cruise_hold_goal = inherited_reference
                controller._patrol_cruise_hold_sources = None
            self.index += 1
            self.phase = "HOME" if self.index == len(self.steps) - 1 else "DEPARTURE"
            self._set_step()
            runlog.emit("stage1_straight_passage", "直穿临时继承旧参考；新双墙后按本道路规则重新采样",
                        step=self.step.index, goal_mmps=self.goal,
                        phase=self.phase, entry_sources=None if candidate is None else candidate.source_keys,
                        source_barrier=self.passage_barrier,
                        fixed_reference_deg=self.display.controller.heading_reference_deg)
            return
        if action not in (coverage.TURN_LEFT, coverage.TURN_RIGHT, coverage.TURN_UTURN):
            raise mcu.McuError("路线产生未授权转向或末段侧路口动作")
        self.response.pause(self.link, time.monotonic())
        origin, speed = self.link.odom_mm, max(0.0, float(self.display.command[0]))
        side = "left" if action in (coverage.TURN_LEFT, coverage.TURN_UTURN) else "right"
        self.alignment_goal = POST_TURN_SPEED_MMPS
        departure_speed_mmps = (self.home_speed_mmps if self.index + 1 == len(self.steps) - 1
                                else self.cruise_speed_mmps)
        try:
            self._advance_departure_junction()
            openings = _confirm_junction(self.link, self.display, candidate,
                                         self.deadline, input_source=self.input_source)
            self._measure_brake(origin, speed)
            self._confirm_road()
            pivot = self._departure_turn_pending() and self.departure_strategy == "40mm_pivot"
            self._coordinate_mode = "PIVOT" if action == coverage.TURN_UTURN or pivot else "TRANSIT"
            _turn_planned(self.link, self.display, openings, self.deadline,
                          turn_speed_mmps=self.turn_speed_mmps,
                          side=side, departure_speed_mmps=departure_speed_mmps,
                          input_source=self.input_source,
                          angle_deg=180 if action == coverage.TURN_UTURN else 90,
                          goal_reference_deg=self._short_road_reference(self.steps[self.index + 1]),
                          pivot=pivot)
        except mcu.McuError as exc:
            runlog.emit("stage1_junction_aborted", "地图转弯路口执行中止",
                        requested_side=side, reason=str(exc),
                        saved_openings=sorted(sequence._pending_junction_openings(self.link)),
                        stable_barrier=getattr(self.link, "_stable_barrier", None),
                        brake_sequence=getattr(self.link, "_last_brake_sequence", None),
                        brake_evidence_at=_brake_evidence_at(self.link),
                        **getattr(self.link, "patrol_route_context", {}))
            raise
        self.first_actual_turn_completed = True
        if action in (coverage.TURN_LEFT, coverage.TURN_RIGHT):
            self.departure_normal_turn_completed = True
        self.index += 1
        self.phase = "HOME" if self.index == len(self.steps) - 1 else "DEPARTURE"
        self.goal = self.home_speed_mmps if self.phase == 'HOME' else departure_speed_mmps
        self._set_step(after_turn=True)
        self._begin_passage(self.link.post_turn_barrier, after_turn=True)
        self.sent_at, self.next_send = None, time.monotonic()
        self.response.pause(self.link, self.next_send)

    def _recover(self):
        """Reference stopped recovery, with straight openings delegated to navigation."""
        link, display = self.link, self.display
        finish = getattr(link, "finish_departure", None)
        if callable(finish):
            finish()
        self.sampler.cancel()
        self.response.pause(link, time.monotonic())
        origin, speed = link.odom_mm, max(0.0, float(display.command[0]))
        if any(display.command) or getattr(link, "_stop_tracker", None) is None:
            _brake(link, display, self.deadline, input_source=self.input_source)
        else:
            _ensure_stable(link, display, self.deadline, input_source=self.input_source)
        self._measure_brake(origin, speed)
        barrier = link._stable_barrier
        display.reset_averages(source_barrier=barrier, reset_controller=False)
        previous, verified = None, []
        while True:
            sequence._check_input_exit(self.input_source)
            sequence._remaining(self.deadline)
            display.tick()
            samples = display.consume_samples(collect_means=False, update_controller=True)
            if not sequence._assert_still_stable(link, display):
                barrier = _wait_stable(link, display, self.deadline,
                                       input_source=self.input_source)
                display.controller.source_barrier = barrier
                previous, verified = None, []
                continue
            now = time.monotonic()
            candidate = self._detect_junction(samples, now)
            if self.phase != "HOME" and candidate is not None:
                self.pending_candidate = candidate
                sequence._clear_side_recovery(link)
                self._junction(candidate)
                return
            for sample in samples:
                if sequence._fresh_post_stop_pair(sample, now, barrier, previous):
                    previous = sample
                    verified.append(sample)
                    verified = verified[-sequence.POST_STOP_PAIRS:]
            covered = False
            if len(verified) >= sequence.POST_STOP_PAIRS:
                controller = display.controller
                controller.retry_pending_pair(now=now)
                poses, latest = controller._poses, controller.latest_source_times
                covered = bool(poses and all(
                    poses[0].at - 1e-9 <= source <= poses[-1].at + 1e-9
                    and (not pair.dual_wall or latest is not None and latest[index] >= source - 1e-9)
                    for pair in verified for index, source in enumerate(pair.source_times)))
            if covered:
                link._verified_side_pairs = verified
                sequence._set_ready_feedback(link, verified, barrier)
                sequence._clear_side_recovery(link)
                controller.begin_alignment(now, source_barrier=barrier)
                if self.phase in ("ALIGN", "SAMPLE") and not self.post_turn_sampler.active:
                    self.phase = "ALIGN"
                if self.phase == "HOME" and not verified[-1].dual_wall:
                    self._begin_passage(barrier)
                if not self.armed:
                    self._begin_passage(barrier)
                    self._arm_on_walls(verified, now)
                self.sent_at, self.next_send = None, now
                self.response.pause(link, now)
                runlog.emit("stage1_laser_recovered", "严格停稳后复用两组健康新反馈",
                            step=self.step.index, source_barrier=barrier, phase=self.phase)
                return
            display.action = "侧激光停车恢复"
            display.maybe_print()
            time.sleep(sequence.CONTROL_INTERVAL_S)

    def home_brake_reserve_mm(self, now):
        controller = self.display.controller
        source = controller._state_at
        age = max(0.0, now - source) if source is not None else config.TELEMETRY_STALE_MS / 1000.0
        speed = max(self.home_speed_mmps, float(self.display.command[0]),
                    controller.actual_speed_mmps or 0.0)
        # A single confirmation allowance plus measured/fallback physical tail.
        return speed * (age + sequence.CONTROL_INTERVAL_S
                        + config.MCU_COMMAND_CONFIRM_TIMEOUT_S + self.brake_delay_s)

    def _finish_home(self):
        home_speed_mmps = getattr(self, "home_speed_mmps", HOME_SPEED_MMPS)
        self.sampler.cancel()
        self.response.pause(self.link, time.monotonic())
        before = self.link.odom_mm
        sequence._brake(self.link, self.display, self.deadline, input_source=self.input_source)
        self._measure_brake(before, home_speed_mmps)
        result = self._arrival("home")
        if not self.display.controller.heading_ready(tolerance_deg=5.0):
            raise mcu.McuError("回出发区已停稳，但相对返回道路参考的航向未达标")
        self.ledger.confirm_home_arrival(self.step, result)
        self.home_result = result
        self.ledger.route_plan_finished = True
        self.ledger.stage1_home_stop_confirmed = True
        runlog.emit("stage1_home_confirmed", "返回n01并严格停稳，包含制动尾段",
                    target_mm=HOME_TRAVEL_MM, stopped_distance_mm=result.odom_distance_mm,
                    target_speed_mmps=home_speed_mmps, wheel_origin_mm=self.progress.origin_mm,
                    origin_source_at=self.progress.origin_source_at,
                    error_mm=result.odom_distance_mm - HOME_TRAVEL_MM,
                    brake_tail_mm=self.display.braking_tail_mm,
                    wheel_positions_mm=self.link.odom_mm, yaw_deg=result.yaw_deg,
                    completed_roads=sorted(self.ledger.completed_roads),
                    confirmed_nodes=sorted(self.ledger.confirmed_nodes))

    def _run_until_fault(self):
        link, display, controller = self.link, self.display, self.display.controller
        control_interval_s = getattr(self, "control_interval_s", sequence.CONTROL_INTERVAL_S)
        link.side_motion_active = link.side_age_required = link.side_candidates_enabled = True
        display._distance_observer = self._observe_distance
        if not self._initialized:
            self._set_step()
            self._initialized = True
        try:
            while True:
                sequence._check_input_exit(self.input_source)
                sequence._remaining(self.deadline)
                display.tick()
                if self._midroad_return_wait:
                    display.consume_samples(collect_means=False, update_controller=False)
                    _ensure_stable(link, display, self.deadline, input_source=self.input_source)
                    display.action = "已返回原道路入口，严格停稳等待人工"
                    display.maybe_print()
                    time.sleep(control_interval_s)
                    continue
                if (self.pending_candidate is not None
                        or getattr(self, "pending_junction_decision", None) is not None):
                    self._junction(self.pending_candidate)
                    continue
                samples = display.consume_samples(collect_means=self.armed and self.phase != "START",
                                                  update_controller=self.armed,
                                                  pair_observer=self._sample_pair_guard)
                now = time.monotonic()
                was_armed = self.armed
                if not self.armed:
                    self._arm_on_walls(samples, now)
                candidate = self._detect_junction(samples, now)
                if self.phase != "HOME" and candidate is not None:
                    if self._midroad_returning:
                        self._finish_midroad_return()
                        continue
                    self._capture_junction(candidate)
                    self._junction(candidate)
                    continue
                if self._midroad_uturn_reason is not None:
                    self._execute_midroad_uturn()
                    continue
                if (self.armed and self.phase != 'HOME' and self.progress.position_mm is not None
                        and not (self._midroad_returning and self._retreat_contract is not None
                                 and not self._retreat_contract.get("short_node_return")
                                 and self.step.from_node != self.field_map.start_node_id)
                        and self.progress.position_mm > self.detector.maximum and not self._odom_warning):
                    self._odom_warning = True
                    runlog.emit('stage1_localization_warning', '已越预期路口窗口但无候选，保持参考行驶等待有效证据',
                                step=self.step.index, position_mm=self.progress.position_mm)
                    display.emitter('定位异常：已越路口窗口，保持有效参考等待路口证据')
                if self.phase == "HOME" and candidate is not None:
                    self._begin_passage(max(candidate.source_times))
                if getattr(link, "side_recovery_active", False):
                    self._recover()
                    continue
                self._update_initial_launch(now)
                self._update_n06_passage(now)
                if self.phase != "HOME" and self.armed and self.first_actual_turn_completed:
                    retreat = getattr(self, "_retreat_contract", None) if self._midroad_returning else None
                    heading_error = controller._heading_error()
                    if (self.phase == "HEADING_RECOVERY" and heading_error is not None
                            and abs(heading_error) <= PATROL_RECOVERY_EXIT_DEG + 1e-9):
                        self.post_turn_low_speed, self.reference_fixed = False, True
                        self.phase, self.goal = (("RETURN_ROAD", retreat["road_goal_mmps"])
                            if retreat is not None else ("CRUISE", self.cruise_speed_mmps))
                        self.cruise_transition_pending = True
                        self._checkpoint()
                    if heading_error is not None and abs(heading_error) > controller.patrol_heading_limit_deg + 1e-9:
                        if retreat is not None:
                            retreat.setdefault("road_goal_mmps", self.goal)
                        self.post_turn_low_speed, self.reference_fixed = False, True
                        self.phase, self.goal = "HEADING_RECOVERY", (retreat["road_goal_mmps"]
                            if retreat is not None else POST_TURN_SPEED_MMPS
                            if controller._patrol_road_policy == "SHORT" else self.cruise_speed_mmps)
                        controller._patrol_heading_recovery = True
                    self._learn_road_heading(now, soft_missing=bool(getattr(link, "side_cached_active", False)))
                executing, confirmed_at, applied = sequence._confirmed_wheel_response(
                    link, self.sent_at, display.wheel_targets)
                self.response.update(link, executing, now, confirmed_at,
                                     display.state_samples, applied_state=applied)
                if self.phase == "HOME" and self.progress.position_mm is not None:
                    reserve = self.home_brake_reserve_mm(now)
                    if self.progress.position_mm + reserve >= HOME_TRAVEL_MM:
                        self._finish_home()
                        return self.home_result
                cached = bool(getattr(link, "side_cached_active", False))
                target_goal = self._target_speed_goal()
                start_heading = self._use_start_heading_control(now)
                display.input_speed_mmps = target_goal
                display.action = (f"道路{self.step.road_id} {self.step.from_node}→{self.step.to_node}；"
                                  f"{self.phase}；目标{target_goal}；下一动作{self.next_action}")
                pending = getattr(link, "motion_target_pending", False)
                pending = pending() if callable(pending) else pending
                current = sequence._side_sample(link) if link.directional is not None else None
                ordinary = self.phase != "HOME" and not start_heading
                laser_reason = None
                if ordinary and self.armed and not getattr(link, "patrol_gyro_only_active", False):
                    sources = controller.latest_source_times
                    if current is None or not current.dual_wall:
                        laser_reason = "当前缺少双墙反馈"
                    elif any(not 0 <= now - source <= sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                             for source in current.source_times):
                        laser_reason = "当前双墙真实源时间已过期"
                    elif (not controller.control_available(now) or not controller._usable_pair
                          or sources is None or any(
                              not 0 <= now - source <= sequence.LASER_MEASUREMENT_MAX_AGE_MS / 1000.0 + 1e-9
                              for source in sources)):
                        laser_reason = "等待新鲜双墙与STATE对齐"
                    if laser_reason is not None:
                        if self._laser_resume_token is None:
                            self._laser_resume_token = (controller._state_session, controller._patrol_feedback_generation,
                                                        controller.observation_token)
                    elif self._laser_resume_token is not None:
                        identity, generation, token = self._laser_resume_token
                        if ((controller._state_session, controller._patrol_feedback_generation) == (identity, generation)
                                and controller.observation_token <= token):
                            laser_reason = "等待新的已对齐双墙周期"
                        else:
                            self._laser_resume_token = None
                gyro_only = (getattr(link, "patrol_gyro_only_active", False)
                             or current is None or not current.dual_wall
                             or not controller.control_available(now) or laser_reason is not None)
                # A varying gyro-only heading target needs the same original
                # ACK/applied protection as a varying laser steering target.
                holding_pending = bool(pending and any(display.wheel_targets) and not cached
                                       and not (self._n06_slow_active and display.command[0] > target_goal))
                display.control_mode = ("首段仅航向控制，左右居中关闭" if start_heading
                                        else f"返家航向控制{self.home_speed_mmps}离口；首组新双墙起计160 mm含制动尾段，再启用居中"
                                        if self.phase == "HOME" and not self.armed
                                        else f"返家{self.home_speed_mmps}目标：首组新双墙后激光居中；160 mm含制动尾段停车优先"
                                        if self.phase == "HOME"
                                        else f"下一道路待确认；直行穿口：陀螺仪保持原参考与目标{self.goal}，等待首组真正新双墙"
                                        if getattr(link, "patrol_gyro_passage_active", False)
                                        else f"下一道路待确认；转后陀螺仪{self.goal}离口，等待首组真正新双墙"
                                        if getattr(link, "patrol_gyro_departure_active", False)
                                        else f"下一道路待确认；目标{self.goal}离口，等待真正新双墙" if not self.armed
                                        else "差速优先航向恢复至±5°，然后按3mm激光居中"
                                        if self.phase == "HEADING_RECOVERY"
                                        else f"n06后前50mm：均速{min(self.cruise_speed_mmps, self.special_speed_mmps)}、激光纠偏{controller.PATROL_LASER_YAW_GAIN_PER_S:g}、暂停参考采样"
                                        if self._n06_slow_active
                                        else f"实际均速{self.cruise_speed_mmps}±3%；本道路合格参考±45°内激光优先")
                if ordinary and self.armed and not getattr(link, "patrol_gyro_only_active", False):
                    display.control_mode = (f"激光缺测，仅保持航向；{laser_reason}" if gyro_only else
                                            "命令待确认；保持上一已发送目标" if holding_pending else
                                            f"激光联合纠偏；{display.control_mode}")
                if now + 1e-9 < self.next_send:
                    display.maybe_print()
                    time.sleep(control_interval_s)
                    continue
                selection = ("HEADING" if start_heading or not self.armed or gyro_only else
                             "PENDING" if holding_pending else "LASER", laser_reason)
                if selection != self._control_selection:
                    runlog.emit("stage1_control_mode", "阶段一控制分支切换",
                                step=self.step.index, phase=self.phase, mode=selection[0],
                                reason=laser_reason or {
                                    "PENDING": "上一非零目标等待ACK及应用STATE确认",
                                    "HEADING": "首段或穿口／离口仅航向控制",
                                    "LASER": "真正新鲜且已对齐的双墙反馈可用"}[selection[0]],
                                command_pending=bool(pending),
                                observation_token=controller.observation_token,
                                paired_source_times=controller.latest_source_times,
                                current_source_times=None if current is None else current.source_times,
                                source_ages_ms=None if current is None else tuple(
                                    (now - source) * 1000.0 for source in current.source_times),
                                control_mode=display.control_mode)
                    self._control_selection = selection
                if holding_pending:
                    target = None
                    linear, angular = display.command
                else:
                    target_method = (controller.target_patrol_heading_recovery
                                     if self.phase == "HEADING_RECOVERY" else
                                     controller.target_heading_only if start_heading or gyro_only
                                     else controller.target)
                    if self.cruise_transition_pending:
                        controller.request_cruise_speed_transition(target_goal)
                    options = dict(cached_feedback=cached,
                                   departure_only=start_heading or not self.armed or gyro_only)
                    if start_heading:
                        options["fixed_reference_deg"] = self.startup_reference_deg
                    if gyro_only:
                        options['road_adjustment'] = self.armed
                    if self.phase == "HEADING_RECOVERY":
                        options = {}
                    elif self.phase == "HOME" and not gyro_only:
                        options["correction_heading_limit_deg"] = PATROL_CORRECTION_HEADING_LIMIT_DEG
                    target = target_method(target_goal, now, **options)
                    if not any(target.wheel_targets):
                        self.sampler.cancel()
                        if any(display.wheel_targets):
                            self._recover()
                        else:
                            display.maybe_print()
                            time.sleep(control_interval_s)
                        continue
                    linear, angular = target.linear_mmps, target.angular_mradps
                    if _patrol_zero_transition(display.wheel_targets, target.wheel_targets):
                        self._alignment_stop()
                        self.sent_at, self.next_send = None, time.monotonic()
                        continue
                    display.centering_target = target
                try:
                    self._prepare_initial_launch()
                    previous_initial_guard = getattr(link, "_patrol_initial_launch_write_guard", None)
                    link._patrol_initial_launch_write_guard = self._initial_launch_write_guard
                    try:
                        number = _send_patrol_feedback_target(link, display, linear, angular, self.deadline, target=target)
                    finally:
                        link._patrol_initial_launch_write_guard = previous_initial_guard
                except sequence._CenteringProposalExpired:
                    self.next_send = time.monotonic()
                    time.sleep(control_interval_s)
                    continue
                except sequence._SideRecoveryRequired:
                    self._recover()
                    continue
                self.sent_at = time.monotonic()
                if (self.index == 0 and self.phase == "START" and not self._initial_confirmed
                        and linear == self.initial_speed_mmps):
                    key = sequence.MessageType.CONTROL_TARGET, number
                    record = getattr(link, "_command_ledger", {}).get(key)
                    command_at = record[0] if record is not None else self.sent_at
                    self._initial_command = (key, (sequence.MessageType.CONTROL_TARGET, linear, angular),
                                             (link.session_nonce, link.boot_id), command_at)
                if target is not None:
                    committed = controller.confirm_target_sent(target)
                    if committed and self.cruise_transition_pending:
                        self.cruise_transition_pending = False
                        if self.index == 0 and self.phase == "START" and not self._initial_complete:
                            runlog.emit("stage1_initial_speed_sent", "首发150目标已成功发送，保持启动航向直到首个实际转弯完成",
                                        requested_mmps=self.goal, actual_target_mmps=linear,
                                        distance_mm=self.initial_distance_mm, sequence=number)
                        self._checkpoint()
                self.next_send = self.sent_at + sequence.COMMAND_INTERVAL_S
                display.maybe_print()
                time.sleep(control_interval_s)
        finally:
            display._distance_observer = self._previous_observer
            link.side_motion_active = link.side_candidates_enabled = False
            finish = getattr(link, "finish_departure", None)
            if callable(finish):
                finish()

    def run(self):
        previous = getattr(self.display, "_post_turn_sampling_observer", None)
        self.display._post_turn_sampling_observer = self._observe_post_turn_sampling
        try:
            return self._run_with_recovery()
        finally:
            self.display._post_turn_sampling_observer = previous

    def _run_with_recovery(self):
        while True:
            try:
                return self._run_until_fault()
            except sequence._CenteringProposalExpired:
                self.next_send = time.monotonic()
                time.sleep(sequence.CONTROL_INTERVAL_S)
            except mcu.McuError as exc:
                if time.monotonic() >= self.deadline:
                    raise _PatrolLimit('达到用户指定的duration，停车') from exc
                self._checkpoint()
                self.sampler.cancel()
                self.response.pause(self.link, time.monotonic())
                recover = getattr(self.link, 'recover_fault', None)
                if not callable(recover):
                    raise
                self.link._latch_hard_fault(str(exc))
                recover(self.display, input_source=self.input_source)
                # Preserve only the confirmed semantic task. Raw laser frames
                # retain their original session and cannot be replayed as live.
                self.pending_candidate = self.pending_wall = None
                self._localization_samples.clear()
                self._post_turn_pending_states.clear()
                self._laser_resume_token = self._control_selection = None
                self.display._ready_batches.clear()
                reference = getattr(self.link, 'patrol_checkpoint', {}).get('fixed_reference')
                if self.display.controller.heading_reference_deg is None and reference is not None:
                    self.display.controller.set_initial_reference(reference)
                self._refresh_sampling_identity()
                self.sent_at, self.next_send = None, time.monotonic()

    def _refresh_sampling_identity(self):
        road_sampler = self.post_turn_sampler
        if road_sampler.started_at is not None and not road_sampler.reference_matches(self.display.controller):
            road_sampler.cancel()
            self.display.controller._patrol_formal_reference = None
            if (not self.first_exit_no_sampling and self.phase not in ("START", "HOME")
                    and not self._n06_slow_active):
                road_sampler.start(self.display.controller, time.monotonic())
        sampler = self.stopped_sampler
        if (self.post_turn_low_speed and sampler.started_at is not None
                and not sampler.reference_matches(self.display.controller)):
            sampler.cancel()
            self.phase, self.goal = "ALIGN", POST_TURN_SPEED_MMPS
            self.reference_fixed = False
            self._alignment_mode_index = 0
            self._checkpoint()
            runlog.emit("stage1_heading_sample_identity_reset", "身份或固定参考改变，停后重新激光矫正及固定1秒采样",
                        step=self.step.index, identity=self.display.controller._state_session,
                        fixed_reference_deg=self.display.controller.heading_reference_deg)


def _components() -> dict:
    return {"mcu": PatrolLink(), "clues": clues.Clues(),
            "audio": PatrolAudio(), "rfid": PatrolRfidReader()}


def _finish_summary(ledger: mission_ledger.MissionLedger, field_map: coverage.FieldMap) -> bool:
    missing_roads = sorted(set(field_map.roads) - ledger.completed_roads)
    missing_nodes = sorted(set(field_map.nodes) - ledger.confirmed_nodes)
    missing_patrols = sorted(set(range(1, 13)) - ledger.confirmed_patrol_nos)
    missing_announcements = sorted(set(range(1, 13))
        - set(getattr(ledger, "stage1_announced_patrol_nos", ())))
    returned = (ledger.last_confirmed_node == field_map.start_node_id
                and ledger.position_confidence == mission_ledger.POSITION_CONFIRMED_NODE
                and bool(getattr(ledger, "stage1_home_stop_confirmed", False))
                and ledger.route_plan_finished)
    traversal = not missing_roads and not missing_nodes and returned
    outcomes = getattr(ledger, "stage1_audio_request_outcomes", {})
    requests_completed = all(value.get("event") == "completed" for value in outcomes.values())
    complete = (traversal and requests_completed
                and not ledger.audio_failures
                and not ledger.terminal_reasons)
    print("\n阶段1结果")
    print(f"道路覆盖 {len(ledger.completed_roads)}/{len(field_map.roads)}: "
          f"{'通过' if not missing_roads else '缺少 ' + ', '.join(missing_roads)}")
    print(f"节点覆盖 {len(ledger.confirmed_nodes)}/{len(field_map.nodes)}: "
          f"{'通过' if not missing_nodes else '缺少 ' + ', '.join(missing_nodes)}")
    print(f"返回n01并严格停稳: {'通过' if returned else '未确认'}")
    print(f"实际遍历任务: {'通过' if traversal else '未完成'}")
    if ledger.audio_failures:
        print("本轮音频播报未全部完成，详情已记录日志")
    print("音频完成按播放器退出状态记录；实际可听播报需现场确认")
    if ledger.terminal_reasons:
        print(f"停止原因: {ledger.terminal_reasons}")
    print(f"完整阶段1任务: {'通过' if complete else '未完成'}")
    runlog.emit("stage1_summary", "实际运动及RFID播报分项完成判定", terminal=False,
                traversal_complete=traversal, complete=complete,
                missing_roads=missing_roads, missing_nodes=missing_nodes,
                returned_and_stopped=returned, missing_patrols=missing_patrols,
                missing_announcements=missing_announcements,
                audio_failures=ledger.audio_failures, rfid_errors=ledger.rfid_errors,
                audio_confirmation_basis="player_exit_status", audible_confirmed=None,
                terminal_reasons=ledger.terminal_reasons)
    return complete


def run_patrol(*, duration_s: float | None = None, components: dict | None = None,
               input_source=None, emitter=None, departure_strategy="10mm_turn") -> bool:
    """Run only the independent reference transport; default has no global timer."""
    terminal = sequence.AsyncTerminal() if emitter is None else None
    output = terminal if terminal is not None else emitter
    c, link, display, services, ledger, field_map = {}, None, None, None, None, None
    finished = False
    stop_requested = False
    try:
        if duration_s is not None and (isinstance(duration_s, bool)
                or not isinstance(duration_s, (int, float))
                or not math.isfinite(duration_s) or duration_s <= 0):
            raise ValueError("duration必须是有限正数秒数")
        deadline = math.inf if duration_s is None else time.monotonic() + duration_s
        field_map = coverage.load_map()
        steps = plan_route(field_map)
        c = components if components is not None else _components()
        link = c["mcu"]
        if isinstance(link, mcu.McuLink) and not isinstance(link, PatrolLink):
            raise mcu.McuError("巡逻实车链路必须使用PatrolLink的参考脚本局部门禁")
        if input_source is None:
            input_source = sequence.TerminalSpeedInput(write_line=output)
        link.set_motion_deadline(None if duration_s is None else deadline)
        display = PatrolDisplay(link, emitter=output)
        link._patrol_input = input_source
        display.patrol_deadline = deadline
        ledger = mission_ledger.MissionLedger(field_map)
        c["_ledger"], c["_field_map"] = ledger, field_map
        services = PatrolServices(c, ledger, active_step=steps[0], display=output)
        services.prepare()
        display.services = services
        services.start_polling()
        # Initial transport setup can be retried without restarting a road.
        for name, operation in (("OPEN", link.open), ("HELLO", link.hello),
                                ("CLOCK", link.synchronize_clock),
                                ("START_STOP", lambda: link.send_stop(sequence.StopReason.MISSION))):
            while True:
                sequence._check_input_exit(input_source)
                if time.monotonic() >= deadline:
                    raise _PatrolLimit('达到用户指定的duration，停车')
                services.tick(phase='启动恢复 ' + name)
                try:
                    operation()
                    break
                except (mcu.McuError, OSError) as exc:
                    runlog.exception('stage1_startup_retry', '启动动作失败，保留历史并等待重试', exc, action=name)
                    until = time.monotonic() + 1.0
                    while time.monotonic() < until:
                        sequence._check_input_exit(input_source)
                        services.tick(phase='启动恢复 ' + name)
                        time.sleep(sequence.CONTROL_INTERVAL_S)
        route_stats = route_statistics(field_map, steps)
        output(f"阶段1最短闭合路线：{route_stats['segments']}段、{route_stats['length_m']:.1f}m；"
               f"覆盖{route_stats['roads']}道路、{route_stats['nodes']}节点；"
               f"90°转弯{route_stats['turns_90']}次、180°掉头{route_stats['uturns']}次、"
               f"直行穿口{route_stats['straight']}次；"
               "首发150仅保持启动航向直到第一次实际转弯完成；转后各道路实际均速150±3%、激光差≤3mm为居中标准；"
               "先用150均速内合法差速，新鲜激光请求超限才临时单轮停转，非零轮速至少61；"
               "最近1秒稳定参考仅辅助，普通道路偏角超±45°回正至±5°；"
               "返家61 mm/s，首组新双墙起计160 mm含制动尾段；90°转弯外轮91、内轮0；"
               "180°掉头左轮-91、右轮+91；q/Ctrl-C停车。")
        for operation in (
            lambda: sequence._prepare(link, display,
                min(deadline, time.monotonic() + config.MCU_SELFCHECK_READY_TIMEOUT_S), input_source=input_source),
            lambda: _brake(link, display, deadline, input_source=input_source),
            lambda: sequence._sample_start(link, display, input_source=input_source),
        ):
            while True:
                try:
                    operation()
                    break
                except mcu.McuError as exc:
                    if time.monotonic() >= deadline:
                        raise _PatrolLimit('达到用户指定的duration，停车') from exc
                    recover = getattr(link, 'recover_fault', None)
                    if not callable(recover):
                        raise
                    link._latch_hard_fault(str(exc))
                    recover(display, input_source=input_source)
        link.motion_checks_enabled = True
        display.input_speed_mmps = INITIAL_SPEED_MMPS
        runner = PatrolRunner(link, display, field_map, steps, ledger, services,
                              deadline=deadline, input_source=input_source,
                              departure_strategy=departure_strategy)
        runner.run()
        # The runner only returns after reference BRAKE, strict stationary
        # evidence, heading verification and a physical home-arrival record.
        def stopped_poll():
            sequence._check_input_exit(input_source)
            sequence._remaining(deadline)
            display.tick()
            display.consume_samples(collect_means=False, update_controller=False)
            if callable(getattr(link, "motion_health_reason", None)):
                reason = link.motion_health_reason()
                if reason:
                    raise mcu.McuError(reason)
            if not sequence._assert_still_stable(link, display):
                _wait_stable(link, display, deadline, input_source=input_source)
            display.maybe_print()

        while True:
            try:
                services.finish_after_stop(on_poll=stopped_poll)
                break
            except mcu.McuError as exc:
                if time.monotonic() >= deadline:
                    raise _PatrolLimit('达到用户指定的duration，停车') from exc
                recover = getattr(link, 'recover_fault', None)
                if not callable(recover):
                    raise
                runlog.exception('stage1_audio_tail_vehicle_fault', '终点播报收尾期间车辆故障，保留全部播报请求并停车恢复', exc)
                link._latch_hard_fault(str(exc))
                recover(display, input_source=input_source)
        finished = True
        display.action = "巡逻路线结束，车辆已严格停稳"
        display.maybe_print(force=True)
    except (KeyboardInterrupt, EOFError, sequence._UserQuit, _PatrolLimit) as exc:
        stop_requested = True
        reason = ("收到Ctrl-C，尝试停车" if isinstance(exc, KeyboardInterrupt)
                  else "终端已关闭，尝试停车" if isinstance(exc, EOFError) else str(exc))
        if ledger is not None:
            ledger.note_terminal(reason)
        if link is not None:
            sequence._report_failure(link, display, reason)
        if display is None:
            output(reason)
    except Exception as exc:
        runlog.exception('stage1_unexpected_recovery', '异常历史保留，停止无依据的非零续发', exc)
        if isinstance(link, PatrolLink) and display is not None:
            link._latch_hard_fault(str(exc))
            if not isinstance(exc, mcu.McuError):
                link.patrol_manual_required = f'上位机异常，保留任务等待人工：{exc}'
            try:
                link.recover_fault(display, input_source=input_source)
            except (KeyboardInterrupt, EOFError, sequence._UserQuit, _PatrolLimit, mcu.McuError) as stop:
                stop_requested = True
                reason = ('达到用户指定的duration，停车' if time.monotonic() >= deadline else str(stop))
                ledger.note_terminal(reason)
                sequence._report_failure(link, display, reason)
        else:
            if ledger is not None:
                ledger.note_terminal(str(exc))
            output(f"阶段1未完成：{exc}")
    finally:
        try:
            if link is not None and link.connected and (finished or stop_requested):
                link.send_stop(sequence.StopReason.CLOSE)
        except Exception:
            pass
        try:
            if input_source is not None:
                input_source.close()
        finally:
            try:
                if services is not None:
                    services.close()
            finally:
                if link is not None:
                    link._patrol_input = None
                    link._patrol_display = None
                try:
                    if link is not None:
                        if finished or stop_requested:
                            link.shutdown()
                        else:
                            link.close()
                finally:
                    if terminal is not None:
                        terminal.close()
    if ledger is None or field_map is None:
        return False
    return _finish_summary(ledger, field_map) and finished


def _duration(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("duration必须是秒数") from exc
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("duration必须是有限正数")
    return result


def run_rfid_audio_check(*, duration_s: float | None = None, components: dict | None = None,
                         input_source=None, emitter=None) -> bool:
    """Replay the live Stage-one RFID/audio service without constructing an MCU link."""
    output = print if emitter is None else emitter
    services = None
    c = {}
    try:
        if duration_s is not None and (isinstance(duration_s, bool)
                or not isinstance(duration_s, (int, float))
                or not math.isfinite(duration_s) or duration_s <= 0):
            raise ValueError("duration必须是有限正数秒数")
        script_path = Path(__file__).resolve()
        identity = dict(script_path=str(script_path),
                        software_sha256=hashlib.sha256(script_path.read_bytes()).hexdigest())
        output("RFID/播报独立检查：只连接读卡器与播放器；q/Ctrl-C结束，排队播报最多收尾30秒。")
        output("[stage1_rfid_audio_check_software] " + json.dumps(identity, ensure_ascii=False))
        output("播放器返回码0仅证明播放进程正常退出；RDK声卡输出、音量与实际声音须现场听验。")
        runlog.emit("stage1_rfid_audio_check_started", "阶段一独立读卡与音频检查", **identity)
        field_map = coverage.load_map()
        ledger = mission_ledger.MissionLedger(field_map)
        c = components if components is not None else {
            "clues": clues.Clues(), "audio": PatrolAudio(), "rfid": PatrolRfidReader()}
        step = coverage.Step(0, field_map.start_node_id, field_map.start_node_id,
                             "RFID_AUDIO_CHECK", False)
        services = PatrolServices(c, ledger, active_step=step, display=output, diagnostic=True)
        c["_ledger"], c["_services"] = ledger, services
        reader, player = c.get("rfid"), c.get("audio")
        services._diagnostic_event("stage1_rfid_audio_check_components",
            port=getattr(reader, "port", None), baud=getattr(reader, "baud", None),
            audio_mode=getattr(player, "mode", None), device=getattr(player, "device", None),
            aplay_path=shutil.which("aplay"))
        if not services.prepare() or not services.start_polling():
            output("RFID/播报检查初始化失败；请根据上述组件或音频错误核对RDK配置。")
            return False
        if input_source is None:
            input_source = sequence.TerminalSpeedInput(write_line=output)
        deadline = math.inf if duration_s is None else time.monotonic() + duration_s
        reason = "达到duration"
        try:
            while time.monotonic() < deadline:
                line = input_source.poll()
                if isinstance(line, str) and line.strip().lower() == "q":
                    reason = "收到q"
                    break
                services.tick(phase="独立读卡与播报检查")
                time.sleep(sequence.CONTROL_INTERVAL_S)
        except (KeyboardInterrupt, EOFError) as exc:
            reason = "收到Ctrl-C" if isinstance(exc, KeyboardInterrupt) else "终端已关闭"
        output(f"{reason}，停止新增读卡并完成已排队播报。")
        ok = services.finish_after_stop(timeout_s=30.0)
        request_count = len(services._audio_requests)
        completed_count = len(services._audio_completed)
        services._diagnostic_event("stage1_rfid_audio_check_summary",
            requests=request_count, completed=completed_count,
            rfid_errors=ledger.rfid_errors, audio_failures=ledger.audio_failures,
            software_chain_ok=ok and request_count > 0 and not ledger.rfid_errors,
            audible_confirmation="现场听验")
        if not request_count:
            output("本次未取得有效完整读块，尚无读卡到播报的成功证据。")
        return ok and request_count > 0 and not ledger.rfid_errors
    except KeyboardInterrupt:
        output("播报收尾被Ctrl-C中断，取消未完成请求。")
        return False
    except Exception as exc:
        runlog.exception("stage1_rfid_audio_check_failed", "独立读卡与播报检查失败", exc)
        output(f"RFID/播报检查失败：{type(exc).__name__}: {exc}")
        return False
    finally:
        if services is not None:
            services.close()
        else:
            for name in ("rfid", "audio"):
                component = c.get(name)
                if component is not None:
                    component.close()
        if input_source is not None:
            input_source.close()


def main_cli(argv: list[str] | None = None, *, departure_strategy="10mm_turn") -> int:
    parser = argparse.ArgumentParser(description="阶段1：实际覆盖29道路20节点、返回n01及RFID播报")
    parser.add_argument("--duration", type=_duration, default=None,
                        help="可选最大运行秒数；默认无全程时限")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--print-route", action="store_true", help="只输出路线，不连接硬件或创建运行日志")
    mode.add_argument("--rfid-audio-check", action="store_true",
                      help="仅检查真实读卡到播报链路，不创建MCU或运行车辆；q/Ctrl-C退出")
    args = parser.parse_args(argv)
    if args.print_route:
        field_map = coverage.load_map()
        steps = plan_route(field_map)
        for step in steps:
            print(f"{step.index:02d}: {step.from_node} -> {step.to_node} ({step.road_id})")
        stats = route_statistics(field_map, steps)
        print(f"{stats['segments']}段，{stats['length_m']:.1f}m；"
              f"覆盖{stats['roads']}道路、{stats['nodes']}节点并返回{field_map.start_node_id}；"
              f"90°转弯{stats['turns_90']}次、180°掉头{stats['uturns']}次、"
              f"直行穿口{stats['straight']}次（返家停车后不计终始接续）")
        return 0
    if args.rfid_audio_check:
        with runlog.RunLog("stage1_rfid_audio_check", terminal=sequence.RawTerminalStream(),
                           periodic_status=False):
            return 0 if run_rfid_audio_check(duration_s=args.duration) else 1
    with runlog.RunLog("stage1_patrol", terminal=sequence.RawTerminalStream(), periodic_status=False):
        return 0 if run_patrol(duration_s=args.duration, departure_strategy=departure_strategy) else 1


if __name__ == "__main__":
    sys.exit(main_cli())
