"""Per-round coverage, interrupted obstacle probes, and return-time estimates.

All arrivals and obstacle returns are supplied by the motion controller only
after physical position confirmation. This module never infers an arrival.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import time
from typing import Callable

import config
import coverage


ProbeKey = tuple[str, str]


@dataclass(frozen=True)
class MissionAction:
    kind: str
    step: coverage.Step
    target: ProbeKey | None = None


@dataclass
class ProbeTask:
    road_id: str
    entry_node: str
    status: str = "pending"
    reason: str = ""


class MissionPlanner:
    """Plan one physical road at a time; newest obstacle probe has priority."""

    def __init__(self, field_map: coverage.FieldMap) -> None:
        self.field_map = field_map
        self.blocked_roads: set[str] = set()
        self.completed_roads: set[str] = set()
        self.probe_tasks: dict[ProbeKey, ProbeTask] = {}
        self.stack: list[ProbeKey] = []
        self.unreachable_roads: set[str] = set()
        self.return_mode = False
        self.return_reason = ""
        self.status = "exploring"
        self._sequence = 0
        self._pending: MissionAction | None = None
        self._obstacle_pending = False

    def _paths(self, start: str, allowed: set[str] | None = None):
        if start not in self.field_map.nodes:
            raise coverage.CoverageError(f"Unknown confirmed node: {start}")
        distances = {start: 0.0}
        predecessors: dict[str, tuple[str, str]] = {}
        queue = [(0.0, start)]
        while queue:
            cost, node = heapq.heappop(queue)
            if cost != distances[node]:
                continue
            for road_id, other in self.field_map.adjacency[node]:
                if road_id in self.blocked_roads or (allowed is not None and road_id not in allowed):
                    continue
                candidate = cost + road_length_mm(self.field_map.roads[road_id])
                if candidate < distances.get(other, math.inf):
                    distances[other] = candidate
                    predecessors[other] = node, road_id
                    heapq.heappush(queue, (candidate, other))
        return distances, predecessors

    def _route(self, start: str, end: str, predecessors) -> tuple[coverage.Step, ...] | None:
        if start == end:
            return ()
        roads = []
        node = end
        while node != start:
            if node not in predecessors:
                return None
            previous, road_id = predecessors[node]
            roads.append((previous, node, road_id))
            node = previous
        return tuple(
            coverage.Step(i, a, b, road_id, self.field_map.roads[road_id].dark_tunnel)
            for i, (a, b, road_id) in enumerate(reversed(roads))
        )

    def known_home_route(self, node: str, *, extra_road: str | None = None):
        allowed = self.completed_roads | ({extra_road} if extra_road else set())
        _, predecessors = self._paths(node, allowed)
        return self._route(node, self.field_map.start_node_id, predecessors)

    def _action(self, kind: str, step: coverage.Step, target=None) -> MissionAction:
        step = coverage.Step(
            self._sequence, step.from_node, step.to_node, step.road_id, step.dark_tunnel
        )
        self._sequence += 1
        self._pending = MissionAction(kind, step, target)
        return self._pending

    def next_action(self, node: str) -> MissionAction | None:
        if self._pending is not None:
            if self._obstacle_pending:
                raise coverage.CoverageError("Obstacle return has not been physically confirmed")
            if self._pending.step.from_node != node:
                raise coverage.CoverageError("Pending action disagrees with confirmed node")
            return self._pending
        if self.return_mode:
            route = self.known_home_route(node)
            if route is None:
                self.status = "no_known_home_route"
                return None
            if not route:
                self.status = "returned"
                return None
            return self._action("return", route[0])

        distances, predecessors = self._paths(node)
        while self.stack:
            key = self.stack[-1]
            task = self.probe_tasks[key]
            if task.status != "pending":
                self.stack.pop()
                continue
            if task.entry_node not in distances:
                task.status = "unreachable"
                task.reason = "entry unreachable with current blocked roads"
                self.stack.pop()
                continue
            if node != task.entry_node:
                route = self._route(node, task.entry_node, predecessors)
                return self._action("transit", route[0], key)
            road = self.field_map.roads[task.road_id]
            other = road.b if road.a == node else road.a
            return self._action(
                "probe", coverage.Step(0, node, other, road.id, road.dark_tunnel), key
            )

        remaining = set(self.field_map.roads) - self.completed_roads - self.blocked_roads
        self.unreachable_roads = {
            road_id for road_id in remaining
            if self.field_map.roads[road_id].a not in distances
            and self.field_map.roads[road_id].b not in distances
        }
        candidates = []
        for road_id in sorted(remaining - self.unreachable_roads):
            road = self.field_map.roads[road_id]
            for entry, other in ((road.a, road.b), (road.b, road.a)):
                if entry in distances:
                    candidates.append((distances[entry], road_id, entry, other))
        if not candidates:
            self.begin_return("all reachable roads and obstacle probes processed")
            return self.next_action(node)
        _, road_id, entry, other = min(candidates)
        if node != entry:
            route = self._route(node, entry, predecessors)
            return self._action("transit", route[0])
        road = self.field_map.roads[road_id]
        return self._action("explore", coverage.Step(0, entry, other, road_id, road.dark_tunnel))

    def _validate_pending(self, action: MissionAction) -> None:
        if action != self._pending:
            raise coverage.CoverageError("Stale or unissued mission action")

    def arrived(self, action: MissionAction) -> None:
        self._validate_pending(action)
        if action.step.road_id in self.blocked_roads:
            raise coverage.CoverageError("Arrival contradicts a fixed obstacle on this road")
        self.completed_roads.add(action.step.road_id)
        self._pending = None

    def observe_obstacle(self, action: MissionAction) -> None:
        self._validate_pending(action)
        self.blocked_roads.add(action.step.road_id)
        self.completed_roads.discard(action.step.road_id)
        key = action.step.road_id, action.step.from_node
        if key not in self.probe_tasks:
            self.probe_tasks[key] = ProbeTask(*key, status="returning")
        self._obstacle_pending = True

    def obstacle_returned(self, action: MissionAction) -> None:
        self._validate_pending(action)
        if not self._obstacle_pending:
            self.observe_obstacle(action)
        key = action.step.road_id, action.step.from_node
        task = self.probe_tasks[key]
        task.status = "complete"
        task.reason = "obstacle observed and original entry physically confirmed"
        self._pending = None
        self._obstacle_pending = False
        while self.stack and self.probe_tasks[self.stack[-1]].status != "pending":
            self.stack.pop()
        opposite = action.step.road_id, action.step.to_node
        if opposite not in self.probe_tasks:
            self.probe_tasks[opposite] = ProbeTask(*opposite)
            if self.return_mode:
                self.probe_tasks[opposite].status = "cancelled"
                self.probe_tasks[opposite].reason = self.return_reason
            else:
                self.stack.append(opposite)

    def begin_return(self, reason: str) -> None:
        self.return_mode = True
        self.return_reason = reason
        self.status = "returning"
        for task in self.probe_tasks.values():
            if task.status == "pending":
                task.status = "cancelled"
                task.reason = reason
        self.stack.clear()
        # A pending road can only be discarded while physically at its entry.
        # Callers retreating mid-road must confirm entry before asking next_action.
        self._pending = None
        self._obstacle_pending = False

    def summary(self) -> dict:
        return {
            "status": self.status,
            "return_reason": self.return_reason,
            "blocked_roads": sorted(self.blocked_roads),
            "completed_roads": sorted(self.completed_roads),
            "unreachable_roads": sorted(self.unreachable_roads),
            "probes": [
                {"road_id": task.road_id, "entry_node": task.entry_node,
                 "status": task.status, "reason": task.reason}
                for _, task in sorted(self.probe_tasks.items())
            ],
        }


def road_length_mm(road: coverage.MapRoad) -> float:
    try:
        length = float(road.length_m) * 1000.0
    except (TypeError, ValueError):
        raise coverage.CoverageError(f"Road {road.id} requires a physical length for time budgeting")
    if not math.isfinite(length) or length <= 0:
        raise coverage.CoverageError(f"Road {road.id} has invalid physical length")
    return length


class ReturnBudget:
    """Conservative configuration estimate, never a physical-time guarantee."""

    def __init__(
        self, field_map: coverage.FieldMap, deadline: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.field_map = field_map
        self.deadline = deadline
        self.clock = clock
        self.calibrated = bool(getattr(config, "RETURN_BUDGET_CALIBRATED", False))
        self.speed_mmps = float(getattr(config, "RETURN_ESTIMATED_SPEED_MMPS", 150.0))
        self.turn_90_s = float(getattr(config, "RETURN_TURN_90_S", 7.0))
        self.uturn_s = float(getattr(config, "RETURN_UTURN_S", 7.0))
        self.edge_overhead_s = float(getattr(config, "RETURN_EDGE_OVERHEAD_S", 1.0))
        self.dark_overhead_s = float(getattr(config, "RETURN_DARK_OVERHEAD_S", 1.0))
        self.final_settle_s = float(getattr(config, "RETURN_FINAL_SETTLE_S", 2.0))
        self.margin_s = float(getattr(config, "RETURN_MARGIN_S", 15.0))
        self.factor = float(getattr(config, "RETURN_TIME_FACTOR", 1.25))
        self.observed_factor = 1.0
        if self.speed_mmps <= 0 or self.factor < 1.0:
            raise ValueError("Return speed must be positive and time factor at least one")
        for value in (self.deadline, self.speed_mmps, self.turn_90_s, self.uturn_s,
                      self.edge_overhead_s, self.dark_overhead_s, self.final_settle_s,
                      self.margin_s, self.factor):
            if not math.isfinite(value) or value < 0:
                raise ValueError("Return timing values must be finite and nonnegative")

    @property
    def remaining_s(self) -> float:
        return max(0.0, self.deadline - self.clock())

    def _heading(self, step: coverage.Step):
        return coverage.direction_between(self.field_map, step.from_node, step.to_node)

    def _turn_time(self, heading, target) -> float:
        if heading is None:
            return self.uturn_s
        kind = coverage.turn_kind(heading, target)
        if kind == coverage.TURN_STRAIGHT:
            return 0.0
        return self.uturn_s if kind == coverage.TURN_UTURN else self.turn_90_s

    def _drive_time(self, road: coverage.MapRoad, distance_mm: float | None = None) -> float:
        distance = road_length_mm(road) if distance_mm is None else max(0.0, distance_mm)
        # Budget the final braking zone at creep speed, even on a short road.
        creep = min(self.speed_mmps, float(config.BRAKE_MIN_MMPS))
        brake_distance = min(distance, float(config.BRAKE_START_MM))
        if creep <= 0 or not math.isfinite(distance):
            return math.inf
        return ((distance - brake_distance) / self.speed_mmps + brake_distance / creep
                + self.edge_overhead_s + (self.dark_overhead_s if road.dark_tunnel else 0.0))

    def _route_time(self, route, heading=None) -> float:
        if route is None:
            return math.inf
        total = 0.0
        for step in route:
            target = self._heading(step)
            total += self._turn_time(heading, target)
            total += self._drive_time(self.field_map.roads[step.road_id])
            heading = target
        return total

    def _reserve(self, movement_s: float, audio_remaining_s: float = 0.0, scan_s=0.0) -> float:
        if not math.isfinite(audio_remaining_s) or not math.isfinite(scan_s):
            return math.inf
        movement = (movement_s + self.final_settle_s) * self.factor * self.observed_factor
        # Queued speech plays while driving/scanning; only its tail adds time.
        return max(movement + max(0.0, scan_s), max(0.0, audio_remaining_s)) + self.margin_s

    def required_home(self, planner: MissionPlanner, node: str, *, heading=None,
                      audio_remaining_s: float = 0.0) -> float:
        return self._reserve(self._route_time(planner.known_home_route(node), heading), audio_remaining_s)

    def required_for_action(self, planner: MissionPlanner, action: MissionAction, *,
                            heading=None, audio_remaining_s: float = 0.0) -> float:
        step = action.step
        entering_turn = self._turn_time(heading, self._heading(step))
        movement = self._road_scenarios(planner, action, 0.0) + entering_turn
        return self._reserve(movement, audio_remaining_s)

    def _road_scenarios(self, planner: MissionPlanner, action: MissionAction,
                        distance_mm: float) -> float:
        step = action.step
        road = self.field_map.roads[step.road_id]
        length = road_length_mm(road)
        if not math.isfinite(distance_mm) or distance_mm < 0:
            return math.inf
        remaining = max(0.0, length - distance_mm)
        heading = self._heading(step)
        reverse_heading = (-heading[0], -heading[1])
        home_entry = self._route_time(planner.known_home_route(step.from_node), reverse_heading)
        # The farthest obstruction requires remaining outward travel, a U-turn,
        # and the entire road back; the partial outward leg is never a graph edge.
        blocked = (self._drive_time(road, remaining) + self.uturn_s
                   + self._drive_time(road) + home_entry)
        if action.kind == "probe" or step.road_id in planner.blocked_roads:
            return blocked
        home_exit = planner.known_home_route(step.to_node, extra_road=step.road_id)
        success = self._drive_time(road, remaining) + self._route_time(home_exit, heading)
        if step.road_id in planner.completed_roads:
            return success
        return max(success, blocked)

    def required_from_road(self, planner: MissionPlanner, action: MissionAction,
                           distance_mm: float, *, heading=None, scan_s: float = 0.0,
                           audio_remaining_s: float = 0.0) -> float:
        return self._reserve(self._road_scenarios(planner, action, distance_mm), audio_remaining_s, scan_s)

    def required_to_retreat(self, planner: MissionPlanner, action: MissionAction,
                            distance_mm: float, *, audio_remaining_s: float = 0.0) -> float:
        if not math.isfinite(distance_mm) or distance_mm < 0:
            return math.inf
        step = action.step
        heading = self._heading(step)
        reverse = (-heading[0], -heading[1])
        movement = (self.uturn_s + self._drive_time(self.field_map.roads[step.road_id], distance_mm)
                    + self._route_time(planner.known_home_route(step.from_node), reverse))
        return self._reserve(movement, audio_remaining_s)

    def can_start(self, planner: MissionPlanner, action: MissionAction, **kwargs) -> bool:
        return self.remaining_s > self.required_for_action(planner, action, **kwargs)

    def can_scan(self, planner: MissionPlanner, action: MissionAction,
                 distance_mm: float, *, scan_s: float, **kwargs) -> bool:
        return self.remaining_s > self.required_from_road(
            planner, action, distance_mm, scan_s=scan_s, **kwargs
        )

    def observe_elapsed(self, action: MissionAction, elapsed_s: float, *, heading=None) -> None:
        """Pass moving/turning time only, excluding time spent in scan/audio waits."""
        estimate = self._route_time((action.step,), heading)
        if math.isfinite(elapsed_s) and elapsed_s > 0 and estimate > 0:
            self.observed_factor = max(self.observed_factor, elapsed_s / estimate)
