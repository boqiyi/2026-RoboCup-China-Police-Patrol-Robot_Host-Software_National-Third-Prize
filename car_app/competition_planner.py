"""Exact remaining-road planning, with no transport or device ownership.

The distance objective is an undirected rural postman problem.  A positive-cost
optimal walk uses an edge at most twice: deleting two copies otherwise preserves
its support, connectivity and degree parity.  We enumerate every parity support,
then solve its necessary doubled connectors by exact Steiner dynamic programming.
Every equal-distance support is considered by an exact Euler-transition search,
so turns are minimized across *all* shortest walks, not just one augmentation.

Search has no time cutoff and no approximate fallback.  Run it in the competition
coordinator's planning process while the vehicle remains stationary.
"""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import heapq
import math
from typing import Iterable

import coverage


class PlanningError(coverage.CoverageError):
    """Invalid planning input or failed independent route validation."""


@dataclass(frozen=True)
class PlanRequest:
    start_node: str
    completed_roads: frozenset[str] = frozenset()
    blocked_roads: frozenset[str] = frozenset()
    heading: tuple[int, int] | None = None
    home_node: str | None = None


@dataclass(frozen=True)
class PlanResult:
    steps: tuple[coverage.Step, ...]
    distance_mm: int
    turn_actions: int
    endpoint: str
    home_reachable: bool
    unreachable_roads: frozenset[str]
    optimal: bool = True
    required_roads: frozenset[str] = frozenset()
    explored_candidates: int = 0


@dataclass(frozen=True)
class _Edge:
    road_id: str
    a: str
    b: str
    length: int
    dark_tunnel: bool


def _road_ids(values: Iterable[str], known: set[str], name: str) -> frozenset[str]:
    if isinstance(values, (str, bytes)):
        raise PlanningError(f"{name} must be a collection of road ids")
    try:
        result = frozenset(values)
    except (TypeError, ValueError) as exc:
        raise PlanningError(f"Invalid {name}") from exc
    if any(not isinstance(value, str) for value in result) or not result <= known:
        raise PlanningError(f"{name} contains an unknown road id")
    return result


def _length_mm(road) -> int:
    value = road.length_m
    if isinstance(value, bool) or not isinstance(value, (int, float, Decimal)):
        raise PlanningError(f"Road {road.id} needs a positive length in metres")
    try:
        exact = Decimal(str(value)) * 1000
    except (InvalidOperation, ValueError) as exc:
        raise PlanningError(f"Invalid length for road {road.id}") from exc
    if not exact.is_finite() or exact <= 0 or exact != exact.to_integral_value():
        raise PlanningError(f"Road {road.id} length must be positive whole millimetres")
    return int(exact)


def _turn(heading, direction) -> int:
    return int(heading is not None and coverage.turn_kind(heading, direction) != coverage.TURN_STRAIGHT)


def _route_cost(field_map, route, lengths, heading=None):
    distance, turns = 0, 0
    for step in route:
        direction = coverage.direction_between(field_map, step.from_node, step.to_node)
        distance += lengths[step.road_id]
        turns += _turn(heading, direction)
        heading = direction
    return distance, turns


def validate_result(field_map: coverage.FieldMap, request: PlanRequest,
                    result: PlanResult) -> bool:
    """Independently check a planning-process result before authorizing motion.

    This rebuilds reachability and costs directly from the map and request,
    without using the map's cached adjacency or rerunning ``solve_plan``.  It
    checks feasibility and result metadata, including an explicit ``True``
    optimal flag.  The flag alone is not an optimality certificate for an
    arbitrary forged route: global optimality is supplied by the trusted exact
    planning process, not proved again by this linear-time validation pass.
    """
    if not isinstance(request, PlanRequest) or not isinstance(result, PlanResult):
        raise PlanningError("PlanRequest and PlanResult instances are required")
    if (not isinstance(field_map, coverage.FieldMap)
            or not isinstance(field_map.nodes, dict) or not field_map.nodes
            or not isinstance(field_map.roads, dict)):
        raise PlanningError("A nonempty FieldMap with node/road dictionaries is required")
    for node_id, node in field_map.nodes.items():
        if (not isinstance(node_id, str) or not node_id
                or not isinstance(node, coverage.MapNode) or node.id != node_id
                or type(node.row) is not int or type(node.col) is not int):
            raise PlanningError("Invalid map node identity or coordinates")
    nodes = set(field_map.nodes)
    start = request.start_node
    home = field_map.start_node_id if request.home_node is None else request.home_node
    if (not isinstance(start, str) or not isinstance(home, str)
            or start not in nodes or home not in nodes):
        raise PlanningError("Start and home must be known map nodes")
    heading = request.heading
    if heading is not None and (not isinstance(heading, tuple) or len(heading) != 2
                               or any(type(value) is not int for value in heading)
                               or heading not in ((-1, 0), (1, 0), (0, -1), (0, 1))):
        raise PlanningError("Heading must be a cardinal (row, column) tuple or None")
    known = set(field_map.roads)
    if any(not isinstance(road_id, str) or not road_id for road_id in known):
        raise PlanningError("Map road ids must be nonempty strings")
    completed = _road_ids(request.completed_roads, known, "completed_roads")
    blocked = _road_ids(request.blocked_roads, known, "blocked_roads")
    lengths, adjacency = {}, {node: [] for node in nodes}
    for road_id, road in field_map.roads.items():
        if (not isinstance(road, coverage.MapRoad) or road.id != road_id
                or not isinstance(road.a, str) or not isinstance(road.b, str)
                or road.a not in nodes or road.b not in nodes or road.a == road.b
                or type(road.dark_tunnel) is not bool):
            raise PlanningError(f"Invalid road identity/endpoints/metadata: {road_id}")
        try:
            coverage.direction_between(field_map, road.a, road.b)
            lengths[road_id] = _length_mm(road)
        except (coverage.CoverageError, ArithmeticError) as exc:
            raise PlanningError(f"Invalid road geometry/length: {road_id}") from exc
        if road_id not in blocked:
            adjacency[road.a].append(road.b)
            adjacency[road.b].append(road.a)
    reachable, pending = {start}, [start]
    while pending:
        for other in adjacency[pending.pop()]:
            if other not in reachable:
                reachable.add(other)
                pending.append(other)
    required = frozenset(road_id for road_id, road in field_map.roads.items()
                         if road_id not in blocked and road_id not in completed
                         and road.a in reachable)
    unreachable = frozenset(road_id for road_id, road in field_map.roads.items()
                            if road_id not in blocked and road_id not in completed
                            and road.a not in reachable)
    home_reachable = home in reachable
    if result.optimal is not True:
        raise PlanningError("Only a completed exact optimal result can authorize motion")
    if (result.home_reachable is not home_reachable
            or not isinstance(result.required_roads, frozenset) or result.required_roads != required
            or not isinstance(result.unreachable_roads, frozenset) or result.unreachable_roads != unreachable):
        raise PlanningError("Result reachability/remaining-road metadata disagrees with the request")
    for name in ("distance_mm", "turn_actions", "explored_candidates"):
        value = getattr(result, name)
        if type(value) is not int or value < 0:
            raise PlanningError(f"Result {name} must be a nonnegative integer")
    if (not isinstance(result.endpoint, str) or result.endpoint not in reachable
            or home_reachable and result.endpoint != home):
        raise PlanningError("Result endpoint violates the reachable HOME/free-endpoint contract")
    if not isinstance(result.steps, tuple):
        raise PlanningError("Result steps must be a tuple of coverage.Step instances")
    current, traversed, distance, turns = start, set(), 0, 0
    for index, step in enumerate(result.steps):
        if (not isinstance(step, coverage.Step) or type(step.index) is not int or step.index != index
                or not isinstance(step.from_node, str) or not isinstance(step.to_node, str)
                or not isinstance(step.road_id, str) or step.road_id not in known
                or type(step.dark_tunnel) is not bool):
            raise PlanningError("Invalid step type, index, road id or metadata")
        road = field_map.roads[step.road_id]
        if (step.from_node != current or step.road_id in blocked
                or not ((step.from_node == road.a and step.to_node == road.b)
                        or (step.from_node == road.b and step.to_node == road.a))
                or step.dark_tunnel is not road.dark_tunnel):
            raise PlanningError("Result step is discontinuous, blocked or disagrees with its road")
        # Independent cardinal direction comparison: all nonstraight actions,
        # including reversal, cost one; there is no post-arrival action.
        direction = coverage.direction_between(field_map, step.from_node, step.to_node)
        turns += int(heading is not None and heading != direction)
        distance += lengths[step.road_id]
        heading, current = direction, step.to_node
        traversed.add(step.road_id)
    if (not required <= traversed or current != result.endpoint
            or distance != result.distance_mm or turns != result.turn_actions):
        raise PlanningError("Result coverage, actual endpoint or objective claims are invalid")
    return True


def _components(nodes, edges, support, start, end):
    """Contract mandatory support; isolated start/end are mandatory terminals."""
    parent = {node: node for node in nodes}

    def root(node):
        while parent[node] != node:
            node = parent[node]
        return node

    touched = {start, end}
    for index, edge in enumerate(edges):
        if support >> index & 1:
            parent[root(edge.a)] = root(edge.b)
            touched.update((edge.a, edge.b))
    groups = {}
    for node in nodes:
        groups.setdefault(root(node), []).append(node)
    ordered = sorted(groups.values(), key=lambda group: min(group))
    labels = {node: index for index, group in enumerate(ordered) for node in group}
    terminals = tuple(sorted({labels[node] for node in touched}))
    return labels, terminals, labels[start], len(ordered)


class _Steiner:
    """Exact numeric DP and streamed enumeration of all cheapest edge supports."""

    def __init__(self, nodes, edges, support, start, end):
        labels, self.terminals, self.root, count = _components(nodes, edges, support, start, end)
        self.count = count
        self.adj = [[] for _ in range(count)]
        self.edge_cost = {}
        for index, edge in enumerate(edges):
            if support >> index & 1:
                continue
            a, b = labels[edge.a], labels[edge.b]
            if a != b:
                cost = 2 * edge.length
                self.adj[a].append((b, cost, index))
                self.adj[b].append((a, cost, index))
                self.edge_cost[index] = cost
        self.full = (1 << len(self.terminals)) - 1
        self.dp = []
        self.cost = 0
        if len(self.terminals) <= 1:
            return
        # Larger than the cost of any edge-simple doubled connected support.
        self.inf = 1 + 2 * sum(edge.length for edge in edges)
        self.dp = [[self.inf] * count for _ in range(self.full + 1)]
        terminal_bits = {terminal: 1 << index for index, terminal in enumerate(self.terminals)}
        for mask in range(1, self.full + 1):
            row = self.dp[mask]
            if mask & (mask - 1) == 0:
                row[self.terminals[mask.bit_length() - 1]] = 0
            else:
                # One representative for each unordered split.
                least = mask & -mask
                sub = (mask - 1) & mask
                while sub:
                    if sub & least:
                        other = mask ^ sub
                        left, right = self.dp[sub], self.dp[other]
                        for vertex in range(count):
                            candidate = left[vertex] + right[vertex]
                            if candidate < row[vertex]:
                                row[vertex] = candidate
                    sub = (sub - 1) & mask
            queue = [(cost, vertex) for vertex, cost in enumerate(row) if cost < self.inf]
            heapq.heapify(queue)
            while queue:
                cost, vertex = heapq.heappop(queue)
                if cost != row[vertex]:
                    continue
                for other, weight, _index in self.adj[vertex]:
                    candidate = cost + weight
                    if candidate < row[other]:
                        row[other] = candidate
                        heapq.heappush(queue, (candidate, other))
        self.terminal_bits = terminal_bits
        self.cost = self.dp[self.full][self.root]
        if self.cost >= self.inf:
            raise PlanningError("Reachable planning graph cannot connect its mandatory support")

    def _support_cost(self, bits):
        result = 0
        while bits:
            bit = bits & -bits
            result += self.edge_cost[bit.bit_length() - 1]
            bits ^= bit
        return result

    def supports(self):
        if not self.dp:
            yield 0
            return

        def generate(mask, vertex):
            cost = self.dp[mask][vertex]
            if mask == self.terminal_bits.get(vertex) and cost == 0:
                yield 0
            least = mask & -mask
            sub = (mask - 1) & mask
            while sub:
                other = mask ^ sub
                if sub & least and self.dp[sub][vertex] + self.dp[other][vertex] == cost:
                    for left in generate(sub, vertex):
                        for right in generate(other, vertex):
                            combined = left | right
                            if self._support_cost(combined) == cost:
                                yield combined
                sub = (sub - 1) & mask
            for previous, weight, edge in self.adj[vertex]:
                if self.dp[mask][previous] + weight == cost:
                    bit = 1 << edge
                    for support in generate(mask, previous):
                        if not support & bit:
                            yield support | bit

        # Bounded deduplication only avoids repeated work; clearing it never
        # removes a feasible support or changes the optimization guarantee.
        recent = set()
        for support in generate(self.full, self.root):
            if support not in recent:
                if len(recent) >= 8192:
                    recent.clear()
                recent.add(support)
                yield support


def _pairings(edge_ids, outward):
    if not edge_ids:
        yield 0, ()
        return
    first = edge_ids[0]
    incoming = tuple(-value for value in outward[first])
    for offset in range(1, len(edge_ids)):
        second = edge_ids[offset]
        for cost, pairs in _pairings(edge_ids[1:offset] + edge_ids[offset + 1:], outward):
            yield cost + _turn(incoming, outward[second]), ((first, second),) + pairs


def _euler_route(field_map, instances, start, end):
    """Produce a feasible upper bound; it is never labelled turn-optimal."""
    adj = {}
    for index, edge in enumerate(instances):
        adj.setdefault(edge.a, []).append(index)
        adj.setdefault(edge.b, []).append(index)
    used = [False] * len(instances)
    stack = [(start, None)]
    walk = []
    while stack:
        node, incoming = stack[-1]
        remaining = adj.get(node, [])
        while remaining and used[remaining[-1]]:
            remaining.pop()
        if not remaining:
            walk.append(stack.pop())
            continue
        index = remaining.pop()
        used[index] = True
        edge = instances[index]
        stack.append((edge.b if node == edge.a else edge.a, index))
    walk.reverse()
    if len(walk) != len(instances) + 1 or walk[0][0] != start or walk[-1][0] != end:
        raise PlanningError("Parity support did not produce its promised Euler walk")
    return tuple(coverage.Step(index, previous[0], current[0], instances[current[1]].road_id,
                               instances[current[1]].dark_tunnel)
                 for index, (previous, current) in enumerate(zip(walk, walk[1:])))


def _minimum_turn_route(field_map, instances, start, end, heading, upper_bound=math.inf):
    """Enumerate local edge transitions, rejecting premature closed subtours."""
    if not instances:
        return (0, ()) if start == end else None
    lengths = {edge.road_id: edge.length for edge in instances}
    initial_route = _euler_route(field_map, instances, start, end)
    _distance, initial_cost = _route_cost(field_map, initial_route, lengths, heading)
    best_cost = min(initial_cost, upper_bound)
    best_route = initial_route if initial_cost < upper_bound else None
    incident, outward = {}, {}
    for index, edge in enumerate(instances):
        for node, other in ((edge.a, edge.b), (edge.b, edge.a)):
            incident.setdefault(node, []).append(index)
            outward[node, index] = coverage.direction_between(field_map, node, other)
    options_by_node = []
    closed = start == end
    for node, ids in sorted(incident.items()):
        ids = tuple(ids)
        directions = {edge: outward[node, edge] for edge in ids}
        options = []
        if not closed and node in (start, end):
            for boundary in ids:
                remaining = tuple(edge for edge in ids if edge != boundary)
                for cost, pairs in _pairings(remaining, directions):
                    if node == start:
                        cost += _turn(heading, directions[boundary])
                    options.append((cost, pairs, boundary if node == start else -1))
        else:
            for cost, pairs in _pairings(ids, directions):
                if closed and node == start:
                    for first, second in pairs:
                        omitted = _turn(tuple(-value for value in directions[first]), directions[second])
                        for boundary in (first, second):
                            options.append((cost - omitted + _turn(heading, directions[boundary]), pairs, boundary))
                else:
                    options.append((cost, pairs, -1))
        if not options:
            raise PlanningError("Euler transition degree parity is invalid")
        options_by_node.append((node, sorted(options)))
    # Force low-branching vertices first, exposing disconnected subtours early.
    options_by_node.sort(key=lambda item: (len(item[1]), item[0]))
    bounds = [0] * (len(options_by_node) + 1)
    for index in range(len(options_by_node) - 1, -1, -1):
        bounds[index] = bounds[index + 1] + options_by_node[index][1][0][0]
    if bounds[0] >= best_cost:
        return (best_cost, best_route) if best_route is not None else None
    count = len(instances)
    parent, sizes, history = list(range(count)), [1] * count, []
    chosen, best_options = [], None

    def root(edge):
        while parent[edge] != edge:
            edge = parent[edge]
        return edge

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

    def search(index, cost, pair_count):
        nonlocal best_cost, best_options
        if cost + bounds[index] >= best_cost:
            return
        if index == len(options_by_node):
            if pair_count != count - int(not closed):
                raise PlanningError("Incomplete Euler transition assignment")
            best_cost, best_options = cost, tuple(chosen)
            return
        node, options = options_by_node[index]
        for option_cost, pairs, boundary in options:
            if cost + option_cost + bounds[index + 1] >= best_cost:
                break
            snapshot, used_pairs, valid = len(history), pair_count, True
            for first, second in pairs:
                if not connect(first, second) and (not closed or used_pairs != count - 1):
                    valid = False
                    break
                used_pairs += 1
            if valid:
                chosen.append((node, pairs, boundary))
                search(index + 1, cost + option_cost, used_pairs)
                chosen.pop()
            rollback(snapshot)
            if best_cost == bounds[0]:
                return

    search(0, 0, 0)
    if best_options is not None:
        partners, first = {}, None
        for node, pairs, boundary in best_options:
            for a, b in pairs:
                partners[node, a], partners[node, b] = b, a
            if node == start:
                first = boundary
        current, edge_id, used, route = start, first, set(), []
        for index in range(count):
            if edge_id is None or edge_id in used:
                raise PlanningError("Euler transition reconstruction entered a subtour")
            used.add(edge_id)
            edge = instances[edge_id]
            other = edge.b if current == edge.a else edge.a
            route.append(coverage.Step(index, current, other, edge.road_id, edge.dark_tunnel))
            current, edge_id = other, partners.get((other, edge_id))
        if current != end:
            raise PlanningError("Euler transition reconstruction has the wrong endpoint")
        best_route = tuple(route)
        if _route_cost(field_map, best_route, lengths, heading)[1] != best_cost:
            raise PlanningError("Euler transition turn cost failed reconstruction validation")
    return (best_cost, best_route) if best_route is not None else None


def solve_plan(field_map: coverage.FieldMap, *, start_node: str,
               completed_roads: Iterable[str] = (), blocked_roads: Iterable[str] = (),
               heading: tuple[int, int] | None = None, home_node: str | None = None) -> PlanResult:
    """Cover every reachable remaining edge, returning home or to a free endpoint.

    Distance uses positive integer millimetres from the map, followed by the
    number of nonstraight node actions (90 and 180 degrees both count as one).
    ``heading`` adds the action before the first road.  No turn is charged after
    the final arrival.  Completed roads are optional shortcuts; blocked roads
    are excluded completely.  The coordinator owns any exceptional retreat
    prefix on the newly blocked road and supplies the original entry as start.
    """
    if not isinstance(field_map, coverage.FieldMap) or not field_map.nodes:
        raise PlanningError("A nonempty FieldMap is required")
    nodes = set(field_map.nodes)
    home = field_map.start_node_id if home_node is None else home_node
    if (not isinstance(start_node, str) or not isinstance(home, str)
            or start_node not in nodes or home not in nodes):
        raise PlanningError("Start and home must be known map nodes")
    if heading is not None and (not isinstance(heading, tuple) or len(heading) != 2
                               or any(type(value) is not int for value in heading)
                               or heading not in ((-1, 0), (1, 0), (0, -1), (0, 1))):
        raise PlanningError("Heading must be a cardinal (row, column) tuple or None")
    known = set(field_map.roads)
    completed = _road_ids(completed_roads, known, "completed_roads")
    blocked = _road_ids(blocked_roads, known, "blocked_roads")
    all_edges, all_lengths, adjacency = [], {}, {node: [] for node in nodes}
    for road_id, road in sorted(field_map.roads.items()):
        if road_id != road.id or road.a not in nodes or road.b not in nodes or road.a == road.b:
            raise PlanningError(f"Invalid road identity/endpoints: {road_id}")
        try:
            coverage.direction_between(field_map, road.a, road.b)
        except coverage.CoverageError as exc:
            raise PlanningError(f"Road {road_id} needs distinct axis-aligned endpoints") from exc
        length = _length_mm(road)
        all_lengths[road_id] = length
        if road_id not in blocked:
            edge = _Edge(road_id, road.a, road.b, length, bool(road.dark_tunnel))
            all_edges.append(edge)
            adjacency[road.a].append(road.b)
            adjacency[road.b].append(road.a)
    reachable, queue = {start_node}, [start_node]
    while queue:
        for other in adjacency[queue.pop()]:
            if other not in reachable:
                reachable.add(other)
                queue.append(other)
    unreachable = frozenset(edge.road_id for edge in all_edges
                            if edge.road_id not in completed and edge.a not in reachable)
    edges = [edge for edge in all_edges if edge.a in reachable]
    required_ids = frozenset(edge.road_id for edge in edges if edge.road_id not in completed)
    required = sum(1 << index for index, edge in enumerate(edges) if edge.road_id in required_ids)
    home_reachable = home in reachable
    endpoints = (home,) if home_reachable else tuple(sorted(reachable))
    # The spanning tree gives one parity support; its fundamental cycles span
    # every other support, including disconnected parity cycles.
    edge_adjacency = {node: [] for node in reachable}
    for index, edge in enumerate(edges):
        edge_adjacency[edge.a].append((edge.b, index))
        edge_adjacency[edge.b].append((edge.a, index))
    paths, tree_edges, queue = {start_node: 0}, set(), [start_node]
    while queue:
        node = queue.pop()
        for other, index in edge_adjacency[node]:
            if other not in paths:
                paths[other] = paths[node] ^ (1 << index)
                tree_edges.add(index)
                queue.append(other)
    cycles = [paths[edge.a] ^ paths[edge.b] ^ (1 << index)
              for index, edge in enumerate(edges) if index not in tree_edges]
    cycle_masks = [0]
    for cycle in cycles:
        cycle_masks += [mask ^ cycle for mask in cycle_masks]
    candidates = []
    for endpoint in endpoints:
        for cycle in cycle_masks:
            parity = paths[endpoint] ^ cycle
            base = sum(edge.length * (1 if parity >> index & 1 else 2)
                       for index, edge in enumerate(edges) if (required | parity) >> index & 1)
            candidates.append((base, endpoint, parity))
    candidates.sort()
    best_distance, shortest = math.inf, []
    explored = 0
    for base, endpoint, parity in candidates:
        if base > best_distance:
            break
        explored += 1
        connectors = _Steiner(reachable, edges, required | parity, start_node, endpoint)
        distance = base + connectors.cost
        if distance < best_distance:
            best_distance, shortest = distance, [(endpoint, parity)]
        elif distance == best_distance:
            shortest.append((endpoint, parity))
    best_turns, best_route, best_endpoint = math.inf, None, None
    for endpoint, parity in shortest:
        connectors = _Steiner(reachable, edges, required | parity, start_node, endpoint)
        for support in connectors.supports():
            instances = []
            for index, edge in enumerate(edges):
                multiplicity = (1 if parity >> index & 1 else
                                2 if (required | support) >> index & 1 else 0)
                instances.extend([edge] * multiplicity)
            result = _minimum_turn_route(field_map, instances, start_node, endpoint, heading, best_turns)
            if result is not None and result[0] < best_turns:
                best_turns, best_route = result
                best_endpoint = endpoint
    if best_route is None or best_endpoint is None:
        raise PlanningError("Exact search failed to produce a feasible route")
    current, covered = start_node, set()
    for index, step in enumerate(best_route):
        road = field_map.roads[step.road_id]
        if (step.index != index or step.from_node != current or step.road_id in blocked
                or {step.from_node, step.to_node} != {road.a, road.b}):
            raise PlanningError("Route continuity/blocked-road validation failed")
        covered.add(step.road_id)
        current = step.to_node
    actual_distance, actual_turns = _route_cost(field_map, best_route, all_lengths, heading)
    if (not required_ids <= covered or current != best_endpoint
            or actual_distance != best_distance or actual_turns != best_turns
            or home_reachable and current != home):
        raise PlanningError("Exact objective/coverage/endpoint validation failed")
    return PlanResult(best_route, actual_distance, actual_turns, best_endpoint,
                      home_reachable, unreachable, True, required_ids, explored)
