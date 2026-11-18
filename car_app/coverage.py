"""地图加载 + DFS 覆盖遍历（纯逻辑，不碰硬件，可桌面单测）。

从 2026 场地拓扑 JSON 建无向图，用深度优先遍历生成"覆盖全部道路、走完
所有巡逻点、回到 START_GATE"的行驶顺序；并按节点网格坐标算每步的转弯方向。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import config


class CoverageError(RuntimeError):
    """地图加载或遍历失败。"""


# 三种节点类型
KIND_PATROL = "PATROL_POINT"
KIND_INTERSECTION = "INTERSECTION"
KIND_START = "START_GATE"

# 转弯类型
TURN_STRAIGHT = "straight"
TURN_LEFT = "left"      # 对应正角速度（w>0）
TURN_RIGHT = "right"    # 对应负角速度（w<0）
TURN_UTURN = "uturn"    # 掉头 180°


@dataclass(frozen=True)
class MapNode:
    id: str
    kind: str
    patrol_no: int | None
    row: int
    col: int


@dataclass(frozen=True)
class MapRoad:
    id: str
    a: str
    b: str
    dark_tunnel: bool
    length_m: float | None


@dataclass(frozen=True)
class Step:
    """覆盖遍历里的一步：从 from_node 沿 road 开到 to_node。"""

    index: int
    from_node: str
    to_node: str
    road_id: str
    dark_tunnel: bool


@dataclass(frozen=True, eq=False)
class ReplanResult:
    """节点图重规划的显式结果，不把 best-effort 误写成完整任务。"""

    steps: tuple[Step, ...]
    reachable_remaining_roads: frozenset[str]
    unreachable_remaining_roads: frozenset[str]
    reachable_nodes: frozenset[str]
    can_return_to_end: bool
    planned_end_node: str
    covers_all_reachable_roads: bool

    def __iter__(self):
        return iter(self.steps)

    def __len__(self) -> int:
        return len(self.steps)

    def __bool__(self) -> bool:
        return bool(self.steps)

    def __getitem__(self, index):
        return self.steps[index]

    def __eq__(self, other: object) -> bool:
        """兼容旧的 list 读取方式，避免桌面旧脚本无谓失效。"""
        if isinstance(other, ReplanResult):
            return (
                self.steps,
                self.reachable_remaining_roads,
                self.unreachable_remaining_roads,
                self.reachable_nodes,
                self.can_return_to_end,
                self.planned_end_node,
                self.covers_all_reachable_roads,
            ) == (
                other.steps,
                other.reachable_remaining_roads,
                other.unreachable_remaining_roads,
                other.reachable_nodes,
                other.can_return_to_end,
                other.planned_end_node,
                other.covers_all_reachable_roads,
            )
        if isinstance(other, (list, tuple)):
            return list(self.steps) == list(other)
        return NotImplemented

@dataclass
class FieldMap:
    nodes: dict[str, MapNode]
    roads: dict[str, MapRoad]
    start_node_id: str
    # adjacency: node_id -> [(road_id, other_node_id), ...]（按 road_id 排序保证确定性）
    adjacency: dict[str, list[tuple[str, str]]] = field(default_factory=dict)


def load_map(path: str | Path | None = None) -> FieldMap:
    """读取并校验 2026 场地拓扑 JSON。"""
    p = Path(path) if path is not None else config.MAP_JSON
    try:
        doc = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CoverageError(f"无法读取地图 {p}: {exc}") from exc

    if not isinstance(doc, dict) or not isinstance(doc.get("nodes"), list) or not isinstance(doc.get("roads"), list):
        raise CoverageError("地图必须包含 nodes 和 roads 数组")
    nodes: dict[str, MapNode] = {}
    positions: set[tuple[int, int]] = set()
    for raw in doc["nodes"]:
        if not isinstance(raw, dict):
            raise CoverageError("节点必须是对象")
        nid = raw.get("id")
        kind = raw.get("kind")
        if not isinstance(nid, str) or not nid:
            raise CoverageError("节点 id 必须是非空字符串")
        if nid in nodes:
            raise CoverageError(f"节点 id 重复: {nid}")
        pno = raw.get("patrol_no")
        if kind not in (KIND_PATROL, KIND_INTERSECTION, KIND_START):
            raise CoverageError(f"节点 {nid} 类型非法: {kind}")
        if kind == KIND_PATROL:
            if type(pno) is not int or not 1 <= pno <= 12:
                raise CoverageError(f"巡逻点 {nid} 的 patrol_no 非法: {pno}")
        elif pno is not None:
            raise CoverageError(f"非巡逻点 {nid} 的 patrol_no 应为 null")
        row, col = raw.get("row"), raw.get("col")
        if type(row) is not int or type(col) is not int:
            raise CoverageError(f"节点 {nid} 的 row/col 必须是整数")
        if (row, col) in positions:
            raise CoverageError(f"节点 {nid} 的坐标与其他节点重合")
        positions.add((row, col))
        nodes[nid] = MapNode(nid, kind, pno if isinstance(pno, int) else None, row, col)

    roads: dict[str, MapRoad] = {}
    for raw in doc["roads"]:
        if not isinstance(raw, dict):
            raise CoverageError("道路必须是对象")
        rid = raw.get("id")
        a, b = raw.get("from"), raw.get("to")
        if not isinstance(rid, str) or not rid:
            raise CoverageError("道路 id 必须是非空字符串")
        if not isinstance(a, str) or not isinstance(b, str):
            raise CoverageError(f"道路 {rid} 的端点必须是节点 id")
        if a not in nodes or b not in nodes:
            raise CoverageError(f"道路 {rid} 引用了不存在的节点: {a}/{b}")
        if rid in roads:
            raise CoverageError(f"道路 id 重复: {rid}")
        if a == b or (nodes[a].row != nodes[b].row and nodes[a].col != nodes[b].col):
            raise CoverageError(f"道路 {rid} 必须连接不同节点且沿单一坐标轴")
        dark_tunnel = raw.get("dark_tunnel", False)
        if type(dark_tunnel) is not bool:
            raise CoverageError(f"道路 {rid} 的 dark_tunnel 必须是布尔值")
        length = raw.get("length_m")
        if type(length) not in (int, float) or not math.isfinite(length) or length <= 0:
            raise CoverageError(f"道路 {rid} 的 length_m 必须是有限正数")
        roads[rid] = MapRoad(rid, a, b, dark_tunnel, float(length))

    start = doc.get("start", {})
    if not isinstance(start, dict):
        raise CoverageError("start 必须是对象")
    start_node = start.get("node_id", "n01")
    if not isinstance(start_node, str) or start_node not in nodes:
        raise CoverageError(f"起点节点不存在: {start_node}")
    if nodes[start_node].kind != KIND_START:
        raise CoverageError("起点必须标记为 START_GATE")

    fm = FieldMap(nodes=nodes, roads=roads, start_node_id=start_node)
    fm.adjacency = _build_adjacency(fm)
    _check_connected(fm)
    return fm


def _build_adjacency(fm: FieldMap) -> dict[str, list[tuple[str, str]]]:
    adj: dict[str, list[tuple[str, str]]] = {nid: [] for nid in fm.nodes}
    for rid, road in fm.roads.items():
        adj[road.a].append((rid, road.b))
        adj[road.b].append((rid, road.a))
    for lst in adj.values():
        lst.sort(key=lambda item: item[0])
    return adj


def _check_connected(fm: FieldMap) -> None:
    seen: set[str] = set()
    stack = [fm.start_node_id]
    while stack:
        nid = stack.pop()
        if nid in seen:
            continue
        seen.add(nid)
        for _, other in fm.adjacency[nid]:
            stack.append(other)
    missing = [nid for nid in fm.nodes if nid not in seen]
    if missing:
        raise CoverageError("地图不连通，无法覆盖: " + ", ".join(missing))


def _road_len(road: MapRoad) -> float:
    """道路长度：缺失或 0 按 1.0 计（最短路计权统一口径）。"""
    return road.length_m if road.length_m else 1.0


def build_coverage_order(fm: FieldMap | None = None) -> list[Step]:
    """深度优先覆盖遍历，返回完整行驶顺序（含回溯）。"""
    if fm is None:
        fm = load_map()
    visited_roads: set[str] = set()
    steps: list[Step] = []

    def dfs(node_id: str) -> None:
        for road_id, other in fm.adjacency[node_id]:
            if road_id in visited_roads:
                continue
            visited_roads.add(road_id)
            road = fm.roads[road_id]
            steps.append(
                Step(
                    index=len(steps),
                    from_node=node_id,
                    to_node=other,
                    road_id=road_id,
                    dark_tunnel=road.dark_tunnel,
                )
            )
            dfs(other)
            # 回溯：回到当前节点（除非已经覆盖完全部道路，最后一次回溯在 dfs 顶层自然结束）
            steps.append(
                Step(
                    index=len(steps),
                    from_node=other,
                    to_node=node_id,
                    road_id=road_id,
                    dark_tunnel=road.dark_tunnel,
                )
            )

    dfs(fm.start_node_id)
    if len(visited_roads) != len(fm.roads):
        missing = [rid for rid in fm.roads if rid not in visited_roads]
        raise CoverageError("存在未覆盖道路: " + ", ".join(missing))
    if not steps or steps[-1].to_node != fm.start_node_id:
        raise CoverageError("覆盖遍历未回到起点")
    return steps


def direction_between(fm: FieldMap, a: str, b: str) -> tuple[int, int]:
    """从节点 a 到 b 的网格方向（单位向量，dr 向下为正、dc 向右为正）。"""
    na, nb = fm.nodes[a], fm.nodes[b]
    dr = 0 if nb.row == na.row else (1 if nb.row > na.row else -1)
    dc = 0 if nb.col == na.col else (1 if nb.col > na.col else -1)
    if (dr == 0 and dc == 0) or (dr != 0 and dc != 0):
        raise CoverageError(f"相邻节点 {a}/{b} 必须沿单一坐标轴且不重合")
    return (dr, dc)


def turn_kind(from_dir: tuple[int, int], to_dir: tuple[int, int]) -> str:
    """由旧方向转新方向，返回 straight/left/right/uturn。"""
    dot = from_dir[0] * to_dir[0] + from_dir[1] * to_dir[1]
    cross = from_dir[0] * to_dir[1] - from_dir[1] * to_dir[0]
    if dot > 0:
        return TURN_STRAIGHT
    if dot < 0:
        return TURN_UTURN
    return TURN_LEFT if cross > 0 else TURN_RIGHT


def turn_angle_deg(turn: str) -> float:
    """转弯类型对应的目标角（正=左转）。"""
    return {
        TURN_STRAIGHT: 0.0,
        TURN_LEFT: config.TURN_ANGLE_DEG,
        TURN_RIGHT: -config.TURN_ANGLE_DEG,
        TURN_UTURN: 180.0,
    }[turn]


def patrol_points(fm: FieldMap | None = None) -> dict[int, str]:
    """返回 {巡逻点号: 节点id}。"""
    if fm is None:
        fm = load_map()
    return {
        node.patrol_no: node.id
        for node in fm.nodes.values()
        if node.patrol_no is not None
    }


def dark_tunnel_roads(fm: FieldMap | None = None) -> list[str]:
    """返回涵洞（暗隧道）道路 id 列表。"""
    if fm is None:
        fm = load_map()
    return [rid for rid, road in fm.roads.items() if road.dark_tunnel]


# ---------------------------------------------------------------------------
# 中国邮差最短闭合路线（覆盖全部道路、尽量少重复、回到起点）
# ---------------------------------------------------------------------------
def _active_adjacency(fm: FieldMap, blocked: set[str]) -> dict[str, list[tuple[str, str]]]:
    adj: dict[str, list[tuple[str, str]]] = {nid: [] for nid in fm.nodes}
    for rid, road in fm.roads.items():
        if rid in blocked:
            continue
        adj[road.a].append((rid, road.b))
        adj[road.b].append((rid, road.a))
    for lst in adj.values():
        lst.sort(key=lambda item: item[0])
    return adj


def _odd_nodes(adj: dict[str, list]) -> list[str]:
    return [nid for nid, lst in adj.items() if len(lst) % 2 == 1]


def _floyd_warshall(fm: FieldMap, adj: dict[str, list]) -> tuple[dict, dict]:
    """返回 (dist, nxt)，以节点 id 为键的嵌套 dict。"""
    nids = list(fm.nodes)
    dist = {a: {b: math.inf for b in nids} for a in nids}
    nxt = {a: {b: b for b in nids} for a in nids}
    for a in nids:
        dist[a][a] = 0.0
    for a in nids:
        for rid, b in adj[a]:
            w = _road_len(fm.roads[rid])
            if w < dist[a][b]:
                dist[a][b] = w
                nxt[a][b] = b
    for k in nids:
        for i in nids:
            dik = dist[i][k]
            for j in nids:
                if dik + dist[k][j] < dist[i][j]:
                    dist[i][j] = dik + dist[k][j]
                    nxt[i][j] = nxt[i][k]
    return dist, nxt


def _min_perfect_matching(odd: list[str], dist: dict) -> list[tuple[str, str]]:
    """8 个奇度节点以内暴力枚举最小权完美匹配。"""
    assert len(odd) % 2 == 0, "奇度节点数必为偶数（握手引理）"
    if not odd:
        return []
    best: float | None = None
    best_pairs: list[tuple[str, str]] = []

    def rec(remaining: list[str], pairs: list[tuple[str, str]], total: float) -> None:
        nonlocal best, best_pairs
        if not remaining:
            if best is None or total < best:
                best = total
                best_pairs = list(pairs)
            return
        first = remaining[0]
        for i in range(1, len(remaining)):
            second = remaining[i]
            rest = remaining[1:i] + remaining[i + 1:]
            pairs.append((first, second))
            rec(rest, pairs, total + dist[first][second])
            pairs.pop()

    rec(odd, [], 0.0)
    return best_pairs


def _road_between(fm: FieldMap, adj: dict, a: str, b: str) -> str:
    for rid, other in adj[a]:
        if other == b:
            return rid
    raise CoverageError(f"节点 {a} 与 {b} 之间无道路")


def _reconstruct_path_roads(fm: FieldMap, adj: dict, nxt: dict, a: str, b: str) -> list[str]:
    """a→b 最短路径上的道路 id 列表（按活动邻接表取道路）。"""
    roads: list[str] = []
    cur = a
    while cur != b:
        nxt_node = nxt[cur][b]
        roads.append(_road_between(fm, adj, cur, nxt_node))
        cur = nxt_node
    return roads


def _eulerian_walk(fm: FieldMap, start: str, edges: list[tuple[str, str, str]]) -> list[str]:
    """edges: [(u, v, road_id)]，求从 start 出发的欧拉回路节点序列（Hierholzer）。"""
    adj: dict[str, list[int]] = {nid: [] for nid in fm.nodes}
    for i, (u, v, _) in enumerate(edges):
        adj[u].append(i)
        adj[v].append(i)
    used = [False] * len(edges)
    stack = [start]
    walk: list[str] = []
    while stack:
        v = stack[-1]
        e = None
        while adj[v]:
            candidate = adj[v].pop()
            if not used[candidate]:
                e = candidate
                break
        if e is None:
            walk.append(v)
            stack.pop()
        else:
            used[e] = True
            u, w, _ = edges[e]
            stack.append(w if u == v else u)
    walk.reverse()
    return walk


def plan_coverage_route(
    fm: FieldMap | None = None,
    blocked_roads: set[str] | None = None,
) -> list[Step]:
    """中国邮差最短闭合路线：覆盖全部道路、尽量少重复、回到起点。

    ``blocked_roads`` 为未来避障规划预留（屏蔽若干道路后重规划）；本次仅做过滤，
    不实现避障。中国邮差失败时回退 DFS（``build_coverage_order``）。
    """
    if fm is None:
        fm = load_map()
    blocked = set(blocked_roads or ())
    adj = _active_adjacency(fm, blocked)

    # 活动子图连通性检查
    active_nodes = {nid for nid in fm.nodes if adj[nid]}
    if active_nodes:
        seen: set[str] = set()
        stack = [next(iter(active_nodes))]
        while stack:
            nid = stack.pop()
            if nid in seen:
                continue
            seen.add(nid)
            for _, other in adj[nid]:
                stack.append(other)
        if seen != active_nodes:
            raise CoverageError("屏蔽道路后地图不连通，无法覆盖")
    if fm.start_node_id not in active_nodes:
        raise CoverageError("起点节点被屏蔽道路孤立，无法覆盖")

    active_road_ids = {rid for rid in fm.roads if rid not in blocked}
    if not active_road_ids:
        return []

    odd = _odd_nodes(adj)
    if len(odd) > 8:
        # 奇度节点过多，最小权完美匹配暴力枚举不可行，回退 DFS
        if blocked:
            raise CoverageError(f"奇度节点 {len(odd)} > 8，无法求最短路线")
        return build_coverage_order(fm)
    dist, nxt = _floyd_warshall(fm, adj)
    pairs = _min_perfect_matching(odd, dist)

    # 多重图边集：原活动道路（每条一次）+ 匹配对之间的最短路（额外遍历）
    edges: list[tuple[str, str, str]] = []
    for rid in sorted(active_road_ids):
        road = fm.roads[rid]
        edges.append((road.a, road.b, rid))
    for a, b in pairs:
        for rid in _reconstruct_path_roads(fm, adj, nxt, a, b):
            road = fm.roads[rid]
            edges.append((road.a, road.b, rid))

    if not edges:
        return []

    walk = _eulerian_walk(fm, fm.start_node_id, edges)
    steps: list[Step] = []
    for a, b in zip(walk, walk[1:]):
        rid = _road_between(fm, adj, a, b)
        road = fm.roads[rid]
        steps.append(
            Step(index=len(steps), from_node=a, to_node=b, road_id=rid,
                 dark_tunnel=road.dark_tunnel)
        )

    # 校验：覆盖全部活动道路 + 回到起点
    covered = {s.road_id for s in steps}
    if covered != active_road_ids or not steps or steps[-1].to_node != fm.start_node_id:
        if blocked:
            raise CoverageError("中国邮差路线覆盖/回起点校验失败（且存在屏蔽道路）")
        return build_coverage_order(fm)
    return steps


# ---------------------------------------------------------------------------
# 乡村邮差重规划（避障后：覆盖剩余道路 + 抄近道连通 + 回起点 + 重复最少）
# ---------------------------------------------------------------------------
def _required_components(fm: FieldMap, required: set[str]) -> list[set[str]]:
    """required 道路子图的连通分量（只沿 required 边走），返回节点集合列表。"""
    adj: dict[str, list[str]] = {nid: [] for nid in fm.nodes}
    for rid in required:
        road = fm.roads[rid]
        adj[road.a].append(road.b)
        adj[road.b].append(road.a)
    seen: set[str] = set()
    comps: list[set[str]] = []
    for nid in fm.nodes:
        if nid in seen or not adj[nid]:
            continue
        comp: set[str] = set()
        stack = [nid]
        seen.add(nid)
        while stack:
            v = stack.pop()
            comp.add(v)
            for w in adj[v]:
                if w not in seen:
                    seen.add(w)
                    stack.append(w)
        comps.append(comp)
    return comps


def _reachable_nodes(fm: FieldMap, adj: dict, start: str) -> set[str]:
    """从 start 沿活动邻接表可达的节点集合。"""
    seen: set[str] = {start}
    stack = [start]
    while stack:
        v = stack.pop()
        for _, w in adj[v]:
            if w not in seen:
                seen.add(w)
                stack.append(w)
    return seen


def _terminal_mst(terminals: list[set[str]], dist: dict) -> list[tuple[str, str]]:
    """对 terminals（节点集合列表）按最短路距离做 Prim MST，返回 MST 边的代表节点对。"""
    n = len(terminals)
    if n <= 1:
        return []
    dmat = [[math.inf] * n for _ in range(n)]
    rep = [[None] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            best = math.inf
            ba = bb = None
            for a in terminals[i]:
                for b in terminals[j]:
                    if dist[a][b] < best:
                        best = dist[a][b]
                        ba, bb = a, b
            dmat[i][j] = dmat[j][i] = best
            rep[i][j] = rep[j][i] = (ba, bb)
    in_tree = [False] * n
    in_tree[0] = True
    mst: list[tuple[str, str]] = []
    for _ in range(n - 1):
        best = math.inf
        bi = bj = -1
        for i in range(n):
            if not in_tree[i]:
                continue
            for j in range(n):
                if in_tree[j]:
                    continue
                if dmat[i][j] < best:
                    best = dmat[i][j]
                    bi, bj = i, j
        if bi < 0:  # 有终端不可达（被 blocked 切断），跳过
            break
        in_tree[bj] = True
        mst.append(rep[bi][bj])
    return mst


def _shortest_path_steps(fm: FieldMap, adj: dict, dist: dict, nxt: dict,
                         a: str, b: str) -> list[Step]:
    """a→b 最短路径的 Step 列表；a==b 或不可达返回空。"""
    if a == b or dist[a][b] == math.inf:
        return []
    steps: list[Step] = []
    cur = a
    while cur != b:
        nxt_node = nxt[cur][b]
        rid = _road_between(fm, adj, cur, nxt_node)
        road = fm.roads[rid]
        steps.append(Step(len(steps), cur, nxt_node, rid, road.dark_tunnel))
        cur = nxt_node
    return steps


def plan_replan_route(
    fm: FieldMap | None = None,
    start_node_id: str | None = None,
    blocked_roads: set[str] | None = None,
    traversed_roads: set[str] | None = None,
    end_node_id: str | None = None,
) -> ReplanResult:
    """从确认节点覆盖可达剩余道路，并显式报告不可达道路与回起点能力。

    已完成道路可作为抄近道，屏蔽道路永不进入后续路线。若终点不可达，仍尽力在
    当前连通分量完成可达道路并回到当前确认节点，但 ``can_return_to_end`` 为 False。
    """
    if fm is None:
        fm = load_map()
    start = start_node_id if start_node_id is not None else fm.start_node_id
    end = end_node_id if end_node_id is not None else fm.start_node_id
    blocked = set(blocked_roads or ())
    traversed = set(traversed_roads or ())
    active_adj = _active_adjacency(fm, blocked)
    dist, nxt = _floyd_warshall(fm, active_adj)
    reachable = _reachable_nodes(fm, active_adj, start)
    remaining = {rid for rid in fm.roads if rid not in blocked and rid not in traversed}
    reachable_required = {
        rid
        for rid in remaining
        if fm.roads[rid].a in reachable and fm.roads[rid].b in reachable
    }
    unreachable_required = remaining - reachable_required
    can_return_to_end = end in reachable
    route_end = end if can_return_to_end else start

    if not reachable_required:
        steps = _shortest_path_steps(fm, active_adj, dist, nxt, start, route_end)
        return ReplanResult(
            tuple(steps),
            frozenset(),
            frozenset(unreachable_required),
            frozenset(reachable),
            can_return_to_end,
            route_end,
            True,
        )

    comps = _required_components(fm, reachable_required)
    terminals = [set(component) for component in comps]
    in_comp = {nid for component in comps for nid in component}
    if start not in in_comp:
        terminals.append({start})
    if route_end != start and route_end not in in_comp:
        terminals.append({route_end})
    mst_pairs = _terminal_mst(terminals, dist)

    edges: list[tuple[str, str, str]] = []
    for rid in sorted(reachable_required):
        road = fm.roads[rid]
        edges.append((road.a, road.b, rid))
    for a, b in mst_pairs:
        for rid in _reconstruct_path_roads(fm, active_adj, nxt, a, b):
            road = fm.roads[rid]
            edges.append((road.a, road.b, rid))

    degree: dict[str, int] = {nid: 0 for nid in fm.nodes}
    for u, v, _ in edges:
        degree[u] += 1
        degree[v] += 1
    if start == route_end:
        odd = [nid for nid in fm.nodes if degree[nid] % 2 == 1]
    else:
        odd = [
            nid
            for nid in fm.nodes
            if (degree[nid] % 2 == 1) != (nid in (start, route_end))
        ]
    if len(odd) > 8:
        sorted_odd = sorted(odd)
        pairs = [
            (sorted_odd[index], sorted_odd[index + 1])
            for index in range(0, len(sorted_odd) - 1, 2)
        ]
    else:
        pairs = _min_perfect_matching(odd, dist)
    for a, b in pairs:
        for rid in _reconstruct_path_roads(fm, active_adj, nxt, a, b):
            road = fm.roads[rid]
            edges.append((road.a, road.b, rid))

    walk = _eulerian_walk(fm, start, edges)
    walk_adj: dict[str, list[int]] = {nid: [] for nid in fm.nodes}
    for index, (u, v, _) in enumerate(edges):
        walk_adj[u].append(index)
        walk_adj[v].append(index)
    used = [False] * len(edges)
    steps: list[Step] = []
    for a, b in zip(walk, walk[1:]):
        road_id = None
        for index in walk_adj[a]:
            if used[index]:
                continue
            u, v, candidate = edges[index]
            if (u == a and v == b) or (u == b and v == a):
                road_id = candidate
                used[index] = True
                break
        if road_id is None:
            raise CoverageError(f"欧拉路径重建失败：{a}-{b} 无未用边")
        road = fm.roads[road_id]
        steps.append(Step(len(steps), a, b, road_id, road.dark_tunnel))

    covered = {step.road_id for step in steps}
    covers_all = reachable_required <= covered
    if not covers_all:
        raise CoverageError("重规划未覆盖全部可达剩余道路")
    if steps and steps[0].from_node != start:
        raise CoverageError("重规划起点不一致")
    if steps and steps[-1].to_node != route_end:
        raise CoverageError("重规划终点不一致")
    return ReplanResult(
        tuple(steps),
        frozenset(reachable_required),
        frozenset(unreachable_required),
        frozenset(reachable),
        can_return_to_end,
        route_end,
        covers_all,
    )
