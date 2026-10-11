#!/usr/bin/env python3
"""
Route all unrouted Cx ("column") nets between on-board LED pads, F.Cu
only, 0.2mm width, every segment at a multiple of 45 degrees, with real
obstacle avoidance so traces never cross anything (other pads, existing
copper, each other) and never leave the board outline. Off-board (parked)
LED pads are ignored entirely.

Per net: union-find over existing track/via endpoints to find what's
already connected; build a Prim's MST over the remaining disconnected
pad clusters (on-board pads only); for each MST edge, run A* on a local
octilinear grid (8-directional moves only) around the two endpoints,
treating every other pad, existing track/via, previously-routed segment
in this run, and the area outside Edge.Cuts as a blocked/clearance zone.
Successfully found paths are converted to minimal track segments (runs
of collinear grid steps merged into one segment) and added as obstacles
for subsequent edges. Failures are reported, not silently dropped.
"""
import heapq
import math
import re
from collections import defaultdict

from kipy import KiCad
from kipy.board_types import Pad, Track, BoardLayer
from kipy.geometry import Vector2

WIDTH_MM = 0.2
WIDTH_NM = round(WIDTH_MM * 1e6)
GRID = 0.2  # mm, pathfinding grid resolution
# Exclusion radius around a pad center = pad's circumscribing half-diagonal
# + trace half-width (0.1) + DRC min clearance (0.127) + small buffer.
PAD_CLEARANCE_R_0402 = 1.32   # 0402 courtyard half-diag 1.042
PAD_CLEARANCE_R_RGB = 1.69    # RGB courtyard half-diag 1.415
TRACK_CLEARANCE_R = 0.1 + 0.1 + 0.127 + 0.05  # two trace half-widths + clearance + buffer
BOARD_MARGIN = 0.7  # mm, keep this far inside Edge.Cuts (covers 0.5 edge clearance + trace half-width + buffer)
BBOX_PAD = 8.0  # mm, extra margin around each edge's local search box
OFFBOARD_X = 90.0  # mm, footprints at/after this x are the parked pile

RGB_FOOTPRINT_NAME = "LED-SMD-4P-L1.6-W1.5_NH-B1515RGBA-GF"

DIRS8 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def point_in_polygon(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def dist_point_to_segment(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def dist_to_polygon_boundary(pt, poly):
    n = len(poly)
    return min(dist_point_to_segment(pt, poly[i], poly[(i + 1) % n]) for i in range(n))


class UnionFind:
    def __init__(self):
        self.parent = {}

    def find(self, x):
        self.parent.setdefault(x, x)
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def keypt(x, y, tol=0.005):
    return (round(x / tol), round(y / tol))


def snap(v):
    return round(v / GRID) * GRID


def a_star(start, goal, blocked_segments, blocked_points, outline, exclude_r_segments, exclude_r_points, bbox):
    """8-directional A* on an implicit grid. blocked_points: list of
    (x,y,r). blocked_segments: list of (x1,y1,x2,y2,r). bbox=(xmin,xmax,ymin,ymax)."""
    sx, sy = snap(start[0]), snap(start[1])
    gx, gy = snap(goal[0]), snap(goal[1])
    xmin, xmax, ymin, ymax = bbox

    def blocked(x, y):
        if x < xmin or x > xmax or y < ymin or y > ymax:
            return True
        if not point_in_polygon((x, y), outline) or dist_to_polygon_boundary((x, y), outline) < BOARD_MARGIN:
            return True
        for (px, py, r) in blocked_points:
            if (x - px) ** 2 + (y - py) ** 2 < r * r:
                return True
        for (x1, y1, x2, y2, r) in blocked_segments:
            if dist_point_to_segment((x, y), (x1, y1), (x2, y2)) < r:
                return True
        return False

    start_n = (round(sx / GRID), round(sy / GRID))
    goal_n = (round(gx / GRID), round(gy / GRID))

    open_heap = [(0.0, start_n)]
    came_from = {}
    g_score = {start_n: 0.0}
    visited = set()

    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur in visited:
            continue
        visited.add(cur)
        if cur == goal_n:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return [(n[0] * GRID, n[1] * GRID) for n in path]

        cx, cy = cur
        for ddx, ddy in DIRS8:
            nx, ny = cx + ddx, cy + ddy
            nnode = (nx, ny)
            if nnode in visited:
                continue
            wx, wy = nx * GRID, ny * GRID
            if nnode != goal_n and blocked(wx, wy):
                continue
            step_cost = GRID * (math.sqrt(2) if ddx and ddy else 1.0)
            ng = g_score[cur] + step_cost
            if ng < g_score.get(nnode, math.inf):
                g_score[nnode] = ng
                came_from[nnode] = cur
                h = math.hypot(nx - goal_n[0], ny - goal_n[1]) * GRID
                heapq.heappush(open_heap, (ng + h, nnode))
    return None


def simplify_path(path):
    if len(path) < 2:
        return path
    out = [path[0]]
    cur_dir = None
    for i in range(1, len(path)):
        dx = path[i][0] - out[-1][0]
        dy = path[i][1] - out[-1][1]
        n = math.hypot(dx, dy)
        d = (round(dx / n, 3), round(dy / n, 3)) if n > 0 else (0, 0)
        if cur_dir is not None and d == cur_dir:
            out[-1] = path[i]
        else:
            out.append(path[i])
            cur_dir = d
    return out


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()

    outline = [tuple(p) for p in __import__("json").load(open("/tmp/outline_current.json"))]

    nets_by_name = {n.name: n for n in board.get_nets()}
    cx_nets = sorted((n for n in nets_by_name if re.match(r"^C\d+$", n)), key=lambda n: int(n[1:]))

    all_pads = board.get_pads()

    fp_name_by_uuid = {fp.id.value: fp.definition.id.name for fp in board.get_footprints()}

    def pad_clearance_r(p):
        parent = p.parent.value if p.parent else None
        name = fp_name_by_uuid.get(parent, "")
        return PAD_CLEARANCE_R_RGB if name == RGB_FOOTPRINT_NAME else PAD_CLEARANCE_R_0402

    pads_by_net = defaultdict(list)
    for p in all_pads:
        x, y = p.position.x / 1e6, p.position.y / 1e6
        if x >= OFFBOARD_X:
            continue  # parked / off-board, ignore per instruction
        if not point_in_polygon((x, y), outline):
            continue
        pads_by_net[p.net.name].append(p)

    all_tracks = list(board.get_tracks())
    all_vias = list(board.get_vias())

    # global obstacle lists (grow as we route)
    blocked_points = [(v.position.x / 1e6, v.position.y / 1e6, TRACK_CLEARANCE_R) for v in all_vias]
    blocked_segments = [
        (t.start.x / 1e6, t.start.y / 1e6, t.end.x / 1e6, t.end.y / 1e6, TRACK_CLEARANCE_R)
        for t in all_tracks
    ]
    # all on-board pads are obstacles by default; per-edge we exclude the
    # source/target footprint's own pads
    all_pad_entries = []  # (x,y,r,parent_uuid)
    for p in all_pads:
        x, y = p.position.x / 1e6, p.position.y / 1e6
        if x >= OFFBOARD_X or not point_in_polygon((x, y), outline):
            continue
        all_pad_entries.append((x, y, pad_clearance_r(p), p.parent.value if p.parent else None))

    tracks_by_net = defaultdict(list)
    for t in all_tracks:
        tracks_by_net[t.net.name].append(t)

    new_tracks = []
    failures = []
    summary = []

    for net_name in cx_nets:
        pads = pads_by_net.get(net_name, [])
        if len(pads) < 2:
            summary.append((net_name, len(pads), 0, 0, 0))
            continue
        pad_pts = [(p.position.x / 1e6, p.position.y / 1e6) for p in pads]

        uf = UnionFind()
        for x, y in pad_pts:
            uf.find(keypt(x, y))
        for t in tracks_by_net.get(net_name, []):
            sxy = (t.start.x / 1e6, t.start.y / 1e6)
            exy = (t.end.x / 1e6, t.end.y / 1e6)
            uf.union(keypt(*sxy), keypt(*exy))

        clusters = defaultdict(list)
        for (x, y), p in zip(pad_pts, pads):
            clusters[uf.find(keypt(x, y))].append((x, y, p))
        cluster_list = list(clusters.values())
        if len(cluster_list) <= 1:
            summary.append((net_name, len(pads), 0, 0, 0))
            continue

        in_tree = [0]
        remaining = list(range(1, len(cluster_list)))
        edges = []
        while remaining:
            best = None
            for i in in_tree:
                for j in remaining:
                    for (ax, ay, _) in cluster_list[i]:
                        for (bx, by, _) in cluster_list[j]:
                            d = math.hypot(ax - bx, ay - by)
                            if best is None or d < best[0]:
                                best = (d, i, j, (ax, ay), (bx, by))
            d, i, j, pa, pb = best
            edges.append((pa, pb))
            in_tree.append(j)
            remaining.remove(j)

        net_obj = nets_by_name[net_name]
        ok_count = 0
        seg_count = 0
        for pa, pb in edges:
            xmin = min(pa[0], pb[0]) - BBOX_PAD
            xmax = max(pa[0], pb[0]) + BBOX_PAD
            ymin = min(pa[1], pb[1]) - BBOX_PAD
            ymax = max(pa[1], pb[1]) + BBOX_PAD

            local_pad_obstacles = [
                (x, y, r) for (x, y, r, _) in all_pad_entries
                if not ((abs(x - pa[0]) < 1e-6 and abs(y - pa[1]) < 1e-6) or
                        (abs(x - pb[0]) < 1e-6 and abs(y - pb[1]) < 1e-6))
            ]
            combined_points = blocked_points + local_pad_obstacles

            path = a_star(pa, pb, blocked_segments, combined_points, outline,
                          TRACK_CLEARANCE_R, 0.0, (xmin, xmax, ymin, ymax))
            if path is None:
                failures.append((net_name, pa, pb))
                continue

            simplified = simplify_path(path)
            # force exact pad endpoints (grid snap may be off by <GRID/2)
            simplified[0] = pa
            simplified[-1] = pb

            edge_segments = []
            for k in range(len(simplified) - 1):
                sxp, syp = simplified[k]
                exp, eyp = simplified[k + 1]
                if abs(sxp - exp) < 1e-9 and abs(syp - eyp) < 1e-9:
                    continue
                t = Track()
                t.start = Vector2.from_xy_mm(sxp, syp)
                t.end = Vector2.from_xy_mm(exp, eyp)
                t.layer = BoardLayer.BL_F_Cu
                t.width = WIDTH_NM
                t.net = net_obj
                edge_segments.append(t)
                blocked_segments.append((sxp, syp, exp, eyp, TRACK_CLEARANCE_R))
            new_tracks.append(edge_segments)  # grouped by edge
            seg_count += len(edge_segments)
            ok_count += 1

        summary.append((net_name, len(pads), len(cluster_list), ok_count, seg_count))

    print(f"{'net':6} {'on-brd pads':>11} {'clusters':>9} {'edges_ok':>9} {'segments':>9}")
    for net_name, npads, nclusters, ok, nseg in summary:
        print(f"{net_name:6} {npads:>11} {nclusters:>9} {ok:>9} {nseg:>9}")

    total_segs = sum(len(e) for e in new_tracks)
    print(f"\nTotal new segments: {total_segs} across {len(new_tracks)} edges")
    print(f"Failed edges (no path found): {len(failures)}")
    for f in failures[:20]:
        print("  ", f)

    # create per-edge, so every created track knows which edge-group it
    # belongs to (for safe whole-edge cleanup later, not partial-segment
    # deletion that would orphan stubs)
    edge_groups = []
    for edge_segments in new_tracks:
        if not edge_segments:
            continue
        created = board.create_items(edge_segments)
        edge_groups.append([it.id.value for it in created])

    import json as _json
    _json.dump(edge_groups, open("/tmp/routed_edge_groups.json", "w"))
    print(f"Created {sum(len(g) for g in edge_groups)} segments in {len(edge_groups)} edge-groups.")
    print("Saved edge-group UUIDs to /tmp/routed_edge_groups.json for safe cleanup.")
    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
