#!/usr/bin/env python3
"""
Route every net that connects 2+ LED pads (0402 or RGB), covering:
  - the Cx "column" (cathode) nets, now bridging the 0402 matrix to the
    RGB tentacle LEDs that share the same net, not just 0402-to-0402
  - the "row" (anode-index) nets: 13 that link 16 0402 pads each, and 7
    that link 5 RGB pads (one per tentacle) + 1 0402 pad each
Net discovery is dynamic (any net with >=2 LED pads), not name-pattern
matched, so this covers both families without hardcoding net names.

Same rules as the column router: same-net crossings already count as
connected (no redundant links), connection target can be any point along
an existing same-net trace (a real T-tap), F.Cu only, 0.2mm width, and
each link tries straight line -> single arc bulge -> multi-segment
obstacle-avoiding A* path as a last resort.
"""
import heapq
import json
import math
import re

from kipy import KiCad
from kipy.board_types import Track, ArcTrack, BoardLayer
from kipy.geometry import Vector2

WIDTH_MM = 0.2
WIDTH_NM = round(WIDTH_MM * 1e6)
TRACE_HALF = WIDTH_MM / 2
DRC_MIN_CLEARANCE = 0.127
BUFFER = 0.03
PAD_GAP = TRACE_HALF + DRC_MIN_CLEARANCE + BUFFER
TRACK_GAP = 2 * TRACE_HALF + DRC_MIN_CLEARANCE + BUFFER
EDGE_GAP = 0.5 + TRACE_HALF + BUFFER
SAMPLE_STEP = 0.1
BULGE_MAGS = (0.3, 0.6, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0)
PT_TOL = 0.01
TAP_SAMPLE_STEP = 0.5  # mm, resolution for candidate tap points along existing same-net copper
GRID = 0.2
BBOX_PAD = 10.0


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


def keypt(x, y, tol=PT_TOL):
    return (round(x / tol), round(y / tol))


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


def dist_to_segment(p, a, b):
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
    return min(dist_to_segment(pt, poly[i], poly[(i + 1) % n]) for i in range(n))


def point_to_rotated_rect_dist(pt, center, half_w, half_h, angle_deg):
    px, py = pt
    cx, cy = center
    a = math.radians(-angle_deg)
    dx, dy = px - cx, py - cy
    lx = dx * math.cos(a) - dy * math.sin(a)
    ly = dx * math.sin(a) + dy * math.cos(a)
    ex = max(0.0, abs(lx) - half_w)
    ey = max(0.0, abs(ly) - half_h)
    return math.hypot(ex, ey)


def seg_intersect(p1, p2, p3, p4):
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def circle_from_3_points(p1, p2, p3):
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None
    ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / d
    uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    return (ux, uy), r


def sample_arc(start, mid, end, n=60):
    c = circle_from_3_points(start, mid, end)
    if c is None:
        return [start, end]
    (cx, cy), r = c
    a0 = math.atan2(start[1] - cy, start[0] - cx)
    am = math.atan2(mid[1] - cy, mid[0] - cx)
    a1 = math.atan2(end[1] - cy, end[0] - cx)

    def norm(a, ref):
        while a - ref > math.pi:
            a -= 2 * math.pi
        while a - ref < -math.pi:
            a += 2 * math.pi
        return a

    am_n = norm(am, a0)
    a1_n = norm(a1, a0)
    if not (min(a0, a1_n) <= am_n <= max(a0, a1_n)):
        a1_n = a1_n - 2 * math.pi if a1_n > a0 else a1_n + 2 * math.pi
    pts = []
    for i in range(n + 1):
        t = i / n
        a = a0 + (a1_n - a0) * t
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def sample_segment(a, b, step=SAMPLE_STEP):
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(d / step))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]


def path_clear(points, obstacle_rects, obstacle_segs, outline):
    for pt in points:
        if not point_in_polygon(pt, outline) or dist_to_polygon_boundary(pt, outline) < EDGE_GAP:
            return False
        for (cx, cy, hw, hh, ang) in obstacle_rects:
            if point_to_rotated_rect_dist(pt, (cx, cy), hw, hh, ang) < PAD_GAP:
                return False
        for (x1, y1, x2, y2) in obstacle_segs:
            if dist_to_segment(pt, (x1, y1), (x2, y2)) < TRACK_GAP:
                return False
    return True


def snap(v, grid=GRID):
    return round(v / grid) * grid


def a_star(start, goal, obstacle_rects, obstacle_segs, outline, bbox, max_expansions=25000, grid=GRID):
    xmin, xmax, ymin, ymax = bbox

    # pre-filter obstacles to the search bbox (+ margin) ONCE, not per node
    margin = PAD_GAP + 1.0
    obstacle_rects = [
        (cx, cy, hw, hh, ang) for (cx, cy, hw, hh, ang) in obstacle_rects
        if xmin - margin <= cx <= xmax + margin and ymin - margin <= cy <= ymax + margin
    ]
    obstacle_segs = [
        (x1, y1, x2, y2) for (x1, y1, x2, y2) in obstacle_segs
        if not (max(x1, x2) < xmin - margin or min(x1, x2) > xmax + margin or
                max(y1, y2) < ymin - margin or min(y1, y2) > ymax + margin)
    ]

    def blocked(x, y):
        if x < xmin or x > xmax or y < ymin or y > ymax:
            return True
        if not point_in_polygon((x, y), outline) or dist_to_polygon_boundary((x, y), outline) < EDGE_GAP:
            return True
        for (cx, cy, hw, hh, ang) in obstacle_rects:
            if point_to_rotated_rect_dist((x, y), (cx, cy), hw, hh, ang) < PAD_GAP:
                return True
        for (x1, y1, x2, y2) in obstacle_segs:
            if dist_to_segment((x, y), (x1, y1), (x2, y2)) < TRACK_GAP:
                return True
        return False

    DIRS8 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    start_n = (round(snap(start[0], grid) / grid), round(snap(start[1], grid) / grid))
    goal_n = (round(snap(goal[0], grid) / grid), round(snap(goal[1], grid) / grid))

    open_heap = [(0.0, start_n)]
    came_from = {}
    g_score = {start_n: 0.0}
    visited = set()
    expansions = 0
    while open_heap:
        expansions += 1
        if expansions > max_expansions:
            return None
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
            return [(n[0] * grid, n[1] * grid) for n in path]
        cx, cy = cur
        for ddx, ddy in DIRS8:
            nx, ny = cx + ddx, cy + ddy
            nnode = (nx, ny)
            if nnode in visited:
                continue
            wx, wy = nx * grid, ny * grid
            if nnode != goal_n and blocked(wx, wy):
                continue
            step_cost = grid * (math.sqrt(2) if ddx and ddy else 1.0)
            ng = g_score[cur] + step_cost
            if ng < g_score.get(nnode, math.inf):
                g_score[nnode] = ng
                came_from[nnode] = cur
                h = math.hypot(nx - goal_n[0], ny - goal_n[1]) * grid
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

    outline = [tuple(p) for p in json.load(open("/tmp/outline_current.json"))]
    nets_by_name = {n.name: n for n in board.get_nets()}

    LED_FOOTPRINTS = {"LED_0402_1005Metric", "LED-SMD-4P-L1.6-W1.5_NH-B1515RGBA-GF"}

    fps = board.get_footprints()
    fp_name_by_uuid = {fp.id.value: fp.definition.id.name for fp in fps}

    all_pads = board.get_pads()
    all_pad_entries = []
    pads_by_net = {}
    for p in all_pads:
        x, y = p.position.x / 1e6, p.position.y / 1e6
        if x >= 90 or not point_in_polygon((x, y), outline):
            continue
        layer_info = p.padstack.copper_layer(BoardLayer.BL_F_Cu)
        if layer_info is None:
            continue
        size = layer_info.size
        hw, hh = (size.x / 1e6) / 2, (size.y / 1e6) / 2
        ang = p.padstack.angle.degrees
        net = p.net.name
        all_pad_entries.append((x, y, hw, hh, ang, net))
        if fp_name_by_uuid.get(p.parent.value if p.parent else None, "") in LED_FOOTPRINTS:
            pads_by_net.setdefault(net, []).append((x, y))

    # route every net with >=2 LED pads, not just ones matching Cx
    cx_nets = sorted((n for n, plist in pads_by_net.items() if len(plist) >= 2))

    all_tracks = list(board.get_tracks())
    plain_tracks = [t for t in all_tracks if not hasattr(t, "mid")]
    arc_tracks = [t for t in all_tracks if hasattr(t, "mid")]

    live_segs = []  # (x1,y1,x2,y2,net)
    for t in plain_tracks:
        live_segs.append((t.start.x / 1e6, t.start.y / 1e6, t.end.x / 1e6, t.end.y / 1e6, t.net.name))
    for a in arc_tracks:
        pts = sample_arc((a.start.x / 1e6, a.start.y / 1e6), (a.mid.x / 1e6, a.mid.y / 1e6),
                          (a.end.x / 1e6, a.end.y / 1e6), n=12)
        for i in range(len(pts) - 1):
            live_segs.append((pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], a.net.name))

    def other_net_rects(net_name):
        return [(x, y, hw, hh, ang) for (x, y, hw, hh, ang, n) in all_pad_entries if n != net_name]

    def other_net_segments(net_name):
        return [(x1, y1, x2, y2) for (x1, y1, x2, y2, n) in live_segs if n != net_name]

    all_new_items = []
    summary = []

    for net_name in cx_nets:
        pads = pads_by_net.get(net_name, [])
        if len(pads) < 2:
            continue

        same_net_segs = [(x1, y1, x2, y2) for (x1, y1, x2, y2, n) in live_segs if n == net_name]

        uf = UnionFind()
        for x, y in pads:
            uf.find(keypt(x, y))
        for (x1, y1, x2, y2) in same_net_segs:
            uf.union(keypt(x1, y1), keypt(x2, y2))
        n = len(same_net_segs)
        seg_root = list(range(n))

        def find_seg(i):
            while seg_root[i] != i:
                seg_root[i] = seg_root[seg_root[i]]
                i = seg_root[i]
            return i

        for i in range(n):
            for j in range(i + 1, n):
                x1, y1, x2, y2 = same_net_segs[i]
                x3, y3, x4, y4 = same_net_segs[j]
                if seg_intersect((x1, y1), (x2, y2), (x3, y3), (x4, y4)):
                    uf.union(keypt(x1, y1), keypt(x3, y3))

        clusters = {}
        for (x, y) in pads:
            clusters.setdefault(uf.find(keypt(x, y)), []).append(("pad", x, y))
        # add tap points sampled along this cluster's own same-net copper
        for (x1, y1, x2, y2) in same_net_segs:
            root = uf.find(keypt(x1, y1))
            if root not in clusters:
                continue
            for (tx, ty) in sample_segment((x1, y1), (x2, y2), step=TAP_SAMPLE_STEP):
                clusters[root].append(("tap", tx, ty))

        cluster_list = list(clusters.values())
        if len(cluster_list) <= 1:
            continue

        in_tree = [0]
        remaining = list(range(1, len(cluster_list)))
        mst_edges = []
        while remaining:
            best = None
            for i in in_tree:
                for j in remaining:
                    for (_, ax, ay) in cluster_list[i]:
                        for (_, bx, by) in cluster_list[j]:
                            d = math.hypot(ax - bx, ay - by)
                            if best is None or d < best[0]:
                                best = (d, i, j, (ax, ay), (bx, by))
            d, i, j, pa, pb = best
            mst_edges.append((pa, pb))
            in_tree.append(j)
            remaining.remove(j)

        net_obj = nets_by_name[net_name]
        obstacle_rects = other_net_rects(net_name)
        obstacle_segs = other_net_segments(net_name)

        ok = 0
        fail = 0
        net_new_items = []
        method_counts = {"straight": 0, "arc": 0, "multi": 0}
        for pa, pb in mst_edges:
            straight_pts = sample_segment(pa, pb)
            if path_clear(straight_pts, obstacle_rects, obstacle_segs, outline):
                t = Track()
                t.start = Vector2.from_xy_mm(*pa)
                t.end = Vector2.from_xy_mm(*pb)
                t.layer = BoardLayer.BL_F_Cu
                t.width = WIDTH_NM
                t.net = net_obj
                all_new_items.append(t)
                net_new_items.append(t)
                live_segs.append((pa[0], pa[1], pb[0], pb[1], net_name))
                ok += 1
                method_counts["straight"] += 1
                continue

            mx, my = (pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2
            dx, dy = pb[0] - pa[0], pb[1] - pa[1]
            length = math.hypot(dx, dy)
            found = None
            if length > 1e-9:
                perp = (-dy / length, dx / length)
                for mag in BULGE_MAGS:
                    for sign in (1, -1):
                        mid = (mx + perp[0] * mag * sign, my + perp[1] * mag * sign)
                        arc_pts = sample_arc(pa, mid, pb)
                        if path_clear(arc_pts, obstacle_rects, obstacle_segs, outline):
                            found = mid
                            break
                    if found:
                        break
            if found is not None:
                a = ArcTrack()
                a.start = Vector2.from_xy_mm(*pa)
                a.mid = Vector2.from_xy_mm(*found)
                a.end = Vector2.from_xy_mm(*pb)
                a.layer = BoardLayer.BL_F_Cu
                a.width = WIDTH_NM
                a.net = net_obj
                all_new_items.append(a)
                net_new_items.append(a)
                arc_pts2 = sample_arc(pa, found, pb, n=12)
                for i in range(len(arc_pts2) - 1):
                    live_segs.append((arc_pts2[i][0], arc_pts2[i][1], arc_pts2[i + 1][0], arc_pts2[i + 1][1], net_name))
                ok += 1
                method_counts["arc"] += 1
                continue

            # multi-segment straight-line fallback via A*. Use a coarser
            # grid for longer spans -- keeps the search tractable (a fine
            # grid over a 20-30mm span with a dense board can take tens of
            # seconds just to fail).
            xmin = min(pa[0], pb[0]) - BBOX_PAD
            xmax = max(pa[0], pb[0]) + BBOX_PAD
            ymin = min(pa[1], pb[1]) - BBOX_PAD
            ymax = max(pa[1], pb[1]) + BBOX_PAD
            search_grid = 0.2 if length < 8 else (0.3 if length < 16 else 0.5)
            raw_path = a_star(pa, pb, obstacle_rects, obstacle_segs, outline, (xmin, xmax, ymin, ymax),
                               max_expansions=25000, grid=search_grid)
            if raw_path is None:
                fail += 1
                continue
            simp = simplify_path(raw_path)
            simp[0] = pa
            simp[-1] = pb
            for k in range(len(simp) - 1):
                sxp, syp = simp[k]
                exp, eyp = simp[k + 1]
                if abs(sxp - exp) < 1e-9 and abs(syp - eyp) < 1e-9:
                    continue
                t = Track()
                t.start = Vector2.from_xy_mm(sxp, syp)
                t.end = Vector2.from_xy_mm(exp, eyp)
                t.layer = BoardLayer.BL_F_Cu
                t.width = WIDTH_NM
                t.net = net_obj
                all_new_items.append(t)
                net_new_items.append(t)
                live_segs.append((sxp, syp, exp, eyp, net_name))
            ok += 1
            method_counts["multi"] += 1

        summary.append((net_name, len(pads), len(cluster_list), ok, fail, method_counts))
        if net_new_items:
            board.create_items(net_new_items)
        print(f"{net_name}: {len(cluster_list)} clusters, {ok} routed ({method_counts}), {fail} failed", flush=True)

    total_ok = sum(s[3] for s in summary)
    total_fail = sum(s[4] for s in summary)
    print(f"\nTotal: {total_ok} routed, {total_fail} failed")
    print(f"Created {len(all_new_items)} items (created incrementally per-net during the run).")
    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
