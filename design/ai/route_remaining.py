#!/usr/bin/env python3
"""
Continue routing the Cx nets' 0402 LEDs (circle + edge/contour together),
on top of whatever's already on the board -- nothing existing is removed
or re-routed.

Router change from before: traces on the SAME net are no longer treated
as obstacles for each other (same-net copper is allowed to cross/overlap
-- it's already electrically one node). Only different-net pads/traces
are hard obstacles. Before adding any new link, existing same-net
connectivity is computed (shared endpoints AND geometric crossings
between same-net segments both count as "connected"), and only pad
clusters that are still disconnected from each other get a new link --
no redundant connections where crossings already joined things.

Per net: union-find the existing same-net tracks/arcs (by endpoint match
and by segment-segment crossing) to find current pad clusters -> MST over
remaining clusters (closest pad pair) -> each new link tries a straight
line first (checked against other-net obstacles only), then a single
arc bulge. F.Cu only, 0.2mm width.
"""
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
PT_TOL = 0.01  # mm, endpoint coincidence tolerance


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


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()

    outline = [tuple(p) for p in json.load(open("/tmp/outline_current.json"))]
    nets_by_name = {n.name: n for n in board.get_nets()}
    cx_nets = sorted((n for n in nets_by_name if re.match(r"^C\d+$", n)), key=lambda n: int(n[1:]))

    fps = board.get_footprints()
    fp_name_by_uuid = {fp.id.value: fp.definition.id.name for fp in fps}

    all_pads = board.get_pads()
    pad_rects_by_net = {}  # net -> list of (x,y,hw,hh,ang)
    pads_by_net = {}       # net -> list of (x,y)
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
        pad_rects_by_net.setdefault(net, []).append((x, y, hw, hh, ang))
        # Only 0402 LED pads are actual routing endpoints; every on-board
        # pad (RGB LEDs, U1, J1, ...) still counts as an obstacle for
        # other nets via pad_rects_by_net above.
        parent = p.parent.value if p.parent else None
        if fp_name_by_uuid.get(parent, "") == "LED_0402_1005Metric":
            pads_by_net.setdefault(net, []).append((x, y))

    # global "other-net" obstacle rects (all 0402 pads, tagged by net)
    all_pad_entries = []
    for net, rects in pad_rects_by_net.items():
        for (x, y, hw, hh, ang) in rects:
            all_pad_entries.append((x, y, hw, hh, ang, net))

    all_tracks = list(board.get_tracks())  # get_tracks() returns both Track and ArcTrack
    plain_tracks = [t for t in all_tracks if not hasattr(t, "mid")]
    arc_tracks = [t for t in all_tracks if hasattr(t, "mid")]

    def seg_list_for(net_name):
        segs = []
        for t in plain_tracks:
            if t.net.name != net_name:
                continue
            segs.append((t.start.x / 1e6, t.start.y / 1e6, t.end.x / 1e6, t.end.y / 1e6))
        for a in arc_tracks:
            if a.net.name != net_name:
                continue
            pts = sample_arc((a.start.x / 1e6, a.start.y / 1e6), (a.mid.x / 1e6, a.mid.y / 1e6),
                              (a.end.x / 1e6, a.end.y / 1e6), n=12)
            for i in range(len(pts) - 1):
                segs.append((pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1]))
        return segs

    # Live list of (x1,y1,x2,y2,net_name) for every existing segment, grown
    # as new tracks/arcs are created during this run so later nets avoid
    # earlier nets' brand-new copper too, not just what existed at startup.
    live_segs = []
    for t in plain_tracks:
        live_segs.append((t.start.x / 1e6, t.start.y / 1e6, t.end.x / 1e6, t.end.y / 1e6, t.net.name))
    for a in arc_tracks:
        pts = sample_arc((a.start.x / 1e6, a.start.y / 1e6), (a.mid.x / 1e6, a.mid.y / 1e6),
                          (a.end.x / 1e6, a.end.y / 1e6), n=12)
        for i in range(len(pts) - 1):
            live_segs.append((pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], a.net.name))

    def other_net_segments(net_name):
        return [(x1, y1, x2, y2) for (x1, y1, x2, y2, n) in live_segs if n != net_name]

    all_new_items = []
    summary = []

    for net_name in cx_nets:
        pads = pads_by_net.get(net_name, [])
        if len(pads) < 2:
            summary.append((net_name, len(pads), 0, 0, 0))
            continue

        same_net_segs = seg_list_for(net_name)

        uf = UnionFind()
        for x, y in pads:
            uf.find(keypt(x, y))
        # union by shared endpoints
        for (x1, y1, x2, y2) in same_net_segs:
            uf.union(keypt(x1, y1), keypt(x2, y2))
        # union pads that coincide with any segment endpoint
        endpoint_keys = []
        for (x1, y1, x2, y2) in same_net_segs:
            endpoint_keys.append(keypt(x1, y1))
            endpoint_keys.append(keypt(x2, y2))
        # union by segment-segment crossing
        n = len(same_net_segs)
        seg_root = list(range(n))

        def find_seg(i):
            while seg_root[i] != i:
                seg_root[i] = seg_root[seg_root[i]]
                i = seg_root[i]
            return i

        def union_seg(i, j):
            ri, rj = find_seg(i), find_seg(j)
            if ri != rj:
                seg_root[ri] = rj

        for i in range(n):
            for j in range(i + 1, n):
                x1, y1, x2, y2 = same_net_segs[i]
                x3, y3, x4, y4 = same_net_segs[j]
                if seg_intersect((x1, y1), (x2, y2), (x3, y3), (x4, y4)):
                    union_seg(i, j)
                    uf.union(keypt(x1, y1), keypt(x3, y3))  # merge their point-space too

        clusters = {}
        for (x, y) in pads:
            clusters.setdefault(uf.find(keypt(x, y)), []).append((x, y))
        cluster_list = list(clusters.values())

        if len(cluster_list) <= 1:
            summary.append((net_name, len(pads), 1, 0, 0))
            continue

        in_tree = [0]
        remaining = list(range(1, len(cluster_list)))
        mst_edges = []
        while remaining:
            best = None
            for i in in_tree:
                for j in remaining:
                    for (ax, ay) in cluster_list[i]:
                        for (bx, by) in cluster_list[j]:
                            d = math.hypot(ax - bx, ay - by)
                            if best is None or d < best[0]:
                                best = (d, i, j, (ax, ay), (bx, by))
            d, i, j, pa, pb = best
            mst_edges.append((pa, pb))
            in_tree.append(j)
            remaining.remove(j)

        net_obj = nets_by_name[net_name]
        obstacle_rects = [(x, y, hw, hh, ang) for (x, y, hw, hh, ang, nname) in all_pad_entries if nname != net_name]
        obstacle_segs = other_net_segments(net_name)

        ok = 0
        fail = 0
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
                live_segs.append((pa[0], pa[1], pb[0], pb[1], net_name))
                ok += 1
                continue

            mx, my = (pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2
            dx, dy = pb[0] - pa[0], pb[1] - pa[1]
            length = math.hypot(dx, dy)
            if length < 1e-9:
                fail += 1
                continue
            perp = (-dy / length, dx / length)
            found = None
            for mag in BULGE_MAGS:
                for sign in (1, -1):
                    mid = (mx + perp[0] * mag * sign, my + perp[1] * mag * sign)
                    arc_pts = sample_arc(pa, mid, pb)
                    if path_clear(arc_pts, obstacle_rects, obstacle_segs, outline):
                        found = mid
                        break
                if found:
                    break
            if found is None:
                fail += 1
                continue
            a = ArcTrack()
            a.start = Vector2.from_xy_mm(*pa)
            a.mid = Vector2.from_xy_mm(*found)
            a.end = Vector2.from_xy_mm(*pb)
            a.layer = BoardLayer.BL_F_Cu
            a.width = WIDTH_NM
            a.net = net_obj
            all_new_items.append(a)
            arc_pts_for_obstacle = sample_arc(pa, found, pb, n=12)
            for i in range(len(arc_pts_for_obstacle) - 1):
                live_segs.append((arc_pts_for_obstacle[i][0], arc_pts_for_obstacle[i][1],
                                   arc_pts_for_obstacle[i + 1][0], arc_pts_for_obstacle[i + 1][1], net_name))
            ok += 1

        summary.append((net_name, len(pads), len(cluster_list), ok, fail))
        print(f"{net_name}: {len(pads)} pads, {len(cluster_list)} clusters, {ok} routed, {fail} failed")

    print(f"\n{'net':6}{'pads':>6}{'clust':>7}{'routed':>8}{'failed':>8}")
    for net_name, npads, nclust, ok, fail in summary:
        print(f"{net_name:6}{npads:>6}{nclust:>7}{ok:>8}{fail:>8}")

    print(f"\nTotal new items: {len(all_new_items)}")

    if all_new_items:
        CHUNK = 150
        created_total = 0
        for i in range(0, len(all_new_items), CHUNK):
            created = board.create_items(all_new_items[i:i + CHUNK])
            created_total += len(created)
        print(f"Created {created_total} items.")
    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
