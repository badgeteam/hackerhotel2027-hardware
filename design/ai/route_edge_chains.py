#!/usr/bin/env python3
"""
Same approach as route_c1_chain.py / route_all_chains.py, but for the
0402 LEDs placed around the board edge (the inset-contour ring) instead
of the center circle matrix. Nearest-neighbor chain per net, straight
lines where clear, single circular-arc bulge fallback. F.Cu only, 0.2mm
width. All 16 Cx nets processed; existing tracks (including the circle
matrix routing already done) are obstacles from the start.
"""
import json
import math

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
CIRCLE_R_MAX = 18.5
SAMPLE_STEP = 0.1
BULGE_MAGS = (0.3, 0.6, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0)


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


def path_clear(points, obstacles_rect, obstacles_seg, outline):
    for pt in points:
        if not point_in_polygon(pt, outline) or dist_to_polygon_boundary(pt, outline) < EDGE_GAP:
            return False
        for (cx, cy, hw, hh, ang) in obstacles_rect:
            if point_to_rotated_rect_dist(pt, (cx, cy), hw, hh, ang) < PAD_GAP:
                return False
        for (x1, y1, x2, y2) in obstacles_seg:
            if dist_to_segment(pt, (x1, y1), (x2, y2)) < TRACK_GAP:
                return False
    return True


def route_net(net_name, net_obj, pads_for_net, all_pad_rects, obstacles_seg, outline):
    """pads_for_net: list of (x, y). all_pad_rects: list of (x,y,hw,hh,ang)
    for every on-board pad (any net). obstacles_seg mutated in place."""
    remaining = list(range(len(pads_for_net)))
    if not remaining:
        return [], [], 0
    start_idx = min(remaining, key=lambda i: math.hypot(*pads_for_net[i]))
    chain = [start_idx]
    remaining.remove(start_idx)
    while remaining:
        cx, cy = pads_for_net[chain[-1]]
        nxt = min(remaining, key=lambda i: math.hypot(pads_for_net[i][0] - cx, pads_for_net[i][1] - cy))
        chain.append(nxt)
        remaining.remove(nxt)

    new_items = []
    failures = []
    for k in range(len(chain) - 1):
        pa = pads_for_net[chain[k]]
        pb = pads_for_net[chain[k + 1]]

        local_rects = [(x, y, hw, hh, ang) for (x, y, hw, hh, ang) in all_pad_rects
                       if not ((abs(x - pa[0]) < 1e-6 and abs(y - pa[1]) < 1e-6) or
                               (abs(x - pb[0]) < 1e-6 and abs(y - pb[1]) < 1e-6))]

        straight_pts = sample_segment(pa, pb)
        if path_clear(straight_pts, local_rects, obstacles_seg, outline):
            t = Track()
            t.start = Vector2.from_xy_mm(*pa)
            t.end = Vector2.from_xy_mm(*pb)
            t.layer = BoardLayer.BL_F_Cu
            t.width = WIDTH_NM
            t.net = net_obj
            new_items.append(t)
            obstacles_seg.append((pa[0], pa[1], pb[0], pb[1]))
            continue

        mx, my = (pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            failures.append((pa, pb, "zero-length"))
            continue
        perp = (-dy / length, dx / length)

        found = None
        for mag in BULGE_MAGS:
            for sign in (1, -1):
                mid = (mx + perp[0] * mag * sign, my + perp[1] * mag * sign)
                arc_pts = sample_arc(pa, mid, pb)
                if path_clear(arc_pts, local_rects, obstacles_seg, outline):
                    found = mid
                    break
            if found:
                break

        if found is None:
            failures.append((pa, pb, "no clear path found"))
            continue

        a = ArcTrack()
        a.start = Vector2.from_xy_mm(*pa)
        a.mid = Vector2.from_xy_mm(*found)
        a.end = Vector2.from_xy_mm(*pb)
        a.layer = BoardLayer.BL_F_Cu
        a.width = WIDTH_NM
        a.net = net_obj
        new_items.append(a)
        arc_pts = sample_arc(pa, found, pb, n=8)
        for i in range(len(arc_pts) - 1):
            obstacles_seg.append((arc_pts[i][0], arc_pts[i][1], arc_pts[i + 1][0], arc_pts[i + 1][1]))

    return new_items, failures, len(chain) - 1


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()

    outline = [tuple(p) for p in json.load(open("/tmp/outline_current.json"))]

    nets_by_name = {n.name: n for n in board.get_nets()}
    import re
    cx_nets = sorted((n for n in nets_by_name if re.match(r"^C\d+$", n)), key=lambda n: int(n[1:]))

    fps = board.get_footprints()
    fp_name_by_uuid = {fp.id.value: fp.definition.id.name for fp in fps}

    all_pads = board.get_pads()
    all_pad_rects = []
    pads_by_net_circle = {}
    for p in all_pads:
        x, y = p.position.x / 1e6, p.position.y / 1e6
        if x >= 90 or not point_in_polygon((x, y), outline):
            continue
        layer_info = p.padstack.copper_layer(BoardLayer.BL_F_Cu)
        size = layer_info.size
        hw, hh = (size.x / 1e6) / 2, (size.y / 1e6) / 2
        ang = p.padstack.angle.degrees
        all_pad_rects.append((x, y, hw, hh, ang))

        parent = p.parent.value if p.parent else None
        if fp_name_by_uuid.get(parent, "") != "LED_0402_1005Metric":
            continue
        if math.hypot(x, y) <= CIRCLE_R_MAX:
            continue  # skip circle-matrix LEDs, only want the edge/contour ones
        pads_by_net_circle.setdefault(p.net.name, []).append((x, y))

    all_tracks = list(board.get_tracks())
    obstacles_seg = [(t.start.x / 1e6, t.start.y / 1e6, t.end.x / 1e6, t.end.y / 1e6) for t in all_tracks]

    all_new_items = []
    summary = []
    for net_name in cx_nets:
        pads_for_net = pads_by_net_circle.get(net_name, [])
        new_items, failures, total_links = route_net(
            net_name, nets_by_name[net_name], pads_for_net, all_pad_rects, obstacles_seg, outline
        )
        all_new_items.extend(new_items)
        summary.append((net_name, len(pads_for_net), total_links, len(new_items), len(failures)))
        print(f"{net_name}: {len(pads_for_net)} pads, {len(new_items)}/{total_links} links routed, {len(failures)} failed")

    print(f"\n{'net':6}{'pads':>6}{'links':>7}{'routed':>8}{'failed':>8}")
    for net_name, npads, total, ok, fail in summary:
        print(f"{net_name:6}{npads:>6}{total:>7}{ok:>8}{fail:>8}")

    total_ok = sum(s[3] for s in summary)
    total_fail = sum(s[4] for s in summary)
    print(f"\nTotal: {total_ok} routed, {total_fail} failed, {len(all_new_items)} new track/arc items")

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
