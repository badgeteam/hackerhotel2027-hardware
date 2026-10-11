#!/usr/bin/env python3
"""
Route net C1's center-matrix (circle) 0402 LEDs as a nearest-neighbor
chain: start at one pad, connect to the closest unvisited pad, repeat
until all are visited. Straight lines where clear (checked against the
*actual* pad rectangles and existing copper, not a conservative circle),
falling back to a single circular-arc bulge around the blocking obstacle
when a straight line isn't clear. F.Cu only, 0.2mm width.

Scope: only 0402 LEDs within ~18.5mm of board center (0,0) -- i.e. the
ring/circle matrix, not the contour or tentacle LEDs.
"""
import math

from kipy import KiCad
from kipy.board_types import Track, ArcTrack, BoardLayer
from kipy.geometry import Vector2

WIDTH_MM = 0.2
WIDTH_NM = round(WIDTH_MM * 1e6)
TRACE_HALF = WIDTH_MM / 2
DRC_MIN_CLEARANCE = 0.127
BUFFER = 0.03
PAD_GAP = TRACE_HALF + DRC_MIN_CLEARANCE + BUFFER          # trace edge to pad edge
TRACK_GAP = 2 * TRACE_HALF + DRC_MIN_CLEARANCE + BUFFER     # trace edge to trace edge
EDGE_GAP = 0.5 + TRACE_HALF + BUFFER                         # trace edge to board edge
CIRCLE_R_MAX = 18.5
SAMPLE_STEP = 0.1  # mm, collision-sampling resolution along candidate paths


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
    # ensure mid lies between a0 and a1 along the chosen direction
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


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()

    import json
    outline = [tuple(p) for p in json.load(open("/tmp/outline_current.json"))]

    nets_by_name = {n.name: n for n in board.get_nets()}
    net_obj = nets_by_name["C1"]

    fps = board.get_footprints()
    fp_name_by_uuid = {fp.id.value: fp.definition.id.name for fp in fps}

    all_pads = board.get_pads()
    obstacles_rect = []
    for p in all_pads:
        x, y = p.position.x / 1e6, p.position.y / 1e6
        if x >= 90 or not point_in_polygon((x, y), outline):
            continue
        layer_info = p.padstack.copper_layer(BoardLayer.BL_F_Cu)
        size = layer_info.size
        hw, hh = (size.x / 1e6) / 2, (size.y / 1e6) / 2
        ang = p.padstack.angle.degrees
        obstacles_rect.append((x, y, hw, hh, ang, p))

    c1_pads = []
    for (x, y, hw, hh, ang, p) in obstacles_rect:
        if p.net.name != "C1":
            continue
        parent = p.parent.value if p.parent else None
        if fp_name_by_uuid.get(parent, "") != "LED_0402_1005Metric":
            continue
        if math.hypot(x, y) > CIRCLE_R_MAX:
            continue
        c1_pads.append((x, y, p))

    print(f"C1 circle-0402 pads: {len(c1_pads)}")

    all_tracks = list(board.get_tracks())
    obstacles_seg = [(t.start.x / 1e6, t.start.y / 1e6, t.end.x / 1e6, t.end.y / 1e6) for t in all_tracks]

    # nearest-neighbor chain
    remaining = list(range(len(c1_pads)))
    start_idx = min(remaining, key=lambda i: math.hypot(c1_pads[i][0], c1_pads[i][1]))
    chain = [start_idx]
    remaining.remove(start_idx)
    while remaining:
        cx, cy, _ = c1_pads[chain[-1]]
        nxt = min(remaining, key=lambda i: math.hypot(c1_pads[i][0] - cx, c1_pads[i][1] - cy))
        chain.append(nxt)
        remaining.remove(nxt)

    print("Chain order (pad coords):")
    for i in chain:
        print("  ", c1_pads[i][0], c1_pads[i][1])

    new_items = []
    failures = []

    for k in range(len(chain) - 1):
        ia, ib = chain[k], chain[k + 1]
        pa = (c1_pads[ia][0], c1_pads[ia][1])
        pb = (c1_pads[ib][0], c1_pads[ib][1])

        local_rects = [(x, y, hw, hh, ang) for (x, y, hw, hh, ang, p) in obstacles_rect
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

        # try arc bulge, both directions, increasing magnitude
        mx, my = (pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            failures.append((pa, pb, "zero-length"))
            continue
        perp = (-dy / length, dx / length)

        found = None
        for mag in (0.3, 0.6, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0):
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
        # approximate the arc as a few segments for future obstacle checks
        arc_pts = sample_arc(pa, found, pb, n=8)
        for i in range(len(arc_pts) - 1):
            obstacles_seg.append((arc_pts[i][0], arc_pts[i][1], arc_pts[i + 1][0], arc_pts[i + 1][1]))

    print(f"\nRouted {len(new_items)} of {len(chain) - 1} links, {len(failures)} failed")
    for f in failures:
        print("  FAILED:", f)

    if new_items:
        created = board.create_items(new_items)
        print(f"Created {len(created)} items.")
    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
