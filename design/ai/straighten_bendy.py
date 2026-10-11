#!/usr/bin/env python3
"""
Find the grid-stepped multi-segment chains left over from the A* fallback
in route_final_gaps.py (short, 0.2mm-grid-aligned segments), group them
into per-net chains by shared endpoints, and replace each chain with the
simplest possible direct connection between its two true endpoints: a
straight line at any angle if clear, otherwise a single arc. Only chains
of 2+ segments are touched (a lone grid-aligned segment is left alone).
"""
import json
import math
import re
from collections import defaultdict

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
BULGE_MAGS = (0.3, 0.6, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 14.0)
PT_TOL = 0.001
GRID = 0.2


def keypt(x, y, tol=PT_TOL):
    return (round(x / tol), round(y / tol))


def is_grid_aligned(v, grid=GRID, tol=0.001):
    return abs(v / grid - round(v / grid)) < tol


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

    all_tracks = list(board.get_tracks())
    plain_tracks = [t for t in all_tracks if not hasattr(t, "mid")]
    arc_tracks = [t for t in all_tracks if hasattr(t, "mid")]

    cx_plain = [t for t in plain_tracks if re.match(r"^C\d+$", t.net.name)]
    grid_tracks = []
    for t in cx_plain:
        sx, sy = t.start.x / 1e6, t.start.y / 1e6
        ex, ey = t.end.x / 1e6, t.end.y / 1e6
        length = math.hypot(ex - sx, ey - sy)
        if (is_grid_aligned(sx) and is_grid_aligned(sy) and is_grid_aligned(ex) and is_grid_aligned(ey)
                and length < 3.0):
            grid_tracks.append(t)

    print(f"Candidate grid-stepped segments: {len(grid_tracks)}")

    # group into chains per net by shared endpoints
    by_net = defaultdict(list)
    for t in grid_tracks:
        by_net[t.net.name].append(t)

    chains = []  # list of (net_name, [track,...], endpoint_a, endpoint_b)
    for net_name, tlist in by_net.items():
        endpoint_count = defaultdict(int)
        adjacency = defaultdict(list)
        for t in tlist:
            sx, sy = t.start.x / 1e6, t.start.y / 1e6
            ex, ey = t.end.x / 1e6, t.end.y / 1e6
            ks, ke = keypt(sx, sy), keypt(ex, ey)
            endpoint_count[ks] += 1
            endpoint_count[ke] += 1
            adjacency[ks].append((t, ke))
            adjacency[ke].append((t, ks))

        visited_tracks = set()
        for t in tlist:
            if t.id.value in visited_tracks:
                continue
            sx, sy = t.start.x / 1e6, t.start.y / 1e6
            ex, ey = t.end.x / 1e6, t.end.y / 1e6
            ks, ke = keypt(sx, sy), keypt(ex, ey)
            # only start chain-walk from a degree-1 endpoint (a true chain end)
            start_key = None
            if endpoint_count[ks] == 1:
                start_key = ks
            elif endpoint_count[ke] == 1:
                start_key = ke
            else:
                continue
            chain_tracks = []

            def coords_for(t, key):
                sx2, sy2 = t.start.x / 1e6, t.start.y / 1e6
                ex2, ey2 = t.end.x / 1e6, t.end.y / 1e6
                return (sx2, sy2) if keypt(sx2, sy2) == key else (ex2, ey2)

            waypoints = [coords_for(t, start_key)]
            cur = start_key
            prev_track_id = None
            guard = 0
            while guard < 200:
                guard += 1
                opts = [(tt, other) for (tt, other) in adjacency[cur] if tt.id.value != prev_track_id]
                step = None
                for tt, other in opts:
                    if tt.id.value not in visited_tracks:
                        step = (tt, other)
                        break
                if step is None:
                    break
                tt, other = step
                chain_tracks.append(tt)
                visited_tracks.add(tt.id.value)
                prev_track_id = tt.id.value
                cur = other
                waypoints.append(coords_for(tt, other))
                if endpoint_count[cur] == 1:
                    break
            if len(chain_tracks) >= 2:
                end_key = cur
                # true coordinates for start_key/end_key
                sxy = None
                exy = None
                first_t = chain_tracks[0]
                fsx, fsy = first_t.start.x / 1e6, first_t.start.y / 1e6
                fex, fey = first_t.end.x / 1e6, first_t.end.y / 1e6
                sxy = (fsx, fsy) if keypt(fsx, fsy) == start_key else (fex, fey)
                last_t = chain_tracks[-1]
                lsx, lsy = last_t.start.x / 1e6, last_t.start.y / 1e6
                lex, ley = last_t.end.x / 1e6, last_t.end.y / 1e6
                exy = (lsx, lsy) if keypt(lsx, lsy) == end_key else (lex, ley)
                chains.append((net_name, chain_tracks, sxy, exy, waypoints))

    print(f"Chains found (>=2 segments): {len(chains)}")
    for net_name, ct, a, b, wp in chains:
        print(f"  {net_name}: {len(ct)} segs, {a} -> {b}")

    # obstacle data (exclude same-net)
    fps = board.get_footprints()
    all_pads = board.get_pads()
    all_pad_entries = []
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
        all_pad_entries.append((x, y, hw, hh, ang, p.net.name))

    live_segs = []
    for t in plain_tracks:
        live_segs.append((t.start.x / 1e6, t.start.y / 1e6, t.end.x / 1e6, t.end.y / 1e6, t.net.name, t.id.value))
    for a in arc_tracks:
        pts = sample_arc((a.start.x / 1e6, a.start.y / 1e6), (a.mid.x / 1e6, a.mid.y / 1e6),
                          (a.end.x / 1e6, a.end.y / 1e6), n=12)
        for i in range(len(pts) - 1):
            live_segs.append((pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], a.net.name, a.id.value))

    replaced = 0
    to_remove_ids = []
    new_items = []

    for net_name, chain_tracks, pa, pb, waypoints in chains:
        chain_ids = {t.id.value for t in chain_tracks}
        obstacle_rects = [(x, y, hw, hh, ang) for (x, y, hw, hh, ang, n) in all_pad_entries if n != net_name]
        obstacle_segs = [(x1, y1, x2, y2) for (x1, y1, x2, y2, n, tid) in live_segs
                          if n != net_name and tid not in chain_ids]

        straight_pts = sample_segment(pa, pb)
        if path_clear(straight_pts, obstacle_rects, obstacle_segs, outline):
            t = Track()
            t.start = Vector2.from_xy_mm(*pa)
            t.end = Vector2.from_xy_mm(*pb)
            t.layer = BoardLayer.BL_F_Cu
            t.width = WIDTH_NM
            t.net = nets_by_name[net_name]
            new_items.append(t)
            to_remove_ids.extend(chain_ids)
            replaced += 1
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
            a.net = nets_by_name[net_name]
            new_items.append(a)
            to_remove_ids.extend(chain_ids)
            replaced += 1
            continue

        # last resort: greedy line-of-sight simplification of the original
        # (already obstacle-clear) waypoint sequence -- walk forward from
        # each kept point to the farthest waypoint still reachable by a
        # clear straight line, at whatever angle that line happens to be.
        simplified = [waypoints[0]]
        i = 0
        while i < len(waypoints) - 1:
            j = len(waypoints) - 1
            step_found = False
            while j > i:
                pts = sample_segment(waypoints[i], waypoints[j])
                if path_clear(pts, obstacle_rects, obstacle_segs, outline):
                    simplified.append(waypoints[j])
                    i = j
                    step_found = True
                    break
                j -= 1
            if not step_found:
                i += 1
                simplified.append(waypoints[i])

        if len(simplified) < len(waypoints):
            for k in range(len(simplified) - 1):
                t1 = Track()
                t1.start = Vector2.from_xy_mm(*simplified[k])
                t1.end = Vector2.from_xy_mm(*simplified[k + 1])
                t1.layer = BoardLayer.BL_F_Cu
                t1.width = WIDTH_NM
                t1.net = nets_by_name[net_name]
                new_items.append(t1)
            to_remove_ids.extend(chain_ids)
            replaced += 1
            print(f"  {net_name} chain simplified from {len(waypoints)-1} to {len(simplified)-1} segments")
        else:
            print(f"  could not simplify {net_name} chain {pa}->{pb}, leaving as-is")

    print(f"\nReplacing {replaced} of {len(chains)} chains")

    if to_remove_ids:
        items = list(board.get_tracks())
        to_remove = [it for it in items if it.id.value in set(to_remove_ids)]
        board.remove_items(to_remove)
        print(f"Removed {len(to_remove)} old segments")

    if new_items:
        created = board.create_items(new_items)
        print(f"Created {len(created)} replacement items")

    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
