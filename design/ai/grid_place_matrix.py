#!/usr/bin/env python3
"""
Rebuild the 0402 LED matrix as a true Cartesian grid (straight rows/columns)
instead of the old dartboard/radial scheme, while keeping the outer
silhouette roughly circular.

Net topology (discovered live from the board, not hardcoded by position):
  - 16 column nets (C1..C16, cathode side, pad '1'). Every column net is
    present in every one of 17 "pure" row nets -> a complete 16x17
    rectangle (272 LEDs).
  - 17 pure row nets (anode side, pad '2'): '/R1b'..'/R7b' and the
    'Net-(Dnnn-A)' auto-named ones, 16 members each.
  - 7 "bridge" row nets (Net-(D308-A)..Net-(D314-A)), each with exactly
    ONE 0402 pad -- always on column C16 -- plus pads on all 5 RGB
    tentacles (shared per-position addressing across tentacles). These
    279 - 272 = 7 extra LEDs only ever sit on column C16.

Plan (per user instruction):
  - All 0402 LEDs get orientation 0 degrees (puts pad 1 on the left,
    pad 2 on the right of each LED body -- so same-column pads line up
    in a straight vertical line, same-row pads line up in a straight
    horizontal line).
  - Lay the 16x17 rectangle out on a uniform-pitch grid, with C16 placed
    at the centermost column (so it has the most vertical room) and its
    7 extra bridge-row LEDs extended straight off one end of that column.
  - Apply a circular mask (radius R from board center) over the
    rectangle: cells outside R are "left away" -- parked off-board in
    the same (150,80)-ish holding pen used earlier in this project,
    exactly as the user said is fine for cells that would otherwise ruin
    the circular silhouette or collide with the RGB tentacles.
  - Verify real pad-to-pad clearance against every RGB tentacle pad for
    every placed cell; park any violators too.

Uses plain update_items() (not begin_commit/push_commit -- that hangs this
KiCad 10.0.6 IPC server).
"""
import json
import math

from kipy import KiCad
from kipy.board_types import Pad, BoardLayer
from kipy.geometry import Vector2, Angle

MONO_FOOTPRINT_NAME = "LED_0402_1005Metric"
RGB_NAME_HINT = "NH-B1515RGBA-GF"

PITCH = 2.0          # mm, both axes
N_COLS = 16
N_ROWS = 17
C16_COL_IDX = 8       # centermost column slot (0..15)
MASK_RADIUS = 16.0    # mm from board center; cells farther than this are parked
GRID_CENTER = (0.0, 0.0)

MIN_PAD_CLEARANCE = 0.25  # mm, copper-to-copper safety margin

PARK_ORIGIN = (150.0, 80.0)
PARK_COLS = 11
PARK_PITCH = 3.0

OUTLINE_FILE = "/tmp/outline_current.json"


def point_in_poly(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xin:
                inside = not inside
    return inside


def pads_of(fp):
    return [p for p in fp.definition.items if isinstance(p, Pad)]


def rotated_rect_corners(cx, cy, w, h, angle_deg):
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    hw, hh = w / 2, h / 2
    pts = []
    for sx, sy in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)):
        rx = sx * ca - sy * sa
        ry = sx * sa + sy * ca
        pts.append((cx + rx, cy + ry))
    return pts


def point_to_seg_dist(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def point_to_rotated_rect_dist(pt, cx, cy, w, h, angle_deg):
    corners = rotated_rect_corners(cx, cy, w, h, angle_deg)
    # If point is inside the rect, distance is 0 (shouldn't happen for valid layouts).
    # Simple check via edges' min distance (good enough; these shapes are small/convex).
    dmin = min(
        point_to_seg_dist(pt, corners[i], corners[(i + 1) % 4]) for i in range(4)
    )
    return dmin


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()
    print("Board:", board.name)

    outline = json.load(open(OUTLINE_FILE))

    fps = board.get_footprints()
    mono_fps = [fp for fp in fps if fp.definition.id.name == MONO_FOOTPRINT_NAME]
    rgb_fps = [fp for fp in fps if RGB_NAME_HINT in fp.definition.id.name]
    print(f"mono={len(mono_fps)} rgb={len(rgb_fps)}")

    # --- RGB pad geometry, for real clearance checks -----------------------
    rgb_pads = []  # (cx, cy, w, h, angle_deg)
    for fp in rgb_fps:
        rot = fp.orientation.degrees
        for p in pads_of(fp):
            sz = p.padstack.copper_layer(BoardLayer.BL_F_Cu).size
            rgb_pads.append((p.position.x / 1e6, p.position.y / 1e6, sz.x / 1e6, sz.y / 1e6, rot))

    # --- Net topology discovery ---------------------------------------------
    mapping = {}  # ref -> (col_net, row_net)
    for fp in mono_fps:
        nets = {p.number: p.net.name for p in pads_of(fp)}
        mapping[fp.reference_field.text.value] = (nets.get("1"), nets.get("2"))

    col_nets = sorted(set(c for c, r in mapping.values()), key=lambda s: int(s[1:]))
    row_nets_all = set(r for c, r in mapping.values())
    bridge_rows = sorted(r for r in row_nets_all
                          if sum(1 for c, rr in mapping.values() if rr == r) == 1)
    pure_rows = sorted(r for r in row_nets_all if r not in bridge_rows)
    assert len(col_nets) == N_COLS and len(pure_rows) == N_ROWS and len(bridge_rows) == 7

    # column index assignment: C16 at the centermost slot, others fill the rest in order
    col_index = {}
    other_cols = [c for c in col_nets if c != "C16"]
    slots = [i for i in range(N_COLS) if i != C16_COL_IDX]
    for c, idx in zip(other_cols, slots):
        col_index[c] = idx
    col_index["C16"] = C16_COL_IDX

    row_index = {r: i for i, r in enumerate(pure_rows)}

    def grid_xy(col_idx, row_idx):
        x = GRID_CENTER[0] + (col_idx - (N_COLS - 1) / 2) * PITCH
        y = GRID_CENTER[1] + (row_idx - (N_ROWS - 1) / 2) * PITCH
        return x, y

    def rgb_clearance_ok(x, y):
        # mono LED at rotation 0: pads at local (+-0.485, 0), size ~0.59x0.64
        for sign in (-1, 1):
            px, py = x + sign * 0.485, y
            for (rx, ry, rw, rh, rang) in rgb_pads:
                d = point_to_rotated_rect_dist((px, py), rx, ry, rw, rh, rang)
                if d < MIN_PAD_CLEARANCE + 0.3:  # 0.3 ~ mono pad half-width
                    return False
        return True

    # --- Decide placement for every mono LED --------------------------------
    placements = {}  # ref -> (x, y) or None (-> park)
    tail_candidates = []  # refs of bridge-row LEDs, to place as a tail off C16

    for ref, (c, r) in mapping.items():
        if r in bridge_rows:
            tail_candidates.append(ref)
            continue
        ci, ri = col_index[c], row_index[r]
        x, y = grid_xy(ci, ri)
        d = math.hypot(x - GRID_CENTER[0], y - GRID_CENTER[1])
        if d <= MASK_RADIUS and point_in_poly((x, y), outline) and rgb_clearance_ok(x, y):
            placements[ref] = (x, y)
        else:
            placements[ref] = None  # parked

    # Tail: continue straight off column C16 in +Y, as far as it stays inside
    # the outline with real RGB clearance.
    c16_x, _ = grid_xy(C16_COL_IDX, 0)
    tail_candidates_sorted = sorted(tail_candidates)  # deterministic order
    row_cursor = N_ROWS  # continue past the last standard row index (16 -> 17, 18, ...)
    for ref in tail_candidates_sorted:
        x, y = grid_xy(C16_COL_IDX, row_cursor)
        if point_in_poly((x, y), outline) and rgb_clearance_ok(x, y):
            placements[ref] = (x, y)
        else:
            placements[ref] = None
        row_cursor += 1

    placed = {ref: xy for ref, xy in placements.items() if xy is not None}
    parked = [ref for ref, xy in placements.items() if xy is None]
    print(f"placed={len(placed)} parked={len(parked)}")

    fp_by_ref = {fp.reference_field.text.value: fp for fp in mono_fps}

    updated = []
    for ref, (x, y) in placed.items():
        fp = fp_by_ref[ref]
        fp.position = Vector2.from_xy_mm(x, y)
        fp.orientation = Angle.from_degrees(0.0)
        updated.append(fp)

    for i, ref in enumerate(sorted(parked)):
        fp = fp_by_ref[ref]
        col = i % PARK_COLS
        row = i // PARK_COLS
        fp.position = Vector2.from_xy_mm(PARK_ORIGIN[0] + col * PARK_PITCH, PARK_ORIGIN[1] + row * PARK_PITCH)
        fp.orientation = Angle.from_degrees(0.0)
        updated.append(fp)

    board.update_items(updated)
    print(f"Updated {len(updated)} footprints total.")

    board.save()
    print("Saved board to disk.")

    print(f"Parked refs: {sorted(parked)}")


if __name__ == "__main__":
    main()
