#!/usr/bin/env python3
"""
Two placement steps on top of the already-built central grid (which is
left completely untouched):

1. Complete two more rows of the logical matrix for the 6 centermost
   columns (C6,C7,C8,C9,C10,C16 -- the ones that already reach the grid's
   full +-14mm extent). The row nets that are entirely missing from the
   placed grid are '/R1b' (row index 0, y=-16) and 'Net-(D116-A)' (row
   index 16, y=+16) -- discovered live from which parked LEDs share a
   column net with the already-placed grid. Using those specific LEDs
   means this is a real, flush, zero-gap continuation of the existing
   columns -- not just a visual approximation.

2. Every other still-parked 0402 LED gets placed around the board edge,
   inset 1.5mm from Edge.Cuts, evenly spaced by arc length, oriented to
   the nearest 45-degree multiple of the local contour tangent. Any that
   land too close to an RGB tentacle pad stay parked.

Does not move any of the 196 LEDs already on the central grid.
Uses plain update_items() (not begin_commit/push_commit -- hangs this
KiCad 10.0.6 IPC server).
"""
import json
import math
import re

from kipy import KiCad
from kipy.board_types import Pad, BoardLayer
from kipy.geometry import Vector2, Angle
from shapely.geometry import Polygon

MONO_FOOTPRINT_NAME = "LED_0402_1005Metric"
RGB_NAME_HINT = "NH-B1515RGBA-GF"

OUTLINE_FILE = "/tmp/outline_live.json"
CONTOUR_MARGIN = 1.5
RGB_CLEARANCE = 2.7

# Other on-board components near the edge that the contour must also avoid
# (radius is a generous component-body estimate + LED half-width + margin).
OTHER_OBSTACLES = [
    (0.0, 15.0, 5.5),       # U1 LQFP-64
    (11.725, 27.875, 7.0),  # J1 USB-C receptacle
    (16.1, 21.925, 2.5),    # U2 USON-10
    (17.875, 24.025, 1.8),  # R25
    (13.85, 20.675, 1.8),   # R26
]

PARK_ORIGIN = (150.0, 80.0)
PARK_COLS = 11
PARK_PITCH = 3.0

# (ref, x, y) for the two new flush rows -- derived live, see docstring.
TOP_ROW = [  # y = -16, row net '/R1b'
    ("D100", -5.0, -16.0),
    ("D117", -3.0, -16.0),
    ("D134", -1.0, -16.0),
    ("D291", 1.0, -16.0),
    ("D158", 3.0, -16.0),
    ("D175", 5.0, -16.0),
]
BOTTOM_ROW = [  # y = +16, row net 'Net-(D116-A)'
    ("D116", -5.0, 16.0),
    ("D133", -3.0, 16.0),
    ("D150", -1.0, 16.0),
    ("D307", 1.0, 16.0),
    ("D174", 3.0, 16.0),
    ("D191", 5.0, 16.0),
]


def build_contour_points(n_target, outline, rgb_pts):
    poly = Polygon(outline)
    inset = poly.buffer(-CONTOUR_MARGIN, join_style=2)
    if inset.geom_type == "MultiPolygon":
        inset = max(inset.geoms, key=lambda g: g.area)
    boundary = list(inset.exterior.coords)

    cum = [0.0]
    for i in range(1, len(boundary)):
        cum.append(cum[-1] + math.hypot(boundary[i][0] - boundary[i - 1][0],
                                         boundary[i][1] - boundary[i - 1][1]))
    total = cum[-1]

    def point_at(s):
        s = s % total
        for i in range(1, len(cum)):
            if cum[i] >= s:
                t = (s - cum[i - 1]) / (cum[i] - cum[i - 1]) if cum[i] > cum[i - 1] else 0
                x = boundary[i - 1][0] + t * (boundary[i][0] - boundary[i - 1][0])
                y = boundary[i - 1][1] + t * (boundary[i][1] - boundary[i - 1][1])
                dx = boundary[i][0] - boundary[i - 1][0]
                dy = boundary[i][1] - boundary[i - 1][1]
                return x, y, math.degrees(math.atan2(dy, dx))
        return boundary[-1][0], boundary[-1][1], 0.0

    points = []
    skipped = 0
    for k in range(n_target):
        s = k * total / n_target
        x, y, tangent = point_at(s)
        if min(math.hypot(x - rx, y - ry) for rx, ry in rgb_pts) < RGB_CLEARANCE:
            skipped += 1
            continue
        if any(math.hypot(x - ox, y - oy) < orad for ox, oy, orad in OTHER_OBSTACLES):
            skipped += 1
            continue
        snapped_rot = round(tangent / 45.0) * 45.0
        if snapped_rot == 360.0:
            snapped_rot = 0.0
        points.append((round(x, 2), round(y, 2), snapped_rot))

    print(f"Contour: perimeter={total:.1f}mm, target pitch={total/n_target:.2f}mm, "
          f"{len(points)} valid / {n_target} ({skipped} skipped near RGB LEDs)")
    return points


def pads_of(fp):
    return [p for p in fp.definition.items if isinstance(p, Pad)]


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()
    print("Board:", board.name)

    outline = json.load(open(OUTLINE_FILE))

    fps = board.get_footprints()
    mono_fps = [fp for fp in fps if fp.definition.id.name == MONO_FOOTPRINT_NAME]
    rgb_fps = [fp for fp in fps if RGB_NAME_HINT in fp.definition.id.name]
    rgb_pts = [(fp.position.x / 1e6, fp.position.y / 1e6) for fp in rgb_fps]

    fp_by_ref = {fp.reference_field.text.value: fp for fp in mono_fps}
    row_refs = {ref for ref, x, y in TOP_ROW + BOTTOM_ROW}

    def is_core_grid(fp):
        x, y = fp.position.x / 1e6, fp.position.y / 1e6
        return -15.0 <= x <= 15.0 and -16.0 <= y <= 16.0

    core_refs = {ref for ref, fp in fp_by_ref.items() if is_core_grid(fp) and ref not in row_refs}
    print(f"core grid (untouched): {len(core_refs)}, rows (re-placed idempotently): {len(row_refs)}")

    updated = []

    for ref, x, y in TOP_ROW + BOTTOM_ROW:
        fp = fp_by_ref[ref]
        fp.position = Vector2.from_xy_mm(x, y)
        fp.orientation = Angle.from_degrees(0.0)
        updated.append(fp)

    all_refs = set(fp_by_ref.keys())
    remaining_parked_refs = sorted(
        all_refs - core_refs - row_refs,
        key=lambda r: int(re.match(r"D(\d+)$", r).group(1)),
    )
    print(f"remaining to (re)place on contour: {len(remaining_parked_refs)}")

    contour_pts = build_contour_points(len(remaining_parked_refs), outline, rgb_pts)

    used_refs = remaining_parked_refs[:len(contour_pts)]
    leftover_refs = remaining_parked_refs[len(contour_pts):]

    for ref, (x, y, rot) in zip(used_refs, contour_pts):
        fp = fp_by_ref[ref]
        fp.position = Vector2.from_xy_mm(x, y)
        fp.orientation = Angle.from_degrees(rot)
        updated.append(fp)

    for i, ref in enumerate(leftover_refs):
        fp = fp_by_ref[ref]
        col = i % PARK_COLS
        row = i // PARK_COLS
        fp.position = Vector2.from_xy_mm(
            round(PARK_ORIGIN[0] + col * PARK_PITCH, 2),
            round(PARK_ORIGIN[1] + row * PARK_PITCH, 2),
        )
        fp.orientation = Angle.from_degrees(0.0)
        updated.append(fp)

    board.update_items(updated)
    print(f"Updated {len(updated)} footprints total "
          f"({len(TOP_ROW)+len(BOTTOM_ROW)} new rows, {len(used_refs)} on contour, "
          f"{len(leftover_refs)} still parked).")

    board.save()
    print("Saved board to disk.")

    if leftover_refs:
        print("Still parked:", leftover_refs)


if __name__ == "__main__":
    main()
