#!/usr/bin/env python3
"""
Place the remaining spare LED_0402_1005Metric footprints along the board
outline, inset from the edge, evenly spaced by arc length, rotated to
follow the local contour tangent.

Does NOT touch the 166 footprints already placed as the center ring
pattern (center + rings 1-8) -- only the next unused ones (by D-number)
get moved from their parked position onto the contour. Any that don't fit
(too close to an RGB LED) stay parked.

Uses plain update_items() (not begin_commit/push_commit -- that hangs/
crashes this KiCad 10.0.6 build's IPC server).
"""
import json
import math
import re

from kipy import KiCad
from kipy.geometry import Vector2, Angle
from shapely.geometry import Polygon

MONO_FOOTPRINT_NAME = "LED_0402_1005Metric"
RING_FOOTPRINT_COUNT = 166  # already placed by place_ring_step.py, don't touch
MARGIN = 2.0
RGB_CLEARANCE = 2.7


def build_contour_candidates(n_target):
    outline = json.load(open("/tmp/outline_pts_new.json"))
    rgb_pts = json.load(open("/tmp/rgb_pts_current.json"))

    poly = Polygon(outline)
    inset = poly.buffer(-MARGIN, join_style=2)
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
        x, y, rot = point_at(s)
        if min(math.hypot(x - rx, y - ry) for rx, ry in rgb_pts) < RGB_CLEARANCE:
            skipped += 1
            continue
        points.append((x, y, rot))

    print(f"Contour: perimeter={total:.1f}mm, target pitch={total/n_target:.2f}mm, "
          f"{len(points)} valid / {n_target} ({skipped} skipped near RGB LEDs)")
    return points


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()
    print("Board:", board.name)

    mono_fps = [fp for fp in board.get_footprints() if fp.definition.id.name == MONO_FOOTPRINT_NAME]

    def refnum(fp):
        return int(re.match(r"D(\d+)$", fp.reference_field.text.value).group(1))

    mono_fps.sort(key=refnum)
    spare_fps = mono_fps[RING_FOOTPRINT_COUNT:]
    print(f"{len(spare_fps)} spare footprints available for the contour")

    contour_pts = build_contour_candidates(len(spare_fps))

    used_fps = spare_fps[:len(contour_pts)]
    leftover_fps = spare_fps[len(contour_pts):]

    updated = []
    for fp, (x, y, rot) in zip(used_fps, contour_pts):
        fp.position = Vector2.from_xy_mm(x, y)
        fp.orientation = Angle.from_degrees(rot)
        updated.append(fp)

    PARK_ORIGIN = (150.0, 80.0)
    PARK_COLS = 11
    PARK_PITCH = 3.0
    for i, fp in enumerate(leftover_fps):
        col = i % PARK_COLS
        row = i // PARK_COLS
        fp.position = Vector2.from_xy_mm(PARK_ORIGIN[0] + col * PARK_PITCH, PARK_ORIGIN[1] + row * PARK_PITCH)
        fp.orientation = Angle.from_degrees(0.0)
        updated.append(fp)

    board.update_items(updated)
    print(f"Placed {len(used_fps)} LEDs on the contour; parked {len(leftover_fps)}.")
    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
