#!/usr/bin/env python3
"""
Step-by-step rebuild of the center matrix as a spoke-aligned concentric
circle pattern (going back to the circle idea, not the square grid).

Each LED's rotation matches the angle of its own spoke (radial orientation),
so the footprint's long axis points straight out from the center.

Step 1 (this run): park every LED_0402_1005Metric off-board, then place
only the center LED + ring 1 (6 LEDs). Later rings will be added in
subsequent steps once this looks right.

"Axis position 0,0" for the center LED is this pattern's own local origin;
it's placed at the board's actual central-cavity point
(48.6557, 54.5562mm) so it lines up with the tentacle chains and board
outline from all the earlier confirmed work -- same point used throughout
this session as CENTER. Flagging this in case a literal board-origin
placement was intended instead.

Uses plain update_items() (not begin_commit/push_commit -- that hangs/
crashes this KiCad 10.0.6 build's IPC server).
"""
import json
import math
import re

from kipy import KiCad
from kipy.geometry import Vector2, Angle

RGB_CLEARANCE = 2.7
RGB_POINTS = json.load(open("/tmp/rgb_pts_current.json"))

MONO_FOOTPRINT_NAME = "LED_0402_1005Metric"

CENTER = (0.0, 0.0)  # board has been moved so its visual center is now here
PITCH = 2.2

PARK_ORIGIN = (150.0, 80.0)
PARK_COLS = 11
PARK_PITCH = 3.0


# Minimum safe tangential spacing between same-ring 0402s at these radial
# pitches. Empirically: 1.728mm (a naive sqrt(2)-threshold doubling scheme)
# produced real KiCad courtyard overlaps; 2.304mm did not. Using 2.1mm as a
# safe floor with margin below the confirmed-good value.
SAFE_MIN_SPACING = 2.1
RING_COUNT = 8  # ring 1 through ring 8


def spoke_count_for_ring(r, prev_S):
    """Largest power-of-2 multiple of 6 spoke count (>= prev_S, so spokes
    only ever split further out, never merge) whose tangential spacing at
    radius r stays >= SAFE_MIN_SPACING."""
    S = prev_S
    while 2 * math.pi * r / (S * 2) >= SAFE_MIN_SPACING:
        S *= 2
    return S


def ring_positions(max_ring=RING_COUNT):
    points = [(CENTER[0], CENTER[1], 0.0)]  # center LED at local (0,0)
    S = 6
    for k in range(1, max_ring + 1):
        r = k * PITCH
        S = spoke_count_for_ring(r, S)
        spacing = 2 * math.pi * r / S
        skipped = 0
        for i in range(S):
            a = 2 * math.pi * i / S
            x = CENTER[0] + r * math.cos(a)
            y = CENTER[1] + r * math.sin(a)
            if min(math.hypot(x - rx, y - ry) for rx, ry in RGB_POINTS) < RGB_CLEARANCE:
                skipped += 1
                continue
            points.append((x, y, math.degrees(a)))
        print(f"  ring {k}: r={r:.2f}mm S={S} spacing={spacing:.3f}mm"
              + (f" ({skipped} skipped, too close to RGB LEDs)" if skipped else ""))
    return points


def park_positions(n):
    out = []
    for i in range(n):
        col = i % PARK_COLS
        row = i // PARK_COLS
        out.append((PARK_ORIGIN[0] + col * PARK_PITCH, PARK_ORIGIN[1] + row * PARK_PITCH, 0.0))
    return out


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()
    print("Board:", board.name)

    mono_fps = [fp for fp in board.get_footprints() if fp.definition.id.name == MONO_FOOTPRINT_NAME]
    print(f"Found {len(mono_fps)} {MONO_FOOTPRINT_NAME} footprints")

    def refnum(fp):
        return int(re.match(r"D(\d+)$", fp.reference_field.text.value).group(1))

    mono_fps.sort(key=refnum)

    ring_pts = ring_positions()
    print(f"Placing {len(ring_pts)} LEDs (center + rings 1-{RING_COUNT}), parking the rest")

    used_fps = mono_fps[:len(ring_pts)]
    leftover_fps = mono_fps[len(ring_pts):]

    updated = []
    for fp, (x, y, rot) in zip(used_fps, ring_pts):
        fp.position = Vector2.from_xy_mm(x, y)
        fp.orientation = Angle.from_degrees(rot)
        updated.append(fp)

    for fp, (x, y, rot) in zip(leftover_fps, park_positions(len(leftover_fps))):
        fp.position = Vector2.from_xy_mm(x, y)
        fp.orientation = Angle.from_degrees(rot)
        updated.append(fp)

    board.update_items(updated)
    print(f"Updated {len(updated)} footprints total ({len(used_fps)} placed, {len(leftover_fps)} parked).")
    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
