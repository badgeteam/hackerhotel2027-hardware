#!/usr/bin/env python3
"""
Place the center matrix (279 of the LED_0402_1005Metric footprints) on the
live, open hackerhotel2027 PCB via KiCad's IPC API.

Layout: a 217-LED core circle (dartboard spokes, 2.2mm pitch, same scheme
as before) plus 62 "petal" LEDs that flow out toward each of the 5
tentacles, snapped to real nearby silkscreen dots and filtered to keep
>=2.7mm clearance from every RGB tentacle LED (verified zero violations
before this script was written -- see design/ai/flow_matrix_check.html).

RGB tentacle LEDs (LED-SMD-4P-L1.6-W1.5_NH-B1515RGBA-GF) are NOT touched.

Uses plain update_items() (not begin_commit/push_commit -- that hangs/
crashes this KiCad 10.0.6 build's IPC server).
"""
import json
import re

from kipy import KiCad
from kipy.geometry import Vector2, Angle

MONO_FOOTPRINT_NAME = "LED_0402_1005Metric"
POINTS_FILE = "/tmp/final_grid_points.json"


def main():
    points = json.load(open(POINTS_FILE))
    print(f"Loaded {len(points)} target positions")

    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()
    print("Board:", board.name)

    mono_fps = [fp for fp in board.get_footprints() if fp.definition.id.name == MONO_FOOTPRINT_NAME]
    print(f"Found {len(mono_fps)} {MONO_FOOTPRINT_NAME} footprints")

    def refnum(fp):
        return int(re.match(r"D(\d+)$", fp.reference_field.text.value).group(1))

    mono_fps.sort(key=refnum)
    assert len(points) <= len(mono_fps)

    used_fps = mono_fps[:len(points)]
    leftover_fps = mono_fps[len(points):]
    print(f"Placing {len(used_fps)} in the flow matrix, parking {len(leftover_fps)} off-board")

    updated = []
    for fp, (x, y, rot) in zip(used_fps, points):
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
    print(f"Updated {len(updated)} footprints total.")

    board.save()
    print("Saved board to disk.")


if __name__ == "__main__":
    main()
