#!/usr/bin/env python3
"""Snap every placed LED_0402_1005Metric footprint's rotation to the
nearest multiple of 45 degrees. Position is untouched. RGB tentacle LEDs
are not touched (not in scope for this request based on the session so
far -- flag if that's wrong)."""
import re

from kipy import KiCad
from kipy.geometry import Angle

MONO_FOOTPRINT_NAME = "LED_0402_1005Metric"


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()

    fps = [fp for fp in board.get_footprints() if fp.definition.id.name == MONO_FOOTPRINT_NAME]
    # only touch ones actually placed on the board, not the parked pile
    placed = [fp for fp in fps if fp.position.x / 1e6 <= 100]
    print(f"{len(placed)} placed LEDs to snap (of {len(fps)} total)")

    changed = 0
    updated = []
    for fp in placed:
        cur = fp.orientation.degrees
        snapped = round(cur / 45.0) * 45.0
        if snapped == 360.0:
            snapped = 0.0
        if abs(snapped - cur) > 1e-6:
            changed += 1
        fp.orientation = Angle.from_degrees(snapped)
        updated.append(fp)

    board.update_items(updated)
    print(f"Updated {len(updated)} footprints, {changed} actually changed angle.")
    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
