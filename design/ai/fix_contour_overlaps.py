#!/usr/bin/env python3
"""Park the 5 contour LEDs that DRC flagged as courtyard-overlapping their
neighbors (near-tight 'pinch' zones in the inward-offset outline close to
the armpits, where the inset boundary's arc-length parametrization brings
physically-close points from different arc positions)."""
from kipy import KiCad
from kipy.geometry import Vector2, Angle

MONO_FOOTPRINT_NAME = "LED_0402_1005Metric"
TO_PARK = ["D243", "D275", "D144", "D241", "D232"]
PARK_ORIGIN = (150.0, 80.0)
PARK_COLS = 11
PARK_PITCH = 3.0


def main():
    kicad = KiCad()
    board = kicad.get_board()
    mono_fps = {fp.reference_field.text.value: fp for fp in board.get_footprints()
                if fp.definition.id.name == MONO_FOOTPRINT_NAME}

    # find next free park grid slot by counting footprints already out past x=100mm
    parked_count = sum(1 for fp in mono_fps.values() if fp.position.x / 1e6 > 100)

    updated = []
    for i, ref in enumerate(TO_PARK):
        fp = mono_fps[ref]
        idx = parked_count + i
        col = idx % PARK_COLS
        row = idx // PARK_COLS
        fp.position = Vector2.from_xy_mm(PARK_ORIGIN[0] + col * PARK_PITCH, PARK_ORIGIN[1] + row * PARK_PITCH)
        fp.orientation = Angle.from_degrees(0.0)
        updated.append(fp)

    board.update_items(updated)
    print(f"Parked {len(updated)} conflicting contour LEDs.")


if __name__ == "__main__":
    main()
