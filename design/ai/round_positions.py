#!/usr/bin/env python3
"""Round every LED footprint's X/Y position to 2 decimal places (hundredths
of a mm). Applies to both LED_0402_1005Metric and the RGB tentacle LEDs.
Rotation is untouched. Max shift per LED is <=0.005mm (sub-hundredth),
well under normal pad/trace overlap tolerance."""
from kipy import KiCad
from kipy.geometry import Vector2

FOOTPRINT_NAMES = ("LED_0402_1005Metric", "LED-SMD-4P-L1.6-W1.5_NH-B1515RGBA-GF")


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()

    fps = [fp for fp in board.get_footprints() if fp.definition.id.name in FOOTPRINT_NAMES]
    print(f"{len(fps)} LED footprints total")

    changed = 0
    updated = []
    for fp in fps:
        x_mm = fp.position.x / 1e6
        y_mm = fp.position.y / 1e6
        rx, ry = round(x_mm, 2), round(y_mm, 2)
        if abs(rx - x_mm) > 1e-9 or abs(ry - y_mm) > 1e-9:
            changed += 1
        fp.position = Vector2.from_xy_mm(rx, ry)
        updated.append(fp)

    board.update_items(updated)
    print(f"Updated {len(updated)} footprints, {changed} actually shifted.")
    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
