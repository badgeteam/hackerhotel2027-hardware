#!/usr/bin/env python3
"""Connect to the running KiCad instance via the IPC API and list LED
footprint instances on the open hackerhotel2027 board: reference, current
position, and the net on each pad (to see how columns/rows are wired)."""
from kipy import KiCad
from kipy.proto.common.types import DocumentType

FOOTPRINT_NAME_HINT = "NH-B1515RGBA-GF"

kicad = KiCad()
print("Connected to KiCad", kicad.get_version())

board = kicad.get_board()
print("Board:", board.name)

fps = board.get_footprints()
leds = [fp for fp in fps if FOOTPRINT_NAME_HINT in fp.definition.id.name]
print(f"Total footprints: {len(fps)}, LED footprints: {len(leds)}")

for fp in leds[:10]:
    ref = fp.reference_field.text.value
    pos = fp.position
    pads = fp.definition.items
    from kipy.board_types import Pad
    pad_nets = [(p.number, p.net.name) for p in pads if isinstance(p, Pad)]
    print(f"{ref}: pos=({pos.x/1e6:.2f},{pos.y/1e6:.2f})mm rot={fp.orientation.degrees} pads={pad_nets}")

if len(leds) > 10:
    print(f"... and {len(leds) - 10} more")
