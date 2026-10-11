#!/usr/bin/env python3
"""
Place the D1-D120 LED footprints in the live, open hackerhotel2027 PCB via
KiCad's IPC API.

Layout:
  - 5 row-groups (RGB cathode triplets), each wired to 24 LEDs in the
    schematic: D1-24, D25-48, D49-72, D73-96, D97-120.
  - Within each block of 24, the first 17 (by D-number) are one row of the
    center matrix (17 cols x 5 rows = 85 LEDs) -- currently PARKED off-board
    while we focus on the tentacles. The last 7 are one tentacle's LED
    chain, positions taken directly from the confirmed circles in
    design/ai/starfish_silkscreen.svg (tip = chain index 0, body = index 6).

Row-group -> arm assignment (arbitrary, not yet constrained by any net):
    row_group 0 (D1-24)   -> arm_bl
    row_group 1 (D25-48)  -> arm_top
    row_group 2 (D49-72)  -> arm_right
    row_group 3 (D73-96)  -> arm_br
    row_group 4 (D97-120) -> arm_left

Only repositions/rotates existing footprints -- creates/deletes nothing,
touches no nets. Uses plain update_items() (NOT begin_commit/push_commit --
that hangs/crashes this KiCad 10.0.6 build's IPC server). Nothing is saved
to disk; review in the PCB editor and save manually if it looks right.
"""
import math
import re

from kipy import KiCad
from kipy.geometry import Vector2, Angle

FOOTPRINT_NAME_HINT = "NH-B1515RGBA-GF"

# --- matrix: parked off-board for now -------------------------------------
MATRIX_COLS = 17
MATRIX_ROWS = 5
MATRIX_PITCH_X = 2.8
MATRIX_PITCH_Y = 2.0
MATRIX_PARK_OFFSET = (150.0, 0.0)  # pushes the whole block clear of the board
BOARD_CENTER = (47.5, 50.0)

# --- tentacles: confirmed chains from starfish_silkscreen.svg (mm) --------
TENTACLE_CHAINS = [
    [(20.868944666286364, 87.90881916714204), (23.697952187640954, 83.97612990527739),
     (27.541330902461258, 79.54195077484046), (31.457284946236555, 75.0889623655914),
     (39.581789709172256, 74.45097688292319), (34.39942492012779, 69.87012779552715),
     (27.77929993577392, 66.50125883108542)],  # arm_bl
    [(47.17118047673098, 10.444659477866061), (47.21055353901996, 15.276293103448275),
     (46.743094582185485, 21.11965564738292), (46.22395035078251, 27.037188343227196),
     (40.27050295857988, 32.04855769230769), (46.9480991359709, 32.976971350613915),
     (53.70720492866407, 31.856809338521398)],  # arm_top
    [(85.82794386821232, 40.00869432580841), (81.37747506799637, 41.686935630099725),
     (75.69314022988506, 43.16930574712643), (69.94618562231759, 44.618127682403426),
     (64.5716590909091, 47.255027272727276), (66.79257372654155, 51.980395442359246),
     (63.65143390589993, 41.830463032113514)],  # arm_right
    [(69.88709806157354, 88.26928164196123), (67.09758196721312, 84.3137568306011),
     (64.17156207054512, 79.23217590471828), (61.24054662379421, 74.06491425509111),
     (53.67985849056603, 75.1902358490566), (57.27541020966272, 69.5826162260711),
     (63.617994034302754, 65.73396718866518)],  # arm_br
    [(8.521298627002288, 38.00867848970252), (13.041789759855005, 39.755020389669234),
     (18.296298165137614, 42.357096330275226), (23.6092688172043, 45.01805376344086),
     (29.064504918032785, 40.59573114754099), (29.393851784080514, 46.53968892955169),
     (25.686664184661204, 52.61842144452718)],  # arm_left
]


def matrix_position(row, col):
    w = (MATRIX_COLS - 1) * MATRIX_PITCH_X
    h = (MATRIX_ROWS - 1) * MATRIX_PITCH_Y
    x0 = BOARD_CENTER[0] - w / 2 + MATRIX_PARK_OFFSET[0]
    y0 = BOARD_CENTER[1] - h / 2 + MATRIX_PARK_OFFSET[1]
    return (x0 + col * MATRIX_PITCH_X, y0 + row * MATRIX_PITCH_Y, 0.0)


def tentacle_position(tentacle_index, index_in_tentacle):
    chain = TENTACLE_CHAINS[tentacle_index]
    x, y = chain[index_in_tentacle]
    if index_in_tentacle == 0:
        dx, dy = chain[1][0] - chain[0][0], chain[1][1] - chain[0][1]
    elif index_in_tentacle == len(chain) - 1:
        dx, dy = chain[-1][0] - chain[-2][0], chain[-1][1] - chain[-2][1]
    else:
        dx = chain[index_in_tentacle + 1][0] - chain[index_in_tentacle - 1][0]
        dy = chain[index_in_tentacle + 1][1] - chain[index_in_tentacle - 1][1]
    rot_deg = math.degrees(math.atan2(dy, dx))
    return (x, y, rot_deg)


def target_for_ref(d_number):
    n = d_number - 1
    row_group, local = divmod(n, 24)
    if local < MATRIX_COLS:
        return matrix_position(row_group, local)
    return tentacle_position(row_group, local - MATRIX_COLS)


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()
    print("Board:", board.name)

    fps = [fp for fp in board.get_footprints() if FOOTPRINT_NAME_HINT in fp.definition.id.name]
    print(f"Found {len(fps)} LED footprints")

    def refnum(fp):
        m = re.match(r"D(\d+)$", fp.reference_field.text.value)
        if not m:
            raise ValueError(f"Unexpected reference {fp.reference_field.text.value!r}")
        return int(m.group(1))

    updated = []
    for fp in fps:
        n = refnum(fp)
        x_mm, y_mm, rot_deg = target_for_ref(n)
        fp.position = Vector2.from_xy_mm(x_mm, y_mm)
        fp.orientation = Angle.from_degrees(rot_deg)
        updated.append(fp)

    board.update_items(updated)

    print(f"Repositioned {len(updated)} LEDs.")
    print("Not saved to disk -- review in the PCB editor and Ctrl+S if it looks right.")


if __name__ == "__main__":
    main()
