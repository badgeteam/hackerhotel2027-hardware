#!/usr/bin/env python3
"""
Prototype LED placement generator for the starfish badge.

Generates a standalone .kicad_pcb containing just the LED footprints,
laid out as:
  - a 17x5 "matrix" grid in the center
  - 5 tentacle arms of 7 LEDs each, following a simple parametric
    centerline (placeholder arm shape -- swap get_arm_waypoints()
    for traced centerlines once the pitch/addressing look right)

Run standalone (no need to open KiCad):
    python3 led_layout.py

Then open the generated .kicad_pcb in pcbnew to look at it, and run
`kicad-cli pcb drc` on it (this script does that automatically and
prints a summary).
"""
import math
import pathlib
import subprocess
import sys

import pcbnew

# ---------------------------------------------------------------------------
# Footprint facts (LED-SMD-4P-L1.6-W1.5_NH-B1515RGBA-GF / LCSC C52212029)
# ---------------------------------------------------------------------------
# Body (silkscreen) is 1.6 x 1.5mm, but the pads are gull-wing and stick out
# past the body in X: real copper bounding box is 2.4mm (X) x 1.2mm (Y).
# The footprint's courtyard was previously drawn at the *body* size (1.6x1.5)
# which did not enclose the pads -- fixed in the library file itself
# (library/led.pretty/..., courtyard now 2.4 x 1.5mm). Use the pad bbox here
# to size pitches so DRC clearance checks mean something.
PAD_HALF_X = 1.2
PAD_HALF_Y = 0.6
PAD_CLEARANCE = 0.2  # desired minimum copper-to-copper gap

LIB_PATH = pathlib.Path(__file__).resolve().parents[2] / "library" / "led.pretty"
FOOTPRINT_NAME = "LED-SMD-4P-L1.6-W1.5_NH-B1515RGBA-GF"

OUT_PCB = pathlib.Path(__file__).resolve().parent / "led_layout_test.kicad_pcb"

# ---------------------------------------------------------------------------
# Matrix parameters
# ---------------------------------------------------------------------------
MATRIX_COLS = 17
MATRIX_ROWS = 5
# Safe minimum pitch given the pad bbox above, with a bit of headroom.
MATRIX_PITCH_X = 2.8  # mm, along the row (pad-wide axis, LED rotation 0)
MATRIX_PITCH_Y = 2.0  # mm, between rows

# ---------------------------------------------------------------------------
# Tentacle parameters
# ---------------------------------------------------------------------------
TENTACLE_COUNT = 5
LEDS_PER_TENTACLE = 7
# Conservative spacing along the tentacle path: regardless of how the LED is
# rotated to follow the local tangent, its pad bbox diagonal is
# sqrt(1.2^2 + 0.6^2) ~= 1.34mm, so 2x that + clearance is always safe.
TENTACLE_PITCH = 2 * math.hypot(PAD_HALF_X, PAD_HALF_Y) + PAD_CLEARANCE  # ~2.88mm

# Board is ~95x100mm (from the real hackerhotel2027.kicad_pcb). Center the
# matrix on the board center and radiate tentacles outward from there.
BOARD_CENTER = (47.5, 50.0)


def matrix_positions():
    """Return list of (x, y, rotation_deg) for the center matrix, centered
    on BOARD_CENTER."""
    w = (MATRIX_COLS - 1) * MATRIX_PITCH_X
    h = (MATRIX_ROWS - 1) * MATRIX_PITCH_Y
    x0 = BOARD_CENTER[0] - w / 2
    y0 = BOARD_CENTER[1] - h / 2
    out = []
    for row in range(MATRIX_ROWS):
        for col in range(MATRIX_COLS):
            x = x0 + col * MATRIX_PITCH_X
            y = y0 + row * MATRIX_PITCH_Y
            out.append((x, y, 0.0))
    return out


def get_arm_waypoints(angle_deg):
    """PLACEHOLDER centerline for one tentacle arm: a gentle outward curve
    from the edge of the matrix to a tip, at the given angle from center.

    This is NOT traced from starfish_outline.svg yet -- it's a stand-in so
    we can validate pitch/rotation/DRC before investing in real centerline
    extraction (e.g. skeletonizing the outline polygon). Swap this out once
    the rest of the pipeline looks right.
    """
    inner_r = 28.0  # roughly matrix half-width, so arms start at its edge
    outer_r = 45.0  # arm tip distance from center
    bulge_deg = 12.0  # how much the arm curves sideways, for "organic" feel

    a0 = math.radians(angle_deg)
    a1 = math.radians(angle_deg + bulge_deg)

    p0 = (
        BOARD_CENTER[0] + inner_r * math.cos(a0),
        BOARD_CENTER[1] + inner_r * math.sin(a0),
    )
    p1 = (
        BOARD_CENTER[0] + (inner_r + outer_r) / 2 * math.cos(a1),
        BOARD_CENTER[1] + (inner_r + outer_r) / 2 * math.sin(a1),
    )
    p2 = (
        BOARD_CENTER[0] + outer_r * math.cos(a0),
        BOARD_CENTER[1] + outer_r * math.sin(a0),
    )
    return [p0, p1, p2]


def quad_bezier(p0, p1, p2, t):
    x = (1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t ** 2 * p2[0]
    y = (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t ** 2 * p2[1]
    return x, y


def resample_by_arclength(waypoints, n_points, spacing):
    """Sample n_points along a quadratic bezier through waypoints, spaced by
    `spacing` mm of arc length starting from the first waypoint. Returns
    list of (x, y, tangent_deg)."""
    p0, p1, p2 = waypoints
    # Dense sampling to approximate arc length.
    steps = 2000
    pts = [quad_bezier(p0, p1, p2, i / steps) for i in range(steps + 1)]
    cum = [0.0]
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        cum.append(cum[-1] + math.hypot(dx, dy))
    total = cum[-1]

    out = []
    for k in range(n_points):
        target = k * spacing
        if target > total:
            target = total
        idx = min(range(len(cum)), key=lambda i: abs(cum[i] - target))
        idx = max(1, min(idx, len(pts) - 2))
        x, y = pts[idx]
        dx = pts[idx + 1][0] - pts[idx - 1][0]
        dy = pts[idx + 1][1] - pts[idx - 1][1]
        tangent_deg = math.degrees(math.atan2(dy, dx))
        out.append((x, y, tangent_deg))
    return out


def tentacle_positions():
    out = []
    for i in range(TENTACLE_COUNT):
        angle = 90.0 + i * (360.0 / TENTACLE_COUNT)  # first arm pointing "up"
        waypoints = get_arm_waypoints(angle)
        pts = resample_by_arclength(waypoints, LEDS_PER_TENTACLE, TENTACLE_PITCH)
        out.append(pts)
    return out


def build_board():
    board = pcbnew.BOARD()

    def add_led(x_mm, y_mm, rot_deg, ref):
        fp = pcbnew.FootprintLoad(str(LIB_PATH), FOOTPRINT_NAME)
        if fp is None:
            raise RuntimeError(f"Could not load footprint {FOOTPRINT_NAME} from {LIB_PATH}")
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x_mm), pcbnew.FromMM(y_mm)))
        fp.SetOrientationDegrees(rot_deg)
        fp.SetReference(ref)
        board.Add(fp)

    idx = 1
    for x, y, rot in matrix_positions():
        add_led(x, y, rot, f"DM{idx}")
        idx += 1

    for t_i, pts in enumerate(tentacle_positions()):
        for p_i, (x, y, tangent_deg) in enumerate(pts):
            add_led(x, y, tangent_deg, f"DT{t_i + 1}_{p_i + 1}")

    # Simple rectangular board edge so DRC has an outline to work with.
    margin = 10.0
    xs = [fp.GetPosition().x for fp in board.GetFootprints()]
    ys = [fp.GetPosition().y for fp in board.GetFootprints()]
    x0, x1 = pcbnew.ToMM(min(xs)) - margin, pcbnew.ToMM(max(xs)) + margin
    y0, y1 = pcbnew.ToMM(min(ys)) - margin, pcbnew.ToMM(max(ys)) + margin
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    for (ax, ay), (bx, by) in zip(corners, corners[1:]):
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(ax), pcbnew.FromMM(ay)))
        seg.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(bx), pcbnew.FromMM(by)))
        seg.SetLayer(pcbnew.Edge_Cuts)
        board.Add(seg)

    return board


def main():
    print(f"Matrix: {MATRIX_COLS}x{MATRIX_ROWS} = {MATRIX_COLS * MATRIX_ROWS} LEDs, "
          f"pitch {MATRIX_PITCH_X}x{MATRIX_PITCH_Y}mm")
    print(f"Tentacles: {TENTACLE_COUNT} x {LEDS_PER_TENTACLE} = "
          f"{TENTACLE_COUNT * LEDS_PER_TENTACLE} LEDs, pitch {TENTACLE_PITCH:.2f}mm")
    total = MATRIX_COLS * MATRIX_ROWS + TENTACLE_COUNT * LEDS_PER_TENTACLE
    print(f"Total: {total} / 120 chip budget")

    board = build_board()
    board.Save(str(OUT_PCB))
    print(f"Wrote {OUT_PCB}")

    drc_report = OUT_PCB.with_suffix(".drc.json")
    result = subprocess.run(
        [
            "kicad-cli", "pcb", "drc",
            "--format", "json",
            "--output", str(drc_report),
            "--exit-code-violations",
            str(OUT_PCB),
        ],
        capture_output=True, text=True,
    )
    print(result.stdout)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        print(f"DRC reported violations (see {drc_report}) -- exit code {result.returncode}")
    else:
        print("DRC: no violations.")


if __name__ == "__main__":
    main()
