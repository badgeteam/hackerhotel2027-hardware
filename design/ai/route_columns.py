#!/usr/bin/env python3
"""
Route all unrouted Cx ("column") nets between LED pads, F.Cu only,
0.2mm width, every segment at a multiple of 45 degrees.

Approach per net:
  1. Find all pads on the net (any footprint, not just LEDs).
  2. Union-find over existing track/via endpoints (within tolerance) to see
     which pads are already electrically connected to each other.
  3. If already one cluster, skip (net fully routed).
  4. Otherwise build a minimum spanning tree over the clusters (edge
     weight = closest pad-to-pad Euclidean distance between two clusters)
     and add exactly the edges needed to join everything into one net.
  5. Each new edge is routed as a 2-segment octilinear path (one 45
     diagonal covering min(|dx|,|dy|), one orthogonal leg covering the
     remainder) -- the shortest possible path using only 45-multiple
     angles. No obstacle avoidance; collisions are expected to be found
     and fixed via a DRC pass afterward, consistent with how every other
     step in this session has been validated.

Uses create_items for new tracks (not update_items). Doesn't touch any
existing track/via/footprint.
"""
import math
import re
from collections import defaultdict

from kipy import KiCad
from kipy.board_types import Pad, Track
from kipy.board_types import BoardLayer
from kipy.geometry import Vector2

WIDTH_NM = round(0.2 * 1e6)
TOL = 0.005  # mm, tolerance for matching endpoints


class UnionFind:
    def __init__(self):
        self.parent = {}

    def find(self, x):
        self.parent.setdefault(x, x)
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def key(x, y):
    return (round(x / TOL), round(y / TOL))


def octilinear_path(p1, p2):
    """Return list of (x,y) waypoints (incl. both ends) for a 2-segment
    45-degree-only path from p1 to p2."""
    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    adx, ady = abs(dx), abs(dy)
    if adx < 1e-9 and ady < 1e-9:
        return [p1, p2]
    if abs(adx - ady) < 1e-9:
        return [p1, p2]  # pure diagonal
    diag = min(adx, ady)
    sx = 1 if dx >= 0 else -1
    sy = 1 if dy >= 0 else -1
    mid = (x1 + diag * sx, y1 + diag * sy)
    return [p1, mid, p2]


def main():
    kicad = KiCad()
    print("Connected to KiCad", kicad.get_version())
    board = kicad.get_board()

    nets_by_name = {n.name: n for n in board.get_nets()}

    cx_nets = sorted((n for n in nets_by_name if re.match(r"^C\d+$", n)),
                      key=lambda n: int(n[1:]))
    print("Cx nets found:", cx_nets)

    all_pads = board.get_pads()
    pads_by_net = defaultdict(list)
    for p in all_pads:
        pads_by_net[p.net.name].append(p)

    all_tracks = list(board.get_tracks())
    all_vias = list(board.get_vias())
    tracks_by_net = defaultdict(list)
    for t in all_tracks:
        tracks_by_net[t.net.name].append(t)
    vias_by_net = defaultdict(list)
    for v in all_vias:
        vias_by_net[v.net.name].append(v)

    new_tracks = []
    summary = []

    for net_name in cx_nets:
        pads = pads_by_net.get(net_name, [])
        if len(pads) < 2:
            continue
        pad_pts = [(p.position.x / 1e6, p.position.y / 1e6) for p in pads]

        uf = UnionFind()
        for x, y in pad_pts:
            uf.find(key(x, y))
        for t in tracks_by_net.get(net_name, []):
            sx, sy = t.start.x / 1e6, t.start.y / 1e6
            ex, ey = t.end.x / 1e6, t.end.y / 1e6
            uf.union(key(sx, sy), key(ex, ey))
        # vias connect same-net items that share their position; since we
        # don't have per-layer segment chains here, just treat a via as a
        # pass-through point already implied by tracks touching it (the
        # union above already links any track endpoints that coincide).

        clusters = defaultdict(list)
        for (x, y), p in zip(pad_pts, pads):
            clusters[uf.find(key(x, y))].append((x, y, p))

        cluster_list = list(clusters.values())
        if len(cluster_list) <= 1:
            summary.append((net_name, len(pads), 0, 0))
            continue

        # MST over clusters (Prim's), edge weight = closest pad pair
        in_tree = [0]
        remaining = list(range(1, len(cluster_list)))
        edges = []
        while remaining:
            best = None
            for i in in_tree:
                for j in remaining:
                    for (ax, ay, _) in cluster_list[i]:
                        for (bx, by, _) in cluster_list[j]:
                            d = math.hypot(ax - bx, ay - by)
                            if best is None or d < best[0]:
                                best = (d, i, j, (ax, ay), (bx, by))
            d, i, j, pa, pb = best
            edges.append((pa, pb))
            in_tree.append(j)
            remaining.remove(j)

        net_obj = nets_by_name[net_name]
        seg_count = 0
        for pa, pb in edges:
            waypoints = octilinear_path(pa, pb)
            for i in range(len(waypoints) - 1):
                sx, sy = waypoints[i]
                ex, ey = waypoints[i + 1]
                if abs(sx - ex) < 1e-9 and abs(sy - ey) < 1e-9:
                    continue
                t = Track()
                t.start = Vector2.from_xy_mm(sx, sy)
                t.end = Vector2.from_xy_mm(ex, ey)
                t.layer = BoardLayer.BL_F_Cu
                t.width = WIDTH_NM
                t.net = net_obj
                new_tracks.append(t)
                seg_count += 1

        summary.append((net_name, len(pads), len(cluster_list), seg_count))

    print(f"{'net':6} {'pads':>5} {'clusters':>9} {'new_segments':>13}")
    for net_name, npads, nclusters, nseg in summary:
        print(f"{net_name:6} {npads:>5} {nclusters:>9} {nseg:>13}")

    print(f"\nTotal new track segments to create: {len(new_tracks)}")

    if new_tracks:
        board.create_items(new_tracks)
        print("Created.")
    print("Not saved to disk -- review in the PCB editor.")


if __name__ == "__main__":
    main()
