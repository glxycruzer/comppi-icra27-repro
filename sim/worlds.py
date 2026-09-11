#!/usr/bin/env python3
"""Randomized benchmark worlds (10x10 m @ 0.05 m/px) + A* global planner.

Each world gives two grids: grid_true (what the costmap sees) and grid_plan
(what the global planner knew). For 'blocked' worlds they differ — the
surprise obstacle is unmapped, which is the local-detour test.
"""
import heapq
import json
import math
import os
import zlib

import numpy as np

RES = 0.05
N = 200  # 10 m
ROBOT_R = 0.19


def _empty():
    g = np.zeros((N, N), dtype=bool)  # [row=y, col=x], True = occupied
    g[:2, :] = g[-2:, :] = True
    g[:, :2] = g[:, -2:] = True
    return g


def _box(g, cx, cy, sx, sy):
    x0, x1 = int((cx - sx / 2) / RES), int((cy - sy / 2) / RES)
    j0, j1 = max(0, x0), max(0, x1)
    g[j1:int((cy + sy / 2) / RES) + 1, j0:int((cx + sx / 2) / RES) + 1] = True


def make_world(kind, seed):
    # deterministic per-kind offset (Python's hash() is salted per process,
    # which made worlds irreproducible across runs; zlib.crc32 is not)
    rng = np.random.default_rng(seed * 1000 + zlib.crc32(kind.encode()) % 1000)
    start, goal = (1.2, 5.0), (8.8, 5.0)
    g = _empty()
    if kind == "open":
        gp = g.copy()
    elif kind == "clutter":
        for _ in range(10):
            while True:
                cx, cy = rng.uniform(1.0, 9.0, 2)
                if math.hypot(cx - start[0], cy - start[1]) > 1.0 and \
                   math.hypot(cx - goal[0], cy - goal[1]) > 1.0:
                    break
            _box(g, cx, cy, rng.uniform(0.3, 0.5), rng.uniform(0.3, 0.5))
        gp = g.copy()
    elif kind == "gap":
        gap_y = rng.uniform(2.5, 7.5)
        gap_w = rng.uniform(0.7, 1.0)
        wall = _empty()
        x0 = int(4.9 / RES)
        wall[:, x0:x0 + 4] = True
        wall[int((gap_y - gap_w / 2) / RES):int((gap_y + gap_w / 2) / RES), x0:x0 + 4] = False
        g |= wall
        gp = g.copy()
    elif kind == "blocked":
        gp = g.copy()  # planner saw an empty world
        n_boxes = rng.integers(1, 3)
        xs = rng.uniform(3.0, 7.0, n_boxes)
        for cx in xs:
            _box(g, cx, 5.0 + rng.uniform(-0.1, 0.1), 0.4, 0.4)
    elif kind == "blocked_sym":
        # perfectly left/right-symmetric blockage dead ahead: the worst case
        # for softmax mode averaging (tests the temperature-sharpening claim)
        gp = g.copy()
        _box(g, rng.uniform(3.5, 6.5), 5.0, 0.4, 0.4)
    elif kind.startswith("blocked_off"):
        # asymmetry sweep: one 0.4 m box whose centre is offset laterally from
        # the planned path by <cm>/100 m ("blocked_off20" = 0.20 m). The box x
        # and the offset side are drawn from a seed-only rng so that, for a
        # given seed, every offset shares the same box position (paired design).
        gp = g.copy()
        off = int(kind[len("blocked_off"):]) / 100.0
        rng_o = np.random.default_rng(seed * 1000 + 777)
        cx = rng_o.uniform(3.5, 6.5)
        side = 1.0 if rng_o.uniform() < 0.5 else -1.0
        _box(g, cx, 5.0 + side * off, 0.4, 0.4)
    elif kind == "crossing":
        # dynamic obstacle: an empty corridor world; a 0.5 m box waits beside
        # the path and crosses it (perpendicular, 0.25-0.35 m/s) timed so that
        # box and robot reach the crossing point together at nominal cruise.
        # The box is NOT in either grid -- dyn_obstacle.py injects it live.
        gp = g.copy()
        xc = rng.uniform(4.0, 6.0)
        vy_mag = rng.uniform(0.25, 0.35)
        side = 1.0 if rng.uniform() < 0.5 else -1.0
        y0 = 5.0 + side * 1.5
        t_cross = 1.5 / vy_mag                       # box travel time to the path
        trigger_dx = 0.38 * t_cross + rng.uniform(-0.3, 0.3)  # robot distance at trigger
        dyn = {"xc": round(float(xc), 3), "y0": round(float(y0), 3),
               "vy": round(float(-side * vy_mag), 3), "size": 0.5,
               "trigger_dx": round(float(trigger_dx), 3)}
        return {"grid_true": g, "grid_plan": gp, "start": start, "goal": goal, "dyn": dyn}
    else:
        raise ValueError(kind)
    return {"grid_true": g, "grid_plan": gp, "start": start, "goal": goal}


def write_map(grid, out_dir):
    pgm = os.path.join(out_dir, "map.pgm")
    img = np.where(grid, 0, 254).astype(np.uint8)
    with open(pgm, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (N, N))
        f.write(np.flipud(img).tobytes())  # PGM row 0 = top = max y
    yaml_path = os.path.join(out_dir, "map.yaml")
    with open(yaml_path, "w") as f:
        f.write("image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
    return yaml_path


def _los(grid, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
    for t in np.linspace(0, 1, max(n, 2)):
        i = int(round(a[0] + t * (b[0] - a[0])))
        j = int(round(a[1] + t * (b[1] - a[1])))
        if grid[i, j]:
            return False
    return True


def plan_path(grid_plan, start, goal, spacing=0.05):
    """A* on the inflated planning grid, line-of-sight pruning, resampling."""
    from scipy.ndimage import binary_dilation
    r = int((ROBOT_R + 0.06) / RES)
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    inflated = binary_dilation(grid_plan, structure=(xx * xx + yy * yy) <= r * r)

    def cell(p):
        return (int(p[1] / RES), int(p[0] / RES))  # (row=y, col=x)

    s, t = cell(start), cell(goal)
    if inflated[s] or inflated[t]:
        return None
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    gcost = {s: 0.0}
    came = {}
    pq = [(0.0, s)]
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == t:
            break
        for di, dj in dirs:
            nb = (cur[0] + di, cur[1] + dj)
            if not (0 <= nb[0] < N and 0 <= nb[1] < N) or inflated[nb]:
                continue
            ng = gcost[cur] + math.hypot(di, dj)
            if ng < gcost.get(nb, 1e18):
                gcost[nb] = ng
                came[nb] = cur
                h = math.hypot(t[0] - nb[0], t[1] - nb[1])
                heapq.heappush(pq, (ng + h, nb))
    if t not in came and s != t:
        return None
    cells = [t]
    while cells[-1] != s:
        cells.append(came[cells[-1]])
    cells.reverse()

    # line-of-sight shortcut pruning
    pruned = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _los(inflated, cells[i], cells[j]):
            j -= 1
        pruned.append(cells[j])
        i = j

    pts = [((c[1] + 0.5) * RES, (c[0] + 0.5) * RES) for c in pruned]
    pts[0], pts[-1] = start, goal
    # densify to fixed spacing
    dense = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        for k in range(1, int(d / spacing) + 1):
            f = k * spacing / d
            dense.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
    if dense[-1] != goal:
        dense.append(goal)
    return dense


def write_scenario(out_dir, world):
    path = plan_path(world["grid_plan"], world["start"], world["goal"])
    if path is None:
        return None
    yaw0 = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
    scen = {"start": [world["start"][0], world["start"][1], yaw0],
            "goal": list(world["goal"]), "path": path}
    with open(os.path.join(out_dir, "scenario.json"), "w") as f:
        json.dump(scen, f)
    np.save(os.path.join(out_dir, "grid_true.npy"), world["grid_true"])
    return scen
