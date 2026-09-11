#!/usr/bin/env python3
"""Metrics from the kin_sim trajectory CSV: goal error, obstacle/wall
clearance, acceleration-limit compliance, smoothness, time to goal."""
import csv
import math
import sys

name, gx, gy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
path = sys.argv[4]

rows = []
with open(path) as f:
    for r in csv.DictReader(f):
        rows.append({k: float(v) for k, v in r.items()})
if len(rows) < 10:
    print("VERDICT: FAIL (no trajectory data)")
    sys.exit(1)

# steady-state tail: ignore the trailing stopped segment for time-to-goal
moving = [i for i, r in enumerate(rows) if abs(r["v"]) > 0.01 or abs(r["w"]) > 0.01]
if not moving:
    print("VERDICT: FAIL (robot never moved)")
    sys.exit(1)
last_moving = moving[-1]
t_goal = rows[last_moving]["t"]
xf, yf = rows[-1]["x"], rows[-1]["y"]
goal_err = math.hypot(xf - gx, yf - gy)

# clearance to the 0.4x0.4 box at (5,5) (blocked scenario only)
def box_clearance(x, y):
    dx = max(4.8 - x, 0.0, x - 5.2)
    dy = max(4.8 - y, 0.0, y - 5.2)
    return math.hypot(dx, dy)

min_box = min(box_clearance(r["x"], r["y"]) for r in rows) if name == "blocked" else float("inf")
min_wall = min(min(r["x"] - 0.1, 9.9 - r["x"], r["y"] - 0.1, 9.9 - r["y"]) for r in rows)

# per-command jump: commands arrive at 10 Hz and the sim applies them
# instantly, so the largest consecutive-sample delta (50 Hz log) is the
# size of a single command step. Limit = ax_max/f + feedback-lag margin.
max_up, max_dn, max_wstep = 0.0, 0.0, 0.0
for i in range(1, len(rows)):
    dv = rows[i]["v"] - rows[i - 1]["v"]
    max_up = max(max_up, dv)
    max_dn = max(max_dn, -dv)
    max_wstep = max(max_wstep, abs(rows[i]["w"] - rows[i - 1]["w"]))

# jitter: RMS of linear-command change between consecutive 0.1 s windows
diffs = [rows[i]["v"] - rows[i - 5]["v"] for i in range(5, len(rows), 5)]
rms_dv = math.sqrt(sum(d * d for d in diffs) / max(1, len(diffs)))
vmax = max(r["v"] for r in rows)
reversed_ = min(r["v"] for r in rows) < -0.01

print(f"time_to_goal_s:      {t_goal:.1f}")
print(f"goal_error_m:        {goal_err:.3f}")
if name == "blocked":
    print(f"min_box_clearance_m: {min_box:.3f}  (robot radius 0.19)")
print(f"min_wall_clear_m:    {min_wall:.3f}")
print(f"max_cmd_step_up:     {max_up:.3f}  (ax_max/10Hz = 0.025 + lag margin)")
print(f"max_cmd_step_down:   {max_dn:.3f}  (|ax_min|/10Hz = 0.100 + lag margin)")
print(f"max_cmd_step_w:      {max_wstep:.3f}  (shim 0.5 / comppi az 0.3)")
print(f"rms_dv_per_100ms:    {rms_dv:.4f}")
print(f"v_max_reached:       {vmax:.3f}  (limit 0.4)")
print(f"reverse_motion:      {reversed_}")

ok = goal_err < 0.30 and min_wall > 0.19 and max_up < 0.045 and max_dn < 0.16 and vmax < 0.42
if name == "blocked":
    ok = ok and min_box > 0.19
print("VERDICT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
