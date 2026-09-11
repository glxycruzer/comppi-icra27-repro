#!/usr/bin/env python3
"""Offline field-campaign analysis. Plain Python (no ROS) -- run on the dev
machine after copying the extracted CSVs back:

    python3 analyze_trials.py bags/            # aggregate every extracted trial
    python3 analyze_trials.py bags/ --csv out.csv   # also dump per-trial rows

Reads each <trial>/extracted.csv + extracted_meta.json (from extract_bag.py)
and the manifest, computes per-trial metrics, and prints an aggregate table by
(course, controller). Velocity is derived from odometry POSE (this base reports
zero twist), which is why extraction keeps pose, not twist.
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics as st
from collections import defaultdict

ROBOT_RADIUS = 0.19
# Success radius on goal_err_map (the plan-derived map-frame closest approach).
# Set above the plan-rate floor: the planner stops republishing once nav
# declares the goal reached (~0.25 m out), so a perfect arrival still reads
# ~0.30 m; 0.6 cleanly separates "reached" (~0.3-0.5) from "stalled/aborted".
GOAL_TOL = 0.60
MOVE_EPS = 0.02          # m/s, "moving" threshold on derived speed


def load_course_goals(bags_dir):
    """course -> (goal_x, goal_y, tol). Looks for course_goals.yaml next to the
    field_campaign scripts (parent of bags_dir), tolerating its absence."""
    path = os.path.join(os.path.dirname(os.path.abspath(bags_dir.rstrip("/"))),
                        "course_goals.yaml")
    goals = {}
    if not os.path.exists(path):
        return goals
    try:
        import yaml
        for course, cfg in (yaml.safe_load(open(path)) or {}).items():
            gx, gy = cfg.get("goal_x"), cfg.get("goal_y")
            if gx is not None and gy is not None:
                goals[course] = (float(gx), float(gy),
                                 float(cfg.get("tol_m", GOAL_TOL)))
    except Exception as e:
        print("warning: could not read course_goals.yaml:", e)
    return goals


def controller_of(bagname):
    # <course>__<controller>__t<trial>
    parts = bagname.split("__")
    return parts[1] if len(parts) >= 2 else "unknown"


def course_of(bagname):
    return bagname.split("__")[0]


def derive(rows):
    """rows: list of (t,x,y,yaw,cmd_vx,cmd_wz,scan_min). Returns derived speed
    per row via central pose differencing, low-pass smoothed to tame the
    pose-differentiation noise."""
    n = len(rows)
    spd = [0.0] * n
    for i in range(1, n - 1):
        dt = rows[i + 1][0] - rows[i - 1][0]
        if dt > 1e-3:
            dx = rows[i + 1][1] - rows[i - 1][1]
            dy = rows[i + 1][2] - rows[i - 1][2]
            spd[i] = math.hypot(dx, dy) / dt
    if n > 1:
        spd[0], spd[-1] = spd[1], spd[-2]
    # 5-sample moving-average low-pass
    k = 5
    sm = list(spd)
    for i in range(n):
        lo, hi = max(0, i - k // 2), min(n, i + k // 2 + 1)
        sm[i] = sum(spd[lo:hi]) / (hi - lo)
    return sm


def trial_metrics(bagdir, course_goals=None):
    csv_path = os.path.join(bagdir, "extracted.csv")
    meta_path = os.path.join(bagdir, "extracted_meta.json")
    if not (os.path.exists(csv_path) and os.path.exists(meta_path)):
        return None
    meta = json.load(open(meta_path))
    rows = []
    map_xy = []          # map-frame (x,y) per row for goal-reaching
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            rows.append((float(r["t"]), float(r["x"]), float(r["y"]),
                         float(r["yaw"]), float(r["cmd_vx"]), float(r["cmd_wz"]),
                         float(r["scan_min"])))
            # map_x/map_y present in new extractions; fall back to odom x,y
            mx = float(r.get("map_x", "nan"))
            my = float(r.get("map_y", "nan"))
            map_xy.append((mx, my) if not math.isnan(mx) else (rows[-1][1], rows[-1][2]))
    if len(rows) < 5:
        return None
    spd = derive(rows)

    # goal: prefer the per-trial captured goal; else the fixed course goal.
    # tolerance always comes from the course entry when present.
    course = course_of(meta["bag"])
    tol = GOAL_TOL
    if course_goals and course in course_goals:
        tol = course_goals[course][2]
    gx, gy = meta.get("goal_x"), meta.get("goal_y")
    if gx is None and course_goals and course in course_goals:
        gx, gy, _ = course_goals[course]
    # Success by CLOSEST APPROACH over the trajectory, not the final pose: this
    # robot does a small end-of-goal rotation that nudges it off the point after
    # arrival, so the last recorded pose understates goal-reaching. Closest
    # approach answers "did the robot reach the goal" and is immune to that
    # post-goal motion (and to cross-trial odometry drift).
    # Goal-reaching in the MAP frame. Best source is goal_err_map, the closest
    # approach of the plan-derived robot track to the goal (both map-frame,
    # computed in extraction) -- robust to odom drift and the end-of-goal
    # rotation. Fall back to the odom-frame final pose only if unavailable.
    gem = meta.get("goal_err_map")
    if gem is not None:
        goal_err = gem
        success = goal_err < tol
    elif gx is not None:
        goal_err = min(math.hypot(mx - gx, my - gy) for mx, my in map_xy)
        success = goal_err < tol
    else:
        goal_err = float("nan")
        success = False

    # motion window (first/last time the robot is actually moving). The window
    # is gated to the interval in which the navigation stack was actually
    # commanding the base (first .. last non-zero /cmd_vel_nav, with a small
    # margin for actuation lag): pose-differentiation noise while the robot
    # is parked before the goal is accepted registers as 0.05-0.08 m/s of
    # "motion" and would otherwise add the pre-goal wait (5-12 s, unrelated
    # to the controller) to time-to-goal for a random subset of trials.
    cmd_on = [i for i, r in enumerate(rows) if abs(r[4]) > 0.02 or abs(r[5]) > 0.05]
    if cmd_on:
        t_lo = rows[cmd_on[0]][0] - 0.5
        t_hi = rows[cmd_on[-1]][0] + 1.0
    else:
        t_lo, t_hi = -float("inf"), float("inf")
    moving = [i for i, v in enumerate(spd)
              if v > MOVE_EPS and t_lo <= rows[i][0] <= t_hi]
    # Time-to-goal is COMMAND-derived: from the first sustained forward command
    # (cmd_vx > 0.05 m/s for 5 consecutive samples, i.e. the end of the
    # RotationShim's in-place heading alignment, which is a wrapper shared by
    # all controllers and takes ~6 s in every cell) to the last non-zero
    # command. Pose-derived windows proved session-fragile: odometry
    # translation noise during the in-place rotation differed between
    # recording sessions by up to 3 s for otherwise identical command traces.
    rot_s = 0.0
    if cmd_on:
        j = cmd_on[0]
        while j < len(rows) - 5 and not all(rows[k][4] > 0.05 for k in range(j, j + 5)):
            j += 1
        rot_s = rows[j][0] - rows[cmd_on[0]][0]
        ttg_cmd = rows[cmd_on[-1]][0] - rows[j][0]
    else:
        ttg_cmd = 0.0
    if moving:
        ttg = ttg_cmd if cmd_on else rows[moving[-1]][0] - rows[moving[0]][0]
        cruise = st.median(spd[i] for i in moving)
        cruise_p90 = sorted(spd[i] for i in moving)[int(0.9 * len(moving)) - 1]
        # stall fraction: fraction of the motion window spent near-stopped
        # (< 0.05 m/s) -- captures the stall/recover behavior at a blockage
        # without depending on absolute speed.
        seg = spd[moving[0]:moving[-1] + 1]
        stall_frac = sum(1 for v in seg if v < 0.05) / max(1, len(seg))
    else:
        ttg = cruise = cruise_p90 = stall_frac = 0.0

    # path length (odom)
    plen = sum(math.hypot(rows[i][1] - rows[i - 1][1], rows[i][2] - rows[i - 1][2])
               for i in range(1, len(rows)))
    # clearance: nearest lidar return over the trial. Report the raw minimum,
    # but flag a COLLISION only on SUSTAINED contact -- the 1st-percentile
    # clearance below the radius -- so a single near-threshold lidar sample
    # (e.g. a 0.189 m grazing return) does not falsely fail a trial. The
    # operator's manifest note is the ground truth for genuine contact.
    clears = sorted(r[6] for r in rows if not math.isnan(r[6]))
    min_clear = clears[0] if clears else float("nan")
    p1_clear = clears[max(0, int(0.01 * len(clears)) - 1)] if clears else float("nan")
    collision = (not math.isnan(p1_clear)) and (p1_clear < ROBOT_RADIUS)
    # smoothness (primary): RMS of realized acceleration (d speed / dt) over
    # the moving window, from the low-passed pose-derived speed. Robust -- needs
    # only /odom, not the command topic (which this robot does not reliably
    # expose). Lower = smoother.
    accs = []
    for i in range(1, len(rows)):
        dt = rows[i][0] - rows[i - 1][0]
        if dt > 1e-3 and spd[i] > MOVE_EPS:
            accs.append((spd[i] - spd[i - 1]) / dt)
    accel_rms = math.sqrt(sum(a * a for a in accs) / len(accs)) if accs else 0.0
    # command jitter (secondary): only meaningful if /cmd_vel_nav was captured
    dvs = [rows[i][4] - rows[i - 1][4] for i in range(1, len(rows))
           if spd[i] > MOVE_EPS]
    rms_dv = math.sqrt(sum(d * d for d in dvs) / len(dvs)) if dvs else 0.0

    return {
        "bag": meta["bag"], "course": course_of(meta["bag"]),
        "controller": controller_of(meta["bag"]),
        "success": success and not collision, "collision": collision,
        "goal_err": round(goal_err, 3), "ttg": round(ttg, 2), "rot_s": round(rot_s, 2),
        "path_len": round(plen, 2), "cruise": round(cruise, 3),
        "cruise_p90": round(cruise_p90, 3), "min_clear": round(min_clear, 3),
        "accel_rms": round(accel_rms, 4), "rms_dv": round(rms_dv, 4),
        "cmd_jitter": meta.get("cmd_jitter"), "cmd_out_jitter": meta.get("cmd_out_jitter"),
        "stall_frac": round(stall_frac, 4),
        "duration": meta["duration_s"],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags_dir")
    ap.add_argument("--csv", help="also write per-trial metrics here")
    args = ap.parse_args()

    course_goals = load_course_goals(args.bags_dir)
    if course_goals:
        print("course goals:", {c: (round(g[0], 2), round(g[1], 2), g[2])
                                 for c, g in course_goals.items()}, "\n")
    bags = sorted(d for d in glob.glob(os.path.join(args.bags_dir, "*__*__t*"))
                  if os.path.isdir(d))
    trials = [m for m in (trial_metrics(b, course_goals) for b in bags) if m]
    if not trials:
        print("no extracted trials found under", args.bags_dir)
        print("(run extract_bag.py on the robot first, then copy bags back)")
        return

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(trials[0].keys()))
            w.writeheader()
            w.writerows(trials)
        print("per-trial ->", args.csv, f"({len(trials)} trials)\n")

    groups = defaultdict(list)
    for t in trials:
        groups[(t["course"], t["controller"])].append(t)

    print(f"{'course':<12} {'controller':<8} {'n':>3} {'succ%':>6} {'coll':>4} "
          f"{'ttg(s)':>10} {'cruise':>7} {'clear':>6} {'cmd_jit':>8}")
    for (course, ctrl), ts in sorted(groups.items()):
        n = len(ts)
        succ = sum(1 for t in ts if t["success"])
        coll = sum(1 for t in ts if t["collision"])
        ok = [t for t in ts if t["success"]]
        ttgs = [t["ttg"] for t in ok] or [0]
        cru = [t["cruise"] for t in ok] or [0]
        clr = [t["min_clear"] for t in ts if not math.isnan(t["min_clear"])] or [0]
        cj = [t["cmd_jitter"] for t in ok if t["cmd_jitter"] is not None]
        ttg = f"{st.mean(ttgs):5.1f}±{(st.stdev(ttgs) if len(ttgs)>1 else 0):3.1f}"
        cjs = f"{st.mean(cj):.4f}" if cj else "   n/a"
        print(f"{course:<12} {ctrl:<8} {n:>3} {100*succ/n:>5.0f}% {coll:>4} "
              f"{ttg:>10} {st.mean(cru):>7.3f} {min(clr):>6.3f} {cjs:>8}")


if __name__ == "__main__":
    main()
