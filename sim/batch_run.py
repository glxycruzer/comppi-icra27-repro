#!/usr/bin/env python3
"""Monte-Carlo benchmark: worlds x seeds x controllers -> results.csv.

Run from a shell with ROS Humble + the workspace sourced:
  python3 batch_run.py --controllers comppi,dwb,mppi,comppi_iid,comppi_nodetour \
      --worlds open,clutter,gap,blocked --seeds 25 --outdir results/
"""
import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import time

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt

import worlds

HERE = os.path.dirname(os.path.abspath(__file__))
ESS_LOG = False

CONTROLLERS = {
    "comppi": ("sim_params.yaml", {}),
    "comppi_iid": ("sim_params.yaml", {"noise_block_steps": 1}),
    "comppi_colored": ("sim_params.yaml", {"noise_mode": "colored"}),
    "comppi_nodetour": ("sim_params.yaml", {"detour_enabled": False}),
    "comppi_noanneal": ("sim_params.yaml", {"detour_lambda_scale": 1.0}),
    "dwb": ("params_dwb.yaml", {}),
    "mppi": ("params_mppi.yaml", {}),
    "mppi_tuned": ("params_mppi_tuned.yaml", {}),
    # block-size sweep (b1 == comppi_iid, b8 == comppi default, b32 == 1 block)
    "comppi_b2": ("sim_params.yaml", {"noise_block_steps": 2}),
    "comppi_b4": ("sim_params.yaml", {"noise_block_steps": 4}),
    "comppi_b16": ("sim_params.yaml", {"noise_block_steps": 16}),
    "comppi_b32": ("sim_params.yaml", {"noise_block_steps": 32}),
    # C1 discriminating experiments (all i.i.d. noise): is the stall caused by
    # sigma/sqrt(T) alone, or by i.i.d. noise passing through the rollout's
    # acceleration limit (Nav2-MPPI Humble has none and cruises with i.i.d.)?
    "comppi_iid_nolimit": ("sim_params.yaml", {"noise_block_steps": 1, "ax_max": 100.0, "ax_min": -100.0}),
    "comppi_iid_amax1":   ("sim_params.yaml", {"noise_block_steps": 1, "ax_max": 1.0, "ax_min": -1.0}),
    "comppi_iid_symacc":  ("sim_params.yaml", {"noise_block_steps": 1, "ax_min": -0.25}),
    "comppi_iid_l010":    ("sim_params.yaml", {"noise_block_steps": 1, "lambda": 0.1}),
    "comppi_iid_l003":    ("sim_params.yaml", {"noise_block_steps": 1, "lambda": 0.03}),
    # ESS-stall boundary: lambda between the stalling default (0.35, ESS 633) and the released 0.10
    "comppi_iid_l015":    ("sim_params.yaml", {"noise_block_steps": 1, "lambda": 0.15}),
    "comppi_iid_l020":    ("sim_params.yaml", {"noise_block_steps": 1, "lambda": 0.20}),
    "comppi_iid_l025":    ("sim_params.yaml", {"noise_block_steps": 1, "lambda": 0.25}),
    "comppi_iid_l030":    ("sim_params.yaml", {"noise_block_steps": 1, "lambda": 0.30}),
    # what the latency headroom buys: larger sample count / longer horizon at the same 10 Hz
    "comppi_k2048": ("sim_params.yaml", {"batch_size": 2048}),
    "comppi_t48":   ("sim_params.yaml", {"time_steps": 48}),
    # adaptive-temperature baselines (rebuttal Q1): i.i.d. noise, lambda set each cycle to hit an ESS target,
    # or costs normalised by their std before the softmax. Require the adaptive-lambda overlay build.
    "comppi_iid_ess05":  ("sim_params.yaml", {"noise_block_steps": 1, "ess_target_frac": 0.05}),
    "comppi_iid_ess10":  ("sim_params.yaml", {"noise_block_steps": 1, "ess_target_frac": 0.10}),
    "comppi_iid_ess20":  ("sim_params.yaml", {"noise_block_steps": 1, "ess_target_frac": 0.20}),
    "comppi_iid_cnorm":  ("sim_params.yaml", {"noise_block_steps": 1, "cost_normalize": True}),
    # reviewer 6(c): does Nav2-MPPI recover the blocked path if its own path-align gate fires at
    # CoMPPI's sensitivity (occupancy ratio 0.01 instead of 0.05) and/or its temperature is sharpened?
    "mppi_align001":     ("params_mppi.yaml", {"PathAlignCritic": {"max_path_occupancy_ratio": 0.01}}),
    "mppi_align001_t01": ("params_mppi.yaml", {"PathAlignCritic": {"max_path_occupancy_ratio": 0.01}, "temperature": 0.1}),
    "mppi_t01":          ("params_mppi.yaml", {"temperature": 0.1}),
    # C1 scope: raise a_max at the same 4:1 asymmetry (kappa = a_max*dt/sigma_v = 0.25, 0.5)
    "comppi_iid_a05r4":  ("sim_params.yaml", {"noise_block_steps": 1, "ax_max": 0.5, "ax_min": -2.0}),
    "comppi_iid_a10r4":  ("sim_params.yaml", {"noise_block_steps": 1, "ax_max": 1.0, "ax_min": -4.0}),
    # detour-mode obstacle-weight scaling (dynamic-obstacle clearance remedy)
    "comppi_dos2": ("sim_params.yaml", {"detour_obstacle_scale": 2.0}),
    "comppi_dos4": ("sim_params.yaml", {"detour_obstacle_scale": 4.0}),
    # blockage-detection threshold sensitivity (default: cost 253 = inscribed,
    # range 1.0 x auto lookahead)
    "comppi_bc200": ("sim_params.yaml", {"detour_block_cost": 200}),
    "comppi_bc150": ("sim_params.yaml", {"detour_block_cost": 150}),
    "comppi_bc100": ("sim_params.yaml", {"detour_block_cost": 100}),
    "comppi_rs05": ("sim_params.yaml", {"detour_range_scale": 0.5}),
    "comppi_rs15": ("sim_params.yaml", {"detour_range_scale": 1.5}),
    "comppi_rs20": ("sim_params.yaml", {"detour_range_scale": 2.0}),
    # C1 sigma sweep (i.i.d. noise): does the stall need sigma_v >> a_max*dt?
    "comppi_iid_s003": ("sim_params.yaml", {"noise_block_steps": 1, "noise_std_v": 0.03}),
    "comppi_iid_s005": ("sim_params.yaml", {"noise_block_steps": 1, "noise_std_v": 0.05}),
    "comppi_iid_s010": ("sim_params.yaml", {"noise_block_steps": 1, "noise_std_v": 0.10}),
    "comppi_iid_s020": ("sim_params.yaml", {"noise_block_steps": 1, "noise_std_v": 0.20}),
    "comppi_iid_s030": ("sim_params.yaml", {"noise_block_steps": 1, "noise_std_v": 0.30}),
    # C2 sharpening-ratio sweep (rho = detour_lambda_scale; 0.25 = default, 1.0 = comppi_noanneal)
    "comppi_rho010": ("sim_params.yaml", {"detour_lambda_scale": 0.10}),
    "comppi_rho050": ("sim_params.yaml", {"detour_lambda_scale": 0.50}),
    # C1 external-validity test: stock Nav2-MPPI (Humble 1.1.20) built from source with a
    # 30-line patch that adds a per-step acceleration limit INSIDE the rollouts (paper Eq. 1)
    # and optional block noise. Requires the patched overlay workspace to be sourced.
    "mppi_patch":       ("params_mppi.yaml", {}),                                   # patch present, disabled (control)
    "mppi_patch_lim":   ("params_mppi.yaml", {"rollout_ax_max": 0.25, "rollout_ax_min": -1.0}),   # our limits, i.i.d.
    "mppi_patch_sym":   ("params_mppi.yaml", {"rollout_ax_max": 0.25, "rollout_ax_min": -0.25}),  # symmetric limit
    "mppi_patch_block": ("params_mppi.yaml", {"rollout_ax_max": 0.25, "rollout_ax_min": -1.0, "noise_block_steps": 8}),
    # temperature leg of the C1 conjunction inside Nav2-MPPI: flatten lambda with / without the rollout limit
    "mppi_patch_lim_t10": ("params_mppi.yaml", {"rollout_ax_max": 0.25, "rollout_ax_min": -1.0, "temperature": 1.0}),
    "mppi_patch_lim_t30": ("params_mppi.yaml", {"rollout_ax_max": 0.25, "rollout_ax_min": -1.0, "temperature": 3.0}),
    "mppi_patch_t30":     ("params_mppi.yaml", {"temperature": 3.0}),
}


def kill_stray(rundir):
    """Kill leftover nodes of THIS trial only (their command lines carry rundir
    paths). Never pattern-kill unscoped: concurrent campaigns in other ROS
    domains would lose their nodes mid-trial."""
    import re
    for pat in ("lib/nav2_controller/controller_server",
                "lib/nav2_map_server/map_server",
                "simval/kin_sim.py", "simval/dyn_obstacle.py"):
        subprocess.run(["pkill", "-9", "-f", pat + ".*" + re.escape(rundir)], capture_output=True)
    time.sleep(1.0)


def kill_procs(procs):
    """Kill each launched process group (ros2 run wrapper + its child binary)."""
    import signal
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def write_params(base, overrides, out):
    cfg = yaml.safe_load(open(os.path.join(HERE, base)))
    fp = cfg["controller_server"]["ros__parameters"]["FollowPath"]
    for k, v in overrides.items():   # deep-merge so nested critic blocks keep their other keys
        if isinstance(v, dict) and isinstance(fp.get(k), dict):
            fp[k].update(v)
        else:
            fp[k] = v
    with open(out, "w") as f:
        yaml.safe_dump(cfg, f)


def metrics(traj_csv, scen, grid_true):
    rows = []
    with open(traj_csv) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    if len(rows) < 10:
        return None
    edt = distance_transform_edt(~grid_true) * worlds.RES
    gx, gy = scen["goal"]
    moving = [i for i, r in enumerate(rows) if abs(r["v"]) > 0.01 or abs(r["w"]) > 0.01]
    ttg = (rows[moving[-1]]["t"] - rows[moving[0]]["t"]) if moving else 0.0
    goal_err = math.hypot(rows[-1]["x"] - gx, rows[-1]["y"] - gy)
    plen, min_clear, vmax = 0.0, 1e9, 0.0
    for i, r in enumerate(rows):
        if i:
            plen += math.hypot(r["x"] - rows[i - 1]["x"], r["y"] - rows[i - 1]["y"])
        ci = min(int(r["y"] / worlds.RES), worlds.N - 1)
        cj = min(int(r["x"] / worlds.RES), worlds.N - 1)
        min_clear = min(min_clear, edt[ci, cj])
        vmax = max(vmax, r["v"])
    diffs = [rows[i]["v"] - rows[i - 5]["v"] for i in range(5, len(rows), 5)]
    rms_dv = math.sqrt(sum(d * d for d in diffs) / max(1, len(diffs)))
    return {"ttg": round(ttg, 2), "goal_err": round(goal_err, 3),
            "path_len": round(plen, 2), "min_clear": round(min_clear, 3),
            "rms_dv": round(rms_dv, 4), "vmax": round(vmax, 3),
            "collision": min_clear < worlds.ROBOT_R}


def dyn_clearance(traj_csv, dyn_csv, size):
    """Minimum clearance between the robot disc and the moving box (axis-aligned
    square of side `size`) over the trial, aligned on absolute ROS time.
    Negative = overlap = collision."""
    rob = [(float(r["t_abs"]), float(r["x"]), float(r["y"]), abs(float(r["v"])))
           for r in csv.DictReader(open(traj_csv)) if r.get("t_abs")]
    box = [(float(r["t_abs"]), float(r["bx"]), float(r["by"]))
           for r in csv.DictReader(open(dyn_csv)) if r.get("t_abs")]
    if not rob or not box:
        return None
    bt = np.array([b[0] for b in box])
    h = size / 2.0
    best_moving, best_any = 1e9, 1e9
    for t, x, y, v in rob:
        # box pose in force at time t = the LAST published pose (the costmap
        # holds it until the next 5 Hz update), not the next one
        k = int(np.clip(np.searchsorted(bt, t, side="right") - 1, 0, len(box) - 1))
        _, bx, by = box[k]
        dx = max(abs(x - bx) - h, 0.0)
        dy = max(abs(y - by) - h, 0.0)
        c = math.hypot(dx, dy) - worlds.ROBOT_R
        best_any = min(best_any, c)
        if v > 0.05:          # robot-at-fault: contact while the robot is moving
            best_moving = min(best_moving, c)
    return best_moving, best_any


def run_trial(world_kind, seed, controller, outdir):
    rundir = os.path.join(outdir, f"{world_kind}_s{seed}_{controller}")
    os.makedirs(rundir, exist_ok=True)
    w = worlds.make_world(world_kind, seed)
    scen = worlds.write_scenario(rundir, w)
    if scen is None:
        return {"success": False, "note": "planner_failed"}
    map_yaml = worlds.write_map(w["grid_true"], rundir)
    base, overrides = CONTROLLERS[controller]
    params = os.path.join(rundir, "params.yaml")
    write_params(base, overrides, params)

    kill_stray(rundir)
    x0, y0, yaw0 = scen["start"]
    traj = os.path.join(rundir, "traj.csv")
    dyn_log = os.path.join(rundir, "dyn.csv")
    logf = open(os.path.join(rundir, "nodes.log"), "w")
    if w.get("dyn"):
        d = w["dyn"]
        map_cmd = ["python3", os.path.join(HERE, "dyn_obstacle.py"), "--ros-args",
                   "-p", f"grid_path:={os.path.join(rundir, 'grid_true.npy')}",
                   "-p", f"log_path:={dyn_log}",
                   "-p", f"xc:={d['xc']}", "-p", f"y0:={d['y0']}", "-p", f"vy:={d['vy']}",
                   "-p", f"size:={d['size']}", "-p", f"trigger_dx:={d['trigger_dx']}"]
    else:
        map_cmd = ["ros2", "run", "nav2_map_server", "map_server", "--ros-args",
                   "-p", f"yaml_filename:={map_yaml}"]
    env = dict(os.environ)
    if ESS_LOG:  # per-cycle ESS and detour flag from the controller (opt-in diagnostic)
        env["COMPPI_ESS_LOG"] = os.path.join(rundir, "ess.csv")
        env["MPPI_ESS_LOG"] = env["COMPPI_ESS_LOG"]  # same file, read by the patched Nav2-MPPI
    procs = [subprocess.Popen(c, stdout=logf, stderr=subprocess.STDOUT, env=env,
                              start_new_session=True) for c in (
        map_cmd,
        ["ros2", "run", "nav2_controller", "controller_server", "--ros-args",
         "--params-file", params],
        ["python3", os.path.join(HERE, "kin_sim.py"), "--ros-args",
         "-p", f"x0:={x0}", "-p", f"y0:={y0}", "-p", f"yaw0:={yaw0}",
         "-p", f"log_path:={traj}"],
        ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args",
         "-p", "autostart:=true",
         "-p", "node_names:=[controller_server]" if w.get("dyn")
         else "node_names:=[map_server, controller_server]"],
    )]
    time.sleep(8.0)
    try:
        res = subprocess.run(
            ["python3", os.path.join(HERE, "run_trial.py"),
             os.path.join(rundir, "scenario.json")],
            capture_output=True, text=True, timeout=150)
        success = res.returncode == 0
        note = res.stdout.strip().splitlines()[-1] if res.stdout else ""
    except subprocess.TimeoutExpired:
        success, note = False, "driver_timeout"
    finally:
        kill_procs(procs)
        kill_stray(rundir)
        logf.close()

    m = metrics(traj, scen, w["grid_true"]) if os.path.exists(traj) else None
    row = {"world": world_kind, "seed": seed, "controller": controller,
           "success": success, "note": note}
    if m:
        row.update(m)
        if m["collision"]:
            row["success"] = False
    if w.get("dyn") and m and os.path.exists(dyn_log):
        dc = dyn_clearance(traj, dyn_log, w["dyn"]["size"])
        if dc is not None:
            # collision = overlap while the ROBOT is moving. A box walking into
            # a robot that has already stopped is the box's doing (a pedestrian
            # would not), and is reported separately as min_dyn_clear_any.
            row["min_dyn_clear"] = round(dc[0], 3)
            row["min_dyn_clear_any"] = round(dc[1], 3)
            # half-cell tolerance (2.5 cm): the box is rendered on the 5 cm
            # costmap grid and the static-world check carries the same
            # quantization; the raw clearance is kept for re-thresholding.
            if dc[0] < -worlds.RES / 2:
                row["collision"] = True
                row["success"] = False
                row["note"] = (row.get("note", "") + " dyn_collision").strip()
    # keep only small artifacts unless the trial failed
    if row["success"]:
        for fn in ("map.pgm", "grid_true.npy", "nodes.log"):
            try:
                os.remove(os.path.join(rundir, fn))
            except OSError:
                pass
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controllers", default="comppi,dwb,mppi")
    ap.add_argument("--worlds", default="open,clutter,gap,blocked")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--seed-offset", type=int, default=0)
    ap.add_argument("--ess-log", action="store_true", help="write per-cycle ESS + detour flag per trial (ess.csv)")
    ap.add_argument("--outdir", default=os.path.join(HERE, "results"))
    args = ap.parse_args()
    global ESS_LOG
    ESS_LOG = args.ess_log

    os.makedirs(args.outdir, exist_ok=True)
    results_csv = os.path.join(args.outdir, "results.csv")
    fields = ["world", "seed", "controller", "success", "collision", "ttg",
              "goal_err", "path_len", "min_clear", "min_dyn_clear", "min_dyn_clear_any",
              "rms_dv", "vmax", "note"]
    new = not os.path.exists(results_csv)
    with open(results_csv, "a", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        if new:
            wr.writeheader()
        for wk in args.worlds.split(","):
            for seed in range(args.seed_offset, args.seed_offset + args.seeds):
                for ctrl in args.controllers.split(","):
                    t0 = time.time()
                    row = run_trial(wk, seed, ctrl, args.outdir)
                    wr.writerow(row)
                    f.flush()
                    print(f"[{time.strftime('%H:%M:%S')}] {wk} s{seed} {ctrl}: "
                          f"{'OK' if row['success'] else 'FAIL'} "
                          f"({time.time()-t0:.0f}s) {row.get('note','')}",
                          flush=True)
    print("done ->", results_csv)


if __name__ == "__main__":
    main()
