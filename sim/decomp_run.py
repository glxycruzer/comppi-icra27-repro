#!/usr/bin/env python3
"""Capture the per-critic cost decomposition for i.i.d. / block / colored noise
on one open-world cycle. Launches the controller with COMPPI_CRITIC_DUMP set so
the optimizer writes per-sample (mean vx, per-critic cost) for its first batch.

  source /opt/ros/humble/setup.bash && source ../../../install/setup.bash
  ROS_DOMAIN_ID=77 python3 decomp_run.py

Outputs decomp_<mode>.csv in the outdir.
"""
import os
import subprocess
import time

import yaml

import worlds

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "decomp")
os.makedirs(OUT, exist_ok=True)

MODES = {
    "iid":     {"noise_block_steps": 1},
    "block":   {"noise_block_steps": 8},
    "colored": {"noise_mode": "colored"},
}


def kill():
    for p in ("lib/nav2_controller/controller_server", "lib/nav2_map_server/map_server",
              "lib/nav2_lifecycle_manager/lifecycle_manager", "simval/kin_sim.py"):
        subprocess.run(["pkill", "-9", "-f", p], capture_output=True)
    time.sleep(1)


def run(mode, override):
    w = worlds.make_world("open", 0)
    scen = worlds.write_scenario(OUT, w)
    mapy = worlds.write_map(w["grid_true"], OUT)
    cfg = yaml.safe_load(open(os.path.join(HERE, "sim_params.yaml")))
    fp = cfg["controller_server"]["ros__parameters"]["FollowPath"]
    fp.update(override)
    params = os.path.join(OUT, f"params_{mode}.yaml")
    yaml.safe_dump(cfg, open(params, "w"))
    dump = os.path.join(OUT, f"decomp_{mode}.csv")
    if os.path.exists(dump):
        os.remove(dump)

    kill()
    env = dict(os.environ, COMPPI_CRITIC_DUMP=dump)
    logf = open(os.path.join(OUT, f"log_{mode}.txt"), "w")
    procs = [
        subprocess.Popen(["ros2", "run", "nav2_map_server", "map_server", "--ros-args",
                          "-p", f"yaml_filename:={mapy}"], stdout=logf, stderr=subprocess.STDOUT),
        subprocess.Popen(["ros2", "run", "nav2_controller", "controller_server", "--ros-args",
                          "--params-file", params], stdout=logf, stderr=subprocess.STDOUT, env=env),
        subprocess.Popen(["python3", os.path.join(HERE, "kin_sim.py"), "--ros-args",
                          "-p", "x0:=1.0", "-p", "y0:=3.0", "-p", "yaw0:=0.0",
                          "-p", f"log_path:={OUT}/traj_{mode}.csv"], stdout=logf, stderr=subprocess.STDOUT),
        subprocess.Popen(["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args",
                          "-p", "autostart:=true", "-p", "node_names:=[map_server, controller_server]"],
                         stdout=logf, stderr=subprocess.STDOUT),
    ]
    time.sleep(8)
    subprocess.run(["python3", os.path.join(HERE, "run_trial.py"),
                    os.path.join(OUT, "scenario.json")], capture_output=True, timeout=30)
    time.sleep(1)
    for p in procs:
        p.kill()
    kill()
    logf.close()
    ok = os.path.exists(dump) and sum(1 for _ in open(dump)) > 1
    print(f"{mode}: {'dumped ' + dump if ok else 'NO DUMP'}")


if __name__ == "__main__":
    for m, o in MODES.items():
        run(m, o)
    print("done ->", OUT)
