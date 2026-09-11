#!/usr/bin/env python3
"""Extract one field-campaign rosbag into a flat CSV + meta JSON for offline
analysis. Runs ON the robot (needs rosbag2_py / the ROS 2 message types).

    source /opt/ros/humble/setup.bash
    python3 extract_bag.py bags/corridor__comppi__t3

Writes <bagdir>/extracted.csv  (t, x, y, yaw, cmd_vx, cmd_wz, scan_min)
   and <bagdir>/extracted_meta.json  (goal, t0, t1, controller, ...)

Velocity is intentionally NOT read from odometry twist -- this base reports a
zero twist -- it is derived from pose in the offline analyzer instead.
"""
import json
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(bagdir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bagdir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}

    odom, amcl, cmd, scan = [], [], [], []   # (t, ...) rows per topic
    cmd_out = []                              # /cmd_vel after the velocity smoother
    # goal candidates, in priority order (this robot's custom UI first)
    g_update = g_smoothed = g_coord = g_pose = g_plan = None
    # A global plan is map-frame and STARTS at the robot's current pose, so the
    # sequence of plan first-poses is the robot's map-frame trajectory -- the
    # only reliable map-frame position source on this robot (/amcl_pose is
    # silent, no map->odom TF). Used to score goal-reaching in the goal's frame.
    plan_robot = []          # [(x,y), ...] robot map-frame positions
    while reader.has_next():
        topic, data, tstamp = reader.read_next()
        t = tstamp * 1e-9
        typ = tmap.get(topic, "")
        if topic == "/odom":
            m = deserialize_message(data, get_message(typ))
            p = m.pose.pose
            odom.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic == "/amcl_pose":            # MAP-frame robot pose (goal frame)
            m = deserialize_message(data, get_message(typ))
            p = m.pose.pose
            amcl.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic == "/cmd_vel_nav":
            m = deserialize_message(data, get_message(typ))
            cmd.append((t, m.linear.x, m.angular.z))
        elif topic == "/cmd_vel":              # after the platform velocity smoother
            m = deserialize_message(data, get_message(typ))
            cmd_out.append((t, m.linear.x, m.angular.z))
        elif topic == "/scan":
            m = deserialize_message(data, get_message(typ))
            rs = [r for r in m.ranges
                  if not math.isinf(r) and not math.isnan(r) and r > m.range_min]
            scan.append((t, min(rs) if rs else float("nan")))
        elif topic == "/goal_update":          # custom UI goal (PoseStamped)
            m = deserialize_message(data, get_message(typ))
            gp = m.pose.position
            g_update = (gp.x, gp.y)
        elif topic == "/plan_smoothed":         # custom global plan (Path)
            m = deserialize_message(data, get_message(typ))
            if m.poses:
                gp = m.poses[-1].pose.position
                g_smoothed = (gp.x, gp.y)
                rp = m.poses[0].pose.position
                plan_robot.append((rp.x, rp.y))
        elif topic == "/path_planning/coordiante":  # destination (Float64MultiArray)
            m = deserialize_message(data, get_message(typ))
            if len(m.data) >= 2:
                g_coord = (m.data[0], m.data[1])
        elif topic == "/goal_pose":             # standard Nav2 (fallback)
            m = deserialize_message(data, get_message(typ))
            gp = m.pose.position
            g_pose = (gp.x, gp.y)
        elif topic == "/plan":                  # standard Nav2 (fallback)
            m = deserialize_message(data, get_message(typ))
            if m.poses:
                gp = m.poses[-1].pose.position
                g_plan = (gp.x, gp.y)
                rp = m.poses[0].pose.position
                plan_robot.append((rp.x, rp.y))
    goal = g_update or g_smoothed or g_pose or g_plan or g_coord
    goal_source = ("goal_update" if g_update else "plan_smoothed" if g_smoothed
                   else "goal_pose" if g_pose else "plan" if g_plan
                   else "coordiante" if g_coord else None)
    # map-frame closest approach to the goal (robust to odom drift and the
    # end-of-goal rotation): min distance from the plan-derived robot track,
    # which is in the same (map) frame as the goal.
    goal_err_map = None
    if goal is not None and plan_robot:
        goal_err_map = min(math.hypot(px - goal[0], py - goal[1])
                           for px, py in plan_robot)

    # command jitter: RMS of consecutive linear-command changes at the native
    # /cmd_vel_nav rate, over the moving portion. Speed-independent (unlike the
    # pose-derived acceleration), so it is the clean smoothness metric and
    # matches the simulation study's rms_dv. None if /cmd_vel_nav absent.
    cmd.sort()
    mv = [c for c in cmd if abs(c[1]) > 0.02]
    cmd_jitter = None
    if len(mv) > 5:
        dvs = [mv[i][1] - mv[i - 1][1] for i in range(1, len(mv))]
        cmd_jitter = round(math.sqrt(sum(d * d for d in dvs) / len(dvs)), 5)
    # the same metric on /cmd_vel, i.e. after the platform's velocity smoother
    # (identical for every controller): the system-level smoothness.
    cmd_out.sort()
    mvo = [c for c in cmd_out if abs(c[1]) > 0.02]
    cmd_out_jitter = None
    if len(mvo) > 5:
        dvs = [mvo[i][1] - mvo[i - 1][1] for i in range(1, len(mvo))]
        cmd_out_jitter = round(math.sqrt(sum(d * d for d in dvs) / len(dvs)), 5)

    if not odom:
        print("no /odom in bag:", bagdir)
        return 1

    # merge onto the odom time base: carry latest cmd + scan + amcl pose
    cmd.sort(); scan.sort(); amcl.sort()
    ci = si = ai = 0
    last_cmd = (0.0, 0.0)
    last_scan = float("nan")
    last_amcl = (float("nan"), float("nan"))   # map-frame x,y
    rows = []
    for t, x, y, yaw in sorted(odom):
        while ci < len(cmd) and cmd[ci][0] <= t:
            last_cmd = (cmd[ci][1], cmd[ci][2]); ci += 1
        while si < len(scan) and scan[si][0] <= t:
            last_scan = scan[si][1]; si += 1
        while ai < len(amcl) and amcl[ai][0] <= t:
            last_amcl = (amcl[ai][1], amcl[ai][2]); ai += 1
        rows.append((t, x, y, yaw, last_cmd[0], last_cmd[1], last_scan,
                     last_amcl[0], last_amcl[1]))

    t0 = rows[0][0]
    with open(os.path.join(bagdir, "extracted.csv"), "w") as f:
        # x,y (odom frame) drive velocity; map_x,map_y (/amcl_pose) drive
        # goal-reaching. map_* are nan if /amcl_pose was not recorded.
        f.write("t,x,y,yaw,cmd_vx,cmd_wz,scan_min,map_x,map_y\n")
        for r in rows:
            f.write(f"{r[0]-t0:.4f},{r[1]:.4f},{r[2]:.4f},{r[3]:.4f},"
                    f"{r[4]:.4f},{r[5]:.4f},{r[6]:.4f},{r[7]:.4f},{r[8]:.4f}\n")

    meta = {
        "bag": os.path.basename(bagdir.rstrip("/")),
        "goal_x": goal[0] if goal else None,
        "goal_y": goal[1] if goal else None,
        "goal_source": goal_source,
        "goal_err_map": round(goal_err_map, 4) if goal_err_map is not None else None,
        "cmd_jitter": cmd_jitter, "cmd_out_jitter": cmd_out_jitter,
        "n_plan": len(plan_robot),
        "duration_s": round(rows[-1][0] - t0, 2),
        "n_odom": len(odom), "n_amcl": len(amcl),
        "n_cmd": len(cmd), "n_scan": len(scan),
    }
    with open(os.path.join(bagdir, "extracted_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print(f"extracted {os.path.basename(bagdir)}: {len(rows)} samples, "
          f"{meta['duration_s']}s, goal={goal}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: extract_bag.py <bagdir> [<bagdir> ...]")
        sys.exit(2)
    rc = 0
    for d in sys.argv[1:]:
        rc |= main(d)
    sys.exit(rc)
