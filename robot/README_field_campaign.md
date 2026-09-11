# Field campaign toolkit

Tooling to run the ICRA hardware campaign: swap controllers fairly, record each
trial as a rosbag, and compute metrics offline. Target: 3 courses × 3
controllers × ≥10 trials.

## What's here

```
controllers/                three MATCHED Nav2 configs — identical costmap,
  navigation_params_dwb.yaml     only the FollowPath block differs, so the
  navigation_params_comppi.yaml  controller is the single changed variable
  navigation_params_mppi.yaml
swap_controller.sh          deploy one config on the robot (backs up first)
record_trial.sh            record one trial as a rosbag (run on the robot)
extract_bag.py             bag -> CSV, on the robot (needs rosbag2_py)
analyze_trials.py          CSVs -> per-trial + aggregate metrics (dev machine)
courses.md                 course definitions
```

The three configs were built from the robot's own deployed `navigation_params`
by swapping only the FollowPath block; their `local_costmap` sections are
byte-identical (verified by md5), which is the fairness property reviewers check
first.

## Before the campaign (once)

On the robot, make discovery survive restarts (otherwise recording/introspection
break after each bringup restart — we hit this repeatedly):
```
sudo chmod 777 /dev/shm         # or add to the bringup script permanently
```
Also fix NTP so bag timestamps are sane. Copy this `field_campaign/` folder to
the robot (e.g. it's already in `~/<robot_ws>/src/comppi_controller/`).

## Per-controller block (do all trials for one controller, then switch)

1. **Swap** (on robot): `./swap_controller.sh comppi`   (or `dwb` / `mppi`)
2. **Restart bringup**, undock. Verify the controller loaded:
   `ROS_DOMAIN_ID=30 ros2 param get /controller_server FollowPath.plugin`
3. For each course, run ≥10 trials:
   `./record_trial.sh <course> comppi <trial_no>`
   - the script starts the bag, you **send the goal** through the normal robot
     UI, then press **ENTER** when it reaches the goal (or you stop it), and
     type a one-word note (`clean` / `grazed cone` / `stalled`).
4. Keep the **e-stop in hand** every trial — comppi is new on this robot.
   Rollback anytime: `cp <params>.prev <params>` (or `.orig`) + restart.

Recommended order to minimize controller swaps: all of controller A's courses,
then B, then C. Randomize trial order within a course if you can.

## After the campaign

On the robot, extract every bag to CSV:
```
source /opt/ros/humble/setup.bash
python3 extract_bag.py bags/*__*__t*
```
Copy the bags (or just the small `extracted*.csv/json`) back to the dev machine,
then:
```
python3 analyze_trials.py bags/ --csv hardware_results.csv
```
That prints the aggregate table (success %, collisions, time-to-goal, cruise
speed, clearance, jitter) by course × controller, and writes per-trial rows for
the paper. Feed `hardware_results.csv` into the figures pipeline for the
camera-ready hardware figure (replaces the preliminary `fig_hw_velocity`).

## Metric notes (robot-specific)

- **Velocity is derived from odometry pose, not twist** — this base publishes a
  zero twist. `extract_bag.py` keeps pose; `analyze_trials.py` differentiates it.
- **Clearance** is the nearest `/scan` return over the trial (raw range). A
  value below the robot radius (0.19 m) is flagged as a collision.
- **Success** = final pose within 0.30 m of the recorded goal (last `/plan`
  pose) and no collision.
