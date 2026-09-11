#!/bin/bash
# ARM SoC solve-time profiling: one closed-loop sim trial per batch size.
# Runs on an isolated ROS domain so the robot's live stack is untouched.
source /opt/ros/humble/setup.bash
source ~/<robot_ws>/install/setup.bash
export ROS_DOMAIN_ID=88
SV=~/<robot_ws>/src/comppi_controller/test/simval
A=$SV/profile_assets

cleanup() {
  pkill -9 -f "lib/nav2_controller/controller_server" 2>/dev/null
  pkill -9 -f "lib/nav2_map_server/map_server" 2>/dev/null
  pkill -9 -f "lib/nav2_lifecycle_manager/lifecycle_manager" 2>/dev/null
  pkill -9 -f "simval/kin_sim.py" 2>/dev/null
  sleep 1
}

for BS in 256 512 1024 2048; do
  echo "=== batch_size $BS ==="
  cleanup
  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$A/map.yaml >/tmp/prof_map.log 2>&1 &
  ros2 run nav2_controller controller_server --ros-args --params-file $A/params_b$BS.yaml >/tmp/prof_ctrl_$BS.log 2>&1 &
  CTRL=$!
  python3 $SV/kin_sim.py --ros-args -p x0:=1.2 -p y0:=5.0 -p yaw0:=0.0 -p log_path:=/tmp/prof_traj_$BS.csv >/tmp/prof_sim.log 2>&1 &
  ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true -p "node_names:=[map_server, controller_server]" >/tmp/prof_lm.log 2>&1 &
  sleep 10
  timeout 90 python3 $SV/run_trial.py $A/scenario.json
  ps -o %cpu= -p $(pgrep -f "lib/nav2_controller/controller_server" | head -1) 2>/dev/null | xargs echo "controller CPU% (proc avg):"
  grep -c "missed its desired rate" /tmp/prof_ctrl_$BS.log | xargs echo "loop overruns:"
  cleanup
done
echo "=== profile line counts ==="
wc -l /tmp/comppi_profile_*.txt
exit 0
