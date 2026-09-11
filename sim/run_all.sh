#!/bin/bash
# Closed-loop validation: real controller_server + comppi plugin + static map
# + kinematic robot. Two scenarios: free-space run, path blocked by a box.
DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source <your_ros2_workspace>/install/setup.bash
export ROS_DOMAIN_ID=77
export RCUTILS_LOGGING_BUFFERED_STREAM=1

cleanup() {
  # ros2 run wrappers don't forward signals reliably: kill the binaries.
  pkill -9 -f "lib/nav2_controller/controller_server" 2>/dev/null
  pkill -9 -f "lib/nav2_map_server/map_server" 2>/dev/null
  pkill -9 -f "lib/nav2_lifecycle_manager/lifecycle_manager" 2>/dev/null
  pkill -9 -f "simval/kin_sim.py" 2>/dev/null
  sleep 1
}

run_scenario() {
  local name=$1 x0=$2 y0=$3 yaw0=$4 gx=$5 gy=$6
  echo "=== scenario: $name ==="
  cleanup
  ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$DIR/map.yaml" >"$DIR/log_map_$name.txt" 2>&1 &
  local MAP=$!
  ros2 run nav2_controller controller_server --ros-args \
    --params-file "$DIR/sim_params.yaml" >"$DIR/log_ctrl_$name.txt" 2>&1 &
  local CTRL=$!
  python3 "$DIR/kin_sim.py" --ros-args -p x0:=$x0 -p y0:=$y0 -p yaw0:=$yaw0 \
    -p log_path:="$DIR/traj_$name.csv" >"$DIR/log_sim_$name.txt" 2>&1 &
  local SIM=$!
  ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -p autostart:=true -p "node_names:=[map_server, controller_server]" \
    >"$DIR/log_lm_$name.txt" 2>&1 &
  local LM=$!
  sleep 8

  timeout 120 python3 "$DIR/run_scenario.py" "$name" | tee "$DIR/result_$name.txt"
  ps -o %cpu= -p $CTRL > "$DIR/cpu_$name.txt" 2>/dev/null

  kill $MAP $CTRL $SIM $LM 2>/dev/null
  cleanup
  wait 2>/dev/null

  echo "--- metrics: $name ---"
  python3 "$DIR/analyze.py" "$name" $gx $gy "$DIR/traj_$name.csv" | tee "$DIR/metrics_$name.txt"
  echo "controller CPU% (proc avg): $(cat "$DIR/cpu_$name.txt" 2>/dev/null)"
  grep -c "missed its desired rate" "$DIR/log_ctrl_$name.txt" | \
    xargs -I{} echo "control-loop overruns: {}"
}

python3 "$DIR/make_map.py"
run_scenario free 1.0 3.0 0.0 8.0 3.0
run_scenario blocked 2.0 5.0 1.57 8.0 5.0
