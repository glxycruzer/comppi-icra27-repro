#!/bin/bash
# Same-method compute comparison on the robot's ARM SoC: Nav2-MPPI vs CoMPPI, both at
# K=1024, T=32, dt=0.1, 10 Hz, in the standalone sim harness (isolated ROS
# domain: the robot's live stack and motors are untouched). For each controller
# one closed-loop open-world trial is run while cpu_cycle_probe.py measures the
# controller_server process's CPU time per control cycle (all threads and the
# busiest thread). CoMPPI additionally writes its internal wall-clock timer
# (profile_log) so the two methods can be cross-calibrated.
#
# Run on the robot:   bash run_profile_cpu.sh
# Best with the live navigation stack stopped (otherwise its CPU load inflates
# both numbers equally; note it either way).
source /opt/ros/humble/setup.bash
source ~/<robot_ws>/install/setup.bash
export ROS_DOMAIN_ID=88
SV=~/<robot_ws>/src/comppi_controller/test/simval
A=$SV/profile_assets
OUT=/tmp/cpu_profile; mkdir -p $OUT

cleanup() {
  pkill -9 -f "lib/nav2_controller/controller_server" 2>/dev/null
  pkill -9 -f "lib/nav2_map_server/map_server" 2>/dev/null
  pkill -9 -f "lib/nav2_lifecycle_manager/lifecycle_manager" 2>/dev/null
  pkill -9 -f "simval/kin_sim.py" 2>/dev/null
  pkill -9 -f "cpu_cycle_probe.py" 2>/dev/null
  sleep 1
}

run_one() {   # label params.yaml
  local L=$1 P=$2
  echo "=== $L ($P) ==="
  cleanup
  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$A/map.yaml >$OUT/${L}_map.log 2>&1 &
  ros2 run nav2_controller controller_server --ros-args --params-file $P >$OUT/${L}_ctrl.log 2>&1 &
  python3 $SV/kin_sim.py --ros-args -p x0:=1.2 -p y0:=5.0 -p yaw0:=0.0 -p log_path:=$OUT/${L}_traj.csv >$OUT/${L}_sim.log 2>&1 &
  ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true -p "node_names:=[map_server, controller_server]" >$OUT/${L}_lm.log 2>&1 &
  sleep 10
  PID=$(pgrep -f "lib/nav2_controller/controller_server" | head -1)
  echo "controller_server pid $PID, threads $(ls /proc/$PID/task | wc -l)"
  python3 $A/cpu_cycle_probe.py $PID 45 $OUT/${L}_cpu.csv > $OUT/${L}_cpu.txt 2>&1 &
  PROBE=$!
  timeout 90 python3 $SV/run_trial.py $A/scenario.json | tail -1
  echo "cur MHz during run (cpu4/cpu6): $(($(cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq)/1000)) $(($(cat /sys/devices/system/cpu/cpu6/cpufreq/scaling_cur_freq)/1000))"
  wait $PROBE
  cat $OUT/${L}_cpu.txt
  echo "loop overruns: $(grep -c 'missed its desired rate' $OUT/${L}_ctrl.log)"
  cleanup
}

# uptime/load and CPU clock state before, so the reader knows the condition.
# For a clean per-cycle number pin the clocks first:
#   sudo sh -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > $g; done'
# and restore afterwards with 'ondemand'.
echo "load before: $(cut -d' ' -f1-3 /proc/loadavg)   nav container running: $(pgrep -c -f component_container_isolated)"
echo "governor: $(cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_governor)   cur MHz (cpu0/4/6): $(($(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)/1000)) $(($(cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq)/1000)) $(($(cat /sys/devices/system/cpu/cpu6/cpufreq/scaling_cur_freq)/1000))   max $(($(cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq)/1000))"
run_one mppi   $SV/params_mppi.yaml
run_one comppi $A/params_b1024.yaml
echo "=== CoMPPI internal wall-clock timer (us per cycle) ==="
python3 - <<'EOF'
import statistics as st
try:
    v=[float(l.split()[-1]) for l in open('/tmp/comppi_profile_1024.txt') if l.strip()]
    v=[x for x in v if x>0]
    print(f"n={len(v)} median {st.median(v)/1000:.2f} ms  p90 {sorted(v)[int(0.9*len(v))-1]/1000:.2f} ms  max {max(v)/1000:.2f} ms")
except Exception as e:
    print("profile log not found:", e)
EOF
echo "results in $OUT  (please send ${OUT}/*_cpu.txt and *_cpu.csv back)"
exit 0
