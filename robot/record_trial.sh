#!/bin/bash
# Record one field-campaign trial as a rosbag2 bag. Run ON the robot.
# The operator starts this, sends the goal through the usual robot UI, and
# presses ENTER when the robot reaches the goal (or is stopped) to end the bag.
#
# Usage:  ./record_trial.sh <course> <controller> <trial_no>
#   e.g.  ./record_trial.sh corridor comppi 3
#
# Bags land in ./bags/<course>__<controller>__t<trial>/ and every run appends a
# line to ./bags/manifest.csv (course,controller,trial,bag,operator_note,stamp).
# NOTE: no `set -u` — sourcing the ROS setup scripts references unbound vars
# (AMENT_TRACE_SETUP_FILES) and would abort under it.
set -o pipefail

COURSE="${1:-}"; CTRL="${2:-}"; TRIAL="${3:-}"
[ -z "$COURSE" ] || [ -z "$CTRL" ] || [ -z "$TRIAL" ] && {
  echo "usage: $0 <course> <controller> <trial_no>"; exit 2; }

HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source "$HOME/<robot_ws>/install/setup.bash" 2>/dev/null || true
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -w /dev/shm ] || echo "WARN: /dev/shm not writable — run: sudo chmod 777 /dev/shm"

BAGDIR="$HERE/bags/${COURSE}__${CTRL}__t${TRIAL}"
rm -rf "$BAGDIR"
mkdir -p "$HERE/bags"

# Core topics for offline metrics. /odom (pose; twist is zero on this base),
# /cmd_vel_nav (controller command), /scan (clearance), /goal_pose (the goal
# the operator sends -- reliable, published once after the prompt) with /plan
# as a fallback goal source, /tf(+static) (frames), and the local plan for viz.
# /amcl_pose is the MAP-frame robot pose (this robot's localizer publishes it
# there, not via a map->odom TF) -- it is what goal-reaching must be scored
# against, since goals are in the map frame while /odom is in the odom frame.
# /odom stays for velocity (speed is frame-independent). Goal topics: this
# robot's UI sends via the /navigate_to_pose action, so /plan(_smoothed) last
# pose is the fallback goal source; per-course fixed goals are the primary.
TOPICS="/odom /amcl_pose /cmd_vel_nav /cmd_vel /scan /goal_update /plan_smoothed /path_planning/coordiante /goal_pose /plan /tf /tf_static /local_plan"

echo "=== recording  $COURSE / $CTRL / trial $TRIAL ==="
echo "topics: $TOPICS"
ros2 bag record -o "$BAGDIR" $TOPICS >/tmp/bag_$$.log 2>&1 &
BAGPID=$!
sleep 2
if ! kill -0 $BAGPID 2>/dev/null; then
  echo "bag recorder failed to start:"; tail -5 /tmp/bag_$$.log; exit 1; fi

echo ">>> SEND THE GOAL NOW. Press ENTER when the robot reaches the goal (or stops)."
read -r -p "operator note (optional, e.g. 'clean' / 'grazed cone'): " NOTE

kill -INT $BAGPID 2>/dev/null
wait $BAGPID 2>/dev/null
sleep 1

STAMP="$(date -Iseconds)"
echo "${COURSE},${CTRL},${TRIAL},${BAGDIR},\"${NOTE}\",${STAMP}" >> "$HERE/bags/manifest.csv"
echo "saved: $BAGDIR"
echo "manifest updated."
