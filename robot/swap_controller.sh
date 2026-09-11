#!/bin/bash
# Swap the deployed local controller on the robot for a field-campaign run.
# Copies one of the three MATCHED navigation_params variants (identical costmap,
# only the FollowPath block differs) into the installed Nav2 params, backing up
# the current one first. Does NOT restart bringup — it prints the next step so
# the operator restarts with the e-stop in hand.
#
# Usage (run ON the robot):   ./swap_controller.sh {comppi|dwb|mppi}
set -euo pipefail

CTRL="${1:-}"
case "$CTRL" in comppi|dwb|mppi) ;; *)
  echo "usage: $0 {comppi|dwb|mppi}"; exit 2;; esac

HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/controllers/navigation_params_${CTRL}.yaml"
DST="$HOME/<robot_ws>/install/<robot_nav_package>/share/<robot_nav_package>/params/navigation_params.yaml"

[ -f "$SRC" ] || { echo "missing $SRC"; exit 1; }
python3 -c "import yaml; yaml.safe_load(open('$SRC'))" || { echo "invalid YAML"; exit 1; }

# one-time pristine backup, then per-swap backup
[ -f "${DST}.orig" ] || cp "$DST" "${DST}.orig"
cp "$DST" "${DST}.prev"
cp "$SRC" "$DST"

echo "deployed: $CTRL"
grep primary_controller "$DST" | head -1
echo
echo "NEXT:  restart bringup, undock, then send goals."
echo "       verify after restart that the controller loaded:"
echo "         ROS_DOMAIN_ID=30 ros2 param get /controller_server FollowPath.plugin"
echo "ROLLBACK to previous: cp ${DST}.prev $DST   (or .orig for the pristine one)"
