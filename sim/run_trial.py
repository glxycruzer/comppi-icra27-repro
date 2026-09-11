#!/usr/bin/env python3
"""Send the scenario.json path as a FollowPath goal; exit 0 on success."""
import json
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def main():
    scen = json.load(open(sys.argv[1]))
    rclpy.init()
    node = Node("trial_driver")
    client = ActionClient(node, FollowPath, "follow_path")
    if not client.wait_for_server(timeout_sec=30.0):
        print("RESULT: FAILURE (no server)")
        return 1

    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = node.get_clock().now().to_msg()
    pts = scen["path"]
    for i, (x, y) in enumerate(pts):
        nx, ny = pts[min(i + 1, len(pts) - 1)]
        yaw = math.atan2(ny - y, nx - x) if i + 1 < len(pts) else \
            math.atan2(y - pts[i - 1][1], x - pts[i - 1][0])
        p = PoseStamped()
        p.header = path.header
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(p)

    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = "FollowPath"
    goal.goal_checker_id = "general_goal_checker"

    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print("RESULT: FAILURE (rejected)")
        return 1
    result_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut, timeout_sec=120.0)
    res = result_fut.result()
    ok = res is not None and res.status == 4
    print(f"RESULT: {'SUCCESS' if ok else 'FAILURE'} "
          f"(status={res.status if res else 'timeout'})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
