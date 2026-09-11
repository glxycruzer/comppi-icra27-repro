#!/usr/bin/env python3
"""FollowPath action driver: builds the scenario path, sends it, waits for
the result, prints SUCCESS/FAILURE."""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def straight_path(node, x0, y0, x1, y1, step=0.05):
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = node.get_clock().now().to_msg()
    dist = math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    n = max(2, int(dist / step))
    for i in range(n + 1):
        a = i / n
        p = PoseStamped()
        p.header = path.header
        p.pose.position.x = x0 + a * (x1 - x0)
        p.pose.position.y = y0 + a * (y1 - y0)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(p)
    return path


def main():
    scenario = sys.argv[1]
    rclpy.init()
    node = Node("scenario_driver")
    client = ActionClient(node, FollowPath, "follow_path")
    if not client.wait_for_server(timeout_sec=30.0):
        print("RESULT: FAILURE (follow_path server not available)")
        return 1

    if scenario == "free":       # free-space straight run along y=3
        path = straight_path(node, 1.0, 3.0, 8.0, 3.0)
    elif scenario == "blocked":  # straight path THROUGH the box at (5,5)
        path = straight_path(node, 2.0, 5.0, 8.0, 5.0)
    else:
        print("unknown scenario")
        return 2

    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = "FollowPath"
    goal.goal_checker_id = "general_goal_checker"

    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print("RESULT: FAILURE (goal rejected)")
        return 1

    result_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut, timeout_sec=90.0)
    res = result_fut.result()
    if res is None:
        print("RESULT: FAILURE (timeout)")
        return 1
    # status 4 = SUCCEEDED
    print(f"RESULT: {'SUCCESS' if res.status == 4 else 'FAILURE'} (status={res.status})")
    return 0 if res.status == 4 else 1


if __name__ == "__main__":
    sys.exit(main())
