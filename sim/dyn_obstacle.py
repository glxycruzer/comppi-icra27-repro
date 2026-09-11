#!/usr/bin/env python3
"""Dynamic obstacle for the sim harness: publishes /map (transient-local, so the
Nav2 static layer picks up every update) containing the static world plus one
moving box, and logs the box centre. Replaces map_server for 'crossing' worlds.

The box waits at (xc, y0) until the robot's odom x passes xc - trigger_dx, then
crosses the planned path (y = 5) at vy m/s and stops once it has cleared the
far side. Timing is chosen so that, at nominal cruise, box and robot reach the
crossing point together -- a pedestrian stepping into the corridor.
"""
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

RES = 0.05
N = 200


class DynObstacle(Node):
    def __init__(self):
        super().__init__("dyn_obstacle")
        for name, default in (("grid_path", ""), ("log_path", "/tmp/dyn.csv"),
                              ("xc", 5.0), ("y0", 6.5), ("vy", -0.3),
                              ("size", 0.5), ("trigger_dx", 1.9), ("rate", 5.0)):
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value
        self.static = np.load(p("grid_path")) if p("grid_path") else np.zeros((N, N), bool)
        self.xc, self.y, self.vy = p("xc"), p("y0"), p("vy")
        self.y_start = self.y
        self.size, self.trig = p("size"), p("trigger_dx")
        self.moving, self.done = False, False
        self.robot_x = -1.0
        self.log = open(p("log_path"), "w")
        self.log.write("t,t_abs,bx,by,moving\n")
        self.t0 = self.get_clock().now()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, "map", qos)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.dt = 1.0 / p("rate")
        self.create_timer(self.dt, self.step)

    def on_odom(self, msg):
        self.robot_x = msg.pose.pose.position.x

    def step(self):
        if not self.moving and not self.done and self.robot_x >= self.xc - self.trig:
            self.moving = True
        if self.moving:
            self.y += self.vy * self.dt
            # stop once the box has fully crossed to the mirror position
            if abs(self.y - 5.0) >= abs(self.y_start - 5.0) and \
               (self.y - 5.0) * (self.y_start - 5.0) < 0:
                self.moving, self.done = False, True
        g = self.static.copy()
        h = self.size / 2.0
        j0, j1 = int((self.xc - h) / RES), int((self.xc + h) / RES) + 1
        i0, i1 = int((self.y - h) / RES), int((self.y + h) / RES) + 1
        g[max(0, i0):min(N, i1), max(0, j0):min(N, j1)] = True

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = RES
        msg.info.width = msg.info.height = N
        msg.info.origin.orientation.w = 1.0
        msg.data = np.where(g, 100, 0).astype(np.int8).flatten().tolist()
        self.pub.publish(msg)
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        self.log.write(f"{t:.3f},{now.nanoseconds * 1e-9:.3f},{self.xc:.3f},{self.y:.3f},{int(self.moving)}\n")
        self.log.flush()


def main():
    rclpy.init()
    n = DynObstacle()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.log.close()


if __name__ == "__main__":
    main()
