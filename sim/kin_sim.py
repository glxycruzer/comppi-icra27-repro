#!/usr/bin/env python3
"""Kinematic stand-in for the robot base: integrates cmd_vel (unicycle),
publishes map->odom (identity), odom->base_link TF, /odom, and logs state CSV."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class KinSim(Node):
    def __init__(self):
        super().__init__("kin_sim")
        self.declare_parameter("x0", 2.0)
        self.declare_parameter("y0", 5.0)
        self.declare_parameter("yaw0", 0.0)
        self.declare_parameter("log_path", "/tmp/kin_sim.csv")
        self.x = self.get_parameter("x0").value
        self.y = self.get_parameter("y0").value
        self.yaw = self.get_parameter("yaw0").value
        self.v = 0.0
        self.w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.log = open(self.get_parameter("log_path").value, "w")
        self.log.write("t,x,y,yaw,v,w,t_abs\n")
        self.t0 = self.get_clock().now()

        self.tfb = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)

    def on_cmd(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        # command watchdog: stop if the controller went silent
        if (now - self.last_cmd_time).nanoseconds > 5e8:
            self.v = 0.0
            self.w = 0.0
        self.yaw += self.w * self.dt
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        stamp = now.to_msg()

        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = "map"
        t1.child_frame_id = "odom"
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = "odom"
        t2.child_frame_id = "base_link"
        t2.transform.translation.x = self.x
        t2.transform.translation.y = self.y
        t2.transform.rotation.z = qz
        t2.transform.rotation.w = qw
        self.tfb.sendTransform([t1, t2])

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.odom_pub.publish(od)

        te = (now - self.t0).nanoseconds * 1e-9
        self.log.write(f"{te:.3f},{self.x:.4f},{self.y:.4f},{self.yaw:.4f},{self.v:.4f},{self.w:.4f},"
                       f"{now.nanoseconds * 1e-9:.3f}\n")
        self.log.flush()


def main():
    rclpy.init()
    rclpy.spin(KinSim())


if __name__ == "__main__":
    main()
