#!/usr/bin/env python3
"""Republish Gazebo ground-truth odometry (/odom) as the odom -> base_link TF."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTF(Node):
    def __init__(self):
        super().__init__("odom_tf")
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self.cb, 10)

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id, t.child_frame_id = "odom", "base_link"   # force clean frame names
        p = msg.pose.pose
        t.transform.translation.x, t.transform.translation.y = p.position.x, p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init(); rclpy.spin(OdomTF()); rclpy.shutdown()


if __name__ == "__main__":
    main()
