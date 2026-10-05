#!/usr/bin/env python3
"""Put a fallen robot back on its feet in Gazebo (simulation only).

Reads the current pose from /odom, keeps x, y and heading, and teleports the model upright
0.45 m above the floor with Gazebo's set_pose service; pd_controller then lands it in the
standing pose and the policy carries on.
usage: ros2 run go_navigation reset_robot            (reset where it fell)
       ros2 run go_navigation reset_robot --origin   (reset at the spawn point)"""
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from go_navigation import gz_util
from go_navigation.nav_util import yaw_of

MODEL, HEIGHT = "go1", 0.45


def teleport(x, y, yaw):
    """Stand the robot up at (x, y, yaw) in the world frame. Returns True on success."""
    return gz_util.set_pose(MODEL, x, y, HEIGHT, yaw)


def current_pose(timeout=3.0):
    """(x, y, yaw) from /odom, or None if nothing arrives."""
    node = Node("reset_robot")
    msg = []
    node.create_subscription(Odometry, "/odom", msg.append, 1)
    end = node.get_clock().now().nanoseconds + timeout * 1e9
    while not msg and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if not msg:
        return None
    p = msg[0].pose.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


def main():
    rclpy.init()
    pose = None if "--origin" in sys.argv else current_pose()
    rclpy.shutdown()
    x, y, yaw = pose or (0.0, 0.0, 0.0)
    try:
        ok = teleport(x, y, yaw)
    except RuntimeError as e:
        sys.exit(str(e))
    print(f"{'reset' if ok else 'FAILED to reset'} {MODEL} at x={x:.2f} y={y:.2f} "
          f"yaw={math.degrees(yaw):.0f} deg in world '{gz_util.world_name()}'")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
