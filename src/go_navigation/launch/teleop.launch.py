import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(share("go_navigation"), "config")
    joy, mux = os.path.join(cfg, "joy.yaml"), os.path.join(cfg, "twist_mux.yaml")
    return LaunchDescription([
        Node(package="joy", executable="joy_node", parameters=[joy]),
        Node(package="teleop_twist_joy", executable="teleop_node",
             parameters=[joy], remappings=[("cmd_vel", "cmd_vel_joy")]),
        # joystick (priority 100) beats Nav2 (priority 10); output feeds the walking policy
        Node(package="twist_mux", executable="twist_mux", parameters=[mux],
             remappings=[("cmd_vel_out", "cmd_vel_mux")]),
    ])
