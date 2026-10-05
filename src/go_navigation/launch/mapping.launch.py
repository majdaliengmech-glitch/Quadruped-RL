import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Phase 1: build a map with slam_toolbox while driving with the joystick.
    # Run after bringup.launch.py (sim + policy + teleop). slam_toolbox publishes map -> odom.
    slam_params = os.path.join(share("go_navigation"), "config", "slam_params.yaml")
    # map, scan, robot model and TF frames + the slam_toolbox panel (no Nav2 panels while mapping)
    rviz_cfg = os.path.join(share("go_navigation"), "config", "mapping.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share("slam_toolbox"), "launch", "online_async_launch.py")),
            launch_arguments={"use_sim_time": "true", "slam_params_file": slam_params}.items()),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
