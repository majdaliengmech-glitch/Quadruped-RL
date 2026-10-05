import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(pkg, name, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share(pkg), "launch", name)), **kwargs)


def generate_launch_description():
    # one command for sim + walking policy + joystick; policy starts with the sim so the robot stands
    return LaunchDescription([
        DeclareLaunchArgument("teleop", default_value="true", description="start joy + twist_mux"),
        include("go_gazebo", "sim.launch.py"),
        include("policy_node", "policy.launch.py"),
        include("go_navigation", "teleop.launch.py", condition=IfCondition(LaunchConfiguration("teleop"))),
    ])
