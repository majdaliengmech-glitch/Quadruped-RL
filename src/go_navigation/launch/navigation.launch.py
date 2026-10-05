import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Phases 2-5: map_server + AMCL + Nav2 on a saved map. Run after bringup.launch.py.
    # Nav2's final /cmd_vel goes to twist_mux (priority 10); the joystick still overrides it.
    nav = share("go_navigation")
    rviz_cfg = os.path.join(share("nav2_bringup"), "rviz", "nav2_default_view.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(nav, "maps", "factory.yaml")),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(nav, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share("nav2_bringup"), "launch", "bringup_launch.py")),
            launch_arguments={"use_sim_time": "true", "autostart": "true",
                              "map": LaunchConfiguration("map"),
                              "params_file": LaunchConfiguration("params_file")}.items()),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
