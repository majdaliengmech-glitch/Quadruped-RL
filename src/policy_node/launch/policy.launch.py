from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # params_file / weights_file default to the installed share/ directories
    sim = {"use_sim_time": True}
    return LaunchDescription([
        Node(package="policy_node", executable="pd_controller", parameters=[sim], output="screen"),
        Node(package="policy_node", executable="policy_node", parameters=[sim], output="screen"),
    ])
