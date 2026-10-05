import os, xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg, gz_pkg = share("go_description"), share("go_gazebo")
    urdf = os.path.join(desc_pkg, "urdf", "go1.urdf")
    robot_xml = xacro.process_file(urdf).toxml()          # resolves $(find go_gazebo)
    # world:=factory.sdf (default) or world:=patrol_room.sdf
    world_arg = DeclareLaunchArgument("world", default_value="factory.sdf")
    world = PathJoinSubstitution([gz_pkg, "worlds", LaunchConfiguration("world")])
    # Humble ships Fortress (ignition.msgs), Jazzy ships Harmonic (gz.msgs)
    M = "ignition.msgs" if os.environ.get("ROS_DISTRO") == "humble" else "gz.msgs"

    # lets Gazebo resolve package://go1_description/meshes/... URIs
    humble = os.environ.get("ROS_DISTRO") == "humble"
    var = "IGN_GAZEBO_RESOURCE_PATH" if humble else "GZ_SIM_RESOURCE_PATH"
    res = SetEnvironmentVariable(
        var, os.path.dirname(desc_pkg) + os.pathsep + os.environ.get(var, ""))

    # gui:=false runs the server only (sensors still render, so /scan works); saves ~1 GB and a GPU
    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    server_only = PythonExpression(["'' if '", LaunchConfiguration("gui"), "' == 'true' else '-s '"])
    gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", server_only, world]}.items())

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": robot_xml, "use_sim_time": True}])

    spawn = Node(package="ros_gz_sim", executable="create",
                 arguments=["-topic", "robot_description", "-name", "go1",
                            "-x", "0", "-y", "0", "-z", "0.45"])   # drop from just above the floor

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  arguments=[f"/clock@rosgraph_msgs/msg/Clock[{M}.Clock",
                             f"/imu@sensor_msgs/msg/Imu[{M}.IMU",
                             f"/scan@sensor_msgs/msg/LaserScan[{M}.LaserScan",
                             f"/model/go1/odometry@nav_msgs/msg/Odometry[{M}.Odometry"],
                  remappings=[("/model/go1/odometry", "/odom")],
                  parameters=[{"use_sim_time": True}])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager"],
                    parameters=[{"use_sim_time": True}])

    jsb, eff = spawner("joint_state_broadcaster"), spawner("effort_controller")
    chain = [RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
             RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[eff]))]
    odom_tf = Node(package="policy_node", executable="odom_tf",
                   parameters=[{"use_sim_time": True}])
    return LaunchDescription([world_arg, gui_arg, res, gz, rsp, spawn, bridge, odom_tf] + chain)
