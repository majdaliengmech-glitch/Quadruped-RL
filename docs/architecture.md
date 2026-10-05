# Architecture

## Design decisions

1. **The walking policy is blind.** It maps a velocity command (vx, vy, wz) plus proprioception to joint targets. Joystick, keyboard and Nav2 all just publish velocity commands, and `twist_mux` decides who wins. Obstacle avoidance is Nav2's job (lidar → costmaps), not the policy's.
2. **Export from the model the policy was trained on.** The URDF, masses, joint limits, joint order and default pose come from the compiled `MjModel` (`src/tools/mjcf_to_urdf.py`), not from a look-alike description package.
3. **PD runs at physics rate in its own node.** The policy publishes position targets at 50 Hz; `pd_controller` turns them into torques at 500 Hz, as MuJoCo's position actuators do inside the physics step.
4. **Joint order is FR, FL, RR, RL** (Menagerie actuator order). Every node maps joints **by name** using `joint_names` from `robot_params.yaml`.

## Data flow

```text
 Gamepad ─ joy_node ─ teleop_twist_joy ─ /cmd_vel_joy (prio 100) ─┐
 Keyboard ─ teleop_twist_keyboard ───── /cmd_vel_key (prio  90) ──┼─ twist_mux ─ /cmd_vel_mux
 Nav2: controller_server (MPPI) ─ /cmd_vel_nav                    │                 │
        ─ velocity_smoother ─ /cmd_vel_smoothed                   │                 ▼
        ─ collision_monitor ─ /cmd_vel (prio 10) ─────────────────┘   policy_node (50 Hz)
                                                                            │ /policy/q_target
 Gazebo ◄─ /effort_controller/commands ◄─ pd_controller (500 Hz) ◄─────────┘
   └─► /joint_states (ros2_control) · /imu /scan /odom /clock (ros_gz_bridge)
```

## Packages

| Package | Type | Contents |
|---|---|---|
| `go_description` | ament_cmake | `urdf/go1.urdf` (links, joints, ros2_control block, IMU, lidar, odometry plugin), STL meshes, `robot_params.yaml` (policy side), `robot_standing_params.yaml` (PD side) |
| `go_gazebo` | ament_cmake | `worlds/factory.sdf` (generated), `worlds/patrol_room.sdf`, `launch/sim.launch.py`, `config/controllers.yaml` |
| `policy_node` | ament_python | `policy_node`, `pd_controller`, `odom_tf`, `config/policy_weights.npz`, `launch/policy.launch.py` |
| `go_navigation` | ament_python | teleop / mapping / navigation / bringup launches, Nav2 + SLAM + twist_mux + joy config, maps, patrol routes, `goto`, `patrol`, `obstacles`, `reset_robot` |

## Nodes

| Node | Rate | Subscribes | Publishes |
|---|---|---|---|
| `policy_node` | 50 Hz | `/joint_states`, `/imu`, `/odom`, `/cmd_vel_mux` | `/policy/q_target` |
| `pd_controller` | 500 Hz | `/joint_states`, `/policy/q_target` | `/effort_controller/commands` |
| `odom_tf` | with `/odom` | `/odom` | TF `odom → base_link` |
| `twist_mux` | event | `/cmd_vel_joy`, `/cmd_vel_key`, `/cmd_vel` | `/cmd_vel_mux` |
| `robot_state_publisher` | | `/joint_states` | TF `base_link → legs, imu_link, lidar_link` |
| AMCL | | `/scan`, `/map` | TF `map → odom`, `/amcl_pose` |

`policy_node` zeroes the command if `/cmd_vel_mux` is silent for `cmd_timeout` (1.0 s), because `twist_mux` does not send a zero when its inputs stop.

## Frames

```text
map ──(AMCL)── odom ──(odom_tf, Gazebo ground truth)── base_link ─┬─ FR/FL/RR/RL legs
                                                                    ├─ imu_link   (trunk origin)
                                                                    └─ lidar_link (0.12 m above trunk, ~0.39 m above floor)
```

There is no `base_footprint`; every Nav2 / SLAM `base_frame` is set to `base_link`. The map frame equals the Gazebo world frame because the map was built from the spawn point at the origin.

## Sensors (from the URDF)

| Sensor | Topic | Rate | Notes |
|---|---|---|---|
| IMU | `/imu` | 200 Hz | on `imu_link` |
| 2D GPU lidar | `/scan` | 10 Hz | 360 samples, 0.15–12 m, needs the Sensors system with a render engine |
| OdometryPublisher | `/odom` | 50 Hz | ground-truth pose and twist |
| ros2_control | `/joint_states` | 500 Hz | effort command interface, position/velocity/effort state |

## Launch files

| Launch | Arguments | Starts |
|---|---|---|
| `go_gazebo sim.launch.py` | `world:=factory.sdf\|patrol_room.sdf`, `gui:=true\|false` | Gazebo, robot_state_publisher, spawn, bridge, controllers, odom_tf |
| `policy_node policy.launch.py` | | pd_controller, policy_node |
| `go_navigation teleop.launch.py` | | joy_node, teleop_twist_joy, twist_mux |
| `go_navigation bringup.launch.py` | `teleop:=true`, plus the sim arguments | sim + policy + teleop |
| `go_navigation mapping.launch.py` | `rviz:=true` | slam_toolbox (online async) + RViz |
| `go_navigation navigation.launch.py` | `map:=…`, `params_file:=…`, `rviz:=true` | nav2_bringup (map_server, AMCL, Nav2) + RViz |
