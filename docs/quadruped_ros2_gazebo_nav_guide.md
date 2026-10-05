# Quadruped in ROS 2 + Gazebo

**Workspace and packages, MuJoCo → URDF/SDF export, joystick control, and Nav2 patrol with obstacle avoidance**

| | |
|---|---|
| Companion to | "Quadruped RL: Colab Training + Local Gazebo/ROS 2 Deployment" (your project guide) |
| Robot | Unitree Go1 / Go2 class, 12 actuated joints, model taken from MuJoCo Playground / Menagerie |
| Pairings | Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress, or Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic |
| Result | Robot spawns in Gazebo, stands, walks under a joystick, and patrols waypoints around obstacles |

## What was tested and what was not

The exporter script in this guide (Section 3 and Appendix A) was run against the Menagerie Go1 model. The URDF it wrote was reloaded in MuJoCo: all 12 joint ranges and axes matched, total mass matched, and the centre-of-mass position of every leg link matched the original to numerical precision.

The launch files, world file, controller config, ROS nodes and Nav2 settings were syntax-checked only. ROS 2 and Gazebo could not be run, so expect to adjust package names, plugin names and Nav2 keys for your exact distro. Section 11 lists the likely trouble spots.

## Three decisions that shape everything

- **The walking policy is blind.** It only turns a velocity command (vx, vy, yaw rate) into leg motion. Joystick and Nav2 therefore both just publish velocity commands, and a `twist_mux` node decides who wins. Obstacle avoidance is Nav2's job (lidar → costmaps), not the policy's.
- **Export from the model the policy was trained on** (`env.mj_model`), not from a similar-looking description package. Masses, limits, gains, default pose and joint order then match by construction.
- **Two corrections to the earlier guide** (Sections 3.4 and 5): the example joint order (FL, FR, RL, RR) does not match Menagerie Go1/Go2, which use FR, FL, RR, RL; and the PD law should run at a high rate in its own node, not once per 50 Hz policy step.

---

## 1. Architecture

```text
 Gamepad -> joy_node -> teleop_twist_joy -> /cmd_vel_joy --+  (priority 100)
                                                           +--> twist_mux --> /cmd_vel_mux
 Nav2 (planner, DWB controller, costmaps) --> /cmd_vel ----+  (priority 10)        |
   ^                                                                                v
   | /scan, map->odom (AMCL), odom->base_link (odom_tf)          policy_node (50 Hz, NumPy MLP)
   |                                                                                |
   |                                                                       /policy/q_target
   |                                                                                v
 Gazebo <-- /effort_controller/commands <-- pd_controller (500 Hz): tau = Kp(q_target - q) - Kd*dq
 (go1 model, ros2_control effort interface, IMU, lidar, odometry plugin)
   |
   +--> /joint_states, /imu, /scan, /odom, /clock  (ros2_control and ros_gz_bridge) --> nodes above
```

| Package | Type | Holds |
|---|---|---|
| go_description | ament_cmake | Generated URDF, STL meshes, robot_params.yaml (joint order, gains, default pose) |
| go_gazebo | ament_cmake | World SDF, sim launch file, ros2_control controller config |
| policy_node | ament_python | policy_node, pd_controller, odom_tf, policy.npz |
| go_navigation | ament_python | Joystick + twist_mux config, Nav2 params, maps, patrol script |

---

## 2. Install and create the workspace

### 2.1 Install (Ubuntu + ROS 2 + Gazebo)

Install ROS 2 from the official docs for your distro first (desktop variant). Then add the packages below. Gazebo packages are the part most likely to differ: run `apt search` if a name is not found.

```bash
# ---- Humble + Fortress (Ubuntu 22.04) ----
sudo apt install ros-dev-tools ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-tf2-ros ros-humble-rviz2 \
  ros-humble-joy ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard ros-humble-twist-mux \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control   # Fortress side
#   if ros-humble-ros-gz is not found, try:  apt search ros-humble-ros-ign

# ---- Jazzy + Harmonic (Ubuntu 24.04): same list with 'jazzy', and ----
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
```

Python side (separate from ROS): the exporter needs MuJoCo and trimesh. Reuse the venv from your project guide.

```bash
source ~/quadruped_rl/.venv/bin/activate
pip install mujoco trimesh numpy
```

### 2.2 Create the workspace and the four packages

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/quadruped_rl/ros2_ws/src && cd ~/quadruped_rl/ros2_ws/src

ros2 pkg create --build-type ament_cmake go_description
ros2 pkg create --build-type ament_cmake go_gazebo
ros2 pkg create --build-type ament_python policy_node \
    --dependencies rclpy sensor_msgs geometry_msgs nav_msgs std_msgs tf2_ros
ros2 pkg create --build-type ament_python go_navigation \
    --dependencies rclpy geometry_msgs nav2_simple_commander

mkdir -p go_description/{urdf,meshes,config}  go_gazebo/{worlds,launch,config}
mkdir -p policy_node/{config,launch}          go_navigation/{launch,config,maps}
```

### 2.3 Make each package install its folders

**go_description/CMakeLists.txt** and **go_gazebo/CMakeLists.txt**: add before `ament_package()`.

```cmake
# go_description
install(DIRECTORY urdf meshes config DESTINATION share/${PROJECT_NAME})
# go_gazebo
install(DIRECTORY worlds launch config DESTINATION share/${PROJECT_NAME})
```

**go_gazebo/package.xml**: add run dependencies so `rosdep` and launch find everything.

```xml
<exec_depend>go_description</exec_depend>
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>controller_manager</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>effort_controllers</exec_depend>
<exec_depend>policy_node</exec_depend>
```

**policy_node/setup.py** and **go_navigation/setup.py**: install config/launch files and register the executables.

```python
# policy_node/setup.py  (only the parts that change)
from glob import glob
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/policy_node']),
    ('share/policy_node', ['package.xml']),
    ('share/policy_node/config', glob('config/*')),
    ('share/policy_node/launch', glob('launch/*.py')),
],
entry_points={'console_scripts': [
    'policy_node = policy_node.policy_node:main',
    'pd_controller = policy_node.pd_controller:main',
    'odom_tf = policy_node.odom_tf:main',
]},

# go_navigation/setup.py: same idea, with config/launch/maps globs and
#    'patrol = go_navigation.patrol:main'
```

### 2.4 Build, source, and check

```bash
cd ~/quadruped_rl/ros2_ws
rosdep install --from-paths src --ignore-src -y     # after: sudo rosdep init; rosdep update
colcon build --symlink-install
source install/setup.bash
echo "source ~/quadruped_rl/ros2_ws/install/setup.bash" >> ~/.bashrc
ros2 pkg list | grep -E "go_|policy_node"           # all four must appear
```

> **Note.** `--symlink-install` lets you edit Python nodes, launch files and YAML without rebuilding. Re-run `colcon build` after adding new files or changing setup.py / CMakeLists.txt.

---

## 3. Exporting the quadruped from MuJoCo

### 3.1 What does not carry over automatically

MJCF (MuJoCo) and URDF/SDF describe the same robot differently. Nothing converts one into the other without loss, so it helps to know where the differences are.

| MuJoCo concept | URDF / Gazebo reality | What this guide does |
|---|---|---|
| Meshes are recentred on their centre of mass at compile time; geom poses are adjusted to compensate | URDF meshes are placed exactly as the file was authored | Export the *compiled* mesh vertices from the model itself, so the compiled geom poses apply directly |
| Capsule collision geoms | URDF has no capsule | Write one cylinder plus two spheres |
| Position actuators (Kp, Kd) running inside the physics step | ros2_control effort interface; torques come from a node | Read Kp/Kd/limits from the model into robot_params.yaml; PD runs in pd_controller at 500 Hz |
| Joint damping, armature, frictionloss | Not reliably supported by Gazebo physics; armature has no URDF field | Damping applied once, in pd_controller. Armature and frictionloss are listed as known gaps |
| Root has a free joint | Root link is the model's pose in the world | Root becomes `base_link`; no joint written |
| Default pose lives in keyframe "home" | Not part of URDF | Written to robot_params.yaml as default_q and to ros2_control initial_value |
| Joint order = actuator order | Gazebo orders joints however it likes | robot_params.yaml stores the action order; every node maps by name |

### 3.2 Three routes, and why route B is the default

| Route | How | Good | Watch out |
|---|---|---|---|
| A. Official description | Use Unitree's go1_description / go2_description (xacro + meshes) | Maintained, familiar to ROS users | Inertias, limits and gains may differ from the model you trained on, so the policy can fail for reasons unrelated to physics engines |
| **B. Export from your MjModel (recommended)** | Run the script in Appendix A on env.mj_model | Exact masses, COMs, limits, joint order, gains and default pose from training | Collision shapes are what MuJoCo uses (primitives); visuals are the Menagerie meshes |
| C. Community converters | Third-party MJCF→URDF tools | Less code to write | Quality varies and most do not handle mesh recentring or actuator gains; verify with Section 3.5 |

### 3.3 Run the export

**Option 1: from Menagerie XML** (good for a first test).

```bash
cd ~/quadruped_rl
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git
cp /path/to/mjcf_to_urdf.py training/            # Appendix A, or the code bundle
python training/mjcf_to_urdf.py mujoco_menagerie/unitree_go1/go1.xml \
       --out ros2_ws/src/go_description --name go1 --flavor fortress    # or: --flavor harmonic
```

**Option 2: from the Playground environment** (what you actually trained). Run this in the same Python environment where Playground is installed.

```python
from mujoco_playground import registry
from mjcf_to_urdf import export_urdf

env = registry.load("Go1JoystickFlatTerrain")        # the env you trained
export_urdf(env.mj_model, "ros2_ws/src/go_description",
            robot_name="go1", flavor="fortress")      # or "harmonic"
```

If `env.mj_model` is not available in your Playground version, check the environment class for the attribute that holds the compiled `MjModel` and pass that.

Result inside `go_description/`:

```text
urdf/go1.urdf               links, joints, collision + visual, ros2_control block, IMU, lidar, odometry plugin
meshes/*.stl                recentred meshes taken from the compiled model
config/robot_params.yaml    joint_names (ACTION ORDER), default_q, kp, kd_actuator, joint_damping, torque_limit
```

### 3.4 Read robot_params.yaml before doing anything else

This is the output for the Menagerie Go1. **Your Playground model may use different gains**, so treat these numbers as an example of the format, not as values to copy.

```yaml
timestep_sim: 0.002
joint_names: [FR_hip_joint, FR_thigh_joint, FR_calf_joint, FL_hip_joint, FL_thigh_joint, FL_calf_joint,
              RR_hip_joint, RR_thigh_joint, RR_calf_joint, RL_hip_joint, RL_thigh_joint, RL_calf_joint]
default_q:      [0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8]
kp:             [100, 100, 100, ...]            # position-actuator stiffness
kd_actuator:    [0, 0, 0, ...]                  # damping term inside the actuator (zero here)
joint_damping:  [1, 2, 2, 1, 2, 2, 1, 2, 2, 1, 2, 2]   # passive damping: this is where Kd lives in this model
torque_limit:   [23.7, 23.7, 35.55, ...]
action_scale: 0.5   # FILL IN: copy from the environment config; the model does not contain it
```

- **Joint order is FR, FL, RR, RL.** The sample list in the earlier guide (FL, FR, RL, RR) would swap legs. Always use `joint_names` from this file for the policy, the PD node and the controller config.
- **Kd may hide in joint damping.** pd_controller adds `kd_actuator + joint_damping`, and the URDF writes damping 0 so the damping is applied exactly once.
- **action_scale, observation scales and control rate** come from the Playground environment config, not from the model. Fill in action_scale by hand and keep ctrl_dt (policy rate) equal to your 50 Hz timer.

### 3.5 Verify the export (do not skip)

**1. The URDF is well formed and the tree is right.**

```bash
source ~/quadruped_rl/ros2_ws/install/setup.bash
xacro ros2_ws/src/go_description/urdf/go1.urdf > /tmp/go1_flat.urdf     # $(find go_gazebo) needs go_gazebo built
check_urdf /tmp/go1_flat.urdf        # sudo apt install liburdfdom-tools   (prints the link tree)
```

**2. Physical parameters match MuJoCo.** Reload the URDF in MuJoCo and compare. (The MuJoCo URDF importer folds a fixed root into the world, so the trunk mass is absent from the reloaded model. That is expected.)

```python
import mujoco, numpy as np
m0 = mujoco.MjModel.from_xml_path("go1.xml")                 # original
m1 = mujoco.MjModel.from_xml_path("go1_for_check.urdf")      # URDF with ros2_control/gazebo blocks removed,
                                                              # mesh paths changed to ../meshes/<name>.stl
d0, d1 = mujoco.MjData(m0), mujoco.MjData(m1)
q = m0.key_qpos[0]; d0.qpos[:] = q; d1.qpos[:] = q[7:]
mujoco.mj_forward(m0, d0); mujoco.mj_forward(m1, d1)
t0 = d0.body("trunk").xpos
for i in range(2, m0.nbody):
    n = mujoco.mj_id2name(m0, mujoco.mjtObj.mjOBJ_BODY, i)
    print(n, np.abs((d0.body(n).xipos - t0) - d1.body(n).xipos).max())   # expect ~1e-15
```

**3. It looks right.** In RViz2: add RobotModel with description topic `/robot_description`, fixed frame `base_link`; compare with the pose in `python -m mujoco.viewer`.

### 3.6 About SDF

Gazebo reads URDF directly (it converts internally), so keep the URDF as the single source of truth. If you want to see or hand-edit the SDF that Gazebo will use:

```bash
xacro go1.urdf > /tmp/go1_flat.urdf
gz sdf -p /tmp/go1_flat.urdf > go1.sdf          # Fortress:  ign sdf -p /tmp/go1_flat.urdf > go1.sdf
```

Only switch to a hand-maintained SDF if you need features URDF cannot express (for example native capsule collisions). In that case the ros2_control hardware interface still has to be described in a URDF given to the plugin.

> **Known gaps.** Things the export cannot reproduce: joint armature (reflected rotor inertia), frictionloss, MuJoCo's soft contact model and solver. Gazebo uses a different physics engine (DART by default in the world file here). These are the real sim-to-sim gaps your study measures, so write them down rather than trying to hide them.

---

## 4. Bring the robot up in Gazebo

### 4.1 What the generated URDF already contains

- **ros2_control** block: 12 joints, effort command interface, position/velocity/effort state interfaces, initial joint positions from default_q.
- **Gazebo plugin** for ros2_control (flavor-specific names are written for you) pointing at `$(find go_gazebo)/config/controllers.yaml`.
- **Sensors:** an IMU on `imu_link` (200 Hz) and a 360-sample 2D lidar on `lidar_link` (10 Hz, 0.12 m above the trunk origin; change `lidar_xyz`).
- **OdometryPublisher** plugin: ground-truth odometry for Nav2 (see Section 8 for why this is a simplification).

### 4.2 Controllers (`go_gazebo/config/controllers.yaml`)

```yaml
controller_manager:
  ros__parameters:
    use_sim_time: true
    update_rate: 500          # Hz; should divide the physics rate evenly
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    effort_controller:
      type: effort_controllers/JointGroupEffortController

effort_controller:
  ros__parameters:
    # SAME order as joint_names in robot_params.yaml (= policy action order)
    joints: [FR_hip_joint, FR_thigh_joint, FR_calf_joint,
             FL_hip_joint, FL_thigh_joint, FL_calf_joint,
             RR_hip_joint, RR_thigh_joint, RR_calf_joint,
             RL_hip_joint, RL_thigh_joint, RL_calf_joint]
```

The joint list must be identical to `joint_names` in robot_params.yaml, otherwise torques go to the wrong legs.

### 4.3 World (`go_gazebo/worlds/patrol_room.sdf`)

A 12 × 12 m walled room with friction 1.0 on the floor and three obstacles. The full file is in Appendix B. It uses Harmonic plugin names; for Fortress run the one-line `sed` shown in the file's comment. The lidar needs the Sensors system with a render engine, so on a laptop without a display start Gazebo with `-s --headless-rendering` rather than a plain server.

Make the world step match training as closely as you can: Playground environments define `sim_dt` and `ctrl_dt` in the config (the world uses 0.002 s; pd_controller runs at 500 Hz, a multiple of it).

### 4.4 Launch file (`go_gazebo/launch/sim.launch.py`)

```python
import os, xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg, gz_pkg = share("go_description"), share("go_gazebo")
    urdf = os.path.join(desc_pkg, "urdf", "go1.urdf")
    robot_xml = xacro.process_file(urdf).toxml()          # resolves $(find go_gazebo)
    world = os.path.join(gz_pkg, "worlds", "patrol_room.sdf")
    # Humble ships Fortress (ignition.msgs), Jazzy ships Harmonic (gz.msgs)
    M = "ignition.msgs" if os.environ.get("ROS_DISTRO") == "humble" else "gz.msgs"

    # lets Gazebo resolve package://go1_description/meshes/... URIs
    humble = os.environ.get("ROS_DISTRO") == "humble"
    var = "IGN_GAZEBO_RESOURCE_PATH" if humble else "GZ_SIM_RESOURCE_PATH"
    res = SetEnvironmentVariable(
        var, os.path.dirname(desc_pkg) + os.pathsep + os.environ.get(var, ""))

    gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world}"}.items())   # add " -s" for headless

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
    return LaunchDescription([res, gz, rsp, spawn, bridge, odom_tf] + chain)
```

The launch file sets the Gazebo resource path so `package://go1_description/meshes/…` resolves, starts Gazebo, robot_state_publisher, spawns the robot in the air, bridges clock/IMU/lidar/odometry, and starts the two ros2_control controllers one after the other.

### 4.5 Run it and check that everything is alive

```bash
cd ~/quadruped_rl/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch go_gazebo sim.launch.py                       # terminal 1

ros2 control list_controllers                             # joint_state_broadcaster + effort_controller: active
ros2 topic hz /joint_states                               # ~500 Hz
ros2 topic hz /imu                                        # ~200 Hz
ros2 topic hz /scan                                       # ~10 Hz
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames                            # odom -> base_link -> legs, imu_link, lidar_link
```

### 4.6 Stand test with the PD node only (no policy)

Copy `robot_params.yaml` into `policy_node/config/` as well (or point the parameter at the go_description copy), then run the PD node. With no policy running, its target is default_q.

```bash
ros2 run policy_node pd_controller --ros-args -p use_sim_time:=true \
    -p params_file:=$HOME/quadruped_rl/ros2_ws/src/go_description/config/robot_params.yaml
```

**Expected:** the robot lands from 0.45 m and stands. **Checkpoint:** it stands for a full minute without drifting or shaking.

| Symptom | Likely cause | Try |
|---|---|---|
| Robot collapses flat | Torques not reaching joints, or wrong joint order | `ros2 topic echo /effort_controller/commands`; compare controller joint list with robot_params.yaml |
| Legs vibrate | Kd too small for the loop rate, or timer running on wall time | Confirm use_sim_time true on the node; raise Kd; lower physics step |
| Falls over sideways at landing | Spawn height or default pose mismatch | Spawn at 0.35 m; check default_q against the MuJoCo keyframe |
| Body sinks slowly | Torque limit clipping | Compare torque_limit with the torque needed to hold the pose; check units |
| Mesh missing or at origin | Resource path not set | Check the printed Gazebo warnings; confirm package:// resolves under share/ |

---

## 5. Connect the walking policy

Use the NumPy policy and node structure from Sections 9 and 10 of your project guide, with these changes:

- **Publish position targets, not torques.** Replace the torque computation and publisher with `Float64MultiArray(data=q_target.tolist())` on `/policy/q_target`. pd_controller then does the PD law at 500 Hz, closer to how MuJoCo's position actuators work (policy at 50 Hz, PD at physics rate).
- **Subscribe to `/cmd_vel_mux`** instead of `/cmd_vel`, and clip it to the command ranges used in training (read them from the environment config).
- **Load joint order, default_q and action_scale from robot_params.yaml** instead of hard-coding them.
- **Gravity vector:** rotate world gravity (0, 0, −1) into the body frame using the IMU orientation quaternion: g_body = Rᵀ · (0, 0, −1). The gyro is already in the body frame.
- **Observation layout:** copy it from the Playground observation function (order, scales, history). Do not assume the list in the project guide matches it exactly.

```python
# changes inside policy_node.py (sketch)
self.declare_parameter("params_file", "")
p = yaml.safe_load(open(self.get_parameter("params_file").value))
self.joints = p["joint_names"]; self.default_q = np.array(p["default_q"]); self.scale = p["action_scale"]
self.create_subscription(Twist, "/cmd_vel_mux", self.on_cmd, 10)
self.pub = self.create_publisher(Float64MultiArray, "/policy/q_target", 10)
...
def on_cmd(self, msg):   # clip to the ranges used in training
    self.cmd = np.clip([msg.linear.x, msg.linear.y, msg.angular.z], self.cmd_lo, self.cmd_hi)
...
q_target = self.default_q + self.scale * action
self.pub.publish(Float64MultiArray(data=q_target.tolist()))
```

Latency note: the ROS loop (joint_states → policy → q_target → pd_controller → effort command) adds delay that the MuJoCo training did not have. It is a major sim-to-sim factor, and a natural experiment for your latency study. A custom C++ ros2_control controller would remove most of it later.

```bash
# run order for walking tests
ros2 launch go_gazebo sim.launch.py
ros2 run policy_node pd_controller --ros-args -p use_sim_time:=true -p params_file:=<robot_params.yaml>
ros2 run policy_node policy_node   --ros-args -p use_sim_time:=true -p params_file:=<robot_params.yaml>
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_mux   # quick test
```

> **Order matters.** Get walking working with the keyboard before adding the joystick and Nav2. If it does not walk under a keyboard, navigation will not fix it.

---

## 6. Joystick control with twist_mux

Hold the enable button (deadman) and use the sticks; release it and Nav2 takes over again. Joystick commands have priority 100 and Nav2 has priority 10, and twist_mux falls back to Nav2 after the joystick times out (0.5 s).

### 6.1 Config (`go_navigation/config/joy.yaml` and `twist_mux.yaml`)

```yaml
# --- config/joy.yaml ---------------------------------------------------------
joy_node:
  ros__parameters:
    deadzone: 0.1
    autorepeat_rate: 20.0
teleop_twist_joy_node:
  ros__parameters:
    require_enable_button: true      # deadman: robot moves only while held
    enable_button: 4                 # LB on most Xbox-style pads (check: ros2 topic echo /joy)
    axis_linear: {x: 1, y: 0}        # left stick: forward/back and strafe
    scale_linear: {x: 0.6, y: 0.4}   # keep inside the command range used in TRAINING
    axis_angular: {yaw: 3}           # right stick: turn
    scale_angular: {yaw: 0.8}

# --- config/twist_mux.yaml ---------------------------------------------------
twist_mux:
  ros__parameters:
    use_sim_time: true
    topics:
      joystick:   {topic: cmd_vel_joy,       timeout: 0.5, priority: 100}
      navigation: {topic: cmd_vel,           timeout: 0.5, priority: 10}  # Nav2's final output
```

### 6.2 Launch file (`go_navigation/launch/teleop.launch.py`)

```python
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
```

```bash
ros2 launch go_navigation teleop.launch.py
ros2 topic echo /joy                 # press buttons to find your pad's enable button and axes
ros2 topic echo /cmd_vel_mux         # should follow the sticks while the enable button is held
```

Keep `scale_linear` and `scale_angular` inside the velocity ranges the policy was trained on. Commands outside that range are the easiest way to make a working policy fall over.

---

## 7. Navigation: mapping, patrol routes, obstacle avoidance

### 7.1 Frames and data Nav2 needs

| Need | Provided by | Note |
|---|---|---|
| map → odom | AMCL (or slam_toolbox while mapping) | Needs /scan and the map |
| odom → base_link | odom_tf node, from Gazebo ground-truth /odom | Perfect odometry; see Section 8 for the honest caveat |
| base_link → legs, imu_link, lidar_link | robot_state_publisher | From the generated URDF |
| /scan | Gazebo lidar via ros_gz_bridge | Obstacles must reach lidar height (0.12 m above trunk centre) |
| /cmd_vel output | Nav2 controller (DWB) | Goes to twist_mux, then policy_node |

### 7.2 Nav2 parameters

Copy `nav2_bringup/params/nav2_params.yaml` from your install into `go_navigation/config/nav2_params.yaml` and change the keys below. Key names vary slightly between Humble and Jazzy, so edit the copy from your own distro rather than pasting this over it.

```yaml
# Edit a COPY of nav2_bringup/params/nav2_params.yaml; these are the keys that matter.
amcl:
  ros__parameters:
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    scan_topic: scan
    robot_model_type: "nav2_amcl::OmniMotionModel"   # the policy can strafe (vy)
bt_navigator:
  ros__parameters:
    robot_base_frame: base_link
    odom_topic: /odom
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0          # no reversing at first (training range permitting)
      max_vel_x: 0.5          # <= training command range
      min_vel_y: -0.3
      max_vel_y: 0.3
      max_vel_theta: 0.8
      vy_samples: 10          # >1 enables sideways samples (holonomic)
      acc_lim_x: 1.0
      acc_lim_y: 1.0
      acc_lim_theta: 2.0
local_costmap:
  local_costmap:
    ros__parameters:
      robot_base_frame: base_link
      robot_radius: 0.35      # trunk ~0.65 m long; inflate a bit for leg swing
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        observation_sources: scan
        scan: {topic: /scan, data_type: "LaserScan", marking: true, clearing: true}
global_costmap:
  global_costmap:
    ros__parameters:
      robot_base_frame: base_link
      robot_radius: 0.35
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        observation_sources: scan
        scan: {topic: /scan, data_type: "LaserScan", marking: true, clearing: true}
```

- **Velocity limits** must sit inside the policy's training command range. Start slow (0.3–0.5 m/s) and raise later.
- **Holonomic:** the policy accepts vy, so sideways samples in DWB and the Omni motion model in AMCL are valid choices.
- **Inflation:** legs swing wide and the body is long; if Nav2 refuses to enter gaps, reduce inflation before reducing robot_radius.

### 7.3 Make a map by driving with the joystick

```bash
# terminals: sim, pd_controller, policy_node, teleop.launch.py are already running
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
rviz2                                    # add Map (/map), LaserScan (/scan), RobotModel; fixed frame: map
# drive around the room slowly with the joystick, then save:
ros2 run nav2_map_server map_saver_cli -f ~/quadruped_rl/ros2_ws/src/go_navigation/maps/room
```

### 7.4 Run Nav2 with the saved map

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true \
    map:=$HOME/quadruped_rl/ros2_ws/src/go_navigation/maps/room.yaml \
    params_file:=$HOME/quadruped_rl/ros2_ws/src/go_navigation/config/nav2_params.yaml
rviz2    # set the initial pose with '2D Pose Estimate', then try a '2D Goal Pose'
```

Stop here until a single goal pose works: the robot should plan around the obstacles, walk there, and stop. Only then run a patrol.

### 7.5 Patrol routes (`go_navigation/go_navigation/patrol.py`)

```python
#!/usr/bin/env python3
"""Loop through waypoints forever with Nav2. Run after Nav2 is up and localized."""
import math, rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# (x, y, yaw_deg) in the MAP frame - edit for your map
WAYPOINTS = [(3.0, 3.0, 0), (3.0, -3.0, -90), (-3.0, -3.0, 180), (-3.0, 3.0, 90)]


def pose(nav, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    yaw = math.radians(yaw_deg)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.setInitialPose(pose(nav, 0.0, 0.0, 0))   # must match where the robot spawned
    nav.waitUntilNav2Active()
    lap = 0
    while rclpy.ok():
        lap += 1
        nav.followWaypoints([pose(nav, *w) for w in WAYPOINTS])
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb:
                nav.get_logger().info(f"lap {lap}: heading to waypoint {fb.current_waypoint + 1}",
                                      throttle_duration_sec=5.0)
        result = nav.getResult()
        if result != TaskResult.SUCCEEDED:
            nav.get_logger().warn(f"lap {lap} ended with {result}; restarting")
    nav.lifecycleShutdown()


if __name__ == "__main__":
    main()
```

```bash
ros2 run go_navigation patrol --ros-args -p use_sim_time:=true
```

Edit `WAYPOINTS` to points that are free space on *your* map (check them in RViz). To take over manually, hold the joystick enable button; release it and the patrol resumes. To stop the patrol, press Ctrl-C in the patrol terminal.

### 7.6 Test obstacle avoidance

- Place a goal directly behind box1 or the pillar: the path should curve around it.
- Drive the joystick toward a wall with the enable button held: check how the robot behaves when you release it next to an obstacle.
- Add a new box in Gazebo while the patrol runs (Gazebo's resource spawner or `ros_gz_sim create`): the local costmap should react and replan.
- Add an obstacle *below* the lidar plane: it will be invisible. That is a sensor limit, not a Nav2 bug.

| Symptom | Likely cause | Try |
|---|---|---|
| "No valid trajectories" / robot spins | Velocity limits below what the controller needs, or costmap inflated too much | Lower inflation_radius; raise max_vel_theta slightly; check footprint |
| Robot stops, then oscillates near goal | Walking policy cannot track tiny velocity commands | Raise min speeds, loosen goal tolerances (xy_goal_tolerance, yaw_goal_tolerance) |
| Falls when turning sharply | Yaw rate or acceleration outside training range | Lower max_vel_theta and acc_lim_theta; clip commands in policy_node |
| Drifts off the planned path | Policy velocity tracking is imperfect; odometry is ground truth so the planner sees the error | Lower speed; check yaw tracking in MuJoCo vs Gazebo |
| Map shows the robot's own legs as obstacles | Lidar too low or min range too small | Raise lidar_xyz z; raise lidar min range |
| Nav2 times out waiting for tf | use_sim_time missing somewhere | Pass use_sim_time:=true to every node and launch |

---

## 8. Honest limits of this setup

- **Ground-truth odometry makes navigation look better than it will on a real robot.** For a realistic study later, replace the OdometryPublisher with leg odometry plus IMU fused in `robot_localization` (EKF), and compare patrol accuracy.
- **The policy does not see obstacles.** It will walk into anything Nav2 does not route around. Add a collision monitor stage if you want a last-resort stop.
- **Flat floor only.** The baseline policy was trained on flat terrain; slopes and stairs are separate experiments.
- **Gazebo is not MuJoCo.** Expect the policy to degrade. If it falls in Gazebo but not MuJoCo, check in this order: joint order, action scale, gains, default pose, loop timing, then friction and contact.

## 9. Terminal cheat-sheet (full stack)

```bash
T1  ros2 launch go_gazebo sim.launch.py
T2  ros2 run policy_node pd_controller --ros-args -p use_sim_time:=true -p params_file:=<robot_params.yaml>
T3  ros2 run policy_node policy_node   --ros-args -p use_sim_time:=true -p params_file:=<robot_params.yaml>
T4  ros2 launch go_navigation teleop.launch.py
T5  ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=<room.yaml> params_file:=<nav2_params.yaml>
T6  ros2 run go_navigation patrol --ros-args -p use_sim_time:=true
    rviz2     # set initial pose, watch costmaps and path
```

### Suggested checkpoints

| # | Checkpoint | Done when |
|---|---|---|
| 1 | Workspace builds, four packages listed | `ros2 pkg list` shows all of them |
| 2 | Export verified | COM error ≈ 1e-15 against MuJoCo; check_urdf prints the tree |
| 3 | Robot stands in Gazebo with pd_controller | Stable for 60 s |
| 4 | Policy walks under keyboard | Forward, turn and strafe all work |
| 5 | Joystick drives it through twist_mux | Deadman works; releasing it stops the robot |
| 6 | Map saved, single Nav2 goal reached | Robot routes around box1 and the pillar |
| 7 | Patrol completes 3 laps | No falls, no collisions; log time per lap |

**Metrics worth logging for your report:** laps completed before first fall, collision count, time per lap, position error at each waypoint, and the same numbers under changed friction, added latency, and different training seeds.

## 10. Troubleshooting by layer

| Layer | Check | Command |
|---|---|---|
| Build | Sourced the workspace? Package installed its folders? | `ros2 pkg prefix go_description`; look under install/…/share |
| Gazebo | World loads, model appears with meshes | Look for resource and plugin warnings in terminal 1 |
| ros2_control | Controllers active; plugin file name correct for your Gazebo | `ros2 control list_hardware_interfaces` |
| Bridge | Topics exist with the right message type | `ros2 topic info /imu -v`; `gz topic -l` (Fortress: `ign topic -l`) |
| Time | Everything on sim time | `ros2 topic echo /clock --once`; every node has use_sim_time true |
| TF | Tree connected, no duplicate publishers | `ros2 run tf2_tools view_frames` |
| Policy | Same order, scales and rate as training | Log observation and action for one step on both sides and compare |
| Nav2 | Lifecycle nodes active, costmaps show obstacles | `ros2 lifecycle get /controller_server` |

## 11. Likely trouble spots in this guide

These are the parts that could not be tested here and that depend on your exact versions:

- **Gazebo plugin and message-type names** differ between Fortress (ignition.* names) and Harmonic (gz.* names). The exporter and launch file switch on the flavor / ROS_DISTRO, but check against your install.
- **ros2_control initial_value** for joint positions depends on the plugin version. If the legs spawn straight, keep spawning in the air with pd_controller running so they reach the default pose before touchdown.
- **Sensors on fixed-joint links:** if the IMU or lidar appears at the wrong place, add `<gazebo reference='imu_joint'><preserveFixedJoint>true</preserveFixedJoint></gazebo>` (same for lidar_joint) to stop Gazebo merging the links.
- **Odometry frame names** from Gazebo may carry a model prefix; odom_tf deliberately overrides them with odom / base_link.
- **Nav2 parameter names and the final cmd_vel topic** change across releases. Confirm with `ros2 topic info /cmd_vel -v` that Nav2 is the publisher on the topic twist_mux reads.
- **Robot parameters for Go2** differ from Go1: re-run the exporter on the Go2 model and use its own robot_params.yaml.

---

## Appendix A: `mjcf_to_urdf.py`

Run inside the venv with mujoco and trimesh installed.

```python
#!/usr/bin/env python3
"""Export a compiled MuJoCo model (MjModel) to URDF + STL meshes + robot_params.yaml.

Works on the SAME model object the policy was trained on, e.g. env.mj_model in
MuJoCo Playground, so masses, joint limits, gains and joint order all match.

CLI:   python mjcf_to_urdf.py go1.xml --out go1_description --name go1
Python: from mjcf_to_urdf import export_urdf; export_urdf(env.mj_model, "go1_description")
"""
import argparse, math, os
import numpy as np
import mujoco
import trimesh

OBJ = mujoco.mjtObj
GEOM = mujoco.mjtGeom

FLAVORS = {  # Gazebo-version specific strings
    "harmonic": dict(  # Ubuntu 24.04 / ROS 2 Jazzy
        hw="gz_ros2_control/GazeboSimSystem",
        sys_file="gz_ros2_control-system",
        sys_name="gz_ros2_control::GazeboSimROS2ControlPlugin",
        frame_tag="gz_frame_id",
        odom_file="gz-sim-odometry-publisher-system",
        odom_name="gz::sim::systems::OdometryPublisher"),
    "fortress": dict(  # Ubuntu 22.04 / ROS 2 Humble
        hw="ign_ros2_control/IgnitionSystem",
        sys_file="ign_ros2_control-system",
        sys_name="ign_ros2_control::IgnitionROS2ControlPlugin",
        frame_tag="ignition_frame_id",
        odom_file="ignition-gazebo-odometry-publisher-system",
        odom_name="ignition::gazebo::systems::OdometryPublisher"),
}


def f(v):
    return " ".join(f"{float(x):.9g}" for x in np.atleast_1d(v))


def quat_to_rpy(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, dtype=float))
    R = R.reshape(3, 3)
    pitch = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def origin(pos, quat):
    return f'<origin xyz="{f(pos)}" rpy="{f(quat_to_rpy(quat))}"/>'


def name_of(m, kind, i, fallback):
    return mujoco.mj_id2name(m, kind, i) or f"{fallback}{i}"


def export_urdf(m, out_dir, robot_name="go1", root_name="base_link",
                flavor="fortress", default_effort=35.0, max_velocity=30.0,
                lidar_xyz=(0.0, 0.0, 0.12), extras=True, mesh_prefix=None):
    """Write <out_dir>/urdf/<robot>.urdf, meshes/*.stl and config/robot_params.yaml."""
    fl = FLAVORS[flavor]
    mesh_dir = os.path.join(out_dir, "meshes")
    urdf_dir = os.path.join(out_dir, "urdf")
    cfg_dir = os.path.join(out_dir, "config")
    for d in (mesh_dir, urdf_dir, cfg_dir):
        os.makedirs(d, exist_ok=True)
    if mesh_prefix is None:
        mesh_prefix = f"package://{os.path.basename(os.path.normpath(out_dir))}/meshes"

    roots = [b for b in range(1, m.nbody) if m.body_parentid[b] == 0]
    assert len(roots) == 1, "expected exactly one root body"
    link = {b: (root_name if b == roots[0] else name_of(m, OBJ.mjOBJ_BODY, b, "body"))
            for b in range(1, m.nbody)}

    # actuator order == policy action order
    act_joint = {}
    for a in range(m.nu):
        assert m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT, "only joint actuators"
        act_joint[int(m.actuator_trnid[a, 0])] = a

    # one STL per mesh, taken from the compiled model: MuJoCo recentres meshes,
    # so the compiled geom poses only match the compiled (recentred) vertices.
    mesh_file = {}
    for mid in range(m.nmesh):
        nm = name_of(m, OBJ.mjOBJ_MESH, mid, "mesh")
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
        tm = trimesh.Trimesh(vertices=m.mesh_vert[va:va + vn].copy(),
                             faces=m.mesh_face[fa:fa + fn].copy(), process=False)
        tm.export(os.path.join(mesh_dir, nm + ".stl"))
        mesh_file[mid] = nm + ".stl"

    mats, lines = {}, []

    def material(rgba):
        key = tuple(round(float(x), 3) for x in rgba)
        if key not in mats:
            mats[key] = f"mat{len(mats)}"
        return mats[key]

    def geom_xml(g, tag):
        t = int(m.geom_type[g])
        pos, quat, size = m.geom_pos[g], m.geom_quat[g], m.geom_size[g]
        rgba = m.geom_rgba[g]
        mat = f'<material name="{material(rgba)}"/>' if tag == "visual" else ""
        o = origin(pos, quat)
        if t == GEOM.mjGEOM_MESH:
            shp = [f'<mesh filename="{mesh_prefix}/{mesh_file[int(m.geom_dataid[g])]}"/>']
        elif t == GEOM.mjGEOM_BOX:
            shp = [f'<box size="{f(2 * size)}"/>']
        elif t == GEOM.mjGEOM_SPHERE:
            shp = [f'<sphere radius="{f(size[0])}"/>']
        elif t == GEOM.mjGEOM_CYLINDER:
            shp = [f'<cylinder radius="{f(size[0])}" length="{f(2 * size[1])}"/>']
        elif t == GEOM.mjGEOM_CAPSULE:  # URDF has no capsule: cylinder + 2 spheres
            r, h = size[0], size[1]
            out = []
            for dz, kind in ((0.0, "cyl"), (h, "sph"), (-h, "sph")):
                R = np.zeros(9); mujoco.mju_quat2Mat(R, np.asarray(quat, float))
                p = np.asarray(pos) + R.reshape(3, 3) @ np.array([0, 0, dz])
                s = (f'<cylinder radius="{f(r)}" length="{f(2 * h)}"/>' if kind == "cyl"
                     else f'<sphere radius="{f(r)}"/>')
                out.append(f"<{tag}>{origin(p, quat)}<geometry>{s}</geometry>{mat}</{tag}>")
            return out
        else:
            print(f"  skipping geom {g}: type {t} not supported")
            return []
        return [f"<{tag}>{o}<geometry>{shp[0]}</geometry>{mat}</{tag}>"]

    for b in range(1, m.nbody):
        mass = max(float(m.body_mass[b]), 1e-4)
        inertia = np.maximum(m.body_inertia[b], 1e-9)
        vis, col = [], []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b:
                continue
            visual_only = m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0
            if visual_only:
                vis += geom_xml(g, "visual")
            else:
                col += geom_xml(g, "collision")
        if not vis:  # no dedicated visual mesh: show the collision shapes
            for g in range(m.ngeom):
                solid = not (m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0)
                if m.geom_bodyid[g] == b and solid:
                    vis += geom_xml(g, "visual")
        lines.append(f'  <link name="{link[b]}">')
        lines.append(f'    <inertial>{origin(m.body_ipos[b], m.body_iquat[b])}'
                     f'<mass value="{f(mass)}"/>'
                     f'<inertia ixx="{f(inertia[0])}" iyy="{f(inertia[1])}" izz="{f(inertia[2])}"'
                     f' ixy="0" ixz="0" iyz="0"/></inertial>')
        for s in vis + col:
            lines.append("    " + s)
        lines.append("  </link>")

    joint_names = []
    for b in range(1, m.nbody):
        if b == roots[0]:
            continue
        jn = int(m.body_jntnum[b])
        parent = link[int(m.body_parentid[b])]
        o = origin(m.body_pos[b], m.body_quat[b])
        if jn == 0:
            lines.append(f'  <joint name="{link[b]}_fixed" type="fixed"><parent link="{parent}"/>'
                         f'<child link="{link[b]}"/>{o}</joint>')
            continue
        assert jn == 1, f"body {link[b]} has {jn} joints; only 1 supported"
        j = int(m.body_jntadr[b])
        jt = int(m.jnt_type[j])
        J = mujoco.mjtJoint
        assert jt in (int(J.mjJNT_HINGE), int(J.mjJNT_SLIDE)), "hinge/slide only"
        if np.linalg.norm(m.jnt_pos[j]) > 1e-9:
            print(f"  WARNING: joint {j} has a non-zero anchor offset; add it by hand")
        jname = name_of(m, OBJ.mjOBJ_JOINT, j, "joint")
        joint_names.append(jname)
        lo, hi = m.jnt_range[j]
        limited = bool(m.jnt_limited[j])
        kind = ("revolute" if limited else "continuous") if jt == int(mujoco.mjtJoint.mjJNT_HINGE) \
            else "prismatic"
        effort = default_effort
        if j in act_joint and m.actuator_forcelimited[act_joint[j]]:
            effort = float(np.max(np.abs(m.actuator_forcerange[act_joint[j]])))
        lim = (f'<limit lower="{f(lo)}" upper="{f(hi)}" effort="{f(effort)}"'
               f' velocity="{f(max_velocity)}"/>'
               if limited else f'<limit effort="{f(effort)}" velocity="{f(max_velocity)}"/>')
        lines.append(f'  <joint name="{jname}" type="{kind}"><parent link="{parent}"/>'
                     f'<child link="{link[b]}"/>{o}<axis xyz="{f(m.jnt_axis[j])}"/>{lim}'
                     f'<dynamics damping="0" friction="0"/></joint>')

    # ---- policy-facing parameters, in ACTUATOR order (= action order) ----
    names, kp, kd_act, damp, tlim, qdef = [], [], [], [], [], []
    key = None
    for k in range(m.nkey):
        if mujoco.mj_id2name(m, OBJ.mjOBJ_KEY, k) == "home":
            key = k
    if key is None and m.nkey:
        key = 0
    for a in range(m.nu):
        j = int(m.actuator_trnid[a, 0])
        names.append(name_of(m, OBJ.mjOBJ_JOINT, j, "joint"))
        kp.append(float(m.actuator_gainprm[a, 0]))
        kd_act.append(abs(float(m.actuator_biasprm[a, 2])))
        damp.append(float(m.dof_damping[m.jnt_dofadr[j]]))
        tlim.append(float(np.max(np.abs(m.actuator_forcerange[a])))
                    if m.actuator_forcelimited[a] else default_effort)
        qa = m.jnt_qposadr[j]
        qdef.append(float(m.key_qpos[key][qa] if key is not None else m.qpos0[qa]))
    yaml = ["# Generated by mjcf_to_urdf.py - read by pd_controller and policy_node",
            f"timestep_sim: {float(m.opt.timestep)}",
            "joint_names: [" + ", ".join(names) + "]  # ACTION ORDER",
            "default_q: [" + ", ".join(f"{x:.6g}" for x in qdef) + "]",
            "kp: [" + ", ".join(f"{x:.6g}" for x in kp) + "]",
            "kd_actuator: [" + ", ".join(f"{x:.6g}" for x in kd_act) + "]",
            "joint_damping: [" + ", ".join(f"{x:.6g}" for x in damp) + "]",
            "torque_limit: [" + ", ".join(f"{x:.6g}" for x in tlim) + "]",
            "action_scale: 0.5  # FILL IN: copy from the environment config",
            ""]
    with open(os.path.join(cfg_dir, "robot_params.yaml"), "w") as fh:
        fh.write("\n".join(yaml))

    head = [f'<?xml version="1.0"?>',
            f'<robot name="{robot_name}" xmlns:xacro="http://www.ros.org/wiki/xacro">']
    for key_rgba, mn in mats.items():
        head.append(f'  <material name="{mn}"><color rgba="{f(key_rgba)}"/></material>')
    tail = []
    if extras:
        lx = f(lidar_xyz)
        tail += [f'  <link name="imu_link"/>',
                 f'  <joint name="imu_joint" type="fixed"><parent link="{root_name}"/>'
                 f'<child link="imu_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>',
                 f'  <link name="lidar_link"/>',
                 f'  <joint name="lidar_joint" type="fixed"><parent link="{root_name}"/>'
                 f'<child link="lidar_link"/><origin xyz="{lx}" rpy="0 0 0"/></joint>',
                 '  <ros2_control name="GazeboSystem" type="system">',
                 f'    <hardware><plugin>{fl["hw"]}</plugin></hardware>']
        for jn, q0 in zip(names, qdef):
            tail += [f'    <joint name="{jn}">',
                     '      <command_interface name="effort"/>',
                     '      <state_interface name="position">',
                     f'        <param name="initial_value">{q0:.6g}</param>',
                     '      </state_interface>',
                     '      <state_interface name="velocity"/>',
                     '      <state_interface name="effort"/>',
                     '    </joint>']
        tail += ['  </ros2_control>',
                 '  <gazebo>',
                 f'    <plugin filename="{fl["sys_file"]}" name="{fl["sys_name"]}">',
                 '      <parameters>$(find go_gazebo)/config/controllers.yaml</parameters>',
                 '    </plugin>',
                 f'    <plugin filename="{fl["odom_file"]}" name="{fl["odom_name"]}">',
                 '      <odom_frame>odom</odom_frame>',
                 f'      <robot_base_frame>{root_name}</robot_base_frame>',
                 '      <odom_publish_frequency>50</odom_publish_frequency>',
                 '      <dimensions>3</dimensions>',
                 '    </plugin>',
                 '  </gazebo>',
                 '  <gazebo reference="imu_link">',
                 '    <sensor name="imu_sensor" type="imu">',
                 '      <always_on>true</always_on><update_rate>200</update_rate>',
                 '      <topic>imu</topic>',
                 f'      <{fl["frame_tag"]}>imu_link</{fl["frame_tag"]}>',
                 '    </sensor>',
                 '  </gazebo>',
                 '  <gazebo reference="lidar_link">',
                 '    <sensor name="lidar" type="gpu_lidar">',
                 '      <always_on>true</always_on><update_rate>10</update_rate>',
                 '      <topic>scan</topic>',
                 f'      <{fl["frame_tag"]}>lidar_link</{fl["frame_tag"]}>',
                 '      <lidar>',
                 '        <scan><horizontal><samples>360</samples><min_angle>-3.14159</min_angle>'
                 '<max_angle>3.14159</max_angle></horizontal></scan>',
                 '        <range><min>0.15</min><max>12.0</max>'
                 '<resolution>0.01</resolution></range>',
                 '      </lidar>',
                 '    </sensor>',
                 '  </gazebo>']
    path = os.path.join(urdf_dir, f"{robot_name}.urdf")
    with open(path, "w") as fh:
        fh.write("\n".join(head + lines + tail + ["</robot>", ""]))
    print(f"wrote {path}, {len(mesh_file)} meshes, {len(joint_names)} joints")
    print("action order:", names)
    return path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("xml")
    ap.add_argument("--out", default="go1_description")
    ap.add_argument("--name", default="go1")
    ap.add_argument("--flavor", default="fortress", choices=list(FLAVORS))
    ap.add_argument("--no-extras", action="store_true")
    a = ap.parse_args()
    export_urdf(mujoco.MjModel.from_xml_path(a.xml), a.out, a.name,
                flavor=a.flavor, extras=not a.no_extras)
```

## Appendix B: `go_gazebo/worlds/patrol_room.sdf`

```xml
<sdf version="1.8">
  <world name="patrol_room">
    <physics name="p" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- Harmonic names. For Fortress run:  sed -i 's/gz-sim-/ignition-gazebo-/;s/gz::sim::/ignition::gazebo::/' -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <cast_shadows>true</cast_shadows>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="l">
        <collision name="c">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>40 40</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="v">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>40 40</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <!-- 12 x 12 m room: four walls -->
    <model name="walls">
      <static>true</static>
      <link name="n">
        <pose>0 6 0.5 0 0 0</pose>
        <collision name="c">
          <geometry>
            <box>
              <size>12 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="v">
          <geometry>
            <box>
              <size>12 0.2 1</size>
            </box>
          </geometry>
        </visual>
      </link>
      <link name="s">
        <pose>0 -6 0.5 0 0 0</pose>
        <collision name="c">
          <geometry>
            <box>
              <size>12 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="v">
          <geometry>
            <box>
              <size>12 0.2 1</size>
            </box>
          </geometry>
        </visual>
      </link>
      <link name="e">
        <pose>6 0 0.5 0 0 0</pose>
        <collision name="c">
          <geometry>
            <box>
              <size>0.2 12 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="v">
          <geometry>
            <box>
              <size>0.2 12 1</size>
            </box>
          </geometry>
        </visual>
      </link>
      <link name="w">
        <pose>-6 0 0.5 0 0 0</pose>
        <collision name="c">
          <geometry>
            <box>
              <size>0.2 12 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="v">
          <geometry>
            <box>
              <size>0.2 12 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
    <!-- obstacles to test avoidance (tall enough to be seen by the lidar) -->
    <model name="box1">
      <static>true</static>
      <pose>2.5 1.0 0.4 0 0 0.4</pose>
      <link name="l">
        <collision name="c">
          <geometry>
            <box>
              <size>0.8 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="v">
          <geometry>
            <box>
              <size>0.8 0.8 0.8</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
    <model name="pillar1">
      <static>true</static>
      <pose>-1.5 2.5 0.5 0 0 0</pose>
      <link name="l">
        <collision name="c">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="v">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </visual>
      </link>
    </model>
    <model name="box2">
      <static>true</static>
      <pose>0.5 -2.5 0.4 0 0 0</pose>
      <link name="l">
        <collision name="c">
          <geometry>
            <box>
              <size>1.5 0.5 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="v">
          <geometry>
            <box>
              <size>1.5 0.5 0.8</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Appendix C: other files

### `policy_node/pd_controller.py`

```python
#!/usr/bin/env python3
"""High-rate joint PD loop. Policy node sends position targets at 50 Hz;
this node turns them into torques at 500 Hz, like MuJoCo's position actuators do."""
import numpy as np, rclpy, yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PDController(Node):
    def __init__(self):
        super().__init__("pd_controller")
        self.declare_parameter("params_file", "")
        p = yaml.safe_load(open(self.get_parameter("params_file").value))
        self.names = p["joint_names"]
        self.kp = np.array(p["kp"], float)
        # actuator damping + passive joint damping from MuJoCo, applied once, here
        self.kd = np.array(p["kd_actuator"], float) + np.array(p["joint_damping"], float)
        self.tau_max = np.array(p["torque_limit"], float)
        self.target = np.array(p["default_q"], float)      # hold default pose until policy runs
        self.q = self.dq = None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Float64MultiArray, "/policy/q_target", self.on_target, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/effort_controller/commands", 10)
        self.create_timer(1.0 / 500.0, self.step)           # follows sim time (use_sim_time)

    def on_joints(self, msg):
        idx = [msg.name.index(n) for n in self.names]       # map by NAME, never by position
        self.q = np.array(msg.position)[idx]
        self.dq = np.array(msg.velocity)[idx]

    def on_target(self, msg):
        self.target = np.array(msg.data, float)

    def step(self):
        if self.q is None:
            return
        tau = self.kp * (self.target - self.q) - self.kd * self.dq
        tau = np.clip(tau, -self.tau_max, self.tau_max)
        self.pub.publish(Float64MultiArray(data=tau.tolist()))


def main():
    rclpy.init(); rclpy.spin(PDController()); rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### `policy_node/odom_tf.py`

```python
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
```
