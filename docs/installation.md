# Installation

Tested on **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic** (Nav2 1.3.13). Ubuntu 22.04 + Humble + Fortress is supported by the exporter and launch files (they switch plugin and message names on `ROS_DISTRO`), but was not run.

## 1. ROS 2 and Gazebo packages

Install ROS 2 Jazzy (desktop) from the official docs first, then:

```bash
sudo apt install ros-dev-tools \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-rviz2 \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard ros-jazzy-twist-mux \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

The obstacle and reset tools talk to Gazebo through the `gz.transport13` / `gz.msgs10` Python bindings, which come with `gz_transport_vendor` / `gz_msgs_vendor` (pulled in by `ros-jazzy-ros-gz`). Check with:

```bash
python3 -c "import gz.transport13, gz.msgs10; print('ok')"
```

## 2. Build the workspace

The repository root *is* the colcon workspace (packages live in `src/`).

```bash
git clone https://github.com/majdaliengmech-glitch/Quadruped-RL.git
cd Quadruped-RL
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep -E "go_|policy_node"     # go_description, go_gazebo, go_navigation, policy_node
```

`--symlink-install` lets you edit Python nodes, launch files and YAML without rebuilding. Rebuild after adding files or changing `setup.py` / `CMakeLists.txt`. `src/tools` contains a `COLCON_IGNORE` and is not built.

## 3. Python environment for the offline tools (optional)

Only needed to re-export the robot model or convert a newly trained policy. Use a separate venv, not the ROS Python.

```bash
python3 -m venv ~/rl_venv && source ~/rl_venv/bin/activate
pip install mujoco trimesh numpy              # mjcf_to_urdf.py, check_urdf.py
pip install jax brax mujoco_playground        # export_policy.py (to unpickle Brax params)
```

> Keep this venv out of the ROS shell. If its NumPy 2 leaks into `PYTHONPATH`, system packages such as matplotlib fail to import.

## 4. First run

```bash
ros2 launch go_navigation bringup.launch.py       # Gazebo opens, robot drops in and stands
```

Continue with [simulation.md](simulation.md).
