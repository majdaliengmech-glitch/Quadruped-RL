# Navigation

Nav2 on ROS 2 Jazzy, with MPPI as the controller, AMCL for localisation and a SLAM map made by walking the robot through the factory.

## 1. Build a map (slam_toolbox)

```bash
ros2 launch go_navigation bringup.launch.py        # T1: sim + policy + teleop
ros2 launch go_navigation mapping.launch.py        # T2: slam_toolbox + RViz (map, scan, robot, SLAM panel)
# drive slowly through the whole hall with the gamepad or keyboard, then:
ros2 run nav2_map_server map_saver_cli -f src/go_navigation/maps/factory
```

![Gazebo factory and the SLAM map in RViz](images/nav2_slam_mapping_factory.jpg)

Settings are in `config/slam_params.yaml` (`base_frame: base_link`, `max_laser_range` 12 m, scans added every 0.3 m / 0.3 rad). Maps shipped: `maps/factory.{yaml,pgm}` (5 cm cells) and `maps/room.{yaml,pgm}`.

> The shipped factory map has some drift: the north wall is smeared over ~1.5 m and some columns are 0.2–0.4 m off. Navigation copes, and the patrol waypoints avoid those areas.

## 2. Start Nav2

```bash
ros2 launch go_navigation navigation.launch.py                                   # factory map + RViz
ros2 launch go_navigation navigation.launch.py map:=$PWD/src/go_navigation/maps/room.yaml rviz:=false
```

Wait for `Managed nodes are active` twice (localisation and navigation). AMCL starts at the spawn pose (`set_initial_pose: true`, 0/0/0), so no "2D Pose Estimate" is needed unless you moved the robot before starting Nav2.

Velocity chain: `controller_server` (MPPI) → `/cmd_vel_nav` → `velocity_smoother` → `/cmd_vel_smoothed` → `collision_monitor` → `/cmd_vel` → `twist_mux` (priority 10) → `/cmd_vel_mux` → `policy_node`.

## 3. One goal: `goto`

```bash
ros2 run go_navigation goto 6 0          # x y [yaw_deg] in the map frame
ros2 run go_navigation goto 0 7 90
```

It waits for Nav2 (without resetting AMCL), plans, drives, detects falls, and prints:

```text
result SUCCEEDED: 17.2 s sim time (real-time factor 0.79), ground-truth error 0.32 m, planned length 6.0 m
```

Ctrl‑C cancels the goal and the robot stops. In RViz, **2D Goal Pose** does the same without the report.

## 4. Patrol: `patrol`

```bash
ros2 run go_navigation patrol                                    # forever
ros2 run go_navigation patrol --ros-args -p laps:=3
ros2 run go_navigation patrol --ros-args -p waypoints_file:=$PWD/src/go_navigation/config/patrol_room.yaml
```

| Parameter | Default | Meaning |
|---|---|---|
| `waypoints_file` | `config/patrol_factory.yaml` | list of `[x, y, yaw_deg]` in the map frame |
| `laps` | 0 | 0 = until Ctrl‑C |
| `auto_reset` | true | on a fall, stand the robot up where it fell and retry the waypoint; false = stop |
| `log_dir` | `~/.ros/patrol_logs` | one CSV per run |

Factory route (`patrol_factory.yaml`), ~66 m per lap, each point ≥ 2.3 m from walls on the map:

| # | x, y, yaw | Place |
|---|---|---|
| 1 | 12.0, 0.0, 0° | east end of the main aisle |
| 2 | 0.0, −7.5, −90° | south open area, west of the shipping crates |
| 3 | −13.0, 0.5, 180° | west end of the main aisle |
| 4 | 0.0, 7.0, 90° | north bay between racks and machines |

The route deliberately avoids the forklift forks near (9.5, −4.5): they are 6 cm tall, below the lidar, so they are not on the map but can trip the robot.

CSV columns: `lap, waypoint, x, y, reached, error_m, lap_time_s, falls_so_far`. `error_m` is the **ground-truth** distance between the robot and the waypoint when Nav2 declared it reached.

Holding the gamepad deadman takes over at any time; releasing it lets the patrol continue.

## 5. New obstacles: `obstacles`

Obstacles spawned at run time are not on the map. Nav2 must see them with the lidar and avoid them live: the local costmap's voxel layer and the global costmap's obstacle layer mark them, the planner replans (1 Hz), MPPI steers around them, and the collision monitor slows the robot if it still gets too close.

```bash
ros2 run go_navigation obstacles scenario aisle        # 3 crates on legs 4→1, 1→2, 2→3 of the patrol
ros2 run go_navigation obstacles scenario bay          # pallets at the north bay entrance
ros2 run go_navigation obstacles scenario block        # 5 m wall across the aisle at x = 10
ros2 run go_navigation obstacles spawn crate 4 0.5 --size 0.6 0.6 1.0
ros2 run go_navigation obstacles spawn post -3 1 --cylinder 0.2 --size 0 0 1.2
ros2 run go_navigation obstacles shuttle worker 6 -2 2 --speed 0.3   # moving box, Ctrl-C removes it
ros2 run go_navigation obstacles list
ros2 run go_navigation obstacles remove crate
ros2 run go_navigation obstacles clear                 # removes every obs_* model
```

- All names get an `obs_` prefix, so `clear` never touches the factory itself.
- Make obstacles **taller than ~0.4 m**; anything below the lidar plane is invisible (try `--size 0.6 0.6 0.3` to see that limit).
- `shuttle` speed is in wall-clock m/s; with Gazebo at 0.5–0.8× real time the box is effectively faster relative to the robot.

## Key Nav2 settings (`config/nav2_params.yaml`)

Every change from the stock Jazzy file is marked `# Go1:`.

| Area | Setting | Value | Why |
|---|---|---|---|
| AMCL | `robot_model_type` | `OmniMotionModel` | the policy can strafe |
| AMCL | `set_initial_pose` | true, (0, 0, 0) | starts localised at the spawn |
| Frames | `base_frame_id` / `robot_base_frame` | `base_link` | no `base_footprint` |
| Costmaps | `robot_radius` / `inflation_radius` | 0.35 / 0.6 m | trunk ~0.65 m + leg swing |
| Costmaps | `min_obstacle_height` | 0.1 m | ignore floor hits when the trunk pitches |
| Local costmap | size | 6 × 6 m | see obstacles earlier |
| Goal checker | `xy` / `yaw_goal_tolerance` | 0.25 m / 0.3 rad | a legged robot cannot stop precisely |
| MPPI | `motion_model` | `Omni` | holonomic |
| MPPI | `batch_size` | 1000 | 2000 only reached 8–13 Hz next to Gazebo |
| MPPI | `ax/ay_max`, `az_max` | ±5.0, 8.0 | see below |

### Why the MPPI accelerations are so high

Measured in Gazebo, the policy does not move at all for small commands:

| Axis | No motion at | First motion |
|---|---|---|
| vx | ≤ 0.10 m/s | 0.15 m/s → 0.04 m/s actual |
| vy | ≤ 0.20 m/s | — |
| wz | ≤ 0.20 rad/s | 0.30 rad/s → 0.09 rad/s actual |

MPPI rolls every trajectory out from the **measured** speed and limits each 0.05 s step to `a_max × model_dt`. With the stock 1.0 m/s², a standing robot could only be asked for 0.05 m/s, the policy ignored that, the measured speed stayed 0, and Nav2 aborted with *Failed to make progress*. With large MPPI accelerations one step can reach the policy's working range. The real acceleration is still limited by the velocity smoother, which runs open-loop on the commanded speed. **Do not lower `ax_max` / `az_max` to make the robot gentler; lower `velocity_smoother.max_accel` instead.**

## Changing the speed

The robot's real speed limit is the **smaller** of two places, so change them together:

| Setting | MPPI (`controller_server.FollowPath`) | Velocity smoother |
|---|---|---|
| Forward | `vx_max` | `max_velocity[0]` |
| Reverse | `vx_min` | `min_velocity[0]` |
| Sideways | `vy_max` | `max_velocity[1]` / `min_velocity[1]` |
| Turning | `wz_max` | `max_velocity[2]` / `min_velocity[2]` |
| Acceleration | keep high (see above) | `max_accel`, `max_decel` |

Current values: MPPI 0.7 / −0.3 / 0.6 / 0.8, smoother 0.4 / −0.2 / 0.3 / 0.6, so the robot is effectively limited to **0.4 m/s forward, 0.3 m/s sideways, 0.6 rad/s turning**. When raising speeds:

- Keep MPPI and the smoother equal; a larger MPPI limit only makes it plan for speeds the robot cannot reach.
- Keep `prune_distance` above `vx_max × 2.8 s` (MPPI's horizon: 56 steps × 0.05 s).
- Stay inside the training range (±1.5, ±0.8, ±1.2), which `policy_node` also clips to.
- The policy under-tracks (0.25 m/s commanded gives ~0.15 m/s), so expect real speeds below the limits.

Restart `navigation.launch.py` after editing the YAML; no rebuild is needed with `--symlink-install`.
