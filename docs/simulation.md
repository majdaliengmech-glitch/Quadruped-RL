# Simulation and teleoperation

## Start the robot

```bash
ros2 launch go_navigation bringup.launch.py                    # factory world, with Gazebo GUI
ros2 launch go_navigation bringup.launch.py gui:=false         # headless (sensors still render)
ros2 launch go_navigation bringup.launch.py world:=patrol_room.sdf
ros2 launch go_navigation bringup.launch.py teleop:=false      # no joystick / twist_mux
```

The robot spawns at the origin 0.45 m above the floor, `pd_controller` holds the default pose so it lands standing, and `policy_node` starts immediately (a zero command means "stand").

### Check that everything is alive

```bash
ros2 control list_controllers      # joint_state_broadcaster, effort_controller: active
ros2 topic hz /joint_states        # ~500 Hz
ros2 topic hz /imu                 # ~200 Hz
ros2 topic hz /scan                # ~10 Hz
ros2 topic echo /odom --once       # z ≈ 0.28 when standing
ros2 run tf2_tools view_frames     # odom → base_link → legs, imu_link, lidar_link
```

## Worlds

| World | Size | Contents |
|---|---|---|
| `factory.sdf` (default) | 40 × 24 m | main aisle along y = 0 with steel columns, pallet racks (NW), CNC machines and a fenced robot cell (NE), two conveyor lines (SW), shipping crates, barrels, forklift and office (SE) |
| `patrol_room.sdf` | 12 × 12 m | walled room with two boxes and a pillar |

`factory.sdf` is generated: edit `src/tools/make_factory_world.py` and run `python3 src/tools/make_factory_world.py`. Every obstacle in it reaches at least 0.6 m so the lidar (~0.39 m above the floor) sees it.

## Keyboard

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_key
```

`cmd_vel_key` has priority 90 in `twist_mux` (above Nav2). **Hold** the keys: `policy_node` drops the command to zero after 1 s without messages. For the old behaviour (last key holds forever) start the policy with `-p cmd_timeout:=0.0`.

## Gamepad

Plug in an Xbox-style pad; `bringup.launch.py` already runs `joy_node`, `teleop_twist_joy` and `twist_mux`.

| Control | Default (`config/joy.yaml`) |
|---|---|
| Deadman / enable | button 4 (LB) — the robot only follows the sticks while it is held |
| Forward / back | left stick vertical (axis 1), scale 0.6 m/s |
| Strafe | left stick horizontal (axis 0), scale 0.4 m/s |
| Turn | right stick horizontal (axis 3), scale 0.8 rad/s |

Find your pad's button and axis numbers with `ros2 topic echo /joy`. The joystick has priority 100: holding LB overrides Nav2 and the keyboard; releasing it hands control back (after 0.5 s).

## Command priorities (`config/twist_mux.yaml`)

| Source | Topic | Priority | Timeout |
|---|---|---|---|
| Joystick | `cmd_vel_joy` | 100 | 0.5 s |
| Keyboard | `cmd_vel_key` | 90 | 1.0 s |
| Nav2 | `cmd_vel` | 10 | 0.5 s |

## Picking a fallen robot up

```bash
ros2 run go_navigation reset_robot            # stand it up where it fell (keeps x, y, heading)
ros2 run go_navigation reset_robot --origin   # back to the spawn point
```

This teleports the model upright through Gazebo's `set_pose` service; simulation only.

## Stopping

Stop launches with **Ctrl‑C**. Ctrl‑Z only suspends them: the nodes keep their memory and DDS ports, and the next run gets duplicate controllers and `open_and_lock_file failed` errors. See [troubleshooting](troubleshooting.md#leftover-processes) for a clean-up command.
