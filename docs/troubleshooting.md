# Troubleshooting

## Leftover processes

Suspending a launch with Ctrl‑Z, or interrupting one badly, leaves nodes alive. Symptoms: duplicate controllers fighting over the robot, GBs of RAM in use, `RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed`, or a `gz sim -s` server running with no launch. Clean everything from this project:

```bash
pkill -9 -f "[r]os2 (launch|run)"; pkill -9 -f "[g]z sim"; pkill -9 -f "[o]pt/ros/jazzy/lib"; \
pkill -9 -f "[Q]uadruped-RL/install"; pkill -9 -f "[r]viz2"; ros2 daemon stop; \
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

The brackets stop `pkill` from matching (and killing) its own shell. Check with
`ps -eo pid,stat,cmd | grep -E "[g]z sim|[o]pt/ros/jazzy|[Q]uadruped-RL/install"`; a `T` in the STAT column means suspended.

## Robot and walking

| Symptom | Likely cause | Try |
|---|---|---|
| Robot collapses | torques not reaching joints, wrong joint order | `ros2 topic echo /effort_controller/commands`; compare `controllers.yaml` joints with `robot_params.yaml` |
| Legs vibrate | Kd too low for the loop, or a node on wall time | `use_sim_time` true everywhere; raise Kd in `robot_standing_params.yaml` |
| Falls at landing | spawn height or default pose mismatch | spawn lower (`-z` in `sim.launch.py`); check `default_q` |
| Mesh missing | resource path | read the Gazebo warnings; `ros2 pkg prefix go_description` |
| Keeps walking after you stop the keyboard | `cmd_timeout` set to 0 | leave the default 1.0 s |
| Keyboard tap moves it only briefly | `cmd_timeout` 1.0 s | hold the key |
| Small commands do nothing | policy deadzone (see [results](results.md#policy-response-to-small-commands)) | command ≥ 0.2 m/s / 0.3 rad/s |
| Fallen | | `ros2 run go_navigation reset_robot` |

## Navigation

| Symptom | Likely cause | Try |
|---|---|---|
| `Failed to make progress`, robot never starts | MPPI accelerations lowered back towards 1.0 | keep `ax_max` 5, `az_max` 8 ([why](navigation.md#why-the-mppi-accelerations-are-so-high)) |
| Goal plans but nothing reaches the policy | wrong topic chain | `ros2 topic info /cmd_vel -v` must show `collision_monitor` publishing and `twist_mux` subscribing |
| Robot jumps back to the spawn in RViz when a script starts | a script published `/initialpose` | use `goto` / `patrol` from this repo (they never do); set the pose again with "2D Pose Estimate" |
| `Control loop missed its desired rate` | CPU load | `gui:=false`, `rviz:=false`, lower MPPI `batch_size` |
| Stops short / oscillates near the goal | policy deadzone near zero speed | loosen `xy_goal_tolerance` / `yaw_goal_tolerance` |
| New obstacle ignored | below the lidar plane, or out of range | make it ≥ 0.4 m tall; local costmap marks up to 3 m, global up to 2.5 m |
| Robot "sees" obstacles that are not there | floor hits when the trunk pitches | keep `min_obstacle_height` 0.1 |
| Nav2 waits for TF forever | `use_sim_time` missing | every node and launch on sim time |
| `obstacles` / `reset_robot`: no `/world/<name>/create` service | Gazebo not running, or a different `GZ_PARTITION` | start the sim first, same shell environment |

## Python

| Symptom | Fix |
|---|---|
| `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x` | a training venv is on `PYTHONPATH`; open a clean shell or `env -u PYTHONPATH ...` |
| `export_policy.py` cannot unpickle | run it in the training venv with JAX / Brax installed |

## Layer-by-layer checks

| Layer | Command |
|---|---|
| Build | `ros2 pkg prefix go_description` |
| Controllers | `ros2 control list_controllers`, `ros2 control list_hardware_interfaces` |
| Bridge | `ros2 topic info /imu -v`; `gz topic -l` |
| Time | `ros2 topic echo /clock --once` |
| TF | `ros2 run tf2_tools view_frames` |
| Nav2 | `ros2 lifecycle get /controller_server` |
