# Results

All navigation tests ran in the factory world on ROS 2 Jazzy + Gazebo Harmonic, with the velocity limits at 0.4 m/s forward, 0.3 m/s sideways and 0.6 rad/s turning. Times are **simulation** seconds; Gazebo ran at 0.5–0.8× real time. Errors are **ground truth** (Gazebo pose vs. target), not AMCL's estimate.

## Single goals (`goto`)

| Goal | Route | Result | Time | Ground-truth error |
|---|---|---|---|---|
| (6, 0) | straight down the aisle, 6.0 m | succeeded | 17.2 s | 0.32 m |
| (0, 7, 90°) from (6, 0) | ~180° turn, around a column into the north bay, 9.0 m | succeeded, 0 recoveries | 31.8 s | 0.11 m |

Average speed on the straight goal: ~0.35 m/s, against a 0.4 m/s limit.

## Patrol (`patrol`, factory route, ~66 m per lap)

![Patrol accuracy per waypoint over three laps](images/patrol_waypoint_error.png)

| Lap | Lap time | WP1 (12, 0) | WP2 (0, −7.5) | WP3 (−13, 0.5) | WP4 (0, 7) | Falls |
|---|---|---|---|---|---|---|
| 1 | 222.8 s | 0.379 m | 0.293 m | 0.176 m | 0.155 m | 0 |
| 2* | 214.7 s | 0.302 m | 0.264 m | 0.084 m | 0.211 m | 0 |
| 3 | 216.0 s | 0.339 m | 0.257 m | 0.053 m | 0.205 m | 0 |

\* Lap 2 ran with three obstacles spawned live on the route (see below). Laps 1–2 and lap 3 are separate runs.

**12/12 waypoints reached, 0 falls, mean lap 217.8 s, mean error 0.23 m.**

Errors above Nav2's 0.25 m tolerance are expected: Nav2 decides "reached" from AMCL's estimate, so the ground-truth error also contains localisation and map error. It is largest at WP1, near the east wall, where the SLAM map is ~0.4 m off.

## Live obstacles

| Test | What happened |
|---|---|
| 3 obstacles spawned during patrol lap 2 | The two on the robot's path were avoided; the third (an earlier placement, since moved onto the route) was never approached. The cylinder sat on the original path (−4.7, −2.7); the robot detoured south and passed 0.69 m from its surface. Closest pass to a crate: 0.57 m from its surface. Lap 2 was no slower than lap 1. |
| 5 m wall across the whole aisle at x = 10, spawned after the plan was made | The initial 14.5 m plan went through the wall. The robot detected it with the lidar, replanned around its south end (passing at y = −3.2, wall ends at −2.5), and reached the goal: 117.7 s, 0.20 m error, 20.3 m walked. |
| Box shuttling across the aisle at 0.3 m/s | Goal reached, but the box **touched the robot once**: the robot tried to pass the end of the box's track just as the box turned back. Nav2's costmaps have no motion prediction. |

## Policy response to small commands

Measured in Gazebo by publishing a constant command for 7 s and averaging the second half of the odometry:

| Command | Measured |
|---|---|
| vx 0.05 / 0.10 / 0.15 / 0.25 m/s | 0.000 / 0.000 / 0.043 / 0.148 m/s |
| vy 0.10 / 0.20 m/s | 0.000 / 0.000 m/s |
| wz 0.10 / 0.20 / 0.30 / 0.50 rad/s | 0.000 / 0.000 / 0.091 / 0.319 rad/s |

This deadzone is why the stock Nav2 MPPI settings could not start the robot; see [navigation.md](navigation.md#why-the-mppi-accelerations-are-so-high).

## Policy evaluation in MuJoCo

See [training.md](training.md#evaluation-in-mujoco) for the foot-height and velocity-tracking plots from Colab and how to read them.

## Ideas for further experiments

The original guide suggests logging the same patrol metrics under:

- changed floor friction (`<mu>` in the world),
- added latency between `policy_node` and `pd_controller`,
- different training seeds,
- leg odometry + IMU fused with `robot_localization` instead of ground-truth odometry.
