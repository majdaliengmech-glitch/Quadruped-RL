#!/usr/bin/env python3
"""Phase 3: send ONE Nav2 goal, follow it, and report time and ground-truth error at arrival.

usage: ros2 run go_navigation goto X Y [YAW_DEG]
       e.g. goto 6 0        (straight down the aisle)
            goto 0 7 90     (around the column into the north bay)
Run after bringup.launch.py + navigation.launch.py. Ctrl-C cancels the goal."""
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from nav2_simple_commander.robot_navigator import TaskResult

from go_navigation.nav_util import GroundTruth, make_pose, sim_navigator, wait_for_nav2


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(args) not in (2, 3):
        sys.exit(__doc__)
    x, y = float(args[0]), float(args[1])
    yaw = float(args[2]) if len(args) == 3 else 0.0

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl-C must still cancel the goal
    nav = sim_navigator("goto")
    truth = GroundTruth(nav)
    wait_for_nav2(nav)

    goal = make_pose(nav, x, y, yaw)
    path = nav.getPath(make_pose(nav, 0, 0, 0), goal, use_start=False)   # plan from current pose
    if path is None or not path.poses:
        nav.error(f"no path to ({x}, {y}): goal in an obstacle or outside the map?")
        sys.exit(1)
    length = sum(((a.pose.position.x - b.pose.position.x) ** 2 +
                  (a.pose.position.y - b.pose.position.y) ** 2) ** 0.5
                 for a, b in zip(path.poses, path.poses[1:]))
    nav.info(f"planned {length:.1f} m to ({x}, {y}, {yaw:.0f} deg) through {len(path.poses)} poses")

    t0, wall0 = nav.get_clock().now(), time.time()
    if not nav.goToPose(goal):
        sys.exit(1)
    fell = False
    try:
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb:
                nav.get_logger().info(f"{fb.distance_remaining:.2f} m to go, "
                                      f"{fb.number_of_recoveries} recoveries", throttle_duration_sec=3.0)
            if truth.fallen():
                fell = True
                nav.error("robot FELL - cancelling (ros2 run go_navigation reset_robot to stand it up)")
                nav.cancelTask()
                break
    except KeyboardInterrupt:
        nav.cancelTask()
        nav.warn("cancelled by user")
        return

    dt = (nav.get_clock().now() - t0).nanoseconds * 1e-9
    result = "FELL" if fell else nav.getResult().name
    err = truth.error_to(x, y)
    err_s = "n/a" if err is None else f"{err:.2f} m"
    rtf = dt / max(time.time() - wall0, 1e-6)
    nav.info(f"result {result}: {dt:.1f} s sim time (real-time factor {rtf:.2f}), "
             f"ground-truth error {err_s}, planned length {length:.1f} m")
    rclpy.shutdown()
    sys.exit(0 if result == TaskResult.SUCCEEDED.name else 1)


if __name__ == "__main__":
    main()
