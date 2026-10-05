#!/usr/bin/env python3
"""Phase 4: patrol a loop of waypoints with Nav2's waypoint follower, lap after lap.

Run after bringup.launch.py + navigation.launch.py. AMCL already knows the spawn pose
(set_initial_pose in nav2_params.yaml), so this script never publishes /initialpose.
usage: ros2 run go_navigation patrol [--ros-args -p laps:=3 -p waypoints_file:=<yaml>
                                                 -p auto_reset:=false -p log_dir:=<dir>]

Logs one CSV row per waypoint for the report (ground-truth error at arrival, lap time, falls):
~/.ros/patrol_logs/patrol_<date>.csv. On a fall it cancels the lap; with auto_reset (default)
it stands the robot up where it fell (simulation only) and retries the waypoint it was heading to.
Hold the joystick enable button to take over; release it and the patrol carries on."""
import csv
import math
import os
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml
from ament_index_python.packages import get_package_share_directory as share

from go_navigation.nav_util import GroundTruth, make_pose, sim_navigator, sim_sleep, wait_for_nav2
from go_navigation.reset_robot import teleport


def follow(nav, truth, wps, start, on_arrival):
    """Follow wps[start:] once. Calls on_arrival(index) as each waypoint is passed.
    Returns ('done', missed_indices) or ('fell', index_it_was_heading_to)."""
    nav.followWaypoints([make_pose(nav, *w) for w in wps[start:]])
    passed = 0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb and fb.current_waypoint > passed:
            for i in range(passed, fb.current_waypoint):
                on_arrival(start + i)
            passed = fb.current_waypoint
            nav.get_logger().info(f"heading to waypoint {start + passed + 1}/{len(wps)}")
        if truth.fallen():
            nav.cancelTask()
            return "fell", start + passed
    for i in range(passed, len(wps) - start):
        on_arrival(start + i)
    res = nav.result_future.result().result if nav.result_future else None
    missed = {start + m.index for m in res.missed_waypoints} if res else set()
    return "done", missed


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl-C must still cancel the goal
    nav = sim_navigator("patrol")
    nav.declare_parameter("waypoints_file",
                          os.path.join(share("go_navigation"), "config", "patrol_factory.yaml"))
    nav.declare_parameter("laps", 0)                 # 0 = until Ctrl-C
    nav.declare_parameter("auto_reset", True)
    nav.declare_parameter("log_dir", os.path.expanduser("~/.ros/patrol_logs"))
    get = lambda n: nav.get_parameter(n).value  # noqa: E731
    wps = yaml.safe_load(open(get("waypoints_file")))["waypoints"]
    laps, auto_reset = get("laps"), get("auto_reset")

    os.makedirs(get("log_dir"), exist_ok=True)
    log_path = os.path.join(get("log_dir"), time.strftime("patrol_%Y%m%d_%H%M%S.csv"))
    log = open(log_path, "w", newline="")
    out = csv.writer(log)
    out.writerow(["lap", "waypoint", "x", "y", "reached", "error_m", "lap_time_s", "falls_so_far"])

    truth = GroundTruth(nav)
    wait_for_nav2(nav)
    nav.info(f"patrolling {len(wps)} waypoints from {get('waypoints_file')}, "
             f"{'forever' if laps == 0 else f'{laps} laps'}; log: {log_path}")

    lap, falls, totals = 0, 0, []
    try:
        while rclpy.ok() and (laps == 0 or lap < laps):
            lap += 1
            t0, start, errors, missed = nav.get_clock().now(), 0, {}, set()

            def arrived(i):
                errors.setdefault(i, truth.error_to(*wps[i][:2]))

            while start < len(wps):
                status, info = follow(nav, truth, wps, start, arrived)
                if status == "done":
                    missed = info
                    break
                falls += 1
                nav.error(f"lap {lap}: FELL on the way to waypoint {info + 1} (fall #{falls})")
                if not auto_reset:
                    raise KeyboardInterrupt
                x, y, yaw = truth.pose()
                teleport(x, y, yaw)
                sim_sleep(nav, 4.0)                 # land and settle in the standing pose
                start = info                        # retry the waypoint it was heading to
            dt = (nav.get_clock().now() - t0).nanoseconds * 1e-9
            for i, w in enumerate(wps):
                err = errors.get(i)
                out.writerow([lap, i + 1, w[0], w[1], i not in missed,
                              "" if err is None else f"{err:.3f}", f"{dt:.1f}", falls])
            log.flush()
            reached = [e for i, e in errors.items() if i not in missed and e is not None]
            mean = sum(reached) / len(reached) if reached else math.nan
            totals.append(dt)
            nav.info(f"lap {lap} done in {dt:.1f} s: {len(wps) - len(missed)}/{len(wps)} waypoints, "
                     f"mean ground-truth error {mean:.2f} m, falls so far {falls}")
    except KeyboardInterrupt:
        nav.cancelTask()
    finally:
        log.close()
    if totals:
        nav.info(f"{len(totals)} laps, mean lap {sum(totals) / len(totals):.1f} s, "
                 f"{falls} falls; log: {log_path}")
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
