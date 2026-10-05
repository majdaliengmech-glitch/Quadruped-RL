#!/usr/bin/env python3
"""Phase 5: drop NEW obstacles into the running Gazebo world (they are not on the saved map).
Nav2 has to see them with the lidar and avoid them live: the local costmap (voxel_layer) and the
global costmap's obstacle_layer mark them, the planner replans around them (1 Hz), MPPI steers
around them, and the collision monitor slows the robot if it still gets too close.

usage: ros2 run go_navigation obstacles spawn NAME X Y [--size SX SY SZ] [--cylinder R] [--yaw DEG]
       ros2 run go_navigation obstacles scenario aisle|bay|block   (preset sets, see SCENARIOS)
       ros2 run go_navigation obstacles shuttle NAME X Y1 Y2 [--speed V]  (moving box, Ctrl-C stops)
       ros2 run go_navigation obstacles remove NAME [NAME ...]
       ros2 run go_navigation obstacles clear                     (remove every obs_* model)
       ros2 run go_navigation obstacles list
All names get the prefix 'obs_' so 'clear' never touches the factory itself. Every obstacle is at
least 0.8 m tall: the lidar plane is ~0.39 m above the floor, anything lower is invisible to Nav2."""
import argparse
import math
import sys
import time

from go_navigation import gz_util

PREFIX = "obs_"

# name: [(x, y, kind, size)]  kind 'box' size=(sx, sy, sz) | 'cyl' size=(r, h). Coordinates are
# world = map frame. Each one sits ON a leg of the config/patrol_factory.yaml route (checked with
# A* on maps/factory.pgm), away from the waypoints themselves, so the planned path must change.
SCENARIOS = {
    # crates in the main aisle on legs 4->1, 1->2 and 2->3: sidesteps of ~0.5-1 m
    "aisle": [(5.5, 1.8, "box", (0.8, 0.8, 0.9)),
              (9.0, -1.4, "box", (0.6, 1.0, 0.9)),
              (-5.0, -2.5, "cyl", (0.35, 1.0))],
    # pallet stacks where the route enters / leaves the north bay (legs 4-5, ~+1.6 m each)
    "bay": [(-1.0, 4.9, "box", (1.2, 1.0, 0.9)),
            (2.0, 3.5, "box", (0.8, 0.8, 0.9))],
    # a wall across the whole aisle at x=10: the planner must detour around its ends (~+5 m)
    "block": [(10.0, 0.0, "box", (0.4, 5.0, 1.0))],
}


def sdf(name, kind, size, rgb=(0.85, 0.15, 0.1)):
    if kind == "cyl":
        r, h = size
        geom, z = f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder>", h / 2
    else:
        sx, sy, sz = size
        geom, z = f"<box><size>{sx} {sy} {sz}</size></box>", sz / 2
    col = " ".join(str(c) for c in rgb)
    return (f"<sdf version='1.8'><model name='{name}'><static>true</static>"
            f"<link name='l'><pose>0 0 {z} 0 0 0</pose>"
            f"<collision name='c'><geometry>{geom}</geometry></collision>"
            f"<visual name='v'><geometry>{geom}</geometry><material><ambient>{col} 1</ambient>"
            f"<diffuse>{col} 1</diffuse></material></visual></link></model></sdf>")


def full(name):
    return name if name.startswith(PREFIX) else PREFIX + name


def spawn(name, x, y, kind="box", size=(0.6, 0.6, 1.0), yaw_deg=0.0):
    name = full(name)
    ok = gz_util.spawn_sdf(name, sdf(name, kind, size), x, y, 0.0, math.radians(yaw_deg))
    print(f"{'spawned' if ok else 'FAILED to spawn'} {name} ({kind} {size}) at ({x}, {y})")
    return ok


def remove(names):
    for n in names:
        n = full(n)
        print(f"{'removed' if gz_util.remove(n) else 'FAILED to remove'} {n}")


def ours():
    return [m for m in gz_util.models() if m.startswith(PREFIX)]


def shuttle(name, x, y1, y2, speed):
    """Box that walks back and forth between (x, y1) and (x, y2): a crossing 'worker'."""
    name = full(name)
    if name not in ours():
        spawn(name, x, y1, "box", (0.5, 0.5, 1.6))
    span, t0 = abs(y2 - y1), time.time()
    print(f"{name} shuttling between y={y1} and y={y2} at {speed} m/s (wall clock); Ctrl-C to stop")
    try:
        while True:
            s = (time.time() - t0) * speed % (2 * span)                  # triangle wave
            y = y1 + math.copysign(min(s, 2 * span - s), y2 - y1)
            gz_util.set_pose(name, x, y, 0.0)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        remove([name])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("spawn")
    s.add_argument("name"); s.add_argument("x", type=float); s.add_argument("y", type=float)
    s.add_argument("--size", type=float, nargs=3, default=[0.6, 0.6, 1.0], metavar=("SX", "SY", "SZ"))
    s.add_argument("--cylinder", type=float, metavar="R", help="cylinder of radius R (height = SZ)")
    s.add_argument("--yaw", type=float, default=0.0, help="degrees")
    sc = sub.add_parser("scenario"); sc.add_argument("which", choices=sorted(SCENARIOS))
    sh = sub.add_parser("shuttle")
    sh.add_argument("name"); sh.add_argument("x", type=float)
    sh.add_argument("y1", type=float); sh.add_argument("y2", type=float)
    sh.add_argument("--speed", type=float, default=0.3)
    r = sub.add_parser("remove"); r.add_argument("names", nargs="+")
    sub.add_parser("clear"); sub.add_parser("list")
    a = ap.parse_args([x for x in sys.argv[1:] if x != "--ros-args"])

    try:
        if a.cmd == "spawn":
            if a.cylinder:
                ok = spawn(a.name, a.x, a.y, "cyl", (a.cylinder, a.size[2]))
            else:
                ok = spawn(a.name, a.x, a.y, "box", tuple(a.size), a.yaw)
            sys.exit(0 if ok else 1)
        elif a.cmd == "scenario":
            for i, (x, y, kind, size) in enumerate(SCENARIOS[a.which]):
                spawn(f"{a.which}{i}", x, y, kind, size)
        elif a.cmd == "shuttle":
            shuttle(a.name, a.x, a.y1, a.y2, a.speed)
        elif a.cmd == "remove":
            remove(a.names)
        elif a.cmd == "clear":
            names = ours()
            remove(names) if names else print("no obs_* models in the world")
        elif a.cmd == "list":
            print("\n".join(ours()) or "no obs_* models in the world")
    except RuntimeError as e:
        sys.exit(str(e))


if __name__ == "__main__":
    main()
