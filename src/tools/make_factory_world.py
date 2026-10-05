#!/usr/bin/env python3
"""Generate go_gazebo/worlds/factory.sdf: a 40 x 24 m factory hall for mapping / Nav2 tests.

Layout (x: -20..20, y: -12..12, robot spawns at the origin in the main aisle):
  main aisle  y in [-2, 2] along the whole hall, yellow edge lines, steel columns either side
  north-west  pallet racks (3 rows)            north-east  CNC machines + fenced robot cell
  south-west  two conveyor lines               south-east  shipping: crates, barrels, forklift, office

Every obstacle is solid from the floor to at least 0.6 m, so the 2D lidar (~0.39 m above the
floor) sees it. Floor markings are visual only (no collision).
usage: python3 make_factory_world.py [out.sdf]
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "go_gazebo", "worlds", "factory.sdf")

# name: (rgb, metalness, roughness)  -> PBR materials rendered by ogre2
MAT = {
    "concrete":  ((0.42, 0.43, 0.45), 0.0, 0.85),
    "epoxy":     ((0.22, 0.36, 0.30), 0.0, 0.35),   # green work-zone floor coating
    "wall":      ((0.78, 0.80, 0.82), 0.0, 0.70),
    "wall_band": ((0.12, 0.30, 0.55), 0.0, 0.60),
    "shutter":   ((0.55, 0.58, 0.62), 0.8, 0.40),
    "steel":     ((0.60, 0.62, 0.65), 0.9, 0.30),
    "column":    ((0.95, 0.55, 0.10), 0.3, 0.50),   # safety orange
    "yellow":    ((0.98, 0.80, 0.05), 0.0, 0.50),
    "black":     ((0.05, 0.05, 0.05), 0.0, 0.60),
    "white":     ((0.95, 0.95, 0.95), 0.0, 0.60),
    "red":       ((0.80, 0.08, 0.08), 0.2, 0.45),
    "blue":      ((0.10, 0.30, 0.75), 0.4, 0.40),
    "rack_up":   ((0.10, 0.22, 0.55), 0.6, 0.40),   # rack uprights
    "rack_beam": ((0.95, 0.45, 0.05), 0.5, 0.40),   # rack beams
    "pallet":    ((0.55, 0.38, 0.20), 0.0, 0.90),
    "cardboard": ((0.70, 0.52, 0.32), 0.0, 0.95),
    "machine":   ((0.82, 0.84, 0.86), 0.5, 0.35),
    "machine_d": ((0.20, 0.22, 0.25), 0.6, 0.35),
    "glass":     ((0.35, 0.55, 0.65), 0.1, 0.10),
    "belt":      ((0.08, 0.08, 0.08), 0.0, 0.80),
    "green":     ((0.10, 0.60, 0.25), 0.0, 0.50),
}


def material(name):
    (r, g, b), metal, rough = MAT[name]
    return (f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            f"<specular>0.3 0.3 0.3 1</specular>"
            f"<pbr><metal><metalness>{metal}</metalness><roughness>{rough}</roughness></metal></pbr>"
            f"</material>")


class Model:
    """A static model made of many links (one per box/cylinder)."""

    def __init__(self, name):
        self.name, self.links = name, []

    def _link(self, geom, pose, mat, collide):
        i = len(self.links)
        x, y, z, yaw = pose
        col = f"<collision name='c'><geometry>{geom}</geometry></collision>" if collide else ""
        self.links.append(
            f"<link name='l{i}'><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>{col}"
            f"<visual name='v'><geometry>{geom}</geometry>{material(mat)}"
            f"<cast_shadows>{'true' if collide else 'false'}</cast_shadows></visual></link>")

    def box(self, x, y, z, sx, sy, sz, mat, yaw=0.0, collide=True):
        """Box whose BOTTOM sits at height z."""
        self._link(f"<box><size>{sx} {sy} {sz}</size></box>", (x, y, z + sz / 2, yaw), mat, collide)

    def cyl(self, x, y, z, r, h, mat, collide=True):
        self._link(f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder>", (x, y, z + h / 2, 0), mat, collide)

    def sdf(self):
        return f"<model name='{self.name}'><static>true</static>{''.join(self.links)}</model>"


def mark(m, x, y, sx, sy, mat, yaw=0.0):
    """Painted floor marking (visual only)."""
    m.box(x, y, 0.0, sx, sy, 0.004, mat, yaw, collide=False)


# ---------------------------------------------------------------- building
hall = Model("hall")
L, W, H, T = 40.0, 24.0, 4.0, 0.3
for x, y, sx, sy in [(0, W / 2, L + T, T), (0, -W / 2, L + T, T), (-L / 2, 0, T, W), ]:
    hall.box(x, y, 0, sx, sy, H, "wall")
    hall.box(x, y, 1.0, sx + 0.02 if sx > sy else sx + 0.02, sy + 0.02 if sy > sx else sy + 0.02, 0.4, "wall_band", collide=False)
# east wall with a closed loading shutter (y -3..1)
for y0, y1 in [(-W / 2, -3.0), (1.0, W / 2)]:
    hall.box(L / 2, (y0 + y1) / 2, 0, T, y1 - y0, H, "wall")
hall.box(L / 2 - 0.05, -1.0, 0, 0.2, 4.0, 3.2, "shutter")
hall.box(L / 2, -1.0, 3.2, T, 4.0, H - 3.2, "wall")
for y in (-3.1, 1.1):  # hazard posts beside the shutter
    hall.box(L / 2 - 0.3, y, 0, 0.25, 0.25, 1.2, "yellow")
# steel columns along the main aisle
for x in (-16, -8, 0, 8, 16):
    for y in (-2.7, 2.7):
        hall.box(x, y, 0, 0.4, 0.4, H, "column")
        hall.box(x, y, 0, 0.5, 0.5, 0.15, "black")  # base plate

# ---------------------------------------------------------------- floor + markings
floor = Model("floor_marks")
mark(floor, 0, 0, L, 4.0, "concrete")                     # main aisle (slightly different shade)
for y in (-2.05, 2.05):
    mark(floor, 0, y, L - 1, 0.12, "yellow")             # aisle edge lines
for x in range(-18, 19, 3):
    mark(floor, x, 0, 1.2, 0.12, "white")                # centre dashes
mark(floor, 10, 7, 15, 7.5, "epoxy")                      # machine zone
mark(floor, -10.5, -7, 16, 8, "epoxy")                    # conveyor zone
for i in range(10):                                       # hazard stripes at the robot cell
    mark(floor, 13.2 + i * 0.5, 3.4, 0.2, 0.9, "yellow", yaw=0.6)

# ---------------------------------------------------------------- north-west: pallet racks
racks = Model("pallet_racks")
for ry in (4.2, 7.4, 10.6):
    for seg in range(3):
        x0 = -17.5 + seg * 5.0
        x1 = x0 + 4.4
        cx, length = (x0 + x1) / 2, x1 - x0
        racks.box(cx, ry, 0, length, 1.0, 0.65, "pallet")            # floor-level pallets (lidar sees these)
        for k, z in enumerate((0.65, 1.6, 2.6)):                      # stored goods, staggered
            for b in range(4):
                if (b + k + seg) % 3 != 2:
                    racks.box(x0 + 0.6 + b * 1.05, ry, z + 0.1, 0.9, 0.85, 0.75, "cardboard")
        for z in (1.5, 2.5, 3.5):                                     # orange beams
            for dy in (-0.48, 0.48):
                racks.box(cx, ry + dy, z, length, 0.08, 0.1, "rack_beam")
        for x in (x0, x0 + length / 2, x1):                           # blue uprights
            for dy in (-0.48, 0.48):
                racks.box(x, ry + dy, 0, 0.1, 0.1, 3.8, "rack_up")

# ---------------------------------------------------------------- north-east: machines
mach = Model("machines")
for i, (x, y) in enumerate([(4.5, 5.0), (8.5, 5.0), (4.5, 9.5), (8.5, 9.5)]):
    mach.box(x, y, 0, 2.4, 1.8, 2.0, "machine")                       # CNC body
    mach.box(x, y - 0.91, 0.9, 1.2, 0.02, 0.7, "glass", collide=False)  # door window
    mach.box(x, y, 2.0, 2.4, 1.8, 0.12, "machine_d")                  # roof trim
    mach.box(x + 1.3, y - 0.5, 0, 0.35, 0.5, 1.6, "machine_d")        # control cabinet
    mach.box(x + 1.3, y - 0.76, 1.1, 0.25, 0.02, 0.3, "green" if i % 2 else "red", collide=False)  # screen
    mach.cyl(x - 1.0, y + 1.0, 2.12, 0.06, 0.5, "steel")              # signal tower
    mach.cyl(x - 1.0, y + 1.0, 2.62, 0.07, 0.12, "green", collide=False)
    mach.cyl(x - 1.0, y + 1.0, 2.74, 0.07, 0.12, "yellow", collide=False)
    mach.cyl(x - 1.0, y + 1.0, 2.86, 0.07, 0.12, "red", collide=False)
# fenced robot cell: solid lower panels (lidar-visible) + yellow posts
cx0, cx1, cy0, cy1 = 13.0, 18.5, 4.0, 10.5
for (xa, ya, xb, yb) in [(cx0, cy0, cx1, cy0), (cx0, cy1, cx1, cy1), (cx1, cy0, cx1, cy1), (cx0, cy0 + 1.5, cx0, cy1)]:
    length = math.hypot(xb - xa, yb - ya)
    yaw = math.atan2(yb - ya, xb - xa)
    mach.box((xa + xb) / 2, (ya + yb) / 2, 0, length, 0.06, 1.0, "machine_d", yaw)
    mach.box((xa + xb) / 2, (ya + yb) / 2, 1.0, length, 0.03, 1.0, "yellow", yaw, collide=False)
    n = int(length // 1.5)
    for j in range(n + 1):
        t = j / max(n, 1)
        mach.box(xa + (xb - xa) * t, ya + (yb - ya) * t, 0, 0.1, 0.1, 2.0, "yellow")
mach.cyl(15.75, 7.25, 0, 0.35, 0.8, "machine_d")                     # robot arm base
mach.box(15.75, 7.25, 0.8, 0.25, 0.25, 1.0, "red")                   # arm link
mach.box(16.3, 7.25, 1.7, 1.1, 0.2, 0.2, "red")
mach.box(14.5, 8.8, 0, 1.2, 0.8, 0.9, "steel")                       # work table

# ---------------------------------------------------------------- south-west: conveyors
conv = Model("conveyors")
for cy in (-5.0, -8.5):
    conv.box(-10.5, cy, 0, 13.0, 0.9, 0.75, "machine_d")             # solid frame/skirt
    conv.box(-10.5, cy, 0.75, 13.0, 0.8, 0.06, "belt")                # belt
    for dy in (-0.45, 0.45):
        conv.box(-10.5, cy + dy, 0.75, 13.0, 0.06, 0.15, "steel")     # side rails
    for k in range(7):                                                # parcels on the belt
        conv.box(-16 + k * 1.8, cy, 0.81, 0.5, 0.45, 0.35, "cardboard")
    conv.box(-3.6, cy, 0, 0.8, 1.4, 1.6, "blue")                      # drive/motor housing
    conv.box(-17.3, cy, 0, 0.6, 1.2, 1.9, "machine")                  # infeed scanner arch
conv.box(-10.5, -6.75, 0, 0.6, 0.6, 1.3, "red")                       # emergency stop pedestal
conv.box(-10.5, -6.75, 1.3, 0.25, 0.25, 0.08, "yellow", collide=False)

# ---------------------------------------------------------------- south-east: shipping
ship = Model("shipping")
for i, (x, y) in enumerate([(4, -5), (5.4, -5), (4, -6.4), (8, -9), (9.4, -9), (6.5, -10.2)]):
    ship.box(x, y, 0, 1.2, 1.0, 0.15, "pallet")
    for lvl in range(1 + i % 3):
        ship.box(x, y, 0.15 + lvl * 0.6, 1.1, 0.9, 0.58, "cardboard")
for i, (x, y) in enumerate([(11, -4.2), (11.7, -4.2), (11, -4.9), (11.7, -4.9), (12.4, -4.55)]):
    ship.cyl(x, y, 0, 0.29, 0.9, "blue" if i % 2 else "red")          # barrels
# forklift (parked)
ship.box(7.5, -4.5, 0, 2.0, 1.1, 0.9, "yellow")
ship.box(7.1, -4.5, 0.9, 1.0, 1.0, 1.1, "black")
ship.box(8.7, -4.5, 0, 0.1, 1.0, 2.2, "machine_d")
for dy in (-0.3, 0.3):
    ship.box(9.3, -4.5 + dy, 0, 1.1, 0.12, 0.06, "steel")
# office cabin in the corner
ship.box(16.5, -9.0, 0, 6.0, 5.4, 2.8, "white")
ship.box(16.5, -6.29, 0.9, 4.0, 0.02, 1.2, "glass", collide=False)
ship.box(13.49, -9.5, 0, 0.02, 1.0, 2.1, "blue", collide=False)      # door
ship.box(16.5, -9.0, 2.8, 6.2, 5.6, 0.12, "machine_d")

# ---------------------------------------------------------------- lights
lamps = []
for x in (-15, -5, 5, 15):
    for y in (-7, 0, 7):
        lamps.append(
            f"<light type='point' name='lamp_{x}_{y}'><pose>{x} {y} 3.8 0 0 0</pose>"
            f"<diffuse>0.55 0.55 0.5 1</diffuse><specular>0.2 0.2 0.2 1</specular>"
            f"<attenuation><range>14</range><constant>0.3</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation>"
            f"<cast_shadows>false</cast_shadows></light>")
        hall.box(x, y, 3.85, 1.2, 0.3, 0.08, "white", collide=False)  # lamp fixture (visual)

world = f"""<?xml version="1.0"?>
<!-- GENERATED by src/tools/make_factory_world.py - edit the script, not this file -->
<sdf version="1.8">
  <world name="factory">
    <physics name="p" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.45 0.45 0.48 1</ambient>
      <background>0.62 0.72 0.82 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <pose>0 0 20 0 0 0</pose><direction>-0.4 0.25 -0.9</direction>
      <diffuse>0.75 0.73 0.68 1</diffuse><specular>0.3 0.3 0.3 1</specular>
      <cast_shadows>true</cast_shadows>
    </light>
    {''.join(lamps)}
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 40</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 40</size></plane></geometry>
        {material("concrete")}</visual>
    </link></model>
    {hall.sdf()}
    {floor.sdf()}
    {racks.sdf()}
    {mach.sdf()}
    {conv.sdf()}
    {ship.sdf()}
  </world>
</sdf>
"""
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
with open(OUT, "w") as f:
    f.write(world)
n = sum(len(m.links) for m in (hall, floor, racks, mach, conv, ship))
print(f"wrote {os.path.abspath(OUT)}: {n} links, {len(lamps)} lamps")
