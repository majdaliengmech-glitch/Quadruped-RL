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
                flavor="harmonic", default_effort=35.0, max_velocity=30.0,
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
