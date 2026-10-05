import mujoco, numpy as np

m0 = mujoco.MjModel.from_xml_path("/home/majd-ali/Desktop/mujoco_menagerie/unitree_go1/go1.xml")
m1 = mujoco.MjModel.from_xml_path("/home/majd-ali/Desktop/Quadruped-RL/src/go_description/urdf/go1.urdf")
d0, d1 = mujoco.MjData(m0), mujoco.MjData(m1)
q = m0.key_qpos[0]; d0.qpos[:] = q; d1.qpos[:] = q[7:]
mujoco.mj_forward(m0, d0); mujoco.mj_forward(m1, d1)
t0 = d0.body("trunk").xpos
for i in range(2, m0.nbody):
    n = mujoco.mj_id2name(m0, mujoco.mjtObj.mjOBJ_BODY, i)
    print(n, np.abs((d0.body(n).xipos - t0) - d1.body(n).xipos).max()) # expect ~1e-15  