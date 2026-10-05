#!/usr/bin/env python3
"""Runs the Brax/Playground PPO policy (NumPy) at 50 Hz and publishes joint position targets.
Obs layout (48, inferred from the saved normalization stats + Playground Go1 joystick env):
 [0:3] linvel (body frame)  [3:6] gyro  [6:9] gravity (body)  [9:21] q-default_q
 [21:33] dq  [33:45] last_action  [45:48] command (vx, vy, wz)
Verify this against your env's _get_obs before trusting it."""
import os
import numpy as np, rclpy, yaml
from ament_index_python.packages import get_package_share_directory as share
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def quat_to_rot(q):  # (x, y, z, w) -> R (body -> world)
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def required_file(node, param):
    """Return the path in a file parameter, failing loudly if it does not exist."""
    path = node.get_parameter(param).value
    if not os.path.isfile(path):
        raise FileNotFoundError(f"{param}: '{path}' not found (rebuild, or pass -p {param}:=<path>)")
    node.get_logger().info(f"{param}: {path}")
    return path


def silu(x):
    return x / (1.0 + np.exp(-x))


class PolicyNode(Node):
    def __init__(self):
        super().__init__("policy_node")
        self.declare_parameter(
            "params_file", os.path.join(share("go_description"), "config", "robot_params.yaml"))
        self.declare_parameter(
            "weights_file", os.path.join(share("policy_node"), "config", "policy_weights.npz"))
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("odom_topic", "/odom")
        # twist_mux never sends a zero when its inputs go quiet, so without this the robot keeps
        # walking on the last command (e.g. after a keyboard tap or a crashed Nav2). 0 = hold forever.
        self.declare_parameter("cmd_timeout", 1.0)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.cmd_time = None
        p = yaml.safe_load(open(required_file(self, "params_file")))
        self.joints = p["joint_names"]
        self.default_q = np.array(p["default_q"], float)
        self.scale = float(p["action_scale"])
        w = np.load(required_file(self, "weights_file"))
        self.mean, self.std = w["obs_mean"], w["obs_std"]
        self.W = [w[f"W{i}"] for i in range(4)]
        self.b = [w[f"b{i}"] for i in range(4)]
        # env_config: command_config a = [1.5, 0.8, 1.2] -> training range is +-a
        self.cmd_hi = np.array([1.5, 0.8, 1.2]); self.cmd_lo = -self.cmd_hi
        self.q = self.default_q.copy(); self.dq = np.zeros(12)
        self.gyro = np.zeros(3); self.quat = np.array([0, 0, 0, 1.0])
        self.linvel = np.zeros(3); self.cmd = np.zeros(3); self.last_action = np.zeros(12)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 1)
        self.create_subscription(Imu, self.get_parameter("imu_topic").value, self.on_imu, 1)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 1)
        self.create_subscription(Twist, "/cmd_vel_mux", self.on_cmd, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/policy/q_target", 10)
        self.create_timer(0.02, self.step)

    def on_joints(self, m):
        idx = [m.name.index(n) for n in self.joints]
        self.q = np.array(m.position)[idx]; self.dq = np.array(m.velocity)[idx]

    def on_imu(self, m):
        o = m.orientation; g = m.angular_velocity
        self.quat = np.array([o.x, o.y, o.z, o.w]); self.gyro = np.array([g.x, g.y, g.z])

    def on_odom(self, m):  # twist is usually in the child (body) frame; check yours
        v = m.twist.twist.linear
        self.linvel = np.array([v.x, v.y, v.z])

    def on_cmd(self, m):
        self.cmd = np.clip([m.linear.x, m.linear.y, m.angular.z], self.cmd_lo, self.cmd_hi)
        self.cmd_time = self.get_clock().now()

    def policy(self, obs):
        h = (obs - self.mean) / (self.std + 1e-8)
        for i in range(3):
            h = silu(h @ self.W[i] + self.b[i])
        out = h @ self.W[3] + self.b[3]       # 24 = mean(12) + std params(12)
        return np.tanh(out[:12])               # deterministic action

    def step(self):
        if self.cmd_timeout > 0 and self.cmd_time is not None and \
                (self.get_clock().now() - self.cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.cmd, self.cmd_time = np.zeros(3), None      # stale command: stand still
        g_body = quat_to_rot(self.quat).T @ np.array([0, 0, -1.0])
        obs = np.concatenate([self.linvel, self.gyro, g_body, self.q - self.default_q,
                              self.dq, self.last_action, self.cmd])
        a = self.policy(obs)
        self.last_action = a
        self.pub.publish(Float64MultiArray(data=(self.default_q + self.scale * a).tolist()))


def main():
    rclpy.init(); rclpy.spin(PolicyNode()); rclpy.shutdown()


if __name__ == "__main__":
    main()
