#!/usr/bin/env python3
"""High-rate joint PD loop. Policy node sends position targets at 50 Hz;
this node turns them into torques at 500 Hz, like MuJoCo's position actuators do."""
import os
import numpy as np, rclpy, yaml
from ament_index_python.packages import get_package_share_directory as share
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PDController(Node):
    def __init__(self):
        super().__init__("pd_controller")
        self.declare_parameter(
            "params_file", os.path.join(share("go_description"), "config", "robot_standing_params.yaml"))
        path = self.get_parameter("params_file").value
        if not os.path.isfile(path):
            raise FileNotFoundError(f"params_file: '{path}' not found")
        p = yaml.safe_load(open(path))
        self.names = p["joint_names"]
        self.kp = np.array(p["kp"], float)
        # actuator damping + passive joint damping from MuJoCo, applied once, here
        self.kd = np.array(p["kd_actuator"], float) + np.array(p["joint_damping"], float)
        self.tau_max = np.array(p["torque_limit"], float)
        self.target = np.array(p["default_q"], float)      # hold default pose until policy runs
        self.q = self.dq = None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Float64MultiArray, "/policy/q_target", self.on_target, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/effort_controller/commands", 10)
        self.create_timer(1.0 / 500.0, self.step)           # follows sim time (use_sim_time)

    def on_joints(self, msg):
        idx = [msg.name.index(n) for n in self.names]       # map by NAME, never by position
        self.q = np.array(msg.position)[idx]
        self.dq = np.array(msg.velocity)[idx]

    def on_target(self, msg):
        self.target = np.array(msg.data, float)

    def step(self):
        if self.q is None:
            return
        tau = self.kp * (self.target - self.q) - self.kd * self.dq
        tau = np.clip(tau, -self.tau_max, self.tau_max)
        self.pub.publish(Float64MultiArray(data=tau.tolist()))


def main():
    rclpy.init(); rclpy.spin(PDController()); rclpy.shutdown()


if __name__ == "__main__":
    main()
