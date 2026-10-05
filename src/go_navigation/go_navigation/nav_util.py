"""Helpers shared by goto, patrol and obstacles."""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# trunk tilt beyond this, or trunk lower than FALL_Z, counts as a fall (standing: z ~0.27-0.30)
FALL_TILT_DEG, FALL_Z = 60.0, 0.12


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def sim_navigator(name):
    """BasicNavigator on Gazebo's /clock, so goal times are sim seconds without extra flags."""
    nav = BasicNavigator(name)
    nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    return nav


def sim_sleep(node, seconds):
    end = node.get_clock().now() + Duration(seconds=seconds)
    while rclpy.ok() and node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def make_pose(node, x, y, yaw_deg, frame="map"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    yaw = math.radians(yaw_deg)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class GroundTruth:
    """Latest Gazebo ground-truth pose from /odom (odom frame = world frame for a spawn at the origin).
    Used to measure real goal errors and to spot falls; Nav2 itself never sees it except as odometry."""

    def __init__(self, node, topic="/odom"):
        self.msg = None
        node.create_subscription(Odometry, topic, self._cb, 10)

    def _cb(self, msg):
        self.msg = msg

    def pose(self):
        """(x, y, yaw_rad) or None before the first message."""
        if self.msg is None:
            return None
        p = self.msg.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def fallen(self):
        if self.msg is None:
            return False
        p = self.msg.pose.pose
        q = p.orientation
        # z axis of the trunk expressed in the world: cos(tilt) = R[2,2]
        cos_tilt = 1 - 2 * (q.x * q.x + q.y * q.y)
        return cos_tilt < math.cos(math.radians(FALL_TILT_DEG)) or p.position.z < FALL_Z

    def error_to(self, x, y):
        pose = self.pose()
        return None if pose is None else math.hypot(pose[0] - x, pose[1] - y)


def wait_for_nav2(nav, timeout_sec=None):
    """Like BasicNavigator.waitUntilNav2Active, but never publishes /initialpose.

    The stock helper sends a (0, 0, 0) initial pose *before* it checks for /amcl_pose, which
    snaps AMCL back to the spawn point whenever a script is started after the robot has moved.
    AMCL already sets the spawn pose itself (set_initial_pose in nav2_params.yaml)."""
    nav._waitForNodeToActivate("amcl")
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    sub = nav.create_subscription(PoseWithCovarianceStamped, "amcl_pose", got.append, qos)
    start = nav.get_clock().now()
    while not got:
        nav.info("Waiting for AMCL to publish a pose (set one with '2D Pose Estimate' if needed)")
        rclpy.spin_once(nav, timeout_sec=1.0)
        if timeout_sec and (nav.get_clock().now() - start).nanoseconds * 1e-9 > timeout_sec:
            break
    nav.destroy_subscription(sub)
    nav._waitForNodeToActivate("bt_navigator")
    nav.info("Nav2 is ready")
    return bool(got)
