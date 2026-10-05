"""Thin wrappers around Gazebo (Harmonic) world services through gz-transport's Python bindings.
Simulation-only helpers: teleport the robot, spawn / move / remove obstacle models."""
import math

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node

TIMEOUT_MS = 3000
_node = None


def node():
    global _node
    if _node is None:
        _node = Node()
    return _node


def world_name():
    """Name of the running world, found from its /world/<name>/create service."""
    for s in node().service_list():
        if s.startswith("/world/") and s.endswith("/create"):
            return s.split("/")[2]
    raise RuntimeError("no /world/<name>/create service: is Gazebo running "
                       "(and GZ_PARTITION the same as in the sim terminal)?")


def _call(service, req, req_type, rep_type=Boolean):
    ok, rep = node().request(f"/world/{world_name()}/{service}", req, req_type, rep_type, TIMEOUT_MS)
    if not ok:
        raise RuntimeError(f"gz service {service} timed out")
    return rep


def _pose(msg, x, y, z, yaw):
    msg.position.x, msg.position.y, msg.position.z = x, y, z
    msg.orientation.z, msg.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)


def set_pose(name, x, y, z, yaw=0.0):
    req = Pose()
    req.name = name
    _pose(req, x, y, z, yaw)
    return _call("set_pose", req, Pose).data


def spawn_sdf(name, sdf, x, y, z=0.0, yaw=0.0):
    req = EntityFactory()
    req.sdf, req.name, req.allow_renaming = sdf, name, False
    _pose(req.pose, x, y, z, yaw)
    return _call("create", req, EntityFactory).data


def remove(name):
    req = Entity()
    req.name, req.type = name, Entity.MODEL
    return _call("remove", req, Entity).data


def models():
    return [m.name for m in _call("scene/info", Empty(), Empty, Scene).model]
