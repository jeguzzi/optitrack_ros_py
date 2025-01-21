from __future__ import annotations

from collections.abc import Iterable

import PyKDL
import rclpy.duration
import rclpy.node
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped


def transform_from_msg(msg: TransformStamped) -> PyKDL.Frame:
    position_msg = msg.translation
    pos = PyKDL.Vector(position_msg.x, position_msg.y, position_msg.z)
    quaterion_msg = msg.rotation
    rot = PyKDL.Rotation.Quaternion(quaterion_msg.x, quaterion_msg.y,
                                    quaterion_msg.z, quaterion_msg.w)
    return PyKDL.Frame(V=pos, R=rot)


def transform_msg_from_pose_msg(pose_msg: PoseStamped,
                                target_link: str) -> TransformStamped:
    msg = TransformStamped()
    msg.header = pose_msg.header
    msg.child_frame_id = target_link
    msg.transform.translation = pose_msg.pose.position
    msg.transform.rotation = pose_msg.pose.orientation
    return msg


def transform_from_pose_msg(msg: PoseStamped) -> PyKDL.Frame:
    q = msg.pose.orientation
    p = msg.pose.position
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w),
                       PyKDL.Vector(p.x, p.y, p.z))


def transform_to_msg(parent_link: str, child_link: str,
                     transform: PyKDL.Frame) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = msg.header.stamp
    msg.header.frame_id = parent_link
    msg.child_frame_id = child_link
    q = transform.M.GetQuaternion()
    msg.transform.translation.x = transform.p.x()
    msg.transform.translation.y = transform.p.y()
    msg.transform.translation.z = transform.p.z()
    msg.transform.rotation.w = q[3]
    msg.transform.rotation.x = q[0]
    msg.transform.rotation.y = q[1]
    msg.transform.rotation.z = q[2]
    return msg


class TF:

    def __init__(self, node: rclpy.node.Node) -> None:
        self.clock = node.get_clock()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(buffer=self.buffer,
                                                  node=node)
        self.broadcaster = tf2_ros.TransformBroadcaster(node)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(node)

    def get_transform(self,
                      from_frame: str,
                      to_frame: str,
                      at: rclpy.time.Time | None = None,
                      timeout: float = 0.0) -> PyKDL.Frame | None:
        if from_frame == to_frame:
            return PyKDL.Frame()
        if at is None:
            at = rclpy.time.Time()
        try:
            transform_msg = self.buffer.lookup_transform(
                to_frame,
                from_frame,
                at,
                timeout=rclpy.duration.Duration(nanoseconds=timeout *
                                                1e9))  # type: ignore
            return transform_from_msg(transform_msg.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def transform_msg(self, pose_msg: PoseStamped, target_link: str,
                      root_link: str) -> TransformStamped | None:
        if pose_msg.header.frame_id == root_link:
            return transform_msg_from_pose_msg(pose_msg, target_link)
        else:
            t2 = self.get_transform(root_link, target_link)
            if t2:
                t = transform_from_pose_msg(pose_msg) * t2
                msg = transform_to_msg(pose_msg.header.frame_id, root_link, t)
                msg.header.stamp = pose_msg.header.stamp
                return msg
        return None

    def broadcast(self, pose_msg: PoseStamped, target_link: str,
                  root_link: str) -> None:
        msg = self.transform_msg(pose_msg, target_link, root_link)
        if msg:
            self.broadcaster.sendTransform(msg)

    def broadcast_multiple(
            self, transforms: Iterable[tuple[PoseStamped, str, str]]) -> None:
        msgs = filter(lambda x: x is not None,
                      (self.transform_msg(pose, target, root)
                       for pose, target, root in transforms))
        self.broadcaster.sendTransform(msgs)

    def broadcast_static(self, pose: Pose, target_link: str,
                         root_link: str) -> None:
        msg = TransformStamped()
        msg.header.frame_id = root_link
        msg.header.stamp = self.clock.now().to_msg()
        msg.child_frame_id = target_link
        msg.transform.translation = pose.position
        msg.transform.rotation = pose.orientation
        self.static_broadcaster.sendTransform(msg)
