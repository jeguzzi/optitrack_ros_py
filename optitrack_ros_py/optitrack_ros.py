from __future__ import annotations

import asyncio
import dataclasses as dc
import math
import re
from typing import TYPE_CHECKING, Any, TypeVar, cast

try:
    import diagnostic_msgs  # type: ignore[import-untyped]
    import diagnostic_updater  # type: ignore[import-untyped]
    DIAGNOSTICS = True
except ModuleNotFoundError:
    DIAGNOSTICS = False

import geometry_msgs  # type: ignore[import-untyped]
import netifaces
import optitrack_msgs.msg  # type: ignore[import-untyped]
import PyKDL  # type: ignore[import-not-found]
import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.time
from natnet_py import AsyncClient
from rclpy.validate_topic_name import (InvalidTopicNameException,
                                       validate_topic_name)

from optitrack_ros_py.tf import TF

from .interfaces import msg_from_data, msg_from_description

if TYPE_CHECKING:
    from natnet_py.protocol import FrameSuffixData, MoCapData, RigidBodyData

T = TypeVar("T")


@dc.dataclass
class RigidBodyConfig:
    enabled: bool = False
    name: str = ''
    tf: bool = False
    topic: str = ''
    frame_id: str = ''
    root_frame_id: str = ''
    project_to_2d: bool = False

    def replace_name(self, name: str, place_holder: str) -> None:
        self.name = self.name.replace(place_holder, name)
        self.topic = self.topic.replace(place_holder, self.name)
        self.frame_id = self.frame_id.replace(place_holder, self.name)
        self.root_frame_id = self.root_frame_id.replace(
            place_holder, self.name)


@dc.dataclass
class OptionalRigidBodyConfig:
    enabled: bool | None = None
    name: str | None = None
    tf: bool | None = None
    topic: str | None = None
    frame_id: str | None = None
    root_frame_id: str | None = None
    project_to_2d: bool | None = None

    def or_default(self, default: RigidBodyConfig) -> RigidBodyConfig:
        values = dc.asdict(self)
        default_values = dc.asdict(default)
        return RigidBodyConfig(
            **{
                k: v if v is not None else default_values[k]
                for k, v in values.items()
            })


@dc.dataclass
class RigidBody:
    config: RigidBodyConfig
    publisher: rclpy.publisher.Publisher | None = None
    last_pose_msg: geometry_msgs.msg.PoseStamped | None = None

    @property
    def tf_frames(self) -> tuple[str, str] | None:
        if self.config.tf and self.config.frame_id and self.config.root_frame_id:
            return (self.config.frame_id, self.config.root_frame_id)
        return None

    @property
    def enabled(self) -> bool:
        return self.config.enabled

    @property
    def name(self) -> str:
        return self.config.name


class NatNetROSNode(rclpy.node.Node):

    PLACE_HOLDER = 'NAME'

    def try_to_declare_parameter(self, name: str, value: T) -> T:
        if not self.has_parameter(name):
            return cast(T, self.declare_parameter(name, value).value)
        return cast(T, self.get_parameter(name).value)

    def __init__(self) -> None:
        super().__init__(
            "natnet_ros",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.clock = self.get_clock()
        self.updating_description = False

        client_address = "0.0.0.0"
        broadcast_address = "255.255.255.255"
        server_address = ""

        iface = self.try_to_declare_parameter("iface", "")
        if iface:
            try:
                addrs = netifaces.ifaddresses(iface)
            except ValueError:
                addrs = {}
            if addrs:
                if netifaces.AF_INET not in addrs or not addrs[
                        netifaces.AF_INET]:
                    self.get_logger().error(f"Interface {iface} not connected")
                else:
                    net = addrs[netifaces.AF_INET][0]
                    print(net)
                    client_address = net["addr"]
                    if "broadcast" in net:
                        broadcast_address = net["broadcast"]
                    else:
                        server_address = client_address
                        broadcast_address = ""
            else:
                self.get_logger().error(f"Unknown interface {iface}")

        client_address = self.try_to_declare_parameter("client_address",
                                                       client_address)
        self.broadcast_address = self.try_to_declare_parameter(
            "broadcast_address", broadcast_address)
        self.server_address = self.try_to_declare_parameter(
            "server_address", server_address)
        # use_multicast = self.try_to_declare_parameter("use_multicast", False)
        self.sync = self.try_to_declare_parameter("sync", True)
        self.restamp = self.try_to_declare_parameter("restamp", False)
        self.optitrack_frame_id = "optitrack"
        self.frame_id = self.try_to_declare_parameter("frame_id", "world")
        self.timeout = self.try_to_declare_parameter("timeout", 5.0)
        self._tf: TF | None = None
        pose = geometry_msgs.msg.Pose()
        pose.orientation.x = math.sqrt(2)
        pose.orientation.w = math.sqrt(2)
        self.tf.broadcast_static(pose, 'optitrack', self.frame_id)
        self._rigid_bodies: dict[int, RigidBody] = {}
        self._default_config = RigidBodyConfig(
            enabled=self.try_to_declare_parameter(
                "rigid_bodies.default.enabled", True),
            name=self.try_to_declare_parameter("rigid_bodies.default.name",
                                               self.PLACE_HOLDER),
            tf=self.try_to_declare_parameter("rigid_bodies.default.tf", False),
            topic=self.try_to_declare_parameter(
                "rigid_bodies.default.topic",
                f"/optitrack/{self.PLACE_HOLDER}"),
            root_frame_id=self.try_to_declare_parameter(
                "rigid_bodies.default.root_frame_id", self.PLACE_HOLDER),
            frame_id=self.try_to_declare_parameter(
                "rigid_bodies.default.frame_id", self.PLACE_HOLDER),
            project_to_2d=self.try_to_declare_parameter(
                "rigid_bodies.default.project_to_2d", False))
        self._config: dict[str, OptionalRigidBodyConfig] = {}
        for name in self.get_parameters_by_prefix(""):
            ns = name.split(".")
            if ns[0] == "rigid_bodies" and len(ns) == 3:
                value = self.get_parameter(name).value
                name = ns[1]
                if name == "default":
                    continue
                if name not in self._config:
                    self._config[name] = OptionalRigidBodyConfig()
                config = self._config[name]
                if ns[2] == "topic":
                    config.topic = value
                elif ns[2] == "name":
                    config.name = value
                elif ns[2] == "enabled":
                    config.enabled = value
                elif ns[2] == "tf":
                    config.tf = value
                elif ns[2] == "root_frame_id":
                    config.root_frame_id = value
                elif ns[2] == "frame_id":
                    config.frame_id = value
                elif ns[2] == "project_to_2d":
                    config.project_to_2d = value

        self.client = AsyncClient(
            address=client_address,
            queue=-1,
            logger=self.get_logger(),  # type: ignore
            now=self.now_ns,
            sync=self.sync,
        )

        self.last_data_stamp: rclpy.time.Time | None = None
        self.last_data: MoCapData | None = None
        self._diagnostics = DIAGNOSTICS and self.try_to_declare_parameter(
            "diagnostics", True)
        self.data_freq: diagnostic_updater.FrequencyStatus | None = None
        self.should_publish_data = self.try_to_declare_parameter(
            "publish_data", False)
        self.should_publish_description = self.try_to_declare_parameter(
            "publish_description", False)

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def stamp(self, data: FrameSuffixData | None) -> rclpy.time.Time:
        if self.restamp or not data:
            return self.get_clock().now()
        if self.sync and self.client.clock:
            ns = self.client.clock.server_ticks_to_client_time(
                data.stamp_camera_mid_exposure)
        else:
            ns = data.stamp_camera_mid_exposure
        return rclpy.time.Time(nanoseconds=ns)

    def update_diagnostics(
        self, stat: diagnostic_updater.DiagnosticStatusWrapper
    ) -> diagnostic_updater.DiagnosticStatusWrapper:
        # if self.last_data_stamp is not None and self.last_data is not None:
        #     stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Connected")
        #     for data in self.last_data.rigid_bodies:
        #         name = self.client.rigid_body_names.get(data.id_num, "")
        #         stat.add(f"{data.id_num}",
        #                  f"{name} ({data.position}, {data.orientation})")
        if self._rigid_bodies:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Connected")
            for _uid, rb in self._rigid_bodies.items():
                if not rb.last_pose_msg:
                    msg = 'not yet seen'
                else:
                    last_time = rclpy.time.Time.from_msg(
                        rb.last_pose_msg.header.stamp)
                    dt = (self.get_clock().now() - last_time).nanoseconds
                    if dt < 1e9:
                        last_seen = 'just seen'
                    else:
                        last_seen = f'last seen {dt / 1e9: %d} seconds ago'
                    x = rb.last_pose_msg.pose.position
                    position = f'(x={x.x:.3f}, y={x.y:.3f}, z={x.z:.3f})'
                    q = rb.last_pose_msg.pose.orientation
                    y, p, r = PyKDL.Rotation.Quaternion(q.x, q.y, q.z,
                                                        q.w).GetEulerZYX()
                    orientation = (f'(yaw={math.degrees(y):.1f}, '
                                   f'pitch={math.degrees(p):.1f}, '
                                   f'roll={math.degrees(r):.1f})')
                    msg = f'{last_seen} at {position}, {orientation}'
                stat.add(f"{rb.name}", msg)
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                         "Not connected")
        return stat

    def get_rigid_body_config(self, name: str) -> RigidBodyConfig:
        opt_config = OptionalRigidBodyConfig()
        for pattern, candidate in self._config.items():
            if re.match(pattern, name):
                opt_config = candidate
                break
        # opt_config = self._config.get(name, OptionalRigidBodyConfig())
        config = opt_config.or_default(self._default_config)
        config.replace_name(name, self.PLACE_HOLDER)
        self.get_logger().info(f"Use {config} for {name}")
        return config

    @property
    def tf(self) -> TF:
        if self._tf is None:
            self._tf = TF(self)
        return self._tf

    def init_rigid_body(self, uid: int, config: RigidBodyConfig) -> None:
        rb = self._rigid_bodies[uid] = RigidBody(config)
        if config.enabled:
            try:
                validate_topic_name(config.topic)
                rb.publisher = self.create_publisher(
                    geometry_msgs.msg.PoseStamped, config.topic, 1)
            except InvalidTopicNameException:
                self.get_logger().warning(f"Topic {config.topic} not valid")

    async def update_description(self):
        self.updating_description = True
        self.get_logger().info("Updating client description")
        await self.client.update_description()
        self.updating_description = False

    def get_rigid_body(self, uid: int) -> RigidBody | None:
        if uid not in self._rigid_bodies:
            name = self.client.rigid_body_names.get(uid, "")
            if not name and not self.updating_description and self.executor:
                self.executor.create_task(self.update_description())
                return None
            config = self.get_rigid_body_config(name)
            self.init_rigid_body(uid, config)
        return self._rigid_bodies[uid]

    async def init_ros(self) -> None:
        if self._diagnostics:
            fps = await self.client.get_framerate()
            self.updater = diagnostic_updater.Updater(self, period=1.0)
            self.updater.setHardwareID("Optitrack")
            self.updater.add(self.get_namespace(), self.update_diagnostics)
            params = diagnostic_updater.FrequencyStatusParam(
                {
                    "min": fps,
                    "max": fps
                }, tolerance=0.1, window_size=10)
            self.data_freq = diagnostic_updater.FrequencyStatus(params,
                                                                name="Data")
            self.updater.add(self.data_freq)
        uids = {v: k for k, v in self.client.rigid_body_names.items()}
        for name in self._config:
            if name in uids:
                config = self.get_rigid_body_config(name)
                self.init_rigid_body(uids[name], config)

        if self.should_publish_description:
            qos = rclpy.qos.QoSProfile(  # type: ignore
                depth=1,
                history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.description_pub = self.create_publisher(
                optitrack_msgs.msg.Description, "description", qos)

            if self.client.description:
                msg = msg_from_description(self.client.description)
                self.description_pub.publish(msg)
        if self.should_publish_data:
            self.data_pub = self.create_publisher(optitrack_msgs.msg.Data,
                                                  "data", 10)
        self.client.data_callback = self.data_callback

    def pose_msg(self, rb: RigidBody,
                 data: RigidBodyData) -> geometry_msgs.msg.Pose:
        msg = geometry_msgs.msg.Pose()
        msg.position.x = data.position[0]
        msg.position.y = -data.position[2]
        q = (data.orientation[0], -data.orientation[2], data.orientation[1],
             data.orientation[3])
        if rb.config.project_to_2d:
            msg.position.z = 0
            y, p, r = PyKDL.Rotation.Quaternion(*q).GetEulerZYX()
            q = PyKDL.Rotation.EulerZYX(y, 0, 0).GetQuaternion()
        else:
            msg.position.z = data.position[1]
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        return msg

    def data_callback(self, stamp_ns: int, data: MoCapData) -> None:
        self.last_data_stamp = self.stamp(data.suffix_data)
        stamp = self.last_data_stamp.to_msg()
        self.last_data = data
        transforms: list[tuple[geometry_msgs.msg.PoseStamped, str, str]] = []
        for rb_data in data.rigid_bodies:
            if not rb_data.tracking_valid:
                continue
            rb = self.get_rigid_body(rb_data.id)
            if not rb or not rb.enabled:
                continue
            rb.last_pose_msg = geometry_msgs.msg.PoseStamped()
            rb.last_pose_msg.header.frame_id = self.frame_id
            rb.last_pose_msg.header.stamp = stamp
            rb.last_pose_msg.pose = self.pose_msg(rb, rb_data)
            frames = rb.tf_frames
            pub = rb.publisher
            if pub:
                pub.publish(rb.last_pose_msg)
            if frames is not None:
                transforms.append((rb.last_pose_msg, *frames))
        if self.data_freq:
            self.data_freq.tick()
        if self.should_publish_data:
            data_msg = msg_from_data(data)
            data_msg.header.frame_id = self.optitrack_frame_id
            data_msg.header.stamp = stamp
            self.data_pub.publish(data_msg)
        if transforms:
            self.tf.broadcast_multiple(transforms)

    async def connect(self) -> bool:
        return await self.client.connect(
            discovery_address=self.broadcast_address,
            server_address=self.server_address,
            timeout=self.timeout,
            start_listening_for_data=False)


async def spinning(node: NatNetROSNode) -> None:
    while rclpy.ok():  # type: ignore
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(node: NatNetROSNode) -> None:
    loop = asyncio.get_event_loop()
    spin = loop.create_task(spinning(node))
    connected = await node.connect()
    if connected:
        await asyncio.sleep(1)
        await node.init_ros()
        await node.client.start_listening_for_data()
        await asyncio.wait([
            spin,
            asyncio.create_task(node.client.wait_until_lost_connection())
        ])


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = NatNetROSNode()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run(node))
    except KeyboardInterrupt:
        pass
    loop.run_until_complete(node.client.unconnect())
    rclpy.try_shutdown()  # type: ignore
    node.destroy_node()  # type: ignore
