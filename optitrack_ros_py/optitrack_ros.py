import asyncio
from typing import Any, Dict, Optional, Tuple, TypeVar, cast

try:
    import diagnostic_msgs
    import diagnostic_updater
    DIAGNOSTICS = True
except ModuleNotFoundError:
    DIAGNOSTICS = False

import geometry_msgs
import netifaces
import optitrack_msgs.msg
import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.time
from natnet_py.client import NatNetClient
from natnet_py.protocol import FrameSuffixData, MoCapData

from optitrack_ros_py.tf import TF

from .interfaces import msg_from_data, msg_from_description

T = TypeVar("T")

PLACE_HOLDER = 'NAME'


class NatNetROSNode(rclpy.node.Node):

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

        client_address = "127.0.0.1"
        broadcast_address = ""
        server_address = ""

        iface = self.try_to_declare_parameter("iface", "")
        if iface:
            addrs = netifaces.ifaddresses(iface)
            if addrs:
                net = addrs[netifaces.AF_INET][0]
                client_address = net["addr"]
                if "broadcast" in net:
                    broadcast_address = net["broadcast"]
                else:
                    server_address = client_address

        client_address = self.try_to_declare_parameter("client_address",
                                                       client_address)
        broadcast_address = self.try_to_declare_parameter(
            "broadcast_address", broadcast_address)
        server_address = self.try_to_declare_parameter("server_address",
                                                       server_address)
        use_multicast = self.try_to_declare_parameter("use_multicast", False)
        self.sync = self.try_to_declare_parameter("sync", True)
        self.restamp = self.try_to_declare_parameter("restamp", False)
        self.frame_id = self.try_to_declare_parameter("frame_id", "world")
        self.timeout = self.try_to_declare_parameter("timeout", 5.0)
        self._default_topic = self.try_to_declare_parameter(
            "rigid_bodies.default.topic", f"/optitrack/{PLACE_HOLDER}")
        self._default_enabled = self.try_to_declare_parameter(
            "rigid_bodies.default.enabled", True)
        self._default_tf = self.try_to_declare_parameter(
            "rigid_bodies.default.tf", False)
        self._default_root_frame_id = self.try_to_declare_parameter(
            "rigid_bodies.default.root_frame_id", PLACE_HOLDER)
        self._default_frame_id = self.try_to_declare_parameter(
            "rigid_bodies.default.frame_id", PLACE_HOLDER)

        self._rigid_bodies_topic: Dict[str, str] = {}
        self._rigid_bodies_enabled: Dict[str, bool] = {}
        self._rigid_bodies_tf: Dict[str, bool] = {}
        self._rigid_bodies_frame_id: Dict[str, str] = {}
        self._rigid_bodies_root_frame_id: Dict[str, str] = {}
        self._rigid_bodies_pub: Dict[int,
                                     Optional[rclpy.publisher.Publisher]] = {}
        self._rigid_bodies = set()
        self._tf_frames: Dict[int, Tuple[str, str]] = {}
        self._tf: Optional[TF] = None

        for name in self.get_parameters_by_prefix(""):
            ns = name.split(".")
            if ns[0] == "rigid_bodies" and len(ns) == 3:
                value = self.get_parameter(name).value
                if ns[2] == "topic":
                    self._rigid_bodies_topic[ns[1]] = value
                elif ns[2] == "enabled":
                    self._rigid_bodies_enabled[ns[1]] = value
                elif ns[2] == "tf":
                    self._rigid_bodies_tf[ns[1]] = value
                elif ns[2] == "root_frame_id":
                    self._rigid_bodies_root_frame_id[ns[1]] = value
                elif ns[2] == "frame_id":
                    self._rigid_bodies_frame_id[ns[1]] = value
                self._rigid_bodies.add(ns[1])

        self.client = NatNetClient(
            client_address=client_address,
            server_address=server_address,
            broadcast_address=broadcast_address,
            use_multicast=use_multicast,
            queue=-1,
            logger=self.get_logger(),  # type: ignore
            now=self.now_ns,
            sync=self.sync,
        )

        self.last_data_stamp: Optional[rclpy.time.Time] = None
        self.last_data: Optional[MoCapData] = None
        self._diagnostics = DIAGNOSTICS and self.try_to_declare_parameter(
            "diagnostics", True)
        self.data_freq: Optional[diagnostic_updater.FrequencyStatus] = None
        self.should_publish_data = self.try_to_declare_parameter(
            "publish_data", False)
        self.should_publish_description = self.try_to_declare_parameter(
            "publish_description", False)

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def stamp(self, data: Optional[FrameSuffixData]) -> rclpy.time.Time:
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
        if self.last_data_stamp is not None and self.last_data is not None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Connected")
            for rb in self.last_data.rigid_bodies:
                name = self.client.rigid_body_names.get(rb.id_num, "")
                stat.add(f"{rb.id_num}",
                         f"{name} ({rb.position}, {rb.orientation})")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                         "Not connected")
        return stat

    def is_enabled(self, name: str) -> bool:
        return self._rigid_bodies_enabled.get(name, self._default_enabled)

    def should_publish_tf(self, name: str) -> bool:
        return self._rigid_bodies_tf.get(name, self._default_tf)

    def get_topic(self, name: str) -> str:
        value = self._rigid_bodies_topic.get(name, self._default_topic)
        return value.replace(PLACE_HOLDER, name)

    def get_frames(self, name: str) -> Tuple[str, str]:
        root = self._rigid_bodies_root_frame_id.get(
            name, self._default_root_frame_id)
        target = self._rigid_bodies_frame_id.get(name, self._default_frame_id)
        return target.replace(PLACE_HOLDER,
                              name), root.replace(PLACE_HOLDER, name)

    @property
    def tf(self) -> TF:
        if self._tf is None:
            self._tf = TF(self)
        return self._tf

    async def init_ros(self) -> None:
        if self._diagnostics:
            fps = await self.client.get_framerate_async()
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
        for name in self._rigid_bodies:
            if name in uids:
                if self.is_enabled(name):
                    self._rigid_bodies_pub[uids[name]] = self.create_publisher(
                        geometry_msgs.msg.PoseStamped, self.get_topic(name), 1)
                if self.should_publish_tf(name):
                    self._tf_frames[uids[name]] = self.get_frames(name)

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

    def get_tf_frames(self, uid: int) -> Optional[Tuple[str, str]]:
        if self._default_enabled and uid not in self._tf_frames:
            name = self.client.rigid_body_names.get(uid, "")
            if name:
                self._tf_frames[uid] = self.get_frames(name)
            else:
                self._tf_frames[uid] = None
        return self._tf_frames.get(uid, None)

    def get_publisher(self, uid: int) -> Optional[rclpy.publisher.Publisher]:
        if self._default_enabled and uid not in self._rigid_bodies_pub:
            name = self.client.rigid_body_names.get(uid, "")
            topic = self.get_topic(name) if name else ""
            if topic:
                self._rigid_bodies_pub[uid] = self.create_publisher(
                    geometry_msgs.msg.PoseStamped, topic, 1)
            else:
                self._rigid_bodies_pub[uid] = None
        return self._rigid_bodies_pub.get(uid, None)

    def data_callback(self, data: MoCapData) -> None:
        self.last_data_stamp = self.stamp(data.suffix_data)
        stamp = self.last_data_stamp.to_msg()
        self.last_data = data
        for rb in data.rigid_bodies:
            pub = self.get_publisher(rb.id_num)
            frames = self.get_tf_frames(rb.id_num)
            # self.get_logger().info(f"rb {rb} {pub} {frames}" )
            if pub or frames is not None:
                msg = geometry_msgs.msg.PoseStamped()
                msg.header.frame_id = self.frame_id
                msg.header.stamp = stamp
                msg.pose.position.x = rb.position[0]
                msg.pose.position.y = -rb.position[2]
                msg.pose.position.z = rb.position[1]
                msg.pose.orientation.x = rb.orientation[0]
                msg.pose.orientation.y = -rb.orientation[2]
                msg.pose.orientation.z = rb.orientation[1]
                msg.pose.orientation.w = rb.orientation[3]
            if pub:
                pub.publish(msg)
            if frames is not None:
                self.tf.broadcast(msg, *frames)
        if self.data_freq:
            self.data_freq.tick()
        if self.should_publish_data:
            data_msg = msg_from_data(data)
            data_msg.header.frame_id = self.frame_id
            data_msg.header.stamp = stamp
            self.data_pub.publish(data_msg)

    async def connect(self) -> bool:
        return await self.client.connect_async(self.timeout,
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
        await node.client.start_listening_for_data_async()
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
    loop.run_until_complete(node.client.unconnect_async())
    rclpy.try_shutdown()  # type: ignore
    node.destroy_node()  # type: ignore
