import asyncio
from typing import Any, Dict, Optional, TypeVar, Tuple, cast

import geometry_msgs
import rclpy
import rclpy.node
import rclpy.publisher

from natnet_py.client import NatNetClient
from natnet_py.protocol import MoCapData
from optitrack_ros_py.tf import TF

T = TypeVar("T")


class NatNetROSNode(rclpy.node.Node):  # type: ignore

    def try_to_declare_parameter(self, name: str, value: T) -> T:
        if not self.has_parameter(name):
            return cast(T, self.declare_parameter(name, value).value)
        return cast(T, self.get_parameter(name).value)

    def __init__(self) -> None:
        super().__init__("natnet_ros",
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.clock = self.get_clock()
        client_address = self.try_to_declare_parameter('client_address',
                                                       "127.0.0.1")
        server_address = self.try_to_declare_parameter('server_address',
                                                       "127.0.0.1")
        use_multicast = self.try_to_declare_parameter('use_multicast', False)
        self.frame_id = self.try_to_declare_parameter('frame_id', "world")
        self.timeout = self.try_to_declare_parameter('timeout', 5.0)
        self._default_topic = self.try_to_declare_parameter(
            'rigid_bodies.default.topic', "/optitrack/*")
        self._default_enabled = self.try_to_declare_parameter(
            'rigid_bodies.default.enabled', True)
        self._default_tf = self.try_to_declare_parameter(
            'rigid_bodies.default.tf', False)

        self._rigid_bodies_topic: Dict[str, str] = {}
        self._rigid_bodies_enabled: Dict[str, bool] = {}
        self._rigid_bodies_tf: Dict[str, bool] = {}
        self._rigid_bodies_frame_id: Dict[str, str] = {}
        self._rigid_bodies_root_frame_id: Dict[str, str] = {}
        self._rigid_bodies_pub: Dict[int,
                                     Optional[rclpy.publisher.Publisher]] = {}
        self._rigid_bodies = set()
        self._tf_frames: Dict[int, Tuple[str, str]] = {}
        self.tf: Optional[TF] = None

        for name in self.get_parameters_by_prefix(""):
            ns = name.split('.')
            if ns[0] == 'rigid_bodies' and len(ns) == 3:
                value = self.get_parameter(name).value
                if ns[2] == 'topic':
                    self._rigid_bodies_topic[ns[1]] = value
                elif ns[2] == 'enabled':
                    self._rigid_bodies_enabled[ns[1]] = value
                elif ns[2] == 'tf':
                    self._rigid_bodies_tf[ns[1]] = value
                elif ns[2] == 'root_frame_id':
                    self._rigid_bodies_root_frame_id[ns[1]] = value
                elif ns[2] == 'frame_id':
                    self._rigid_bodies_frame_id[ns[1]] = value
                self._rigid_bodies.add(ns[1])

        self.client = NatNetClient(client_address=client_address,
                                   server_address=server_address,
                                   use_multicast=use_multicast,
                                   queue=-1,
                                   logger=self.get_logger())
        self.client.data_callback = self.data_callback

    def is_enabled(self, name: str) -> bool:
        return self._rigid_bodies_enabled.get(name, self._default_enabled)

    def should_publish_tf(self, name: str) -> bool:
        return self._rigid_bodies_tf.get(name, self._default_tf)

    def get_topic(self, name: str) -> str:
        value = self._rigid_bodies_topic.get(name, self._default_topic)
        return value.replace("*", name)

    def init_ros(self) -> None:
        uids = {v: k for k, v in self.client.rigid_body_names.items()}
        for name in self._rigid_bodies:
            if name in uids:
                if self.is_enabled(name):
                    self._rigid_bodies_pub[uids[name]] = self.create_publisher(
                        geometry_msgs.msg.PoseStamped, self.get_topic(name), 1)
                if self.should_publish_tf(name):
                    if not self.tf:
                        self.tf = TF(self)
                    self._tf_frames[uids[name]] = (
                        self._rigid_bodies_frame_id.get(name, name),
                        self._rigid_bodies_root_frame_id.get(
                            name, self.frame_id),
                    )

    def get_publisher(self, uid: int) -> Optional[rclpy.publisher.Publisher]:
        if self._default_enabled and uid not in self._rigid_bodies_pub:
            name = self.client.rigid_body_names.get(uid, '')
            topic = self.get_topic(name) if name else ''
            if topic:
                self._rigid_bodies_pub[uid] = self.create_publisher(
                    geometry_msgs.msg.PoseStamped, topic, 1)
            else:
                self._rigid_bodies_pub[uid] = None
        return self._rigid_bodies_pub.get(uid, None)

    def data_callback(self, data: MoCapData) -> None:
        if not data.rigid_body_data or not data.rigid_body_data.rigid_bodies:
            return
        stamp = self.clock.now().to_msg()
        for rb in data.rigid_body_data.rigid_bodies:

            pub = self.get_publisher(rb.id_num)
            frames = self._tf_frames.get(rb.id_num, None)
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
            if frames is not None and self.tf:
                self.tf.broadcast(msg, *frames)

    async def connect(self) -> bool:
        return await self.client.connect_async(self.timeout)


async def spinning(node: NatNetROSNode) -> None:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(node: NatNetROSNode) -> None:
    loop = asyncio.get_event_loop()
    spin = loop.create_task(spinning(node))
    connected = await node.connect()
    if connected:
        node.init_ros()
        await asyncio.wait([spin, node.client.wait_until_lost_connection()])


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = NatNetROSNode()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run(node))
    except KeyboardInterrupt:
        pass
    loop.run_until_complete(node.client.unconnect_async())
    rclpy.try_shutdown()
    node.destroy_node()
