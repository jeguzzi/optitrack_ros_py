from __future__ import annotations

import geometry_msgs.msg
import optitrack_msgs.msg
from natnet_py import protocol


def msg_from_vector(value: protocol.Vector3) -> geometry_msgs.msg.Vector3:
    return geometry_msgs.msg.Vector3(x=value[0], y=value[1], z=value[2])


def msg_from_quaternion(
        value: protocol.Quaternion) -> geometry_msgs.msg.Quaternion:
    return geometry_msgs.msg.Quaternion(x=value[0],
                                        y=value[1],
                                        z=value[2],
                                        w=value[3])


def msg_from_camera_description(
    value: protocol.CameraDescription,
) -> optitrack_msgs.msg.CameraDescription:
    msg = optitrack_msgs.msg.CameraDescription()
    msg.name = value.name
    msg.position = msg_from_vector(value.position)
    msg.orientation = msg_from_quaternion(value.orientation)
    return msg


def msg_from_device_description(
    value: protocol.DeviceDescription,
) -> optitrack_msgs.msg.DeviceDescription:
    msg = optitrack_msgs.msg.DeviceDescription()
    msg.name = value.name
    msg.id = value.id
    msg.serial_number = value.serial_number
    msg.type = value.device_type
    msg.data_type = value.channel_data_type
    msg.channels = value.channels
    return msg


def msg_from_force_plate_description(
    value: protocol.ForcePlateDescription,
) -> optitrack_msgs.msg.ForcePlateDescription:
    msg = optitrack_msgs.msg.ForcePlateDescription()
    msg.id = value.id
    msg.serial_number = value.serial_number
    msg.width = value.width
    msg.length = value.length
    msg.plate_type = value.plate_type
    msg.position = msg_from_vector(value.position)
    msg.data_type = value.channel_data_type
    msg.channels = value.channels
    msg.cal_matrix = value.cal_matrix
    msg.corners = value.corners
    return msg


def msg_from_marker_set_description(
    value: protocol.MarkerSetDescription,
) -> optitrack_msgs.msg.MarkerSetDescription:
    msg = optitrack_msgs.msg.MarkerSetDescription()
    msg.name = value.name
    msg.markers = value.markers
    return msg


def msg_from_skeleton_description(
    value: protocol.SkeletonDescription,
) -> optitrack_msgs.msg.SkeletonDescription:
    msg = optitrack_msgs.msg.SkeletonDescription()
    msg.name = value.name
    msg.id = value.id
    msg.rigid_bodies = [
        msg_from_rigid_body_description(item) for item in value.rigid_bodies
    ]
    return msg


def msg_from_rigid_body_description(
    value: protocol.RigidBodyDescription,
) -> optitrack_msgs.msg.RigidBodyDescription:
    msg = optitrack_msgs.msg.RigidBodyDescription()
    msg.name = value.name
    msg.id = value.id
    msg.parent_id = value.parent_id
    msg.position = msg_from_vector(value.position)
    msg.markers = [msg_from_marker_description(m) for m in value.markers]
    return msg


def msg_from_marker_description(
    value: protocol.RBMarker,
) -> optitrack_msgs.msg.RigidBodyMarkerDescription:
    msg = optitrack_msgs.msg.RigidBodyMarkerDescription()
    msg.name = value.name
    msg.active_label = value.active_label
    msg.position = msg_from_vector(value.position)
    return msg


def msg_from_description(
    value: protocol.MoCapDescription, ) -> optitrack_msgs.msg.Description:
    msg = optitrack_msgs.msg.Description()
    msg.marker_sets = [
        msg_from_marker_set_description(item) for item in value.marker_sets
    ]
    msg.rigid_bodies = [
        msg_from_rigid_body_description(item) for item in value.rigid_bodies
    ]
    msg.skeletons = [
        msg_from_skeleton_description(item) for item in value.skeletons
    ]
    msg.force_plates = [
        msg_from_force_plate_description(item) for item in value.force_plates
    ]
    msg.devices = [msg_from_device_description(item) for item in value.devices]
    msg.cameras = [msg_from_camera_description(item) for item in value.cameras]
    return msg


def msg_from_rigid_body(
        value: protocol.RigidBodyData) -> optitrack_msgs.msg.RigidBody:
    msg = optitrack_msgs.msg.RigidBody()
    msg.id = value.id
    msg.position = msg_from_vector(value.position)
    msg.orientation = msg_from_quaternion(value.orientation)
    msg.markers = [msg_from_marker(m) for m in value.markers]
    msg.tracking = value.tracking_valid
    msg.error = value.error
    return msg


def msg_from_marker(
    value: protocol.RigidBodyMarkerData,
) -> optitrack_msgs.msg.RigidBodyMarker:
    msg = optitrack_msgs.msg.RigidBodyMarker()
    msg.position = msg_from_vector(value.position)
    msg.id = value.id
    msg.size = value.size
    msg.error = value.error
    return msg


def msg_from_marker_set(
        value: protocol.MarkerSetData) -> optitrack_msgs.msg.MarkerSet:
    msg = optitrack_msgs.msg.MarkerSet()
    msg.name = value.name
    msg.positions = [msg_from_vector(p) for p in value.positions]
    return msg


def msg_from_skeleton(
        value: protocol.SkeletonData) -> optitrack_msgs.msg.Skeleton:
    msg = optitrack_msgs.msg.Skeleton()
    msg.id = value.id
    msg.rigid_bodies = [
        msg_from_rigid_body(item) for item in value.rigid_bodies
    ]
    return msg


def msg_from_labeled_marker(
        value: protocol.LabeledMarkerData) -> optitrack_msgs.msg.LabeledMarker:
    msg = optitrack_msgs.msg.LabeledMarker()
    msg.position = msg_from_vector(value.position)
    msg.id = value.id
    msg.size = value.size
    msg.error = value.residual
    msg.param = value.param
    return msg


def msg_from_device(
    value: protocol.DeviceData | protocol.ForcePlateData
) -> optitrack_msgs.msg.Device:
    msg = optitrack_msgs.msg.Device()
    msg.id = value.id
    msg.channels = [msg_from_channel(item) for item in value.channels]
    return msg


def msg_from_channel(
        value: protocol.AnalogChannelData) -> optitrack_msgs.msg.Channel:
    msg = optitrack_msgs.msg.Channel()
    msg.data = value.values
    return msg


def msg_from_data(value: protocol.MoCapData) -> optitrack_msgs.msg.Data:
    msg = optitrack_msgs.msg.Data()
    msg.marker_sets = [msg_from_marker_set(item) for item in value.marker_sets]
    msg.unlabeled_markers_positions = [
        msg_from_vector(p) for p in value.unlabeled_markers_positions
    ]
    msg.rigid_bodies = [
        msg_from_rigid_body(item) for item in value.rigid_bodies
    ]
    msg.skeletons = [msg_from_skeleton(item) for item in value.skeletons]
    msg.labeled_markers = [
        msg_from_labeled_marker(item) for item in value.labeled_markers
    ]
    msg.force_plates = [msg_from_device(item) for item in value.force_plates]
    msg.devices = [msg_from_device(item) for item in value.devices]
    return msg
