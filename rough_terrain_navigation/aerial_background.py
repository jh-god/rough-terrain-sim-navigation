"""Publish a textured map-aligned RViz marker, independent of navigation costs."""
from pathlib import Path
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory


def make_marker(metadata, mesh_uri, frame, z_offset):
    values = [metadata['width'], metadata['height'], metadata['origin_x'],
              metadata['origin_y'], z_offset]
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Background geometry must be finite')
    if min(metadata['width'], metadata['height']) <= 0:
        raise ValueError('Invalid background dimensions')
    msg = Marker()
    msg.header.frame_id = frame
    # Zero stamp deliberately permits rendering without Gazebo /clock or TF history.
    msg.ns, msg.id = 'terrain_aerial_background', 0
    msg.type, msg.action = Marker.MESH_RESOURCE, Marker.ADD
    msg.pose.position.x = float(metadata['origin_x'])
    msg.pose.position.y = float(metadata['origin_y'])
    msg.pose.position.z = float(z_offset)
    msg.pose.orientation.w = 1.
    msg.scale.x, msg.scale.y, msg.scale.z = float(metadata['width']), float(metadata['height']), 1.
    # All-zero color is RViz's sentinel for using embedded materials without tint.
    msg.color.r = msg.color.g = msg.color.b = 0.
    msg.color.a = 0.
    msg.mesh_resource = mesh_uri
    msg.mesh_use_embedded_materials = True
    return msg


class AerialBackground(Node):
    def __init__(self):
        super().__init__('aerial_background')
        share = Path(get_package_share_directory('rough_terrain_navigation'))
        defaults = dict(metadata_file=str(share/'assets/terrain_aerial.yaml'),
                        frame_id='map', z_offset=-.20)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}
        source = Path(p['metadata_file'])
        metadata = yaml.safe_load(source.read_text())
        mesh = (source.parent/metadata['mesh']).resolve()
        image = (source.parent/metadata['image']).resolve()
        if not mesh.is_file() or not image.is_file():
            raise FileNotFoundError('Aerial mesh/texture missing; regenerate the aerial assets')
        self.marker = make_marker(metadata, mesh.as_uri(), p['frame_id'], p['z_offset'])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, '/navigation/aerial_background', qos)
        self.pub.publish(self.marker)
        self.create_timer(2., lambda: self.pub.publish(self.marker))
        self.get_logger().info(f'Visual-only aerial map: {metadata["width"]} x {metadata["height"]} m, '+
                               f'origin=({metadata["origin_x"]}, {metadata["origin_y"]}), frame={p["frame_id"]}')


def main(args=None):
    rclpy.init(args=args)
    node = AerialBackground()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
