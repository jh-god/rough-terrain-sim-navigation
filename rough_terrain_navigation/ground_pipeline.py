"""Gravity-level clouds, serialize Patchwork++, restore stamps, project to Nav2."""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from rclpy.time import Time
from .clouds import read_xyzi, make_cloud, flatten
from .geometry import quaternion_matrix


def transform_arrays(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return np.array([t.x, t.y, t.z]), quaternion_matrix([q.x, q.y, q.z, q.w])


class GroundPipeline(Node):
    """Only one cloud may be outstanding: legacy Patchwork++ truncates stamps."""
    def __init__(self):
        super().__init__('ground_pipeline')
        defaults = dict(input_topic='/sensors/os1_64/points',
                        input_frame_override='', fixed_frame='navigation/odom',
                        physical_frame='base_link', level_frame='navigation/lidar_level',
                        min_range=0.8, max_range=15.0, voxel_size=0.08,
                        processing_timeout=8.0, max_input_age=0.5,
                        self_x_min=-0.60, self_x_max=0.60,
                        self_y_min=-0.38, self_y_max=0.38,
                        self_z_min=-0.55, self_z_max=0.65,
                        max_relative_height=2.5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['min_range'] < 0 or self.p['max_range'] <= self.p['min_range']:
            raise ValueError('Invalid point cloud range')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf = TransformBroadcaster(self)
        self.input_pub = self.create_publisher(PointCloud2, '/navigation/patchwork/input', 1)
        self.ground_pub = self.create_publisher(PointCloud2, '/navigation/ground', 1)
        self.non_ground_pub = self.create_publisher(PointCloud2, '/navigation/nonground', 1)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/navigation/obstacles', 1)
        self.clear_pub = self.create_publisher(PointCloud2, '/navigation/clearing', 1)
        self.status_pub = self.create_publisher(String, '/navigation/perception_status', 1)
        self.create_subscription(PointCloud2, self.p['input_topic'], self.receive,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/navigation/patchwork/ground',
                                 lambda msg: self.result('ground', msg), QoSProfile(depth=2))
        self.create_subscription(PointCloud2, '/navigation/patchwork/nonground',
                                 lambda msg: self.result('nonground', msg), QoSProfile(depth=2))
        self.pending = None
        self.inflight = None
        self.fault = False
        self.create_timer(0.02, self.dispatch)

    def receive(self, msg):
        if self.fault:
            return
        if self.inflight is None and self.pending is None:
            self.pending = msg

    def dispatch(self):
        if self.fault:
            return
        now = self.get_clock().now().nanoseconds
        if self.inflight is not None:
            elapsed = (now-self.inflight['started'])*1e-9
            if elapsed > self.p['processing_timeout'] or elapsed < -0.1:
                self.fault = True
                reason = 'Patchwork++ timeout or simulation reset; restart navigation to avoid pairing wrong frames'
                self.get_logger().error(reason)
                self.status_pub.publish(String(data=reason))
            return
        msg = self.pending
        if msg is None or self.input_pub.get_subscription_count() == 0:
            return
        stamp_ns = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if not -0.1 <= (now-stamp_ns)*1e-9 <= self.p['max_input_age']:
            self.pending = None
            return
        frame = self.p['input_frame_override'] or msg.header.frame_id
        try:
            stamp = Time.from_msg(msg.header.stamp)
            tf = self.buffer.lookup_transform(self.p['fixed_frame'], frame, stamp)
            body_tf = self.buffer.lookup_transform(self.p['physical_frame'], frame, stamp)
        except TransformException as exc:
            self.get_logger().warning('Waiting for cloud TF: '+str(exc), throttle_duration_sec=5.)
            return
        try:
            points = read_xyzi(msg)
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.)
            self.pending = None
            return
        self.pending = None
        body_t, body_r = transform_arrays(body_tf)
        body = points[:, :3] @ body_r.T + body_t
        in_robot = ((body[:, 0] >= self.p['self_x_min']) & (body[:, 0] <= self.p['self_x_max'])
                    & (body[:, 1] >= self.p['self_y_min']) & (body[:, 1] <= self.p['self_y_max'])
                    & (body[:, 2] >= self.p['self_z_min']) & (body[:, 2] <= self.p['self_z_max']))
        distance = np.linalg.norm(points[:, :3], axis=1)
        points = points[(~in_robot) & (distance >= self.p['min_range'])
                        & (distance <= self.p['max_range'])].copy()
        world_t, world_r = transform_arrays(tf)
        points[:, :3] = points[:, :3] @ world_r.T
        if self.p['voxel_size'] > 0 and len(points):
            _, indices = np.unique(np.floor(points[:, :3]/self.p['voxel_size']).astype(np.int32),
                                   axis=0, return_index=True)
            points = points[np.sort(indices)]
        # Never call legacy Patchwork++ with an empty cloud (unsafe adaptive statistics).
        if len(points) < 10:
            return
        header = Header(stamp=copy.deepcopy(msg.header.stamp), frame_id=self.p['level_frame'])
        level_tf = TransformStamped()
        level_tf.header.stamp = header.stamp
        level_tf.header.frame_id = self.p['fixed_frame']
        level_tf.child_frame_id = self.p['level_frame']
        level_tf.transform.translation.x, level_tf.transform.translation.y, level_tf.transform.translation.z = map(float, world_t)
        level_tf.transform.rotation.w = 1.
        self.tf.sendTransform(level_tf)
        self.inflight = dict(header=header, points=points, translation=world_t,
                             started=now, results={})
        self.input_pub.publish(make_cloud(points, header))

    def result(self, kind, msg):
        if self.fault or self.inflight is None:
            return
        flight = self.inflight
        if kind in flight['results']:
            self.fault = True
            self.get_logger().error('Duplicate Patchwork++ output; restart navigation')
            return
        flight['results'][kind] = msg
        if len(flight['results']) != 2:
            return
        # Ground and nonground are produced in the same callback. Preserve original
        # sensor stamp, never the legacy node's truncated output stamp.
        ground = flight['results']['ground']
        non_ground = flight['results']['nonground']
        ground.header = copy.deepcopy(flight['header'])
        non_ground.header = copy.deepcopy(flight['header'])
        self.ground_pub.publish(ground)
        self.non_ground_pub.publish(non_ground)
        try:
            obstacles = read_xyzi(non_ground)
            obstacles = obstacles[obstacles[:, 2] <= self.p['max_relative_height']]
            header = Header(stamp=flight['header'].stamp, frame_id=self.p['fixed_frame'])
            self.clear_pub.publish(make_cloud(flatten(flight['points'], flight['translation']), header))
            self.obstacle_pub.publish(make_cloud(flatten(obstacles, flight['translation']), header))
            self.status_pub.publish(String(data=f'ready ground={ground.width} nonground={non_ground.width}'))
        except (ValueError, TypeError, KeyError) as exc:
            self.fault = True
            self.get_logger().error('Invalid Patchwork++ output: '+str(exc))
        self.inflight = None


def main(args=None):
    rclpy.init(args=args)
    node = GroundPipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
