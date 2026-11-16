"""Preserve physical 6DoF TF while exposing map-aligned planar Nav2 odometry."""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .geometry import quaternion_matrix, split_pose


class StateAdapter(Node):
    def __init__(self):
        super().__init__('simulation_state_adapter')
        defaults = dict(input_topic='/ground_truth/odom', expected_frame='world',
                        map_frame='map', odom_frame='navigation/odom',
                        planar_frame='navigation/base', physical_frame='base_footprint')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.pub = self.create_publisher(Odometry, '/navigation/odom', 10)
        self.pose_pub = self.create_publisher(Odometry, '/navigation/ground_truth', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        identity = TransformStamped()
        identity.header.frame_id = self.p['map_frame']
        identity.child_frame_id = self.p['odom_frame']
        identity.transform.rotation.w = 1.
        self.static_tf.sendTransform(identity)
        self.create_subscription(Odometry, self.p['input_topic'], self.callback,
                                 qos_profile_sensor_data)
        self.previous = None

    def callback(self, msg):
        if msg.header.frame_id != self.p['expected_frame']:
            self.get_logger().error('Ground truth frame mismatch: expected '+self.p['expected_frame'],
                                    throttle_duration_sec=5.)
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        position = np.array([p.x, p.y, p.z])
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if not np.isfinite(position).all():
            return
        try:
            flat_p, flat_q, residual_p, residual_q = split_pose(position, quaternion)
        except ValueError:
            return
        stamp = msg.header.stamp
        stamp_ns = stamp.sec*1_000_000_000+stamp.nanosec
        transforms = []
        for parent, child, xyz, quat in [
            (self.p['odom_frame'], self.p['planar_frame'], flat_p, flat_q),
            (self.p['planar_frame'], self.p['physical_frame'], residual_p, residual_q),
        ]:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, quat)
            transforms.append(tf)
        self.tf.sendTransform(transforms)
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id, out.child_frame_id = self.p['odom_frame'], self.p['planar_frame']
        out.pose.pose.position.x, out.pose.pose.position.y = float(flat_p[0]), float(flat_p[1])
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = float(flat_q[2]), float(flat_q[3])
        # Differentiate world poses rather than assuming Gazebo twist-frame semantics.
        # Invalid dt/reset yields zero velocity, never a teleport-derived command.
        if self.previous is not None:
            prev_ns, prev_p, prev_yaw = self.previous
            dt = (stamp_ns-prev_ns)*1e-9
            if 0. < dt < 0.5:
                local_v = quaternion_matrix(flat_q).T @ ((position-prev_p)/dt)
                yaw = np.arctan2(2*flat_q[2]*flat_q[3], 1-2*flat_q[2]**2)
                dyaw = np.arctan2(np.sin(yaw-prev_yaw), np.cos(yaw-prev_yaw))
                out.twist.twist.linear.x, out.twist.twist.linear.y = float(local_v[0]), float(local_v[1])
                out.twist.twist.angular.z = float(dyaw/dt)
        yaw = np.arctan2(2*flat_q[2]*flat_q[3], 1-2*flat_q[2]**2)
        self.previous = (stamp_ns, position.copy(), yaw)
        self.pub.publish(out)
        raw = copy.deepcopy(msg)
        raw.header.frame_id = self.p['map_frame']
        raw.child_frame_id = self.p['physical_frame']
        self.pose_pub.publish(raw)


def main(args=None):
    rclpy.init(args=args)
    node = StateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
