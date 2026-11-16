"""Require fresh state/commands, and fresh perception when detection is enabled."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def allowed(now_wall, now_sim, records, timeout):
    return all(record is not None and 0 <= now_wall-record[0] <= timeout
               and -0.1 <= now_sim-record[1] <= timeout for record in records)


class CommandGate(Node):
    def __init__(self):
        super().__init__('command_gate')
        for key, value in dict(output_topic='/cmd_vel', timeout=1.0,
                               command_timeout=0.4, max_linear_speed=0.2,
                               max_angular_speed=0.3, require_cloud=True).items():
            self.declare_parameter(key, value)
        self.timeout = self.get_parameter('timeout').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.status = self.create_publisher(String, '/navigation/command_status', 1)
        self.require_cloud = self.get_parameter('require_cloud').value
        self.records = {'state': None}
        if self.require_cloud:
            self.records['cloud'] = None
        self.command, self.command_time = Twist(), -math.inf
        self.create_subscription(Odometry, '/navigation/odom',
                                 lambda msg: self.observe('state', msg), qos_profile_sensor_data)
        if self.require_cloud:
            self.create_subscription(PointCloud2, '/navigation/obstacles',
                                     lambda msg: self.observe('cloud', msg), qos_profile_sensor_data)
        self.create_subscription(Twist, '/navigation/cmd_vel_smoothed', self.receive, 10)
        # A steady timer keeps issuing stop commands even if /clock stops.
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_status = None

    def observe(self, key, msg):
        self.records[key] = (time.monotonic(), msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)

    def receive(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            self.command, self.command_time = Twist(), -math.inf
            return
        self.command, self.command_time = msg, time.monotonic()

    def tick(self):
        now = time.monotonic()
        ready = allowed(now, self.get_clock().now().nanoseconds*1e-9,
                        self.records.values(), self.timeout)
        ready = ready and now-self.command_time <= self.command_timeout
        out = Twist()
        if ready:
            out.linear.x = max(-self.max_linear, min(self.max_linear, self.command.linear.x))
            out.angular.z = max(-self.max_angular, min(self.max_angular, self.command.angular.z))
        self.pub.publish(out)
        waiting_for = 'state, segmentation and command' if self.require_cloud else 'state and command'
        status = 'enabled' if ready else 'stopped: waiting for fresh ' + waiting_for
        if status != self.last_status:
            self.status.publish(String(data=status))
            self.get_logger().info(status)
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = CommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
