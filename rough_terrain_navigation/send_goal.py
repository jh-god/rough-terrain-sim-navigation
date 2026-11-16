"""Send a map-frame NavigateToPose goal and report its actual action result."""
import argparse
import math
import sys
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def main(args=None):
    argv = remove_ros_args(args=sys.argv if args is None else args)[1:]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('--yaw', type=float, default=0.)
    parser.add_argument('--frame', default='map')
    parser.add_argument('--timeout', type=float, default=180.)
    parsed = parser.parse_args(argv)
    if not all(math.isfinite(v) for v in (parsed.x, parsed.y, parsed.yaw, parsed.timeout)) or parsed.timeout <= 0:
        parser.error('Coordinates must be finite and timeout positive')
    rclpy.init(args=args)
    node = Node('send_navigation_goal', parameter_overrides=[])
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    handle = None
    code = 1
    try:
        if not client.wait_for_server(timeout_sec=30.):
            raise RuntimeError('NavigateToPose server unavailable after 30 seconds')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = parsed.frame
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = parsed.x, parsed.y
        goal.pose.pose.orientation.z = math.sin(parsed.yaw/2)
        goal.pose.pose.orientation.w = math.cos(parsed.yaw/2)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        if not future.done() or future.result() is None or not future.result().accepted:
            raise RuntimeError('Goal not accepted')
        handle = future.result()
        result = handle.get_result_async()
        deadline = time.monotonic()+parsed.timeout
        while rclpy.ok() and not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not result.done():
            raise RuntimeError('Goal timed out')
        status = result.result().status
        node.get_logger().info(f'Goal action status: {status}')
        code = 0 if status == GoalStatus.STATUS_SUCCEEDED else 1
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(str(exc) or 'Interrupted')
        if handle is not None:
            future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
