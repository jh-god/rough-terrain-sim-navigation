from pathlib import Path
from types import SimpleNamespace
import time
import pytest
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rough_terrain_navigation.command_gate import CommandGate
from rough_terrain_navigation.navigation_modes import nav2_for_mode


def test_detection_off_removes_both_obstacle_layers_without_mutating_profile():
    params = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/nav2.yaml').read_text())
    result = nav2_for_mode(params, False)
    for name in ['global_costmap', 'local_costmap']:
        actual = result[name][name]['ros__parameters']
        assert actual['plugins'] == ['static_layer', 'inflation_layer']
        assert 'obstacle_layer' not in actual
        assert 'obstacle_layer' in params[name][name]['ros__parameters']['plugins']
    assert result['controller_server'] == params['controller_server']
    assert nav2_for_mode(params, True) == params


@pytest.mark.parametrize('require_cloud', [True, False])
def test_gate_can_skip_cloud_but_never_skips_state_or_command_freshness(require_cloud):
    rclpy.init(args=['--ros-args', '-p', 'require_cloud:='+str(require_cloud).lower()])
    node = CommandGate()
    outputs = []
    node.pub = SimpleNamespace(publish=outputs.append)
    try:
        state = Odometry()
        state.header.stamp = node.get_clock().now().to_msg()
        command = Twist()
        command.linear.x = .1
        node.receive(command)
        node.tick()
        assert outputs[-1].linear.x == 0.  # State is still required in either mode.
        node.observe('state', state)
        node.tick()
        assert outputs[-1].linear.x == (0. if require_cloud else .1)
        if require_cloud:
            node.observe('cloud', state)  # Same stamped-header interface as PointCloud2.
            node.tick()
            assert outputs[-1].linear.x == .1
        else:
            assert 'cloud' not in node.records
        node.records['state'] = (time.monotonic()-2., node.get_clock().now().nanoseconds*1e-9)
        node.tick()
        assert outputs[-1].linear.x == 0.
        node.observe('state', state)
        node.command_time = time.monotonic()-2.
        node.tick()
        assert outputs[-1].linear.x == 0.
    finally:
        node.destroy_node()
        rclpy.shutdown()
