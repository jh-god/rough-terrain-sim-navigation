from pathlib import Path
from types import SimpleNamespace
import time
import pytest
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rough_terrain_navigation.command_gate import CommandGate
from rough_terrain_navigation.navigation_modes import nav2_for_mode, obstacle_detection_for_mode


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


@pytest.mark.parametrize('mode,override,enabled', [
    ('perception', 'auto', True), ('map_only', 'auto', False),
    ('perception', 'false', False), ('map_only', 'true', True),
    ('waypoint', 'auto', False), ('waypoint', 'true', True),
])
def test_mode_selection_and_legacy_explicit_override(mode, override, enabled):
    assert obstacle_detection_for_mode(mode, override) is enabled


@pytest.mark.parametrize('mode,override', [('unknown','auto'), ('perception','invalid')])
def test_invalid_mode_is_rejected_before_starting_processes(mode, override):
    with pytest.raises(ValueError):
        obstacle_detection_for_mode(mode, override)


@pytest.mark.parametrize('mode,expected', [('perception', True), ('map_only', False)])
@pytest.mark.parametrize('robot', ['fwmax', 'husky', 'jackal'])
def test_existing_launches_apply_mode_to_node_conditions_and_costmaps(mode, expected, robot, tmp_path, monkeypatch):
    import runpy
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    root = Path(__file__).resolve().parents[1]
    for filename, enabled_conditions in [('navigation.launch.py', 2+2*expected),
                                         ('simulation.launch.py', int(expected))]:
        module = runpy.run_path(str(root/'launch'/filename))
        context = LaunchContext()
        context.launch_configurations['mode'] = mode
        context.launch_configurations['robot'] = robot
        for action in module['generate_launch_description']().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        if 'tempfile' in module:
            monkeypatch.setattr(module['tempfile'], 'mkdtemp', lambda **kwargs: str(tmp_path))
        actions = module['setup'](context)
        assert sum(bool(a.condition.evaluate(context)) for a in actions if a.condition) == enabled_conditions
        if filename == 'navigation.launch.py':
            params = yaml.safe_load((tmp_path/'nav2.yaml').read_text())
            for name in ['global_costmap', 'local_costmap']:
                assert ('obstacle_layer' in params[name][name]['ros__parameters']['plugins']) == expected
