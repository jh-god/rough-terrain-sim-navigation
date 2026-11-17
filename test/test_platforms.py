from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from rough_terrain_navigation.platforms import platform, clearpath_description, transform_to_link


@pytest.mark.parametrize('robot', ['husky','jackal'])
def test_clearpath_geometry_gt_and_controller_adapter_match_profiles(robot,tmp_path):
    source = get_package_share_directory('rough_terrain_sim')
    xml,meta = clearpath_description(robot,source,tmp_path)
    root = ET.fromstring(xml)
    config = Path(__file__).resolve().parents[1]/'config'
    spec = platform(robot)
    patch = yaml.safe_load((config/spec['patchwork']).read_text())
    profile = yaml.safe_load((config/spec['profile']).read_text())
    assert patch['patchworkpp']['ros__parameters']['sensor_height'] == pytest.approx(meta['sensor_height'])
    assert profile['ground_pipeline']['ros__parameters']['input_frame_override'] == meta['lidar']['frame']
    assert profile['simulation_state_adapter']['ros__parameters']['physical_frame'] == 'base_link'
    assert root.find('ros2_control') is None
    plugins = root.findall('gazebo/plugin')
    assert not any('ROS2Control' in p.get('name','') for p in plugins)
    drive = next(p for p in plugins if p.get('name','').endswith('::DiffDrive'))
    assert len(drive.findall('left_joint')) == len(drive.findall('right_joint')) == 2
    assert drive.findtext('topic') == '/cmd_vel'
    gt = next(p for p in plugins if p.get('name','').endswith('::OdometryPublisher'))
    assert gt.findtext('dimensions') == '3'
    assert gt.findtext('robot_base_frame') == 'base_link'
    # State adapter must attach to the URDF root, not to a child footprint (TF cycle).
    children = {j.find('child').get('link') for j in root.findall('joint')}
    assert 'base_link' not in children
    assert meta['ground_z'] < 0
    assert spec['spawn_z']+meta['ground_z'] > 1.24483
    assert transform_to_link(root,meta['lidar']['frame'])[2,3] > 0
    assert (tmp_path/'navigation.urdf').is_file()


def test_unknown_platform_rejected():
    with pytest.raises(ValueError):
        platform('unknown')


def test_jackal_camera_request_fails_before_spawning(tmp_path,monkeypatch):
    import runpy
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    root = Path(__file__).resolve().parents[1]
    module = runpy.run_path(str(root/'launch/simulation.launch.py'))
    context = LaunchContext()
    context.launch_configurations.update(robot='jackal',camera='true')
    for action in module['generate_launch_description']().entities:
        if isinstance(action,DeclareLaunchArgument):
            action.execute(context)
    monkeypatch.setattr(module['tempfile'],'mkdtemp',lambda **kwargs:str(tmp_path))
    with pytest.raises(ValueError,match='no RGB-D camera'):
        module['setup'](context)
