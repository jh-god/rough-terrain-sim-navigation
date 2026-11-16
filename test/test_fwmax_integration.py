"""Check navigation assumptions against the installed upstream FWmax model."""
import math
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope='module')
def model():
    sim = Path(get_package_share_directory('rough_terrain_sim'))
    control = Path(get_package_share_directory('fwmax_dual_ackermann_controller'))
    return ET.fromstring(xacro.process_file(str(sim/'urdf/fwmax_skid_steer.urdf.xacro'), mappings={
        'drive_mode': 'swerve', 'controllers_file': str(control/'config/fwmax_controllers.yaml')}).toxml())


def link_transform(model, child):
    if child == 'base_footprint':
        return np.eye(4)
    joint = next(j for j in model.findall('joint') if j.find('child').get('link') == child)
    origin = joint.find('origin')
    transform = np.eye(4)
    if origin is not None:
        transform[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
        roll, pitch, yaw = map(float, origin.get('rpy', '0 0 0').split())
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        transform[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                              [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                              [-sp, cp*sr, cp*cr]]
    return link_transform(model, joint.find('parent').get('link')) @ transform


def test_patchwork_sensor_height_matches_upstream_tf(model):
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/patchworkpp.yaml').read_text())
    assert config['patchworkpp']['ros__parameters']['sensor_height'] == pytest.approx(
        link_transform(model, 'os_sensor')[2, 3], abs=1e-9)


def test_separate_meshes_are_installed_as_real_assets(model):
    visuals = [model.find("link[@name='base_link']/visual/geometry/mesh")]
    visuals += [model.find(f"link[@name='{name}_wheel_link']/visual/geometry/mesh")
                for name in ['front_left', 'front_right', 'rear_left', 'rear_right']]
    for visual in visuals:
        assert visual is not None, 'Expected separate body/wheel mesh visual'
        uri = urlparse(visual.get('filename'))
        assert uri.scheme == 'file'
        path = Path(unquote(uri.path))
        assert path.is_file(), f'Missing model asset: {path}; rebuild rough_terrain_sim'
        with path.open('rb') as stream:
            assert not stream.read(64).startswith(b'version https://git-lfs.github.com'), str(path)
    assert len({v.get('filename') for v in visuals}) == 2


def test_wheels_keep_ground_contact_and_controller_joint_names(model):
    control = Path(get_package_share_directory('fwmax_dual_ackermann_controller'))
    config = yaml.safe_load((control/'config/fwmax_controllers.yaml').read_text())
    for name in ['front_left', 'front_right', 'rear_left', 'rear_right']:
        link = model.find(f"link[@name='{name}_wheel_link']")
        radius = float(link.find('collision/geometry/cylinder').get('radius'))
        assert link_transform(model, name+'_wheel_link')[2, 3] - radius == pytest.approx(0., abs=1e-9)
        for kind in ['wheel', 'steering']:
            joint = name+'_'+kind+'_joint'
            assert model.find(f"joint[@name='{joint}']") is not None
            assert joint in config[f'fwmax_{kind}_controller']['ros__parameters']['joints']
