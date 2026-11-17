from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from rough_terrain_navigation.sensors import camera_enabled, sensor_world


@pytest.mark.parametrize('attached', [True, False])
def test_camera_auto_matches_attached_model_and_false_is_explicit_opt_out(attached):
    assert camera_enabled('auto', attached) is attached
    assert camera_enabled('false', attached) is False
    if attached:
        assert camera_enabled('true', attached) is True
    else:
        with pytest.raises(ValueError, match='no RGB-D camera'):
            camera_enabled('true', attached)


def test_navsat_world_preserves_source_and_existing_geographic_reference(tmp_path):
    source = tmp_path/'original.sdf'
    original = '<sdf version="1.8"><world name="test"/></sdf>'
    source.write_text(original)
    assert sensor_world(source,tmp_path,False) == str(source)
    result = Path(sensor_world(source,tmp_path,True))
    assert source.read_text() == original
    tree = ET.parse(result)
    world = tree.getroot().find('world')
    assert world.find('plugin').get('name').endswith('::NavSat')
    assert world.findtext('spherical_coordinates/latitude_deg') == '0'
    world.find('spherical_coordinates/latitude_deg').text = '37'
    tree.write(result)
    sensor_world(result,tmp_path,True)
    world = ET.parse(result).getroot().find('world')
    assert world.findtext('spherical_coordinates/latitude_deg') == '37'
    assert len(world.findall('plugin')) == 1
