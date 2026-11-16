from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from PIL import Image
from visualization_msgs.msg import Marker
from rough_terrain_navigation.aerial_background import make_marker
from rough_terrain_navigation.aerial_assets import UNIT_PLANE_DAE, terrain_manifest


def test_marker_registers_unit_mesh_to_metric_map_without_rotation():
    marker = make_marker(dict(width=50., height=50., origin_x=-25., origin_y=-25.),
                         'file:///tmp/terrain_aerial.dae', 'map', -.04)
    assert marker.type == Marker.MESH_RESOURCE
    assert marker.mesh_use_embedded_materials
    assert marker.header.frame_id == 'map'
    assert (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z) == (-25., -25., -.04)
    assert marker.pose.orientation.w == 1.
    assert (marker.scale.x, marker.scale.y, marker.scale.z) == (50., 50., 1.)
    assert marker.color.r == marker.color.g == marker.color.b == 0.
    # NE corner of the unit mesh must land at world (+25,+25).
    assert marker.pose.position.x + marker.scale.x == 25.
    assert marker.pose.position.y + marker.scale.y == 25.


@pytest.mark.parametrize('width,z', [(0.,0.), (50.,float('nan')), (-50.,0.)])
def test_invalid_background_geometry_rejected(width, z):
    with pytest.raises(ValueError):
        make_marker(dict(width=width,height=50.,origin_x=-25.,origin_y=-25.), 'file:///x', 'map', z)


def test_heightmap_keeps_north_row_and_sdf_height_range(tmp_path):
    terrain = tmp_path/'models/rough_terrain'
    terrain.mkdir(parents=True)
    (tmp_path/'worlds').mkdir()
    (tmp_path/'worlds/rough_terrain.sdf').write_text('<sdf><world><include><uri>model://rough_terrain</uri></include></world></sdf>')
    (terrain/'model.sdf').write_text('''<sdf><model name="terrain"><link name="ground">
      <collision><geometry><heightmap><uri>model://rough_terrain/height.png</uri><size>50 50 4</size><pos>0 0 1</pos></heightmap></geometry></collision>
      <visual><geometry><heightmap><texture><diffuse>model://rough_terrain/dirt.png</diffuse><size>6</size></texture></heightmap></geometry></visual>
    </link></model></sdf>''')
    Image.fromarray(np.array([[65535,32768],[0,16384]],dtype=np.uint16)).save(terrain/'height.png')
    Image.new('RGB',(2,2)).save(terrain/'dirt.png')
    _, heights, metadata = terrain_manifest(tmp_path)
    assert heights[0,0] == 5. and heights[1,0] == 1.
    assert metadata['image_top'] == '+Y' and metadata['image_right'] == '+X'
    assert metadata['origin_x'] == metadata['origin_y'] == -25.
    assert metadata['visual_only'] is True


def test_packaged_texture_and_collada_share_correct_uv_mapping():
    root = Path(__file__).resolve().parents[1]
    metadata = yaml.safe_load((root/'assets/terrain_aerial.yaml').read_text())
    ns = {'c':'http://www.collada.org/2005/11/COLLADASchema'}
    dae = ET.fromstring((root/'assets'/metadata['mesh']).read_text())
    assert (root/'assets'/metadata['mesh']).read_text() == UNIT_PLANE_DAE
    assert dae.find('.//c:image/c:init_from',ns).text == metadata['image']
    assert dae.find('.//c:asset/c:up_axis',ns).text == 'Z_UP'
    assert dae.find('.//c:source[@id="uv"]/c:float_array',ns).text.split() == ['0','0','1','0','1','1','0','1']
    with Image.open(root/'assets'/metadata['image']) as image:
        assert list(image.size) == metadata['pixels']
    assert metadata['width'] == metadata['height'] == 50.
