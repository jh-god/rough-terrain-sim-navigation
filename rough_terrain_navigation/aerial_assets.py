"""Build deterministic, map-aligned aerial imagery from existing simulation assets.

Blender is needed only for regeneration. Runtime uses the checked-in PNG/Collada.
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from PIL import Image


UNIT_PLANE_DAE = '''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
 <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
 <library_images><image id="aerial_image"><init_from>terrain_aerial.png</init_from></image></library_images>
 <library_effects><effect id="aerial_effect"><profile_COMMON>
  <newparam sid="aerial_surface"><surface type="2D"><init_from>aerial_image</init_from></surface></newparam>
  <newparam sid="aerial_sampler"><sampler2D><source>aerial_surface</source></sampler2D></newparam>
  <technique sid="common"><lambert><emission><color>0 0 0 1</color></emission><ambient><color>1 1 1 1</color></ambient><diffuse><texture texture="aerial_sampler" texcoord="UVMap"/></diffuse>
  <transparent opaque="A_ONE"><color>1 1 1 1</color></transparent><transparency><float>1</float></transparency>
  </lambert></technique></profile_COMMON></effect></library_effects>
 <library_materials><material id="aerial_material"><instance_effect url="#aerial_effect"/></material></library_materials>
 <library_geometries><geometry id="plane"><mesh>
  <source id="positions"><float_array id="positions_array" count="12">0 0 0 1 0 0 1 1 0 0 1 0</float_array>
   <technique_common><accessor source="#positions_array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
  <source id="normals"><float_array id="normals_array" count="3">0 0 1</float_array>
   <technique_common><accessor source="#normals_array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
  <source id="uv"><float_array id="uv_array" count="8">0 0 1 0 1 1 0 1</float_array>
   <technique_common><accessor source="#uv_array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
  <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
  <triangles count="2" material="material"><input semantic="VERTEX" source="#vertices" offset="0"/>
   <input semantic="NORMAL" source="#normals" offset="1"/><input semantic="TEXCOORD" source="#uv" offset="2" set="0"/>
   <p>0 0 0 1 0 1 2 0 2 0 0 0 2 0 2 3 0 3</p></triangles>
 </mesh></geometry></library_geometries>
 <library_visual_scenes><visual_scene id="Scene"><node id="plane_node"><instance_geometry url="#plane"><bind_material><technique_common>
  <instance_material symbol="material" target="#aerial_material"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material>
 </technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
 <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
'''


def resolve_model(uri, share):
    if not uri.startswith('model://'):
        raise ValueError('Expected model:// URI, got '+uri)
    path = Path(share)/'models'/uri[len('model://'):]
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open('rb') as stream:
        is_pointer = stream.read(42).startswith(b'version https://git-lfs.github.com')
    if is_pointer:
        raise ValueError('Asset is a Git LFS pointer; run git lfs pull: '+str(path))
    return path


def terrain_manifest(share):
    """Read physical bounds and model transforms from the existing SDF world."""
    share = Path(share)
    world_path = share/'worlds/rough_terrain.sdf'
    world = ET.parse(world_path).getroot().find('world')
    terrain_sdf = share/'models/rough_terrain/model.sdf'
    terrain = ET.parse(terrain_sdf).getroot().find('model')
    hm = terrain.find('.//collision/geometry/heightmap')
    if hm is None:
        raise ValueError('Terrain collision heightmap not found')
    size = list(map(float, hm.findtext('size').split()))
    if len(size) != 3 or min(size) <= 0:
        raise ValueError('Invalid terrain dimensions')
    position = list(map(float, hm.findtext('pos', '0 0 0').split()))
    # The bundled terrain has no model/link/include rotation or translation.
    # Reject unsupported transforms rather than quietly mis-registering imagery.
    for pose in [terrain.findtext('pose'), terrain.findtext('link/pose')]:
        if pose and any(abs(float(v)) > 1e-10 for v in pose.split()):
            raise ValueError('Nonzero terrain pose requires explicit support')
    for include in world.findall('include'):
        if include.findtext('uri') == 'model://rough_terrain':
            pose = include.findtext('pose')
            if pose and any(abs(float(v)) > 1e-10 for v in pose.split()):
                raise ValueError('Nonzero terrain include pose requires explicit support')
    hm_path = resolve_model(hm.findtext('uri'), share)
    texture = terrain.find('.//visual/geometry/heightmap/texture')
    texture_path = resolve_model(texture.findtext('diffuse'), share)
    texture_size = float(texture.findtext('size'))
    objects = []
    for include in world.findall('include'):
        uri = include.findtext('uri', '')
        if uri == 'model://rough_terrain':
            continue
        model_file = resolve_model(uri+'/model.sdf', share)
        model = ET.parse(model_file).getroot().find('model')
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                objects.append(dict(name=include.findtext('name', model.get('name')),
                    mesh=str(resolve_model(mesh.findtext('uri'), share)),
                    include_pose=list(map(float, include.findtext('pose', '0 0 0 0 0 0').split())),
                    model_pose=list(map(float, model.findtext('pose', '0 0 0 0 0 0').split())),
                    link_pose=list(map(float, link.findtext('pose', '0 0 0 0 0 0').split())),
                    visual_pose=list(map(float, visual.findtext('pose', '0 0 0 0 0 0').split())),
                    scale=list(map(float, mesh.findtext('scale', '1 1 1').split()))))
    # PNG top row is world +Y. Normalized PNG height is the SDF elevation range.
    with Image.open(hm_path) as image:
        if image.mode.startswith('I'):
            denominator = 65535.
        elif image.mode == 'L':
            denominator = 255.
        else:
            raise ValueError('Expected grayscale heightmap, got '+image.mode)
        heights = np.asarray(image, dtype=np.float32)*size[2]/denominator+position[2]
    metadata = dict(frame_id='map', width=size[0], height=size[1],
                    origin_x=position[0]-size[0]/2, origin_y=position[1]-size[1]/2,
                    image='terrain_aerial.png', mesh='terrain_aerial.dae',
                    projection='orthographic', image_top='+Y', image_right='+X',
                    visual_only=True, object_count=len(objects),
                    source_world_sha256=hashlib.sha256(world_path.read_bytes()).hexdigest(),
                    source_heightmap_sha256=hashlib.sha256(hm_path.read_bytes()).hexdigest(),
                    source_terrain_sdf_sha256=hashlib.sha256(terrain_sdf.read_bytes()).hexdigest())
    return dict(size=size, position=position, texture=str(texture_path),
                texture_size=texture_size, objects=objects), heights, metadata


def generate(share, output, blender='blender', pixels=2048, samples=24):
    if pixels < 64 or pixels > 8192 or samples < 1:
        raise ValueError('pixels must be 64..8192 and samples positive')
    executable = shutil.which(blender)
    if not executable:
        raise RuntimeError('Blender is needed to regenerate imagery; not for runtime')
    manifest, heights, metadata = terrain_manifest(share)
    if not np.isclose(metadata['width'], metadata['height']):
        raise ValueError('This renderer currently supports square terrain extents only')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='terrain_aerial_build_') as work:
        work = Path(work)
        np.save(work/'heights.npy', heights)
        manifest.update(height_file=str(work/'heights.npy'), output=str(output/'terrain_aerial.png'),
                        pixels=pixels, samples=samples)
        path = work/'manifest.json'
        path.write_text(json.dumps(manifest))
        renderer = Path(__file__).with_name('render_aerial_blender.py')
        subprocess.run([executable, '--background', '--factory-startup', '--threads', '8',
                        '--python', str(renderer), '--', str(path)], check=True)
    (output/'terrain_aerial.dae').write_text(UNIT_PLANE_DAE)
    metadata.update(pixels=[pixels, pixels], meters_per_pixel=[metadata['width']/pixels,
                    metadata['height']/pixels], renderer='Blender Cycles CPU')
    (output/'terrain_aerial.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    return output/'terrain_aerial.yaml'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-share')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--blender', default='blender')
    parser.add_argument('--pixels', type=int, default=2048)
    parser.add_argument('--samples', type=int, default=24)
    args = parser.parse_args()
    if args.simulation_share is None:
        from ament_index_python.packages import get_package_share_directory
        args.simulation_share = get_package_share_directory('rough_terrain_sim')
    print(generate(args.simulation_share, args.output_dir, args.blender, args.pixels, args.samples))
