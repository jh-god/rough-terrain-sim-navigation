"""Run only inside Blender: reconstruct source SDF visuals, render an orthophoto."""
import json
import math
import sys
from pathlib import Path
import bpy
import numpy as np
from mathutils import Euler, Matrix, Vector


def pose_matrix(pose):
    x, y, z, roll, pitch, yaw = pose
    return Matrix.Translation(Vector((x, y, z))) @ Euler((roll, pitch, yaw), 'XYZ').to_matrix().to_4x4()


def main(path):
    manifest = json.loads(Path(path).read_text())
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    heights = np.load(manifest['height_file'])
    width, length, _ = manifest['size']
    px, py, _ = manifest['position']
    # Native endpoints, ~0.1 m mesh spacing; source texture remains full resolution.
    rows = np.unique(np.linspace(0, heights.shape[0]-1, 513).round().astype(int))
    cols = np.unique(np.linspace(0, heights.shape[1]-1, 513).round().astype(int))
    xs = px-width/2+cols/(heights.shape[1]-1)*width
    ys = py+length/2-rows/(heights.shape[0]-1)*length
    xx, yy = np.meshgrid(xs, ys)
    zz = heights[np.ix_(rows, cols)]
    vertices = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel()))
    faces = []
    n = len(cols)
    for r in range(len(rows)-1):
        for c in range(n-1):
            a = r*n+c
            # Counterclockwise as viewed from +Z (rows move towards -Y).
            faces.append((a, a+n, a+n+1, a+1))
    mesh = bpy.data.meshes.new('original_heightmap')
    mesh.from_pydata(vertices.tolist(), [], faces)
    mesh.update()
    ground = bpy.data.objects.new('terrain', mesh)
    bpy.context.collection.objects.link(ground)
    uv = mesh.uv_layers.new(name='UVMap')
    for loop in mesh.loops:
        v = vertices[loop.vertex_index]
        uv.data[loop.index].uv = ((v[0]-(px-width/2))/manifest['texture_size'],
                                 (v[1]-(py-length/2))/manifest['texture_size'])
    material = bpy.data.materials.new('source_dirt')
    material.use_nodes = True
    shader = material.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Roughness'].default_value = .95
    texture = material.node_tree.nodes.new('ShaderNodeTexImage')
    texture.image = bpy.data.images.load(manifest['texture'])
    texture.extension = 'REPEAT'
    material.node_tree.links.new(texture.outputs['Color'], shader.inputs['Base Color'])
    ground.data.materials.append(material)
    for polygon in mesh.polygons:
        polygon.use_smooth = True
    cache = {}
    for item in manifest['objects']:
        source = item['mesh']
        if source not in cache:
            before = set(bpy.data.objects)
            # Gazebo OBJ vertices already use X-forward, Z-up. Prevent default OBJ Y-up conversion.
            bpy.ops.wm.obj_import(filepath=source, forward_axis='Y', up_axis='Z')
            imported = [obj for obj in bpy.data.objects if obj not in before]
            cache[source] = [(obj, obj.matrix_world.copy()) for obj in imported]
        transform = Matrix.Identity(4)
        for key in ['include_pose', 'model_pose', 'link_pose', 'visual_pose']:
            transform = transform @ pose_matrix(item[key])
        scale = Matrix.Diagonal(Vector((*item['scale'], 1.)))
        for original, local in cache[source]:
            instance = original.copy()
            instance.data = original.data
            instance.name = item['name']
            bpy.context.collection.objects.link(instance)
            instance.matrix_world = transform @ scale @ local
    # Remove import prototypes, retaining shared mesh/material datablocks in instances.
    for originals in cache.values():
        for obj, _ in originals:
            bpy.data.objects.remove(obj, do_unlink=True)
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = manifest['samples']
    scene.cycles.use_denoising = True
    scene.cycles.seed = 42
    scene.render.resolution_x = scene.render.resolution_y = manifest['pixels']
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGB'
    scene.render.filepath = manifest['output']
    scene.render.film_transparent = False
    scene.world.use_nodes = True
    scene.world.node_tree.nodes['Background'].inputs[0].default_value = (.65, .72, .82, 1)
    scene.world.node_tree.nodes['Background'].inputs[1].default_value = .45
    scene.view_settings.view_transform = 'Standard'
    scene.view_settings.exposure = 0
    scene.view_settings.gamma = 1
    light = bpy.data.lights.new('source_sun', 'SUN')
    light.energy = 2.5
    light.angle = math.radians(5)
    sun = bpy.data.objects.new('source_sun', light)
    bpy.context.collection.objects.link(sun)
    sun.rotation_euler = Vector((-.5, .1, -.9)).to_track_quat('-Z', 'Y').to_euler()
    camera = bpy.data.cameras.new('orthographic_topview')
    camera.type = 'ORTHO'
    camera.ortho_scale = max(width, length)
    camera.clip_start, camera.clip_end = .1, 200.
    obj = bpy.data.objects.new('orthographic_topview', camera)
    bpy.context.collection.objects.link(obj)
    obj.location = (px, py, 60.)
    obj.rotation_euler = (0, 0, 0)  # Camera looks along local -Z, top of image = +Y.
    scene.camera = obj
    bpy.ops.render.render(write_still=True)


if __name__ == '__main__':
    main(sys.argv[sys.argv.index('--')+1])
