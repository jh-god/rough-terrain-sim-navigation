"""Robot-specific geometry and temporary Clearpath model adaptation."""
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

PLATFORMS = {
    'fwmax': dict(profile='fwmax_swerve.yaml', patchwork='patchworkpp.yaml',
                  footprint=[[.55,.33],[.55,-.33],[-.55,-.33],[-.55,.33]], spawn_z=1.30),
    'husky': dict(profile='husky.yaml', patchwork='patchworkpp_husky.yaml',
                  footprint=[[.55,.36],[.55,-.36],[-.55,-.36],[-.55,.36]], spawn_z=1.45),
    'jackal': dict(profile='jackal.yaml', patchwork='patchworkpp_jackal.yaml',
                   footprint=[[.26,.24],[.26,-.24],[-.26,-.24],[-.26,.24]], spawn_z=1.38),
}


def platform(robot):
    if robot not in PLATFORMS:
        raise ValueError('robot must be fwmax, husky or jackal')
    return PLATFORMS[robot]


def transform_to_link(root, child):
    if root.find(f"link[@name='{child}']") is None:
        raise ValueError('Unknown URDF link: '+child)
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    if child not in joints:
        return np.eye(4)
    joint = joints[child]
    origin = joint.find('origin')
    matrix = np.eye(4)
    if origin is not None:
        matrix[:3,3] = list(map(float, origin.get('xyz','0 0 0').split()))
        r,p,y = map(float,origin.get('rpy','0 0 0').split())
        cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
        matrix[:3,:3] = [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                         [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    return transform_to_link(root,joint.find('parent').get('link')) @ matrix


def add_plugin(root, name, filename, fields):
    gazebo = ET.SubElement(root,'gazebo')
    plugin = ET.SubElement(gazebo,'plugin',name='ignition::gazebo::systems::'+name,filename=filename)
    for key,value in fields:
        ET.SubElement(plugin,key).text = str(value)


def clearpath_description(robot, simulation_share, output):
    """Generate upstream assets in scratch space; replace only actuation/odom plugins."""
    platform(robot)
    if robot == 'fwmax':
        raise ValueError('FWmax uses its upstream Xacro directly')
    from clearpath_generator_common.description.generator import DescriptionGenerator
    import xacro
    output = Path(output)
    output.mkdir(parents=True,exist_ok=True)
    shutil.copy(Path(simulation_share)/'config'/robot/'robot.yaml',output/'robot.yaml')
    DescriptionGenerator(str(output)+'/').generate()
    root = ET.fromstring(xacro.process_file(str(output/'robot.urdf.xacro'),mappings={'is_sim':'true'}).toxml())
    for control in root.findall('ros2_control'):
        root.remove(control)
    for gazebo in root.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if 'ROS2Control' in plugin.get('name','') or 'PosePublisher' in plugin.get('name',''):
                gazebo.remove(plugin)
    wheel_joints = [root.find(f"joint[@name='{corner}_wheel_joint']") for corner in
                    ['front_left','rear_left','front_right','rear_right']]
    if any(j is None for j in wheel_joints):
        raise ValueError('Upstream wheel joint names changed')
    positions = [transform_to_link(root,j.find('child').get('link'))[:3,3] for j in wheel_joints]
    wheel_link = root.find(f"link[@name='{wheel_joints[0].find('child').get('link')}']")
    radius = float(wheel_link.find('collision/geometry/cylinder').get('radius'))
    fields = [('left_joint',j.get('name')) for j in wheel_joints[:2]]
    fields += [('right_joint',j.get('name')) for j in wheel_joints[2:]]
    fields += [('wheel_radius',radius),('wheel_separation',abs(positions[0][1]-positions[2][1])),
               ('topic','/cmd_vel'),('odom_topic','/navigation/wheel_odom'),
               ('tf_topic','/navigation/unused_drive_tf'),('frame_id','wheel_odom'),
               ('child_frame_id','base_link'),('max_linear_velocity',2.),('min_linear_velocity',-2.),
               ('max_angular_velocity',1.5),('min_angular_velocity',-1.5),
               ('max_linear_acceleration',.8),('min_linear_acceleration',-.8)]
    add_plugin(root,'DiffDrive','libignition-gazebo-diff-drive-system.so',fields)
    add_plugin(root,'OdometryPublisher','libignition-gazebo-odometry-publisher-system.so',[
        ('dimensions',3),('odom_topic','/ground_truth/odom'),('tf_topic','/ground_truth/tf'),
        ('odom_frame','world'),('robot_base_frame','base_link'),('odom_publish_frequency',50)])
    add_plugin(root,'JointStatePublisher','libignition-gazebo-joint-state-publisher-system.so',[])
    sensors = {}
    for gazebo in root.findall('gazebo'):
        for sensor in gazebo.findall('sensor'):
            sensors[sensor.get('type')] = dict(frame=gazebo.get('reference'),topic=sensor.findtext('topic'),name=sensor.get('name'))
    lidar = sensors['gpu_lidar']
    ground_z = min(p[2]-radius for p in positions)
    metadata = dict(lidar=lidar, sensor_height=float(transform_to_link(root,lidar['frame'])[2,3]-ground_z),
                    ground_z=float(ground_z),radius=radius,sensors=sensors)
    description = ET.tostring(root,encoding='unicode')
    (output/'navigation.urdf').write_text(description)
    return description,metadata
