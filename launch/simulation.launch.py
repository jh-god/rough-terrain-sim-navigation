"""Spawn a selected upstream robot without competing planar odometry TF."""
import os
import tempfile
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rough_terrain_navigation.navigation_modes import obstacle_detection_for_mode
from rough_terrain_navigation.platforms import platform, clearpath_description


def setup(context):
    sim = get_package_share_directory('rough_terrain_sim')
    own = get_package_share_directory('rough_terrain_navigation')
    value = lambda key: LaunchConfiguration(key).perform(context)
    robot = value('robot')
    spec = platform(robot)
    detection = obstacle_detection_for_mode(value('mode'), value('obstacle_detection'))
    work = Path(tempfile.mkdtemp(prefix='rough_terrain_robot_'+robot+'_'))
    if robot == 'fwmax':
        controller = get_package_share_directory('fwmax_dual_ackermann_controller')
        description = ParameterValue(Command(['xacro ', os.path.join(sim,'urdf','fwmax_skid_steer.urdf.xacro'),
            ' drive_mode:=swerve controllers_file:=', os.path.join(controller,'config','fwmax_controllers.yaml')]),value_type=str)
        ground_bridge = os.path.join(own,'config','ground_truth_bridge.yaml')
        lidar_bridge = os.path.join(own,'config','lidar_bridge.yaml')
        camera_bridge = os.path.join(own,'config','camera_bridge.yaml')
        camera_topic,camera_frame,camera_name = '/sensors/camera','camera_link','camera'
    else:
        xml, metadata = clearpath_description(robot,sim,work)
        description = ParameterValue(xml,value_type=str)
        def bridge(path, entries):
            file = work/path
            file.write_text(yaml.safe_dump(entries))
            return str(file)
        def entry(ros,gz,ros_type,gz_type,direction='GZ_TO_ROS'):
            return dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=ros_type,
                        gz_type_name='ignition.msgs.'+gz_type,direction=direction,
                        subscriber_queue=1,publisher_queue=1)
        ground_bridge = bridge('state_bridge.yaml',[
            entry('/ground_truth/odom','/ground_truth/odom','nav_msgs/msg/Odometry','Odometry'),
            entry('/joint_states',f'/world/rough_terrain_world/model/{robot}/joint_state','sensor_msgs/msg/JointState','Model'),
            entry('/cmd_vel','/cmd_vel','geometry_msgs/msg/Twist','Twist','ROS_TO_GZ')])
        lidar_bridge = bridge('lidar_bridge.yaml',[
            entry('/sensors/lidar3d_0/points',metadata['lidar']['topic']+'/points','sensor_msgs/msg/PointCloud2','PointCloudPacked')])
        camera = metadata['sensors'].get('rgbd_camera')
        if value('camera').lower() == 'true' and camera is None:
            raise ValueError(robot+' upstream configuration has no RGB-D camera; use camera:=false')
        camera_topic = camera['topic'] if camera else '/unused_camera'
        camera_frame = camera['frame'] if camera else 'base_link'
        camera_name = camera['name'] if camera else 'camera'
        camera_bridge = bridge('camera_bridge.yaml',[
            entry('/sensors/camera/color/camera_info',camera_topic+'/camera_info','sensor_msgs/msg/CameraInfo','CameraInfo'),
            entry('/sensors/camera/depth/camera_info',camera_topic+'/camera_info','sensor_msgs/msg/CameraInfo','CameraInfo'),
            entry('/sensors/camera/points',camera_topic+'/points','sensor_msgs/msg/PointCloud2','PointCloudPacked')])
    resources = os.path.join(sim,'models')+':'+os.path.dirname(get_package_share_directory('clearpath_sensors_description'))
    plugin_paths = os.environ.get('LD_LIBRARY_PATH','')+':'+os.path.join(get_package_prefix('gz_ros2_control'),'lib')
    args = ['ign','gazebo','--force-version','6','-r','-v','3']
    if value('gui').lower() != 'true': args.append('-s')
    args.append(os.path.join(sim,'worlds','rough_terrain.sdf'))
    common = dict(output='screen')
    nodes = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')+':'+resources),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',os.environ.get('GZ_SIM_RESOURCE_PATH','')+':'+resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH','')+':'+plugin_paths),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')+':'+plugin_paths),
        ExecuteProcess(cmd=args,name='rough_terrain_gazebo',output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',name=robot+'_robot_state_publisher',
             parameters=[{'use_sim_time':True,'robot_description':description}],**common),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='navigation_clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],**common),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name=robot+'_ground_truth_bridge',
             parameters=[{'config_file':ground_bridge}],**common),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name=robot+'_sensor_bridge',
             namespace='sensors',parameters=[{'use_sim_time':True,'config_file':lidar_bridge}],
             condition=IfCondition(str(detection).lower()),**common),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name=robot+'_camera_bridge',
             namespace='sensors',parameters=[{'use_sim_time':True,'config_file':camera_bridge}],
             condition=IfCondition(LaunchConfiguration('camera')),**common),
        Node(package='ros_gz_image',executable='image_bridge',name=robot+'_image_bridge',
             arguments=[camera_topic+'/image',camera_topic+'/depth_image'],
             remappings=[(camera_topic+'/image','/sensors/camera/color/image'),
                         (camera_topic+'/depth_image','/sensors/camera/depth/image')],
             parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('camera')),**common),
        Node(package='tf2_ros',executable='static_transform_publisher',name='camera_gazebo_frame',
             arguments=['--frame-id',camera_frame,'--child-frame-id',robot+'/base_link/'+camera_name],
             parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('camera')),**common),
        Node(package='rough_terrain_sim',executable='camera_pointcloud_frame_fix',name='camera_pointcloud_frame_fix',
             condition=IfCondition(LaunchConfiguration('camera')),parameters=[{'use_sim_time':True,
             'input_topic':'/sensors/camera/points','output_topic':'/sensors/camera/points_aligned',
             'input_frame':camera_name+'_color_optical_frame','output_frame':camera_frame}],**common),
        TimerAction(period=3.,actions=[Node(package='ros_gz_sim',executable='create',name=robot+'_spawn',
             arguments=['-world','rough_terrain_world','-name',robot,'-x',value('robot_x'),'-y',value('robot_y'),
                        '-z',str(spec['spawn_z']) if value('robot_z')=='auto' else value('robot_z'),
                        '-Y',value('robot_yaw'),'-topic','robot_description'],**common)]),
    ]
    if robot == 'fwmax':
        nodes.append(TimerAction(period=4.,actions=[Node(package='controller_manager',executable='spawner',
            arguments=[name,'--controller-manager','/controller_manager','--controller-manager-timeout','60'],
            name=name+'_spawner',**common) for name in ['fwmax_steering_controller','fwmax_wheel_controller']]+[
            Node(package='fwmax_dual_ackermann_controller',executable='dual_ackermann_controller',
                 name='fwmax_dual_ackermann_controller',parameters=[{'use_sim_time':True}],**common)]))
    forwarded = ['robot','mode','obstacle_detection','rviz','rmw_implementation','params_file',
                 'robot_params','patchwork_params','aerial_background','aerial_metadata',
                 'width','height','resolution','origin_x','origin_y','boundary']
    nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(own,'launch','navigation.launch.py')),
        launch_arguments={key:value(key) for key in forwarded}.items()))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('rough_terrain_navigation')
    defaults = dict(robot='fwmax',mode='perception',obstacle_detection='auto',aerial_background='true',
        aerial_metadata=os.path.join(share,'assets','terrain_aerial.yaml'),gui='true',rviz='true',camera='false',
        rmw_implementation='rmw_fastrtps_cpp',robot_x='-15.259527206420898',robot_y='21.87224578857422',
        robot_z='auto',robot_yaw='0.0',width='50.0',height='50.0',resolution='0.1',origin_x='-25.0',origin_y='-25.0',
        boundary='0.3',params_file=os.path.join(share,'config','nav2.yaml'),robot_params='auto',patchwork_params='auto')
    descriptions = {'robot':'fwmax, husky or jackal', 'mode':'perception or map_only',
                    'robot_z':'auto selects the robot root-link spawn height; explicit values are metres'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v,description=descriptions.get(k,''))
        for k,v in defaults.items()]+[SetEnvironmentVariable('RMW_IMPLEMENTATION',LaunchConfiguration('rmw_implementation')),
        OpaqueFunction(function=setup)])
