"""Reuse original world, Xacro and controllers without the legacy planar TF node."""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rough_terrain_navigation.navigation_modes import obstacle_detection_for_mode


def setup(context):
    sim = get_package_share_directory('rough_terrain_sim')
    controller = get_package_share_directory('fwmax_dual_ackermann_controller')
    own = get_package_share_directory('rough_terrain_navigation')
    value = lambda key: LaunchConfiguration(key).perform(context)
    obstacle_detection = obstacle_detection_for_mode(value('mode'), value('obstacle_detection'))
    description = ParameterValue(Command(['xacro ', os.path.join(sim, 'urdf', 'fwmax_skid_steer.urdf.xacro'),
        ' drive_mode:=swerve controllers_file:=', os.path.join(controller, 'config', 'fwmax_controllers.yaml')]), value_type=str)
    models = os.path.join(sim, 'models')
    resources = models + ':' + os.path.dirname(get_package_share_directory('clearpath_sensors_description'))
    plugin_paths = os.environ.get('LD_LIBRARY_PATH', '') + ':' + os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    args = ['ign', 'gazebo', '--force-version', '6', '-r', '-v', '3']
    if value('gui').lower() != 'true':
        args.append('-s')
    args.append(os.path.join(sim, 'worlds', 'rough_terrain.sdf'))
    common = dict(output='screen')
    return [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')+':'+resources),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.environ.get('GZ_SIM_RESOURCE_PATH', '')+':'+resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')+':'+plugin_paths),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')+':'+plugin_paths),
        ExecuteProcess(cmd=args, name='rough_terrain_gazebo', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='fwmax_robot_state_publisher', parameters=[{'use_sim_time': True, 'robot_description': description}], **common),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='navigation_clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], **common),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='fwmax_ground_truth_bridge',
             parameters=[{'config_file': os.path.join(own, 'config', 'ground_truth_bridge.yaml')}], **common),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='fwmax_sensor_bridge',
             namespace='sensors', parameters=[{'use_sim_time': True,
                 'config_file': os.path.join(own, 'config', 'lidar_bridge.yaml')}],
             condition=IfCondition(str(obstacle_detection).lower()), **common),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='fwmax_camera_bridge',
             namespace='sensors', parameters=[{'use_sim_time': True,
                 'config_file': os.path.join(own, 'config', 'camera_bridge.yaml')}],
             condition=IfCondition(LaunchConfiguration('camera')), **common),
        Node(package='ros_gz_image', executable='image_bridge', name='fwmax_image_bridge',
             condition=IfCondition(LaunchConfiguration('camera')),
             arguments=['/sensors/camera/image', '/sensors/camera/depth_image'],
             remappings=[('/sensors/camera/image', '/sensors/camera/color/image'),
                         ('/sensors/camera/depth_image', '/sensors/camera/depth/image')],
             parameters=[{'use_sim_time': True}], **common),
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_gazebo_frame',
             arguments=['--frame-id', 'camera_link', '--child-frame-id', 'fwmax/base_link/camera'],
             parameters=[{'use_sim_time': True}], **common),
        Node(package='rough_terrain_sim', executable='camera_pointcloud_frame_fix',
             name='camera_pointcloud_frame_fix', condition=IfCondition(LaunchConfiguration('camera')), parameters=[{'use_sim_time': True,
             'input_topic': '/sensors/camera/points', 'output_topic': '/sensors/camera/points_aligned',
             'input_frame': 'camera_color_optical_frame', 'output_frame': 'camera_link'}], **common),
        TimerAction(period=3., actions=[Node(package='ros_gz_sim', executable='create', name='fwmax_spawn',
            arguments=['-world', 'rough_terrain_world', '-name', 'fwmax',
                       '-x', value('robot_x'), '-y', value('robot_y'), '-z', value('robot_z'),
                       '-Y', value('robot_yaw'), '-topic', 'robot_description'], **common)]),
        TimerAction(period=4., actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=[name, '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],
                 name=name+'_spawner', **common)
            for name in ['fwmax_steering_controller', 'fwmax_wheel_controller']
        ] + [Node(package='fwmax_dual_ackermann_controller', executable='dual_ackermann_controller',
                  name='fwmax_dual_ackermann_controller', parameters=[{'use_sim_time': True}], **common)]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(own, 'launch', 'navigation.launch.py')),
            launch_arguments={'mode': value('mode'), 'obstacle_detection': value('obstacle_detection'), 'rviz': value('rviz'), 'rmw_implementation': value('rmw_implementation'), 'params_file': value('params_file'),
                              'robot_params': value('robot_params'), 'patchwork_params': value('patchwork_params'),
                              'aerial_background': value('aerial_background'), 'aerial_metadata': value('aerial_metadata'),
                              **{key: value(key) for key in ['width', 'height', 'resolution', 'origin_x', 'origin_y', 'boundary']}}.items()),
    ]


def generate_launch_description():
    share = get_package_share_directory('rough_terrain_navigation')
    # User-selected map point; clicked map z is not the terrain elevation.
    # Ground elevation is 1.24483 m; base_footprint starts ~5.5 cm above it.
    defaults = dict(mode='perception', obstacle_detection='auto', aerial_background='true', aerial_metadata=os.path.join(share, 'assets', 'terrain_aerial.yaml'),
                    gui='true', rviz='true', camera='false', rmw_implementation='rmw_fastrtps_cpp', robot_x='-15.259527206420898', robot_y='21.87224578857422', robot_z='1.30', robot_yaw='0.0',
                    width='50.0', height='50.0', resolution='0.1', origin_x='-25.0', origin_y='-25.0', boundary='0.3',
                    params_file=os.path.join(share, 'config', 'nav2.yaml'),
                    robot_params=os.path.join(share, 'config', 'fwmax_swerve.yaml'),
                    patchwork_params=os.path.join(share, 'config', 'patchworkpp.yaml'))
    descriptions = {
        'mode': 'perception: map and sensor obstacle avoidance; map_only: map without obstacle detection',
        'obstacle_detection': 'Legacy override: auto follows mode, true/false overrides it',
    }
    return LaunchDescription([DeclareLaunchArgument(key, default_value=val,
                                                   description=descriptions.get(key, ''))
                              for key, val in defaults.items()] + [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        OpaqueFunction(function=setup)])
