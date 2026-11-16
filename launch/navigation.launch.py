"""Navigation on Gazebo ground truth with optional sensor obstacle perception."""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from rough_terrain_navigation.maps import generate_map
from rough_terrain_navigation.navigation_modes import nav2_for_mode, obstacle_detection_for_mode


def setup(context):
    share = get_package_share_directory('rough_terrain_navigation')
    value = lambda key: LaunchConfiguration(key).perform(context)
    obstacle_detection = obstacle_detection_for_mode(value('mode'), value('obstacle_detection'))
    directory = tempfile.mkdtemp(prefix='rough_terrain_navigation_')
    map_file = generate_map(directory, **{key: float(value(key)) for key in
        ['width', 'height', 'resolution', 'origin_x', 'origin_y', 'boundary']})
    params = yaml.safe_load(open(value('params_file'), encoding='utf-8'))
    params = nav2_for_mode(params, obstacle_detection)
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = os.path.join(share, 'behavior_trees', 'navigate.xml')
    for costmap in ['global_costmap', 'local_costmap']:
        params[costmap][costmap]['ros__parameters']['resolution'] = float(value('resolution'))
    generated_params = os.path.join(directory, 'nav2.yaml')
    with open(generated_params, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
    profile = value('robot_params')
    common = dict(output='screen')
    nodes = [
        Node(package='rough_terrain_navigation', executable='aerial_background',
             name='aerial_background', parameters=[{'use_sim_time': True,
                 'metadata_file': value('aerial_metadata')}],
             condition=IfCondition(LaunchConfiguration('aerial_background')), **common),
        Node(package='rough_terrain_navigation', executable='simulation_state_adapter',
             name='simulation_state_adapter', parameters=[profile], **common),
        Node(package='rough_terrain_navigation', executable='ground_pipeline',
             name='ground_pipeline', parameters=[profile],
             condition=IfCondition(str(obstacle_detection).lower()),
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}, **common),
        Node(package='patchworkpp', executable='demo', name='patchworkpp',
             parameters=[value('patchwork_params')],
             condition=IfCondition(str(obstacle_detection).lower()),
             remappings=[('ground', '/navigation/patchwork/ground'),
                         ('nonground', '/navigation/patchwork/nonground'),
                         ('cloud', '/navigation/patchwork/cloud'),
                         ('plane', '/navigation/patchwork/debug')], **common),
        Node(package='rough_terrain_navigation', executable='command_gate',
             name='command_gate', parameters=[profile, {'require_cloud': obstacle_detection}], **common),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': True, 'yaml_filename': map_file}], **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', parameters=[{'use_sim_time': True,
             'autostart': True, 'node_names': ['map_server']}], **common),
        GroupAction(actions=[
            # Every command source, including recoveries, passes through smoother and gate.
            SetRemap(src='behavior_server:cmd_vel', dst='/cmd_vel_nav'),
            SetRemap(src='velocity_smoother:cmd_vel_smoothed', dst='/navigation/cmd_vel_smoothed'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
                launch_arguments={'params_file': generated_params, 'use_sim_time': 'true',
                                  'autostart': 'true', 'use_composition': 'False'}.items()),
        ]),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz',
             arguments=['-d', os.path.join(share, 'rviz', 'navigation.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')), **common),
    ]
    return nodes


def generate_launch_description():
    share = get_package_share_directory('rough_terrain_navigation')
    defaults = dict(mode='perception', obstacle_detection='auto', rviz='true', aerial_background='true',
                    aerial_metadata=os.path.join(share, 'assets', 'terrain_aerial.yaml'), rmw_implementation='rmw_fastrtps_cpp', width='50.0', height='50.0', resolution='0.1',
                    origin_x='-25.0', origin_y='-25.0', boundary='0.3',
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
