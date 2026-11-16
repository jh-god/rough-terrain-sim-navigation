"""Navigation only; requires FW-max sensors/ground-truth, no competing physical TF."""
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


def setup(context):
    share = get_package_share_directory('rough_terrain_navigation')
    value = lambda key: LaunchConfiguration(key).perform(context)
    directory = tempfile.mkdtemp(prefix='rough_terrain_navigation_')
    map_file = generate_map(directory, **{key: float(value(key)) for key in
        ['width', 'height', 'resolution', 'origin_x', 'origin_y', 'boundary']})
    params = yaml.safe_load(open(value('params_file'), encoding='utf-8'))
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
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}, **common),
        Node(package='patchworkpp', executable='demo', name='patchworkpp',
             parameters=[value('patchwork_params')],
             remappings=[('ground', '/navigation/patchwork/ground'),
                         ('nonground', '/navigation/patchwork/nonground'),
                         ('cloud', '/navigation/patchwork/cloud'),
                         ('plane', '/navigation/patchwork/debug')], **common),
        Node(package='rough_terrain_navigation', executable='command_gate',
             name='command_gate', parameters=[profile], **common),
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
    defaults = dict(rviz='true', aerial_background='true',
                    aerial_metadata=os.path.join(share, 'assets', 'terrain_aerial.yaml'), rmw_implementation='rmw_fastrtps_cpp', width='50.0', height='50.0', resolution='0.1',
                    origin_x='-25.0', origin_y='-25.0', boundary='0.3',
                    params_file=os.path.join(share, 'config', 'nav2.yaml'),
                    robot_params=os.path.join(share, 'config', 'fwmax_swerve.yaml'),
                    patchwork_params=os.path.join(share, 'config', 'patchworkpp.yaml'))
    return LaunchDescription([DeclareLaunchArgument(key, default_value=val)
                              for key, val in defaults.items()] + [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        OpaqueFunction(function=setup)])
