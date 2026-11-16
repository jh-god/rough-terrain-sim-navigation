"""Preview the aerial image without starting Gazebo, controllers or Nav2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rough_terrain_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('metadata_file', default_value=os.path.join(share, 'assets', 'terrain_aerial.yaml')),
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        Node(package='rough_terrain_navigation', executable='aerial_background',
             name='aerial_background', output='screen', parameters=[{'use_sim_time': False,
             'metadata_file': LaunchConfiguration('metadata_file')}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='aerial_preview_frame', output='screen',
             arguments=['--frame-id', 'map', '--child-frame-id', 'aerial_preview_origin']),
        Node(package='rviz2', executable='rviz2', name='aerial_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'aerial.rviz')]),
    ])
