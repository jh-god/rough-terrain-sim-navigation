"""Waypoint navigation on the empty map; detection is off by default."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('rough_terrain_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('obstacle_detection', default_value='false',
                              description='Enable LiDAR ground segmentation and obstacle layers'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'simulation.launch.py'))),
    ])
