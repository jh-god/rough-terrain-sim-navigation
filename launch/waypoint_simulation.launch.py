"""Compatibility launcher for map_only navigation; both modes support waypoints."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('rough_terrain_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='map_only',
                              description='Navigation mode: map_only or perception'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'simulation.launch.py'))),
    ])
