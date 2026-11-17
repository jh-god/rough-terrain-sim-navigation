from glob import glob
import os
from setuptools import find_packages, setup

name = 'rough_terrain_navigation'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md', 'README.ko.md', 'LICENSE']),
    ] + [('share/' + name + '/' + folder, [f for f in glob(folder + '/*') if os.path.isfile(f)])
         for folder in ['launch', 'config', 'behavior_trees', 'rviz', 'docs', 'assets']],
    install_requires=['setuptools', 'numpy', 'PyYAML'], tests_require=['pytest'], zip_safe=True,
    maintainer='rough_terrain_navigation maintainers',
    maintainer_email='maintainer@example.com', license='Apache-2.0',
    description='Ground-truth 6DoF FWmax, Husky and Jackal navigation with Patchwork++ and Nav2.',
    entry_points={'console_scripts': [
        'simulation_state_adapter = rough_terrain_navigation.state_adapter:main',
        'ground_pipeline = rough_terrain_navigation.ground_pipeline:main',
        'command_gate = rough_terrain_navigation.command_gate:main',
        'send_goal = rough_terrain_navigation.send_goal:main',
        'generate_empty_map = rough_terrain_navigation.maps:main',
        'aerial_background = rough_terrain_navigation.aerial_background:main',
        'generate_aerial_map = rough_terrain_navigation.aerial_assets:main',
    ]},
)
