**English** | [한국어](README.ko.md)

# rough_terrain_navigation

## Overview

A goal navigation package for FWmax swerve, Husky A200, and Jackal J100 in ROS 2 Humble / Gazebo Fortress.
It reuses the terrain and robot models from `rough_terrain_sim`.
FWmax uses its existing controller; Husky and Jackal use Gazebo DiffDrive.

The package uses Gazebo ground-truth 3D poses for localization and Nav2 for planar path planning and tracking.
LiDAR and Patchwork++ separate the ground and add obstacles such as trees and rocks to the costmaps.
An empty map matching the test area replaces SLAM, and RViz displays a color top view of the terrain.

FWmax also publishes a 200 Hz virtual IMU on `/sensors/imu/data`, fixed to `base_link`
via `imu_link`. Gyroscope and accelerometer noise are configured in the simulator URDF.

## Build

These instructions assume the package is in `~/ros2_ws/src` and that the required dependencies,
including `rough_terrain_sim`, `fwmax_dual_ackermann_controller`, `patchworkpp`, and Nav2,
are already installed or built. Husky and Jackal also require
`clearpath_generator_common` and `clearpath_platform_description`.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select rough_terrain_sim rough_terrain_navigation --symlink-install
source install/setup.bash
```

Rebuild both packages after changing a URDF or mesh to update the installed assets.

## Launch

### Run simulation and navigation together

Launch Gazebo, robot spawning, sensor bridges, controllers, Nav2, and RViz together.

```bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
ros2 launch rough_terrain_navigation simulation.launch.py
```

Use the **Navigation 2 Goal** tool in RViz to set a goal.
Set the same RMW implementation and Domain ID in other terminals that run ROS commands.
Domain ID 42 is an example.

Append any of the following options to the launch command:

| Option | Description |
|---|---|
| `gui:=false rviz:=false` | Run without the Gazebo GUI or RViz |
| `robot:=fwmax` | FWmax swerve (default) |
| `robot:=husky` | Husky A200 with a 128-channel Ouster LiDAR |
| `robot:=jackal` | Jackal J100 with a VLP-16 |
| `mode:=perception` | Enable sensor-based obstacle detection and avoidance (default) |
| `mode:=map_only` | Navigate to goals or waypoints on an empty map without obstacle detection |
| `camera:=auto` | Default. Publish the attached RGB-D camera (FWmax/Husky); use `false` to disable its ROS output |
| `aerial_background:=false` | Disable the color terrain background |
| `robot_x:=... robot_y:=... robot_z:=... robot_yaw:=...` | Set the initial robot position and heading, in metres and radians |

Robot selection examples:

```bash
ros2 launch rough_terrain_navigation simulation.launch.py robot:=husky mode:=map_only
ros2 launch rough_terrain_navigation simulation.launch.py robot:=jackal mode:=perception
```

Each launch runs one selected robot. Stop the current launch before switching robots.
The default `robot_z:=auto` accounts for each robot's root-link height at the default starting point.
An explicit `robot_z` sets the world z coordinate.
FWmax and Husky have RGB-D cameras; the default Jackal model does not.
Do not run the original `rough_terrain_sim` launch alongside this integrated launch.

### Navigate on the map without obstacle detection

```bash
ros2 launch rough_terrain_navigation simulation.launch.py mode:=map_only
```

Use a terminal with the workspace sourced and the same RMW/Domain ID settings as above.
In the RViz Navigation 2 panel, select **Waypoint / Nav Through Poses Mode**,
place points in sequence with **Nav2 Goal**, and click **Start Waypoint Following**.
Nav2 plans paths on the empty map and tracks them with the existing controller.

This mode disables Patchwork++, ground separation, and the global/local obstacle layers.
ROS output from attached sensors, including LiDAR, cameras, IMU, and GPS, remains available in either mode.
The test-area boundary remains enabled. Trees and rocks retain their physical collisions,
but the robot will not avoid obstacles along its path in this mode.

Select `mode:=perception` to enable sensor-based obstacle avoidance; this is also the default when the argument is omitted.
`navigation.launch.py` accepts the same `mode` argument.
Both modes support single goals and multiple waypoints.
The `mode` argument selects obstacle perception behavior, independently of the goal input method.
The existing `waypoint_simulation.launch.py` remains available as a compatibility launch for `map_only`.

### Run navigation only

Use this when the robot, sensors, and Gazebo ground-truth data are already available.
Select the matching robot profile with `robot:=fwmax`, `robot:=husky`, or `robot:=jackal`.
The setup must avoid TF conflicts with an existing `fwmax_planar_odometry` node.
Use the same RMW and Domain ID settings as above.

```bash
ros2 launch rough_terrain_navigation navigation.launch.py
```

### Preview the color terrain background

View the terrain top view in RViz without Gazebo or Nav2.

```bash
ros2 launch rough_terrain_navigation aerial_view.launch.py
```
