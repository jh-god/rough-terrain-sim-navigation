# rough_terrain_navigation

## 패키지 소개

ROS 2 Humble / Gazebo Fortress 환경에서 FWmax swerve 로봇의 목표점 주행을 위한 패키지입니다.
`rough_terrain_sim`의 지형·로봇 모델과 기존 제어기를 재사용합니다.

Gazebo의 실제 3D 자세를 받아 Nav2로 평면 경로를 계획·추종하며,
LiDAR와 Patchwork++로 지면을 분리하여 나무·바위 등의 장애물을 costmap에 반영합니다.
SLAM 없이 시험장 크기에 맞는 빈 지도를 사용하고, RViz에는 컬러 지형 탑뷰를 표시합니다.

## 빌드

`~/ros2_ws/src`에 이 패키지가 있으며, `rough_terrain_sim`,
`fwmax_dual_ackermann_controller`, `patchworkpp`와 Nav2 등 필요한 의존성이
설치·빌드되어 있는 환경을 기준으로 합니다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select rough_terrain_navigation --symlink-install
source install/setup.bash
```

## Launch 실행

### 시뮬레이션과 navigation 통합 실행

Gazebo, 로봇 스폰, 센서 브리지, 제어기, Nav2, RViz를 함께 실행합니다.

```bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
ros2 launch rough_terrain_navigation simulation.launch.py
```

실행 후 RViz의 **Navigation 2 Goal** 도구로 목표점을 지정합니다.
다른 터미널에서 ROS 명령을 사용할 때도 같은 RMW와 Domain ID를 설정하세요.
Domain ID 42는 예시입니다.

필요한 옵션을 실행 명령 뒤에 추가할 수 있습니다.

| 옵션 | 설명 |
|---|---|
| `gui:=false rviz:=false` | Gazebo GUI와 RViz 없이 실행 |
| `camera:=true` | 카메라 RGB·Depth·PointCloud 브리지 활성화 |
| `aerial_background:=false` | 컬러 지형 배경 비활성화 |
| `robot_x:=... robot_y:=... robot_z:=... robot_yaw:=...` | 로봇 초기 위치와 방향 지정. 단위는 m, rad |

기존 `rough_terrain_sim` 런치와 통합 런치를 동시에 실행하지 마세요.

### Navigation만 실행

로봇·센서·Gazebo ground truth가 이미 제공되는 환경에서 실행합니다.
기존 `fwmax_planar_odometry` 노드와 TF가 충돌하지 않도록 준비되어 있어야 합니다.
아래 명령도 위와 동일한 RMW와 Domain ID 환경에서 사용합니다.

```bash
ros2 launch rough_terrain_navigation navigation.launch.py
```

### 컬러 지형 배경만 확인

Gazebo와 Nav2 없이 RViz에서 지형 탑뷰를 확인합니다.

```bash
ros2 launch rough_terrain_navigation aerial_view.launch.py
```
