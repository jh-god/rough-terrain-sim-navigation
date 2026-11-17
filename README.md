# rough_terrain_navigation

## 패키지 소개

ROS 2 Humble / Gazebo Fortress 환경에서 FWmax swerve, Husky A200, Jackal J100의 목표점 주행을 위한 패키지입니다.
`rough_terrain_sim`의 지형·로봇 모델을 재사용합니다.
FWmax는 기존 제어기, Husky·Jackal은 Gazebo DiffDrive로 구동합니다.

Gazebo의 실제 3D 자세를 받아 Nav2로 평면 경로를 계획·추종하며,
LiDAR와 Patchwork++로 지면을 분리하여 나무·바위 등의 장애물을 costmap에 반영합니다.
SLAM 없이 시험장 크기에 맞는 빈 지도를 사용하고, RViz에는 컬러 지형 탑뷰를 표시합니다.

## 빌드

`~/ros2_ws/src`에 이 패키지가 있으며, `rough_terrain_sim`,
`fwmax_dual_ackermann_controller`, `patchworkpp`와 Nav2 등 필요한 의존성이
설치·빌드되어 있는 환경을 기준으로 합니다. Husky·Jackal은
`clearpath_generator_common`, `clearpath_platform_description`도 필요합니다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select rough_terrain_sim rough_terrain_navigation --symlink-install
source install/setup.bash
```

URDF나 mesh를 변경한 경우에도 두 패키지를 함께 빌드하여 설치 경로에 반영하세요.

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
| `robot:=fwmax` | FWmax swerve (기본값) |
| `robot:=husky` | Husky A200 + Ouster 128채널 |
| `robot:=jackal` | Jackal J100 + VLP-16 |
| `mode:=perception` | 센서 기반 장애물 탐지·회피 사용 (기본값) |
| `mode:=map_only` | 장애물 탐지 없이 빈 지도에서 목표점·waypoint 주행 |
| `camera:=true` | 카메라 RGB·Depth·PointCloud 브리지 활성화 |
| `aerial_background:=false` | 컬러 지형 배경 비활성화 |
| `robot_x:=... robot_y:=... robot_z:=... robot_yaw:=...` | 로봇 초기 위치와 방향 지정. 단위는 m, rad |

로봇 선택 예시:

```bash
ros2 launch rough_terrain_navigation simulation.launch.py robot:=husky mode:=map_only
ros2 launch rough_terrain_navigation simulation.launch.py robot:=jackal mode:=perception
```

한 번의 실행에서 로봇 한 대를 선택합니다. 로봇을 바꿀 때는 기존 런치를 종료하세요.
`robot_z` 기본값 auto는 기본 시작점에서 로봇별 root link 높이를 고려합니다. 명시하면 world z 좌표를 사용합니다.
카메라는 FWmax와 Husky에서 지원하며 Jackal 기본 모델에는 RGB-D 카메라가 없습니다.
기존 `rough_terrain_sim` 런치와 통합 런치를 동시에 실행하지 마세요.

### 지도 기반 주행: 장애물 탐지 없이 실행

```bash
ros2 launch rough_terrain_navigation simulation.launch.py mode:=map_only
```

위와 동일하게 workspace를 source하고 RMW/Domain ID를 설정한 터미널에서 실행합니다.
RViz의 Navigation 2 패널에서 **Waypoint / Nav Through Poses Mode**를 누른 뒤,
**Nav2 Goal** 도구로 지점을 순서대로 지정하고 **Start Waypoint Following**을 누릅니다.
각 지점까지 빈 지도 위에 경로를 생성하고 기존 Nav2 제어기로 추종합니다.

이 런치는 Patchwork++, 지면 분리, LiDAR 브리지와 Global/Local 장애물 layer를 끕니다.
시험장 경계는 유지합니다. 나무·돌의 물리 collision은 남아 있으므로 경로상의 장애물을 회피하지 않습니다.
`mode:=perception`으로 기존 센서 기반 회피 모드를 선택할 수 있습니다.
인자를 생략하면 기존처럼 `perception` 모드로 실행합니다.
`navigation.launch.py`도 같은 `mode` 인자를 지원합니다.
두 모드 모두 단일 목표와 여러 waypoint 주행을 지원합니다.
`mode`는 목표 입력 방식이 아닌 장애물 인식 사용 여부를 선택합니다.
기존 `waypoint_simulation.launch.py`는 `map_only` 모드의 호환 런치로 유지합니다.

### Navigation만 실행

로봇·센서·Gazebo ground truth가 이미 제공되는 환경에서 실행합니다.
`robot:=fwmax|husky|jackal`로 해당 로봇의 프로파일을 선택할 수 있습니다.
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
