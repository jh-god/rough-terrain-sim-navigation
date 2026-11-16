# RViz 지형 탑뷰 배경

기본 `simulation.launch.py`와 `navigation.launch.py`에서 컬러 탑뷰를 자동 표시합니다.
`aerial_background:=false`로 배경 발행만 끌 수 있습니다. RViz의 Displays →
Aerial terrain background 체크박스로 표시를 켜고 끌 수도 있습니다.
기존 빈 지도 표시와 Grid는 배경을 가리지 않도록 기본 비활성화했습니다.

## 구성

- `assets/terrain_aerial.png`: 2048×2048 직교 탑뷰. 약 2.44 cm/pixel.
- `assets/terrain_aerial.dae`: 이미지를 붙이는 단위 XY 평면.
- `assets/terrain_aerial.yaml`: 50×50 m, 원점 (-25, -25), 좌표 방향과 원본 해시.
- `aerial_background`: `/navigation/aerial_background`에 embedded texture mesh Marker를
  reliable / transient-local로 발행. 늦게 열린 RViz도 수신합니다.
- `map` 프레임, z=-0.20 m에 배치하여 z=0의 경로와 costmap 아래에 표시합니다.
  Global/Local costmap의 `Draw Behind`는 false, Alpha는 각각 0.55/0.85입니다.
  이미지 오른쪽은 +X, 위쪽은 +Y입니다. RViz 기본 시점은 전체 시험장 탑뷰입니다.

원본 rough_terrain_sim의 heightmap, 흙 텍스처, 나무/그루터기/바위 OBJ와 SDF pose를
Blender에서 재구성한 정적 렌더입니다. 실제 위성사진이나 Gazebo 화면 캡처는 아니며,
렌더러 차이로 색감/그림자가 Gazebo와 조금 다를 수 있습니다.
지면은 약 0.1 m 간격으로 재구성합니다. 2D 바닥 그림이므로 실제 높이 표시는 아닙니다.
로봇과 움직이는 물체는 이 정적 이미지에 포함하지 않습니다.

이 이미지는 시각화용이며 map_server나 costmap의 입력이 아닙니다.
나무/돌은 기존과 같이 센서로 관측해서 회피합니다.
Nav2 알고리즘, 속도 설정, 로봇 스폰 위치와 다른 패키지 소스는 변경하지 않았습니다.

## Gazebo 없이 배경 확인

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rough_terrain_navigation aerial_view.launch.py
```

별도 실행 중인 navigation에 연결할 때는 그 터미널과 ROS_DOMAIN_ID 및 RMW를 맞추세요.
기본 RMW는 rmw_fastrtps_cpp입니다. standalone launch는 시뮬레이션 clock이 필요 없습니다.

## 시험장 변경 시 재생성

현재 기본 50×50 m 시험장을 기준으로 생성되어 있습니다. 재생성 도구는 정사각형 시험장을 지원합니다. 다른 크기의 빈 지도를 만들어도
배경이 자동으로 늘어나지는 않습니다. 원본 지형/장애물 배치를 바꾸면 재생성하세요.
실행 시에는 Blender가 필요 없고 PNG/DAE/YAML만 사용합니다.
재생성에는 Blender의 OBJ importer와 Cycles, Python numpy/Pillow/PyYAML이 필요합니다.
현재 환경의 Blender 5.2에서 생성 확인했습니다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rough_terrain_navigation generate_aerial_map \
  --simulation-share ~/ros2_ws/src/rough_terrain_sim \
  --output-dir ~/ros2_ws/src/rough_terrain_navigation/assets
cd ~/ros2_ws
colcon build --packages-select rough_terrain_navigation --symlink-install
```

사용자 지정 파일은 `aerial_metadata:=/absolute/path/terrain_aerial.yaml`로 지정합니다.
동일 디렉터리에 해당 DAE와 텍스처를 함께 두세요. 자산 출처와 라이선스는
`assets/SOURCES.txt`에 기록했습니다.
