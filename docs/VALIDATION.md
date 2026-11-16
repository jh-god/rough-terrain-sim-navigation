# 구현 및 검증 기록

검증일: 2026-11-13 (Asia/Seoul)
대상: ROS 2 Humble, Gazebo Fortress, 원본 FW-max swerve, 기존 나무/돌 유지.
외부 패키지 소스는 변경하지 않았습니다.

## 자동 검사

- `colcon build --packages-select rough_terrain_navigation --symlink-install`: 성공.
- `colcon test --packages-select rough_terrain_navigation --event-handlers console_direct+`: pytest 6개 통과.
- 실제 roll/pitch가 있는 6DoF pose의 planar/residual 분해 및 재합성.
- 빈 지도 경계/원점/내부 free 확인.
- XYZI round-trip, big-endian/organized/row-padding 점군 입력.
- 좌표 평면 투영 및 거리 보존.
- 오래된 센서 시각/수신 단절에 대한 명령 gate 거부.
- Patchwork++ 두 결과의 대응과 소수 초 timestamp 복원.

## 최종 통합 실행

`simulation.launch.py gui:=false rviz:=false`를 별도 ROS Domain/IGN partition에서 실행했습니다.
사용자 환경은 CycloneDDS였으며 launch가 새 프로세스의 RMW를 Fast DDS로 설정하는 경로도 확인했습니다.

- robot_state_publisher 및 ros2_control의 조향/바퀴 controller 활성화 성공.
- map_server 및 Nav2 lifecycle 노드 전체 활성화 성공.
- Patchwork++ 실제 LiDAR 입력 및 ground/nonground 분리 결과 수신.
- 정상 구간에서 Gazebo/planar odometry 약 50 Hz; 측정 시각은 clock보다 약 0.02 s 이내 뒤처짐.
- 지면 분리 후 costmap 입력은 표본 구간에서 약 5-7 Hz. 센서 및 렌더링/CPU 부하에 따라 변동.
- 빈 `/map`은 500 x 500이며 경계에서 0.5 m 이상 떨어진 내부의 occupied/nonzero cell은 0.
- 같은 시점 global costmap 내부 occupied cell은 2239, local costmap 내부 occupied cell은 49.
  즉, 사전 장애물 없이 센서 관측이 비용지도에 반영되는 것을 확인했습니다.
  수치는 특정 관측 시점의 값이며 장애물 검출 정확도를 뜻하지 않습니다.
- 시작 약 (-7.20, -3.20)에서 goal (-6.20, -3.20)으로 NavigateToPose 성공(status=4).
  goal 도달 판정 허용오차는 0.3 m이며 이후 경사면의 정착/미끄러짐은 별도입니다.
- Spin action target_yaw=0.5 rad 성공(status=4).
  관측 yaw 변화 약 0.502 rad, XY 이동 약 0.0344 m.
- 목표/Spin 종료 후 `/cmd_vel`이 0으로 복귀한 것을 확인했습니다.
- RViz 설정 파일을 실제 로드하여 지도 500 x 500 및 local costmap 100 x 100 생성 로그 확인.
  RViz plugin/설정 로드 오류는 관찰되지 않았습니다.
- 최종 통합 실행의 운행 중 로그에서 Python traceback, 지면분리 timeout, plugin load 오류는 관찰되지 않았습니다.

## 아직 평가하지 않은 항목

- 시험장 전체 또는 임의의 시작/목표 조합에 대한 성공률.
- 나무/돌 각각을 실제 우회하는 반복 주행 및 충돌률의 정량 평가.
- 급경사에서의 Patchwork++ ground/nonground 정확도 및 모든 지형에 대한 파라미터 최적화.
- 전복 방지, 차체 접촉역학, 센서 기반 localization.
- `camera:=true`의 RGB-D를 포함한 장시간 전체 대역폭 부하.
- Husky/Jackal/FW-max skid 실행. 현재 지원 profile은 FW-max swerve입니다.

따라서 이번 결과는 기본 pipeline과 짧은 goal/spin의 통합 smoke test이며,
주행 알고리즘의 연구 성능 평가를 대신하지 않습니다.

## 관찰한 통신 이슈

이 호스트에서 CycloneDDS 기본 구성은 실제 odometry/점군이 clock보다 수 초 늦는 현상이 재현되었습니다.
수신 buffer를 늘린 별도 설정에서도 지연이 남았습니다. 시스템 설정은 변경하지 않았습니다.
Fast DDS 구성 및 reliable Patchwork++ 결과 수신으로 최종 시험을 수행했습니다.
모든 client 터미널에서도 README의 RMW/Domain 환경을 동일하게 사용해야 합니다.

종료 시 upstream Patchwork++ demo의 SIGTERM escalation이 한 번 관찰되었습니다. INTEGRATION_NOTES.md 7절 참고.

## 2026-11-16 평지 스폰 위치 변경

- 기본 위치를 (-7.2, -3.2, 3.2)에서 **(-3.0, 1.0, 1.30) m**로 변경했습니다. yaw는 0입니다.
- 원본 heightmap의 Y축 반전을 고려한 좌표로, 중심 주변 3 x 3 m 구역 높이는 약 1.244830 m로 일정합니다.
- 기존 world/heightmap/로봇 모델은 수정하지 않았습니다.
- Gazebo 실제 정착 자세(시뮬레이션 시간 5.56 s):
  x=-3.000007 m, y=1.000000 m, base_footprint z=1.240966 m,
  roll 약 0 deg, pitch=0.017875 deg.
- 마지막 50개 자세 표본의 roll/pitch가 각각 1도 미만이며 XY 이동이 0.1 m 미만임을 확인했습니다.
- 새 기본 위치에 맞춰 RViz 시점과 README의 예제 목표점도 변경했습니다.
- 빌드 성공, 자동 테스트 6개 통과.


## 2026-11-16: RViz 컬러 탑뷰 배경

- 원본 50×50 m heightmap, 텍스처와 37개 obstacle visual을 Blender 5.2 Cycles로 렌더.
- 패키지 build 성공, colcon test: 12 passed / 0 failed.
- RViz standalone에서 실제 textured mesh 표시와 북쪽(+Y)/동쪽(+X) 방향을 화면 확인.
  all-zero Marker color로 embedded material을 보존하여 RViz tint 문제 해결.
- 격리된 ROS_DOMAIN_ID=98 / IGN_PARTITION으로 gui:=false rviz:=false 통합 launch 실행.
  Nav2 lifecycle 전체 active, 배경 노드 기동 및 늦게 참가한 subscriber의 marker 수신 확인.
- 배경 추가 이후 목표 주행 재실험은 수행하지 않음. Nav2/제어/센서 설정 변경 없음.
- 검증용 Gazebo/RViz 프로세스 종료. 다른 패키지 소스 변경 없음.


## 2026-11-16: FWmax 분리형 mesh URDF 연동

- upstream rough_terrain_sim 기준 commit: 6720fd5.
- 차체/바퀴가 분리된 DAE를 원본 Xacro에서 직접 로드하며, 원본 패키지 소스 변경 없음.
- rough_terrain_sim과 rough_terrain_navigation 동시 빌드 성공; 새 DAE 실파일 설치 확인.
- 확장된 URDF의 base_footprint → base_link는 0.382 m.
  Ouster mount 0.505 m와 내부 offset 0.060695 m를 포함해 실제 측정 프레임 높이는
  0.947695 m. Patchwork++ sensor_height를 이 값으로 수정.
- 자동 검사 15 passed / 0 failed: 기존 12개 및 upstream TF/센서 높이,
  분리 mesh 실파일, nominal 바퀴 접지/제어 조인트 연동 검사 3개.
  upstream xacro 모듈의 Python 버전 관련 warning 1개는 남아 있으며 시험 실패는 아님.
- 격리된 ROS_DOMAIN_ID=98 / IGN_PARTITION으로 통합 실행.
  새 URDF 로드, 조향/바퀴 제어기 및 Nav2 lifecycle 활성화 확인.
- 런타임 Patchwork++ 파라미터와 TF 측정값이 모두 0.947695 m임을 확인.
- 기본 시작점 (-15.259527, 21.872246)에서 목표 (-14.259527, 21.872246)로
  NavigateToPose 성공(status=4). 최종 XY 약 (-14.533504, 21.890764), 위치 오차 약
  0.275 m로 설정된 0.3 m 허용오차 이내. 종료 후 /cmd_vel=(0,0) 확인.
- 갱신된 obstacle cloud 2254 points 수신(특정 관측 시점의 표본).
- RViz에서 새 RobotModel 로드 및 aerial 위 Global/Local costmap 표시 확인.
- 경사 진입 시 visual/collision 일치 및 전복/충돌 성능의 정량 평가는 수행하지 않음.
- 검증용 프로세스 종료. 사용자 속도 파라미터와 스폰 위치는 유지.


## 2026-11-16: 장애물 탐지 없는 waypoint 모드

- 개발 브랜치: feature/waypoint-no-obstacles. 기본 simulation/navigation 모드는 유지.
- waypoint_simulation.launch.py 기본 obstacle_detection=false 전달 확인(--show-args 포함).
- 통합 실행에서 ground_pipeline/Patchwork++/LiDAR ROS bridge 미실행,
  Global/Local plugins=[static_layer, inflation_layer], require_cloud=false 확인.
- /navigation/obstacles publisher 0인 상태에서 FollowWaypoints 2개 지점 주행 성공(status=4).
  목표 (-14.459527, 21.872246), (-13.659527, 21.872246).
  feedback indices=[0,1], missed_waypoints=[], /plan 메시지 13개 수신.
  최종 XY=(-13.916322,21.955581), 최종 /cmd_vel=(0,0).
- 기본 simulation.launch.py 재실행에서 require_cloud=true, 양쪽 obstacle_layer,
  Patchwork++/ground_pipeline 및 실제 obstacle cloud 수신을 확인.
- RViz /waypoints MarkerArray(실제 토픽 타입 조회 확인) 표시 추가, plugin 로드 오류 없음.
- build 성공 및 colcon test 18 passed / 0 failed. cloud 요구 여부와 무관하게
  위치/명령의 최신성 검사를 유지하는 테스트 포함.
- 첫 GUI 검증 종료 중 RViz가 SIGINT 5초 내 종료하지 못해 SIGTERM escalation과
  exit -11이 한 차례 관찰됨. 해당 실행의 주행은 이미 성공했고 주행 중 오류는 없었음.
- 경사 waypoint, 나무/돌과 충돌하는 경로, 연속 통과 주행은 시험하지 않음.


## 2026-11-16: 기존 launch의 mode 인자

- simulation.launch.py / navigation.launch.py에 mode=perception|waypoint 추가.
  생략 시 perception. 기존 obstacle_detection은 auto 기본값으로 mode를 따르며,
  명시적인 true/false 인자는 호환을 위해 우선 적용.
- waypoint_simulation.launch.py는 mode 기본값만 waypoint로 설정하는 호환 런치로 유지.
- 모드 해석/잘못된 입력/이전 인자 우선순위와 실제 두 launch setup의 조건 및
  생성 costmap YAML을 검사. colcon test 26 passed / 0 failed.
- simulation.launch.py mode:=waypoint gui:=false rviz:=false로 실제 실행하여
  두 waypoint 모두 도달(status=4), missed=[], 경로 13개 수신,
  장애물 publisher 0, 종료 명령 (0,0) 확인.
- simulation.launch.py mode:=perception으로 실제 재실행하여 Patchwork++/ground_pipeline,
  양쪽 obstacle_layer, require_cloud=true, 장애물 점군 수신을 확인.
- build 성공. 검증용 프로세스 종료.
