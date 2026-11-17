# Navigation 모드

모드는 목표 입력 방식이 아니라 장애물 인식 사용 여부를 선택합니다.
두 모드 모두 단일 목표(NavigateToPose)와 여러 waypoint(FollowWaypoints)를 지원합니다.

| 인자 | 동작 |
|---|---|
| `mode:=perception` | 기본값. 지도와 LiDAR/Patchwork++ 기반 장애물 회피 |
| `mode:=map_only` | 센서 장애물 탐지 없이 지도 위 경로계획·추종 |

## 실행

```bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
ros2 launch rough_terrain_navigation simulation.launch.py mode:=map_only
```

센서 기반 회피는 `mode:=perception`을 사용합니다. mode를 생략해도 perception입니다.
`simulation.launch.py`와 `navigation.launch.py` 모두 같은 인자를 지원합니다.
모드 변경 시 기존 실행을 종료하고 다시 실행합니다.

RViz Navigation 2 패널에서 `Waypoint / Nav Through Poses Mode`를 누르고,
`Nav2 Goal`로 지점을 순서대로 입력한 뒤 `Start Waypoint Following`을 누릅니다.
이 절차는 두 모드에서 동일합니다. 단일 목표는 누적 모드에 들어가지 않고 지정합니다.
`/waypoints` MarkerArray를 기본 표시하며, 입력 취소는 `Cancel Accumulation`,
주행 중단은 `Cancel`입니다.

FollowWaypoints는 각 지점까지 Smac2D 경로를 생성하고 MPPI로 추종합니다.
위치 0.3 m, 방향 pi rad 허용오차로 도달 판정 후 0.2초 대기합니다.
연속 통과 경로를 한 번에 추종하는 방식은 아닙니다.
실패한 지점은 stop_on_failure=false에 따라 기록하고 다음 지점을 시도합니다.

## map_only 구성

ground_pipeline, Patchwork++를 끄고 Global/Local
obstacle_layer를 실행 시 생성하는 파라미터에서 제거합니다. Static/Inflation layer,
시험장 경계, 속도/위치 최신성 검사는 유지합니다.

현재 지도는 시험장 경계만 포함하는 빈 지도입니다. Gazebo 센서 자체와 나무·돌의
물리 collision은 원본 모델대로 남지만, 센서로 관측하여 회피하지는 않습니다.
따라서 장애물과 겹치지 않는 목표점/waypoint 구간을 지정하세요.

## 이전 명령과 병합

기존 `mode:=waypoint`는 map_only의 호환 별칭으로만 유지합니다.
기존 `waypoint_simulation.launch.py`도 map_only 모드를 기본 선택합니다.
새 실행에는 기존 simulation/navigation 런치에 mode를 지정하는 방식을 사용하세요.

기존 `obstacle_detection:=true/false`는 명시하면 mode보다 우선합니다.
기본값 auto는 mode를 따릅니다.

feature/waypoint-no-obstacles 브랜치에서 개발하며, 공통 Nav2 YAML을 복제하지 않고
모드 해석 함수와 조건 실행을 사용합니다. 기본 명령은 perception으로 동작합니다.

센서 ROS 출력은 navigation 모드와 독립적입니다. `map_only`에서도 장착된 LiDAR,
RGB-D, IMU, GPS를 발행합니다. 카메라는 `camera:=auto`가 기본이며 장착된 모델에서
활성화됩니다. `camera:=false`를 명시하면 해당 카메라의 ROS 출력만 끕니다.
