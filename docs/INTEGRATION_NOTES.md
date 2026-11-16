# 외부 패키지 연동 메모

기존 패키지의 소스/설정은 이 구현에서 수정하지 않았습니다.
제공한 simulation.launch.py를 사용하면 아래 외부 수정 없이 실행하도록 구성했습니다.

## 1. 기존 평면 TF와 충돌

rough_terrain_sim/launch/rough_terrain.launch.py는 `fwmax_planar_odometry`를 실행하고
`odom -> base_footprint`를 발행합니다. 새 패키지는 실제 3D body TF를 발행하므로
두 launch를 그대로 함께 실행하면 안 됩니다.
새 simulation.launch.py는 기존 world/Xacro/controller를 재사용하되 이 평면 노드를 실행하지 않습니다.
기존 launch에 직접 연결하고 싶다면 사용자께서 해당 Node 실행을 끄는 인자를 추가하거나
노드를 제외한 뒤 navigation.launch.py를 실행해야 합니다. 변경 지점은 executable='fwmax_planar_odometry'인 Node입니다.

## 2. Patchwork++ 출력 timestamp 손실

patchwork-plusplus-ros/include/patchworkpp/patchworkpp.hpp의 cloud2msg()는
`stamp.seconds()`를 정수 sec에 대입하고 nanosec 계산에 double seconds를 사용하여
원래 subsecond timestamp가 사라집니다. 새 ground_pipeline은 한 프레임만 진행하고
두 결과를 수신해 원래 header를 복원하므로 외부 수정은 필수가 아닙니다.
upstream을 직접 고친다면 아래처럼 변경할 수 있습니다(이번 구현에서는 변경하지 않음).

```cpp
cloud_ROS.header.stamp = stamp;  // rclcpp::Time -> builtin_interfaces::msg::Time
cloud_ROS.header.frame_id = frame_id;  // 함수에 전달된 실제 입력 frame
```

현재 wrapper는 upstream 설정 frame_id를 navigation/lidar_level로 고정합니다.
다른 Patchwork++ 인스턴스를 같은 /navigation/patchwork/* 토픽에 연결하지 마세요.
wrapper timeout은 잘못된 프레임 대응을 막기 위해 재시작을 요구합니다.

## 3. 설치된 모델/asset

원본 모델의 Git LFS 실파일과 16-bit heightmap용 Gazebo 라이브러리는 기존 시뮬레이션의 요구조건입니다.
이 패키지는 원본 물리 충돌 형상·마찰·서스펜션을 변경하지 않습니다.
원본 FW-max Xacro의 collision 형상과 시각 형상은 다를 수 있으며 시험 결과에 영향을 줍니다.

## 4. 범위와 비교 실험

ground truth 기반 위치 연결은 센서 localization 알고리즘이 아닙니다.
Nav2 계획/제어는 평면 근사이고 전복 위험을 평가하지 않습니다.
Patchwork++가 급경사를 nonground로 분류하면 기본 모드에서도 회피할 수 있습니다.
사용자 알고리즘 추가 전 ground/nonground/costmap을 기록하여 비교 기준을 확인하세요.
완만한 시험 구역에서 기본 goal 도달부터 검증한 뒤 난도를 높이세요.

## 5. 제어권

Nav2 controller와 recovery 명령 모두 velocity_smoother -> command_gate -> /cmd_vel을 사용합니다.
동시에 teleop이나 다른 /cmd_vel publisher를 실행하지 마세요.
새 패키지는 상태·지면분리·명령의 freshness가 없으면 정지를 발행합니다.
시뮬레이션 reset 후에는 전체 launch를 재시작하세요.


## 6. DDS / 프로세스별 환경

이 호스트의 기본 CycloneDDS 환경에서는 clock에 비해 odometry/점군 수신이 수 초씩 지연되는
현상이 재현되었고, 수신 버퍼 설정만으로 해결되지 않았습니다.
Fast DDS를 사용했을 때 최신 state와 점군이 정상 전달되어 goal과 spin 시험이 성공했습니다.
새 launch의 기본 rmw_implementation은 rmw_fastrtps_cpp입니다. 새로 시작하는 자식 프로세스에만
적용하며 사용자 셸·기존 프로세스·시스템 설정은 바꾸지 않습니다.
별도 클라이언트 터미널도 export RMW_IMPLEMENTATION=rmw_fastrtps_cpp를 적용하세요.
원래 환경을 유지해야 한다면 launch 인자를 바꿀 수 있지만 통신 지연을 먼저 해결해야 합니다.

브리지의 입력·출력 queue는 1로 두어 오래된 대용량 관측의 누적을 줄였습니다.
Patchwork++ 결과는 reliable QoS로 구독하며, raw 센서는 SensorData QoS로 수신합니다.

## 7. 설치된 Patchwork++ 종료 동작

최종 통합 시험 종료 시 upstream demo가 SIGINT 후 5초 안에 종료하지 않아
ROS launch가 SIGTERM으로 종료한 경우가 있었습니다. 주행 중 timeout이나 crash와는 구분됩니다.
명령 gate는 이미 정지한 상태였고 시뮬레이터/나머지 노드는 종료되었습니다.
즉시 upstream 소스 수정이 필요한 사항은 아니며, 종료 지연이 문제가 되면
patchwork-plusplus-ros/src/demo.cpp의 rclcpp shutdown/노드 해제 경로를 별도로 점검하세요.


## 8. 분리형 FWmax mesh 모델 연동 (rough_terrain_sim 6720fd5)

현재 통합 launch는 설치된 upstream Xacro를 직접 읽으므로 차체/바퀴 DAE 분리를
자동으로 반영합니다. mesh를 복사하거나 별도의 cylinder visual을 추가하지 않습니다.
새 DAE 파일이 설치되도록 rough_terrain_sim과 rough_terrain_navigation을 함께 빌드하세요.

base_footprint → base_link 높이는 0.382 m입니다. Ouster mount의 0.505 m에
매크로 내부 0.0225 + 0.038195 m가 더해져 base_link → os_sensor는 0.565695 m입니다.
따라서 config/patchworkpp.yaml의 nominal sensor_height는 0.947695 m입니다.
upstream 주석의 의도값 대신 실제 확장된 URDF의 측정 프레임을 기준으로 합니다.
실제 서스펜션 압축과 지형에 따른 순간 높이는 달라질 수 있습니다.

바퀴 collision 반경 0.125 m, wheelbase 0.600 m, track 0.450 m와 프레임/조인트/토픽
이름은 유지됩니다. 바퀴 중심의 X 이동은 -0.000360 m입니다. 기존 body-twist 제어
근사와 footprint를 유지하고, TF는 upstream joint origin을 그대로 사용합니다.
기존 robot_x/y/z/yaw와 사용자가 조절한 Nav2 속도 파라미터는 유지합니다.

자동 연동 검사는 설치된 Xacro를 확장하여 센서 높이와 Patchwork++ 설정의 일치,
새 mesh 실파일 존재, 네 바퀴의 nominal 접지 높이 및 제어 조인트 이름을 확인합니다.
