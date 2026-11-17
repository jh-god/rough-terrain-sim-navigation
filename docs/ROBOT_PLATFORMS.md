# 로봇 플랫폼 연결

## 실행 인터페이스

`simulation.launch.py robot:=fwmax|husky|jackal mode:=perception|map_only`를 사용합니다.
기본값은 fwmax/perception입니다. navigation.launch.py도 같은 robot 인자를 받습니다.
단일 로봇 선택용이며 여러 플랫폼을 동시에 실행하는 namespace 구성은 아닙니다.

| 항목 | FWmax | Husky A200 | Jackal J100 |
|---|---|---|---|
| 원본 설정 | FWmax swerve Xacro | rough_terrain_sim/config/husky/robot.yaml | rough_terrain_sim/config/jackal/robot.yaml |
| 실제 구동 | 기존 조향/바퀴 제어기 | Gazebo DiffDrive | Gazebo DiffDrive |
| GT 부착 프레임 | base_footprint | base_link | base_link |
| LiDAR | OS1-64 | Ouster 128 | VLP-16 |
| 명목 LiDAR 지상고 | 0.947695 m | 0.722975 m | 0.4852 m |
| 기본 root 스폰 z | 1.30 m | 1.45 m | 1.38 m |
| RGB-D (기본 자동 출력) | 지원 | 지원 | 기본 모델에 없음 |

XY 시작 위치, 빈 지도, aerial 배경, Smac2D/MPPI, 최대 전진 1 m/s와 yaw 1 rad/s는 공통입니다.
로봇별 footprint, 자기 점군 제거 범위, LiDAR frame override 및 Patchwork++ 높이를 사용합니다.
이 설정은 시작 프로파일이며 급경사·수풀 전체에서의 분류/회피 성능 최적화는 아닙니다.

## 모델 생성과 TF

FWmax는 원본 Xacro를 직접 사용합니다. Husky/Jackal은 설치된 Clearpath
DescriptionGenerator와 rough_terrain_sim의 robot.yaml로 임시 디렉터리에 모델을 생성합니다.
원본 mesh, collision, 질량/관성, 센서, WheelSlip을 재사용합니다.

생성 결과에서 ros2_control 제어 플러그인과 설정을 제거하고 Gazebo DiffDrive를 연결합니다.
바퀴 반경과 간격은 확장 URDF의 실제 wheel joint/cylinder에서 읽습니다.
기존 Clearpath 플랫폼 서비스/평면 odom TF는 실행하지 않습니다.
따라서 이 패키지에서의 Husky/Jackal 제어는 Clearpath controller_manager API가 아닌
Gazebo DiffDrive의 /cmd_vel 경로입니다.

3D OdometryPublisher와 JointStatePublisher를 추가하여 /ground_truth/odom과 /joint_states를
ROS로 브리지합니다. 원본 root가 base_link이므로 adapter도 base_link를 부모로 부착합니다.
Husky의 base_footprint는 원본 URDF의 자식 프레임으로 유지하여 TF 순환을 피합니다.

로봇·패키지 소스는 변경하지 않고 /tmp/rough_terrain_robot_*에 생성 결과를 남깁니다.
robot_z:=auto는 현재 기본 시작점 지면 높이 약 1.24483 m에 맞춘 값이며,
임의 XY 지형 높이를 자동으로 샘플링하는 기능은 아닙니다.

## 센서와 설정 덮어쓰기

Clearpath LiDAR는 /sensors/lidar3d_0/points로 브리지하며, 원본 센서 프레임을
input_frame_override로 명시합니다. GT localization은 모든 모드에서 유지됩니다.
Husky IMU는 `/sensors/imu_0/data`, Jackal IMU는 `/sensors/imu_0/data_raw`,
Jackal GPS는 `/sensors/gps_0/navsat`으로 발행합니다. FWmax 원본 모델에는 IMU가 없습니다.
Clearpath LaserScan도 `/sensors/lidar3d_0/scan`으로 발행합니다.

Husky의 RGB-D는 FWmax와 같은 /sensors/camera/color/image, depth/image, camera_info,
points 및 points_aligned 이름으로 제공합니다. `camera:=auto`가 기본값이며 FWmax/Husky에서
자동 출력합니다. Jackal에서 camera:=true는 스폰 전에 명시적으로 거부합니다.

세 모델 모두 `map_only`에서도 장착된 센서 출력을 유지합니다. 센서 출력 여부와
장애물을 costmap에 반영하는 여부는 독립적입니다. Jackal GPS를 위해 임시 월드 사본에
NavSat 시스템과 가상 WGS84 기준점(위도 0°, 경도 0°, 고도 0m, ENU)을 추가합니다.
이는 시험장의 실제 지리 좌표가 아니며 원본 시뮬레이션 파일은 수정하지 않습니다.

robot_params:=auto / patchwork_params:=auto는 로봇별 파일을 선택합니다.
사용자 YAML 경로를 주면 해당 파일을 사용합니다. 공통 기본 Nav2 파일에서는 로봇별
footprint를 적용하고, 사용자 params_file을 지정하면 그 footprint를 유지합니다.

일반 Clearpath/원본 rough_terrain_sim 런치에 navigation.launch.py만 붙이는 구성은
그대로 지원하지 않습니다. 해당 실행에 3D GT 발행/브리지와 TF 중복 제거가 준비되어야 합니다.
이 패키지의 simulation.launch.py는 이를 자동으로 구성합니다.
