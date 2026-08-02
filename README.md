# mybot — ROS 2 Jazzy + Gazebo Harmonic 시뮬레이션 워크스페이스

실제 로봇 없이 시뮬레이션에서 개발하기 위한 워크스페이스입니다.
실기 이전을 염두에 두고, 시뮬과 실기가 **동일한 인터페이스**를 갖도록 구성했습니다.

## 로봇 사양

- 차동구동(differential drive): 중앙 양쪽 구동륜 2개
- 캐스터 4개 (앞 2, 뒤 2) — 마찰 없는 구(sphere)로 모델링
- 섀시 0.40 m 정육면체 → **외접반경 0.283 m** (제자리 회전에 필요한 폭)
- Intel RealSense D435i (전면, 15도 하향) — 컬러 + 뎁스 + IMU
- RPLIDAR A2M12 (상단) — 360도, 스캔 평면 지면 0.49 m

기본 치수는 `mybot_description/urdf/mybot.urdf.xacro` 상단의 파라미터 블록에 모여 있습니다.
실제 치수가 확정되면 그 블록과 `mybot_bringup/config/controllers.yaml` 의
`wheel_separation` / `wheel_radius` 를 **같이** 고쳐야 합니다.

## 패키지 구성

| 패키지 | 역할 |
|---|---|
| `mybot_description` | URDF/xacro, RViz 설정, URDF 확인용 런치 |
| `mybot_bringup` | Gazebo 월드, 컨트롤러/브리지/EKF 설정, 시뮬 통합 런치 |
| `mybot_examples_py` | rclpy 예제 (`square_driver`) |
| `mybot_examples_cpp` | rclcpp 예제 (`depth_safety_filter`) |
| `mybot_navigation` | RTAB-Map VSLAM + Nav2 자율주행 + 프론티어 자동 탐사 |
| `tools/` | 측정·회귀시험 스크립트 (패키지 아님 — `tools/README.md` 참고) |

## 최초 설치

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-realsense2-description \
  ros-jazzy-depth-image-proc \
  ros-jazzy-rtabmap-ros ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-spatio-temporal-voxel-layer \
  ros-jazzy-twist-mux \
  ros-jazzy-robot-localization \
  python3-numpy python3-scipy
```

`rosdep install --from-paths src -y --ignore-src` 로도 됩니다 — 위 목록은
각 패키지의 `package.xml` 과 일치시켜 두었습니다.

아래 셋은 nav2 에 포함되지 않은 별도 패키지라 빠뜨리기 쉽습니다.
없어도 **빌드는 되고 런타임에만 실패**하므로 원인을 찾기 어렵습니다.

| 패키지 | 없으면 |
|---|---|
| `spatio-temporal-voxel-layer` | 코스트맵이 3D 장애물을 못 봄 (플러그인 로드 실패) |
| `twist-mux` | `/cmd_vel` 이 아무도 안 내보내 로봇이 안 움직임 |
| `robot-localization` | EKF 가 없어 `odom -> base_footprint` TF 가 끊김 |

### 반드시 필요한 환경변수

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

이 PC 는 네트워크 인터페이스가 3개(lo/enp4s0/wlo1)라 FastDDS 의 멀티캐스트
디스커버리가 여러 인터페이스에 걸칩니다. 그러면 서비스 응답의 writer/reader
매칭이 늦어져 **Nav2 의 lifecycle 전이가 중간에 멈춥니다**
(`failed to send response to /controller_server/change_state`).
로컬호스트로 제한하면 재현되지 않습니다. `~/.bashrc` 의 `jazzy` 함수에
넣어 두는 것을 권합니다.

> Jazzy 는 Gazebo Harmonic 을 `ros-jazzy-gz-*-vendor` 패키지로 제공하므로
> osrfoundation 저장소를 따로 추가할 필요가 없습니다.

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 을 쓰면 Python 노드나 launch/yaml 을 고쳤을 때 재빌드 없이 반영됩니다.

## 실행

### 시뮬레이션 전체

```bash
ros2 launch mybot_bringup sim.launch.py
```

주요 인자:

| 인자 | 기본값 | 설명 |
|---|---|---|
| `world` | `room.sdf` | `mybot_bringup/worlds/` 아래 월드 파일 |
| `x` `y` `z` `yaw` | `0 0 0.15 0` | 스폰 위치 |
| `use_rviz` | `true` | RViz2 동시 실행 |
| `gui` | `true` | `false` 면 Gazebo headless |
| `use_pointcloud` | `true` | `depth_image_proc` 로 포인트클라우드 생성 |

### URDF 만 확인 (Gazebo 없이)

```bash
ros2 launch mybot_description display.launch.py
```

### 키보드 조종

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_footprint
```

### 예제 노드

```bash
# 정사각형 주행 (Python)
ros2 run mybot_examples_py square_driver --ros-args -p use_sim_time:=true

# 뎁스 기반 안전 필터 (C++): /cmd_vel_raw -> /cmd_vel
ros2 run mybot_examples_cpp depth_safety_filter --ros-args -p use_sim_time:=true
```

## 인터페이스

| 토픽 | 타입 | 방향 | 비고 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | 입력 | `twist_mux` 출력 |
| `/cmd_vel_teleop` | `geometry_msgs/TwistStamped` | 입력 | 사람 (우선순위 높음) |
| `/odom` | `nav_msgs/Odometry` | 출력 | 휠 오도메트리 |
| `/odometry/filtered` | `nav_msgs/Odometry` | 출력 | EKF (휠 + 자이로) |
| `/joint_states` | `sensor_msgs/JointState` | 출력 | |
| `/scan` | `sensor_msgs/LaserScan` | 출력 | RPLIDAR, 360도 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 출력 | VSLAM 특징점 입력 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 출력 | |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 출력 | 컬러에 정합된 뎁스 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 출력 | STVL 3D 관측원 |
| `/camera/imu` | `sensor_msgs/Imu` | 출력 | 자이로만 사용 |
| `/map` | `nav_msgs/OccupancyGrid` | 출력 | RTAB-Map, TRANSIENT_LOCAL |

`/map` 을 구독하는 도구는 QoS 를 **RELIABLE + TRANSIENT_LOCAL** 로 맞춰야 합니다.
RTAB-Map 은 지도가 바뀔 때만 발행하므로, 기본 QoS(VOLATILE) 로 붙으면 로봇이
멈춰 있는 동안에는 마지막 지도를 못 받고 영원히 기다립니다.

TF 트리:

```
map --(rtabmap)--> vodom --(rgbd_odometry)--> odom --(EKF)--> base_footprint
                                                              -> base_link
                                                                 -> 바퀴/캐스터
                                                                 -> laser
                                                                 -> camera_*
```

카메라 토픽·프레임 이름은 실제 `realsense2_camera`
(`camera_name:=camera`, `camera_namespace:=""`) 와 동일하게 맞췄습니다.

## 실제 로봇으로 옮길 때

바꿔야 하는 것은 `mybot_description/urdf/mybot.ros2_control.xacro` 의
`<hardware>` 블록 하나입니다. 현재 `sim_mode:=false` 분기는
`mock_components/GenericSystem` 으로 되어 있는데, 이를 직접 만든
`hardware_interface` 플러그인으로 교체하면 `diff_drive_controller` 위쪽
스택(Nav2, 예제 노드 등)은 수정 없이 그대로 동작합니다.


## 자율주행 (VSLAM + Nav2)

**`navigation.launch.py` 하나만 쓰면 됩니다.** 시뮬 → SLAM → Nav2 를
순서대로(선행 토픽을 확인하며) 띄웁니다.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST      # 필수 (위 "환경변수" 절 참고)

ros2 launch mybot_navigation navigation.launch.py
```

RViz 의 **2D Goal Pose** 로 목표를 찍으면 주행합니다.

### 상황별 실행

**지도 없이 시작해 스스로 지도 만들기 (프론티어 자동 탐사)**

```bash
ros2 launch mybot_navigation navigation.launch.py explore:=true
```

빈 지도에서 시작해 "빈 곳인데 옆이 미탐색"인 경계를 찾아다니며 채웁니다.
다 채우면 출발점으로 복귀하고 스스로 끝납니다 (10×8 m 방에서 약 3분).
지도는 `~/.ros/mybot_rtabmap.db` 에 저장됩니다.

**이미 만든 지도로 주행만 (운용 모드)**

```bash
ros2 launch mybot_navigation navigation.launch.py localization:=true
```

지도를 갱신하지 않으므로 유령 장애물이 생겨도 사라지지 않습니다.
자원 절감 효과는 **없습니다** — DB 전체를 작업 메모리에 올리므로 오히려
rtabmap 이 CPU 25.1 → 33.9 %p, 메모리 552 → 604 MB 로 늘어납니다.

**측정·회귀시험 (창 없이 가볍게)**

```bash
ros2 launch mybot_navigation navigation.launch.py gui:=false use_rviz:=false
```

**시뮬레이터를 따로 띄워 두고 SLAM+Nav2 만**

```bash
ros2 launch mybot_navigation navigation.launch.py use_sim:=false
```

**이미 떠 있는 스택에 탐사만 추가**

```bash
ros2 launch mybot_navigation explore.launch.py
```

**Nav2 파라미터만 고쳤을 때** — 전체 재시작(3분 이상) 대신 Nav2 만
(약 45초). 시뮬과 SLAM 은 켜 둔 채 Nav2 프로세스만 죽이고 다시 띄웁니다.

```bash
ros2 launch mybot_navigation nav2.launch.py
```

### 실기(Jetson Orin Nano) 자원 절감

성능이 모자랄 때 효과 순서대로. **둘 다 대가가 있습니다.**

```bash
# rtabmap CPU -38%, 메모리 -106MB / 자세오차 21 -> 27 mm
ros2 launch mybot_navigation navigation.launch.py detection_rate:=1.0

# 시각 오도메트리를 끄고 EKF(휠+IMU)만. 절감이 가장 크지만 위치 정확도 손실도 큼
ros2 launch mybot_navigation navigation.launch.py use_vslam:=false
```

RViz 는 실기에서 띄우지 말고 개발 PC 에서 원격으로 보세요. 그때는 **양쪽 모두**
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` 으로 바꿔야 합니다
(`LOCALHOST` 로는 다른 PC 가 보이지 않습니다).

### 주요 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `explore` | `false` | 미탐색 구역 자동 탐사 |
| `localization` | `false` | `true` 면 기존 지도로 위치추정만 (지도 갱신 안 함) |
| `use_sim` | `true` | Gazebo 도 함께 실행 |
| `use_rviz` / `gui` | `true` | RViz / Gazebo 창 |
| `use_vslam` | `true` | `false` 면 EKF(휠+IMU)만 |
| `detection_rate` | `2.0` | rtabmap 지도 노드 추가 주기 [Hz] |
| `memory_thr` | `300` | 작업 메모리 노드 수 상한 (0=무제한, 메모리가 계속 자람) |
| `map_3d` | `false` | 3D 점유격자. `true` 면 메모리 2.7배 |
| `reg_strategy` | `2` | 루프 클로저 검증. 0=영상만, 2=영상+ICP |

전체 목록은 `ros2 launch mybot_navigation navigation.launch.py --show-args`.

### 띄운 직후 확인할 것

```bash
ros2 topic info /clock | grep Publisher
```

**`Publisher count: 1`** 이어야 합니다. 2 면 이전 실행의 `parameter_bridge` 가
남아 새 Gazebo 에 다시 붙은 것입니다. `/clock` 이 둘에서 나오면 시각 순서가
뒤섞여 모든 노드가 TF 버퍼를 계속 비우고, Nav2·RTAB-Map 의 조회가 전부
실패합니다. 증상이 "가끔 멈춤"이라 원인을 찾기 어렵습니다.

정리 명령 (파이썬으로 실행되는 노드까지 잡으려면 **경로가 명령줄 어디에
있든** 매칭해야 합니다 — 실행파일 이름만 보면 `vodom_tf_relay.py` 같은
것을 놓칩니다):

```bash
ps -eo pid,args | grep -E '/opt/ros/jazzy|ros2_ws/install|[g]z sim' \
  | grep -v shell-snapshots | awk -v me=$$ '$1 != me {print $1}' | xargs -r kill -9
ros2 daemon stop      # 죽은 노드가 `ros2 node list` 에 남아 보일 때
```

URDF 나 월드를 고친 뒤에는 반드시 위 정리를 거치고 다시 띄우세요.
`gz sim` 서버가 남아 있으면 이전 모델을 재사용합니다.

### 구성

```
map --(rtabmap)--> vodom --(rgbd_odometry + vodom_tf_relay)--> odom
                                            --(diff_drive_controller)--> base_footprint
```

- **시각 오도메트리**: `rtabmap_odom/rgbd_odometry`. 휠 오도메트리를 모션
  추정 초기값(guess)으로 받아, 시각 특징이 부족한 구간에서도 버팁니다.
- **SLAM / 루프 클로저 / 점유격자**: `rtabmap_slam/rtabmap` 이 `/map` 과
  `map -> vodom` 을 발행합니다. AMCL 과 map_server 는 쓰지 않습니다.
- **Nav2 코스트맵 관측원은 둘로 나뉩니다.**
  - `obstacle_layer` — RPLIDAR A2M12 의 `/scan`. 360°라 광선 소거가 정상
    동작합니다.
  - `stvl_layer` (3D 복셀) — D435i 의 `/camera/depth/points`. 화각이 좁아
    (87°×58°) 등을 돌리면 지울 기회가 없으므로 시간 감쇠로 지웁니다.
- **속도 명령 경로**: `controller_server → /cmd_vel_nav → velocity_smoother
  → /cmd_vel_smoothed`, 사람은 `/cmd_vel_teleop`. `twist_mux` 가 중재해
  `/cmd_vel` 로 냅니다. 이게 없으면 수동 개입이 불가능합니다.

### 최소 통과 가능 통로 폭은 0.70 m

**직진 능력이 아니라 회전 능력이 기준입니다.** 로봇은 막다른 곳에서 되돌아
나와야 하고, 차동구동은 후진보다 제자리 회전을 먼저 시도합니다
(Nav2 의 `Spin` 복구 행동도 마찬가지).

제자리 회전에 필요한 폭 = 대각선 = √2 × 0.40 = **0.566 m**.

| 통로 | 편측 여유 | 중앙에서 회전 | 직진 시 가능한 편심 |
|---|---|---|---|
| 0.55 m | −8 mm | **실패** | ±75 mm |
| 0.60 m | 17 mm | 통과 | ±100 mm (**끝에서는 회전 불가**) |
| 0.70 m | 67 mm | 통과 | ±150 mm |

0.60 m 가 탈락한 이유: 직진 통과 중 로봇이 놓일 수 있는 범위(±100 mm)의
끝에서는 회전이 아예 안 됩니다. 정상 주행 중 멈추기만 해도 돌아설 수 없는
자세가 됩니다. 편측 여유 17 mm 는 SLAM 자세 오차(중앙값 21 mm)보다도 작습니다.

**이 때문에 SLAM 정확도와 통로 폭은 한 예산입니다.** `detection_rate:=1.0`
처럼 정확도를 깎는 설정은 통과 여유를 직접 갉아먹습니다.

### 알아둘 한계

- **D435i 는 옆과 뒤를 못 봅니다** (뎁스 화각 87°×58°). 360° 라이다가 그
  역할을 하지만, 라이다 스캔 평면은 지면 **0.49 m** 라 그보다 낮은 것은
  원리적으로 못 봅니다. 낮은 장애물은 카메라 담당입니다.
- **VSLAM 은 영상의 특징점으로 움직입니다.** 뎁스는 특징점에 3D 좌표를
  주는 역할이라, 무늬 없는 벽 앞에서는 뎁스가 아무리 정확해도 오도메트리가
  동작하지 않습니다. `worlds/generate_room.py` 가 벽 세그먼트마다 **고유**
  텍스처를 만드는 이유입니다. 같은 무늬를 재사용하면 서로 다른 장소를 같은
  곳으로 오인해 잘못된 루프 클로저가 들어가고 자세가 몇 미터 튑니다.
- **좁은 곳에 들여보내면 지도까지 잃습니다.** 로봇이 벽에 갈리며 회전하면
  바퀴가 미끄러져 오도메트리가 깨지고, 그 오차가 그래프에 누적됩니다.
  실측: 회전 여유를 무시하고 탐사시켰더니 완주에 4526초가 걸리고 지도가
  몇 도 기울었습니다. 회전 기준을 적용하니 188초, 왜곡 없음.

더 자세한 실측 기록과 함정은 `CLAUDE.md` 에 있습니다.
