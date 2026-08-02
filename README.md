# orinbot — ROS 2 Jazzy + Gazebo Harmonic 시뮬레이션 워크스페이스

실물 로봇 없이 시뮬레이션에서 개발하되, 실기 이전을 전제로 시뮬레이션과 실기가 **동일한 인터페이스**를 갖도록 구성한 워크스페이스입니다.

배포 타깃은 **Jetson Orin Nano Super**(6코어 A78AE + 8GB 통합 메모리)이며, 2026-08-02 에 이 보드에서 자원 사용량을 직접 실측했습니다 — 아래 "실물 기기 자원 실측" 절을 참고하세요.

## 로봇 사양

- 차동 구동 방식(Differential drive): 중앙 양쪽에 위치한 구동륜 2개
- 캐스터 4개 (전방 2개, 후방 2개) — 마찰이 없는 구체(Sphere)로 모델링
- 섀시: 0.40 m 정육면체 → **외접 반지름 0.283 m** (제자리 회전에 필요한 최적 폭)
- Intel RealSense D435i (전면, 15도 하향 피치) — 컬러 + 뎁스 + IMU
- RPLIDAR A2M12 (상단) — 360도 스캔, 지면 기준 스캔 평면 높이 0.49 m

기본 치수는 `orinbot_description/urdf/orinbot.urdf.xacro` 파일 상단의 파라미터 블록에 정의되어 있습니다.
실제 로봇의 치수가 확정되면 해당 블록과 `orinbot_bringup/config/controllers.yaml` 파일의 `wheel_separation` 및 `wheel_radius` 항목을 **함께** 수정해야 합니다.

## 패키지 구성

| 패키지 | 역할 |
|---|---|
| `orinbot_description` | URDF/xacro 모델링, RViz 설정 및 URDF 검증용 런치 파일 |
| `orinbot_bringup` | Gazebo 월드, 컨트롤러/브리지/EKF 설정 및 시뮬레이션 통합 런치 파일 |
| `orinbot_examples_py` | rclpy 기반 예제 노드 (`square_driver`) |
| `orinbot_examples_cpp` | rclcpp 기반 예제 노드 (`depth_safety_filter`) |
| `orinbot_navigation` | RTAB-Map VSLAM + Nav2 자율주행 + 프론티어 자동 탐사 |
| `tools/` | 성능 측정 및 회귀 테스트 스크립트 (독립 스크립트 모음 — `tools/README.md` 참고) |

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

`rosdep install --from-paths src -y --ignore-src` 명령으로도 필요한 의존성을 설치할 수 있으며, 위 설치 목록은 각 패키지의 `package.xml`과 맞춰져 있습니다.

다음 3개 패키지는 Nav2의 기본 의존성에 포함되지 않은 별도 패키지이므로 누락되기 쉽습니다.
해당 패키지가 없어도 **빌드는 성공하지만 런타임 시 오류가 발생**하므로 원인을 찾기 어렵습니다.

| 패키지 | 미설치 시 현상 |
|---|---|
| `spatio-temporal-voxel-layer` | 코스트맵에서 3D 장애물을 인식하지 못함 (플러그인 로드 실패) |
| `twist-mux` | `/cmd_vel` 토픽이 발행되지 않아 로봇이 움직이지 않음 |
| `robot-localization` | EKF 노드가 실행되지 않아 `odom -> base_footprint` TF가 끊김 |

### 필수 환경변수 설정

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

이 PC 환경에는 네트워크 인터페이스가 3개(lo, enp4s0, wlo1) 존재하여, FastDDS의 멀티캐스트 디스커버리가 모든 인터페이스로 퍼집니다. 이로 인해 서비스 응답의 writer/reader 매칭이 지연되어 **Nav2의 라이프사이클(Lifecycle) 상태 전이가 중간에 중단되는 현상**이 발생합니다 (`failed to send response to /controller_server/change_state`).
디스커버리 범위를 `LOCALHOST`로 제한하면 이 문제가 발생하지 않습니다. `~/.bashrc`의 `jazzy` 설정 함수에 추가해 두는 것을 권장합니다.

> ROS 2 Jazzy 버전은 Gazebo Harmonic을 `ros-jazzy-gz-*-vendor` 패키지로 기본 제공하므로, osrfoundation 저장소를 별도로 추가할 필요가 없습니다.

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 옵션을 사용하면 Python 노드나 launch/YAML 파일을 수정했을 때 별도의 재빌드 없이 변경 사항이 바로 반영됩니다.

## 실행

### 전체 시뮬레이션 실행

```bash
ros2 launch orinbot_bringup sim.launch.py
```

주요 실행 인자:

| 인자 | 기본값 | 설명 |
|---|---|---|
| `world` | `room.sdf` | `orinbot_bringup/worlds/` 디렉터리 내 월드 파일 |
| `x` `y` `z` `yaw` | `0 0 0.15 0` | 로봇 생성(스폰) 위치 및 방향 |
| `use_rviz` | `true` | RViz2 동시 실행 여부 |
| `gui` | `true` | `false` 설정 시 Gazebo GUI 없이 그래픽리스(Headless) 실행 |
| `use_pointcloud` | `true` | `depth_image_proc`를 사용한 포인트 클라우드 생성 여부 |
| `clock_rate` | `100.0` | `/clock` 발행 주기 [Hz]. `0` 이면 Gazebo 원본(약 1000 Hz)을 그대로 통과 |

> `clock_rate` 는 시뮬레이션 정확도와 무관합니다. 물리 스텝(`max_step_size` 1 ms)은 그대로 두고 **ROS 로 나가는 시계만** 솎습니다. `use_sim_time: true` 인 rclpy 노드는 `/clock` 메시지마다 파이썬 콜백을 돌기 때문에 비용이 발행 주기에 그대로 비례하며, Orin 실측으로 100 Hz 에서도 **노드 1개당 11.3 %p** 입니다. 1000 Hz 로 두면 rclpy 노드 몇 개만으로 보드가 마비됩니다. 실기에는 `/clock` 자체가 없으므로 이 비용은 순수한 시뮬레이션 인공물입니다.

### 월드

네 개가 있고 **서로를 대체하지 않습니다.** 각자 보려는 것이 다릅니다.

| 월드 | 크기 | 무엇을 보는가 | 도크 |
|---|---|---|---|
| `room.sdf` (기본) | 10 × 8 m | **회귀 기준선.** 이 문서와 `CLAUDE.md` 의 수치는 전부 여기서 나왔습니다 | 있음 |
| `maze.sdf` | 6 × 6 m | 좁은 통로 스트레스. 통로 0.75 m 의 7×7 땋은 미로 | 없음 |
| `hall.sdf` | 24 × 18 m | 배포 규모. 긴 주행의 누적 오차, 넓은 곳의 탐사 수렴 | 있음 |
| `office.sdf` | 20 × 14 m | **실전형 최종 시험.** 무늬 없는 벽 + 가구 + 움직이는 사람 3명 | 있음 |

```bash
ros2 launch orinbot_navigation navigation.launch.py world:=office.sdf explore:=true
ros2 launch orinbot_navigation navigation.launch.py world:=maze.sdf dock:=false   # 미로엔 도크가 없습니다
```

**월드를 바꿀 때는 지도 DB 를 지우거나 경로를 나눠야 합니다.** 기본값(`~/.ros/orinbot_rtabmap.db`)을 그대로 두고 다른 월드를 띄우면 RTAB-Map 이 이전 월드의 포즈 그래프에 이어 붙이려 합니다. `database_path:=~/.ros/office.db` 처럼 나누세요.

**미로/홀 텍스처는 저장소에 없습니다** (176 장 × 435 KB). 시드가 고정이라 아래 한 줄이면 같은 파일이 나옵니다 (3.9초):

```bash
python3 src/orinbot_bringup/models/room_materials/generate_textures.py maze hall
```

이걸 건너뛰면 벽이 흰색으로 뜨고, **무늬 없는 벽 앞에서는 시각 오도메트리가 그대로 실패합니다.** 증상은 "SLAM 이 안 붙음"으로만 보여서 원인을 찾기 어렵습니다. `room.sdf` 와 `office.sdf` 텍스처는 커밋되어 있으므로 그대로 뜹니다.

월드는 전부 생성기가 만듭니다 (`worlds/generate_*.py`). SDF 를 직접 고치지 말고 생성기를 고친 뒤 다시 내보내세요 — 설계 근거가 생성기 주석에 있습니다.

```bash
cd src/orinbot_bringup/worlds
python3 generate_maze.py 0.90 > maze90.sdf     # 통로 폭만 바꾼 대조 실험 (시드가 같아 형상은 동일)
```

#### 사무실 월드의 보행자

`office.sdf` 에는 사람 3명이 있고, **속도를 ROS 쪽에서 줍니다.**

```bash
ros2 run orinbot_bringup people_sim.py --ros-args -p use_sim_time:=true
```

| | 속도 | 성격 |
|---|---|---|
| `person_0` | 1.20 m/s | 로봇(0.40)의 3배. 뒤에서 따라잡고 앞을 스쳐 갑니다 |
| `person_1` | 0.55 m/s | 느리고 끝점에서 6초 섭니다. 앞을 오래 막는 역할 |
| `person_2` | 0.85 m/s | 보통 걸음 |

**셋이 서로 다른 속도인 것이 요점입니다.** 같이 움직이면 로봇이 마주치는 상황이 한 가지로 고정되어, 정작 보고 싶은 것(빠른 사람이 갑자기 나타남 / 느린 사람이 앞을 오래 막음)이 만들어지지 않습니다.

사람은 **로봇을 보면 멈춰 섭니다** — 진행 방향 ±75도 안 1.4 m 에 로봇이 들어오면 정지하고, 1.8 m 로 멀어지면 다시 걷습니다(경계에서 떨리지 않도록 히스테리시스). 뒤쪽은 무시합니다. 12초를 기다려도 안 비키면 돌아섭니다 — 로봇도 사람을 피해 멈춰 설 수 있어서 누군가는 양보를 끝내야 하기 때문입니다.

> 자세는 명령 속도를 적분한 **추측항법**으로 들고 있습니다. `gz-sim-velocity-control-system` 이 사실상 기구학 제어라 명령과 실제가 거의 같습니다. 그래서 자세 피드백 토픽을 따로 브리지하지 않습니다.

### URDF 모델만 확인 (Gazebo 미사용)

```bash
ros2 launch orinbot_description display.launch.py
```

### 키보드 원격 조종

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_footprint
```

### 예제 노드 실행

```bash
# 정사각형 경로 주행 (Python)
ros2 run orinbot_examples_py square_driver --ros-args -p use_sim_time:=true

# 뎁스 기반 비상 정지/안전 필터 (C++): /cmd_vel_raw -> /cmd_vel
ros2 run orinbot_examples_cpp depth_safety_filter --ros-args -p use_sim_time:=true
```

## 인터페이스

| 토픽 | 타입 | 방향 | 비고 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | 입력 | `twist_mux` 최종 출력 |
| `/cmd_vel_teleop` | `geometry_msgs/TwistStamped` | 입력 | 사용자 수동 조종 (최우선 순위) |
| `/odom` | `nav_msgs/Odometry` | 출력 | 휠 오도메트리 |
| `/odometry/filtered` | `nav_msgs/Odometry` | 출력 | EKF 융합 데이터 (휠 + 자이로) |
| `/joint_states` | `sensor_msgs/JointState` | 출력 | 관절 상태 정보 |
| `/scan` | `sensor_msgs/LaserScan` | 출력 | RPLIDAR 360도 스캔 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 출력 | VSLAM 특징점 추출용 컬러 영상 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 출력 | 보정된 뎁스 영상 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 출력 | 컬러 프레임에 정합된 뎁스 영상 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 출력 | STVL 3D 장애물 관측원 데이터 |
| `/camera/imu` | `sensor_msgs/Imu` | 출력 | IMU 데이터 (자이로만 사용) |
| `/map` | `nav_msgs/OccupancyGrid` | 출력 | RTAB-Map 점유 격자 지도 (QoS: TRANSIENT_LOCAL) |
| `/battery_state` | `sensor_msgs/BatteryState` | 출력 | 잔량·전압·전류. 시뮬은 `battery_sim.py`, 실기는 BMS 드라이버 |
| `/detected_dock_pose` | `geometry_msgs/PoseStamped` | 출력 | 카메라가 본 도크 자세 (`dock_marker_board.py`, 마커 3장 보드) |
| `/cmd_vel_dock` | `geometry_msgs/TwistStamped` | 내부 | 도킹 속도 명령. `twist_mux` 우선순위 50 |
| `/exploration_enabled` | `std_msgs/Bool` | 입력 | 탐사 일시정지 스위치. `auto_dock` 이 도킹 중에 `false` 를 보냅니다 |
| `/ground_truth/odom` | `nav_msgs/Odometry` | 출력 | **시뮬레이션 전용.** Gazebo 가 아는 실제 자세. 자율주행 스택은 구독하지 않습니다 |

| 액션 | 타입 | 비고 |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | 일반 주행 |
| `/dock_robot` | `nav2_msgs/DockRobot` | 충전 도크 접안 (`dock_id: home_dock`) |
| `/undock_robot` | `nav2_msgs/UndockRobot` | 도크에서 후진 이탈 |

`/map` 토픽을 구독하는 노드나 도구는 QoS 설정을 **RELIABLE + TRANSIENT_LOCAL**로 맞춰야 합니다.
RTAB-Map은 지도가 업데이트될 때만 토픽을 발행하므로, 기본 QoS(VOLATILE)로 접속하면 로봇이 정지해 있는 동안 마지막 지도를 수신하지 못하고 영구 대기 상태에 빠집니다.

TF 트리 구조:

```
map --(rtabmap)--> vodom --(rgbd_odometry)--> odom --(EKF)--> base_footprint
                                                              -> base_link
                                                                 -> 바퀴/캐스터
                                                                 -> laser
                                                                 -> camera_*
```

카메라 토픽 및 프레임 명칭은 실제 하드웨어의 `realsense2_camera` 패키지 표준 설정(`camera_name:=camera`, `camera_namespace:=""`)과 동일하게 맞추었습니다.

## 실물 로봇으로 전환할 때

수정해야 할 부분은 `orinbot_description/urdf/orinbot.ros2_control.xacro` 파일의 `<hardware>` 블록 하나뿐입니다. 현재 `sim_mode:=false` 분기는 `mock_components/GenericSystem`으로 구성되어 있으며, 이를 자체 구현한 `hardware_interface` 플러그인으로 교체하면 `diff_drive_controller` 상위의 전체 소프트웨어 스택(Nav2, 예제 노드 등)은 수정 없이 그대로 동작합니다.

## 자율주행 (VSLAM + Nav2)

**`navigation.launch.py` 런치 파일 하나로 전체 실행이 가능합니다.** 시뮬레이션 → SLAM → Nav2 순서대로 선행 토픽 상태를 확인하며 안전하게 노드를 기동합니다.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST      # 필수 (위 "필수 환경변수 설정" 절 참고)

ros2 launch orinbot_navigation navigation.launch.py
```

RViz 실행 후 **2D Goal Pose** 툴로 목표 지점을 지정하면 자율 주행을 시작합니다.

### 상황별 실행 방법

**지도가 없는 상태에서 자동 탐사 및 지도 작성 (프론티어 자동 탐사)**

```bash
ros2 launch orinbot_navigation navigation.launch.py explore:=true
```

빈 지도에서 출발하여 '미탐색 영역과 접한 경계(프론티어)'를 탐색하며 지도를 채워 나갑니다.
모든 영역 탐색이 완료되면 초기 출발 지점으로 복귀한 뒤 탐사를 자동 종료합니다 (10×8 m 구역 기준 약 3분 소요).
생성된 지도는 `~/.ros/orinbot_rtabmap.db`에 자동 저장됩니다.

**기존 지도를 활용한 주행 전용 모드 (운용 모드)**

```bash
ros2 launch orinbot_navigation navigation.launch.py localization:=true
```

지도를 새롭게 갱신하지 않으므로 잔여 장애물(유령 장애물)이 발생하더라도 제거되지 않습니다.
자원 절감 효과는 **없습니다.** 전체 지도 DB를 작업 메모리에 로드하므로, 오히려 RTAB-Map의 CPU 사용률이 25.1% → 33.9%p로 증가하고 메모리 사용량도 552MB → 604MB로 늘어납니다.

**성능 측정 및 회귀 테스트 (GUI 미사용 가벼운 모드)**

```bash
ros2 launch orinbot_navigation navigation.launch.py gui:=false use_rviz:=false
```

**시뮬레이터가 이미 실행 중인 상태에서 SLAM + Nav2만 기동**

```bash
ros2 launch orinbot_navigation navigation.launch.py use_sim:=false
```

**이미 실행 중인 스택에 자동 탐사 기능만 추가**

```bash
ros2 launch orinbot_navigation explore.launch.py
```

**충전 도킹**

도킹은 기본으로 켜져 있습니다. 로봇은 잔량이 떨어지면 스스로 도크로 복귀하고, 충전이 끝나면 다시 나갑니다.

접근 방식은 `docking_mode` 로 고릅니다 — 기본 `staged`(단계 분리, 정밀), `smooth`(Nav2 순정 곡선 접근, 빠름).

```bash
# 손으로 도킹/언도킹 시키기
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: home_dock}"
ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot "{}"

# 잔량을 직접 낮춰 자동 복귀를 즉시 시험 (몇 시간 기다리지 않아도 됩니다)
ros2 topic pub --once /battery_sim/set_soc std_msgs/msg/Float32 '{data: 0.1}'

# 방전/충전을 60배속으로 돌려 전체 순환을 몇 분 안에 관찰
ros2 launch orinbot_navigation navigation.launch.py battery_speedup:=60 initial_soc:=0.3

# 도킹 기능 없이 (실기에서 도크를 안 쓸 때 / 자원을 아낄 때)
ros2 launch orinbot_navigation navigation.launch.py dock:=false

# 이미 실행 중인 스택에 도킹만 추가하거나, 도킹만 재기동
ros2 launch orinbot_navigation docking.launch.py
```

충전이 시작되면 **인지·항법을 재웁니다** (`power_save`, 기본 켜짐). Nav2 5개 서버는 lifecycle PAUSE 로, RTAB-Map 은 `/rtabmap/pause` 로 멈춥니다 — 죽이지 않으므로 포즈 그래프가 램에 남아 재위치추정 없이 즉시 복귀합니다. 복귀는 SLAM → Nav2 RESUME → 전역 코스트맵 확인 순서로만 진행되며, 확인 전에는 로봇을 움직이지 않습니다.

```bash
# 절전 없이 (디버깅용)
ros2 launch orinbot_navigation navigation.launch.py auto_dock:=false
```

동작 원리와 실측치는 아래 "충전 도킹" 절을 참고하세요.

**Nav2 파라미터만 수정했을 때 빠르게 재기동하는 방법**

전체 스택을 재시작(3분 이상 소요)하는 대신 Nav2 프로세스만 단독 재기동(약 45초 소요)합니다. 시뮬레이터와 SLAM은 유지한 채 Nav2 노드만 종료 후 재실행합니다.

```bash
ros2 launch orinbot_navigation nav2.launch.py
```

### 실물 기기(Jetson Orin Nano Super) 자원 실측

**2026-08-02 실기에서 직접 쟀습니다. 결론부터 — 자원은 부족하지 않습니다.**

| | 실측 |
|---|---|
| 유휴 | **2.8 / 6 코어** |
| 주행 중 (MPPI 가동) | **3.1 / 6 코어** |
| 메모리 최대 | **2.76 / 7.85 GB** |
| 최고 온도 | 60.1도 (`MAXN_SUPER`) |

- **병목은 CPU 이고 메모리가 아닙니다** (CPU 51% 대 메모리 35%). 자원을 더 쓰는 선택은 메모리로는 감당되지만 CPU 에서 막힙니다.
- **주행 시작 시 늘어나는 32 %p 는 거의 전부 `controller_server`(+18.8)와 `bt_navigator`(+8.9)** 입니다.
- `rgbd_odometry` + `rtabmap` 만으로 1.07 코어입니다. 절감을 논한다면 여기가 먼저입니다.
- **기본 이미지에는 스왑도 zram 도 없습니다.** rtabmap 이 튀면 곧바로 OOM 이므로 스왑파일을 두세요 (안전망 용도이므로 `vm.swappiness` 는 낮게).

아래 두 옵션은 **지금은 켤 이유가 없습니다.** 여유가 있는 상태에서 정확도만 버리는 거래이기 때문입니다. 실기에서 무언가를 더 얹어 CPU 가 막혔을 때만 꺼내세요.

```bash
# rtabmap CPU -38%, 메모리 -106MB / 위치 오차 중앙값 21 -> 27 mm 증가
ros2 launch orinbot_navigation navigation.launch.py detection_rate:=1.0

# 시각 오도메트리를 끄고 EKF(휠+IMU)만 사용. 절감은 가장 크지만 위치 정확도 손실도 가장 큼
ros2 launch orinbot_navigation navigation.launch.py use_vslam:=false
```

**먼저 손대야 할 곳은 도킹 노드입니다** — `staged_dock` 과 `dock_marker_board` 가 도크 근처가 아닐 때도 **0.62 코어**를 씁니다 (`dock_marker_board` 는 15 Hz 로 ArUco 검출을 계속 돌립니다). 정확도를 하나도 버리지 않고 회수할 수 있는 유일한 항목이며, `~/pause` 서비스가 이미 있으므로 호출만 붙이면 됩니다. 도크를 아예 안 쓴다면 `dock:=false`.

노드별 실측표, 측정 방법, 이미 시도했다가 기각된 파라미터 변경 목록은 `.claude/skills/orin-resource-budget/SKILL.md` 에 있습니다. **파라미터로 자원을 아끼려 하기 전에 그 문서를 먼저 보세요.**

#### RViz 를 어디서 띄울 것인가

원격(개발 PC)에서 띄우려면 **양쪽 모두** `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` 이어야 합니다 (`LOCALHOST` 면 서로 안 보입니다). 이 방식은 **네트워크가 멀티캐스트를 통과시켜야** 성립합니다 — 일부 공유기는 IGMP 스누핑으로 멀티캐스트를 버려서 유니캐스트는 되는데 ROS 2 탐색만 안 되는 상태가 됩니다. `ros2 multicast send` / `ros2 multicast receive` 로 먼저 확인하고, 막혀 있으면 두 기기를 언매니지드 스위치로 묶으세요.

로봇에 모니터를 달아 실기에서 직접 띄운다면 **`navigation.rviz` 를 그대로 쓰지 마세요.** `StvlVoxels`/`Map3D` 포인트클라우드와 `ColorImage` 가 비용의 대부분이고, 상태 확인에는 필요하지 않습니다. `Map` + `LocalCostmap` + `RobotModel` + `Scan` + `Path` 만 남긴 2D 뷰로 별도 설정을 만들고, 같이 `publish_voxel_map: false` (`nav2_params.yaml`) 로 두세요 — 이건 보는 쪽뿐 아니라 **발행하는 `controller_server` 쪽 부하도 같이 줄입니다.**

### 주요 실행 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `world` | `room.sdf` | 월드 파일. `maze.sdf` / `hall.sdf` / `office.sdf` (위 "월드" 절) |
| `database_path` | `~/.ros/orinbot_rtabmap.db` | RTAB-Map DB. **월드를 바꾸면 이것도 바꾸세요** |
| `explore` | `false` | 미탐색 구역 자동 탐사 여부 |
| `localization` | `false` | `true` 설정 시 기존 지도로 위치 추정만 수행 (지도 갱신 안 함) |
| `use_sim` | `true` | Gazebo 시뮬레이터 동시 실행 여부 |
| `use_rviz` / `gui` | `true` | RViz / Gazebo GUI 창 표시 여부 |
| `use_vslam` | `true` | `false` 설정 시 VSLAM을 끄고 EKF(휠+IMU)만 사용 |
| `detection_rate` | `2.0` | RTAB-Map 지도 노드 추가 주기 [Hz] |
| `memory_thr` | `3000` | 작업 메모리의 노드 개수 상한 (0=무제한). **낮추면 `/map` 이 깨집니다** |
| `map_3d` | `false` | 3D 점유 격자 생성 여부 (`true` 설정 시 메모리 사용량 2.7배) |
| `reg_strategy` | `2` | 루프 클로저 검증 방식 (0=영상 단독, 2=영상+ICP 융합) |
| `dock` | `true` | 충전 도킹 일체(마커 검출 + docking_server + 배터리) 실행 여부 |
| `docking_mode` | `staged` | `staged`=단계 분리(정밀), `smooth`=Nav2 순정 곡선 접근(빠름). 액션 규격은 동일 |
| `auto_dock` | `true` | 잔량이 떨어지면 자동 복귀. `false` 면 사람이 `/dock_robot` 을 직접 호출 |
| `clock_rate` | `100.0` | `/clock` 발행 주기 [Hz]. `0` 이면 Gazebo 원본을 그대로 통과 (위 설명 참고) |
| `battery_speedup` | `1.0` | 방전/충전 시간 배속. 시나리오 검증 시 `60` 정도로 올려 씁니다 |
| `initial_soc` | `0.85` | 시작 잔량 (0~1) |

전체 인자 목록은 `ros2 launch orinbot_navigation navigation.launch.py --show-args` 명령으로 확인할 수 있습니다.

### 실행 직후 필수 점검 사항

```bash
ros2 topic info /clock | grep Publisher
```

**`Publisher count: 1`** 상태인지 반드시 확인해야 합니다. 만약 출력이 2로 나타난다면, 이전 실행의 `parameter_bridge` 프로세스가 종료되지 않고 남아 새로 기동된 Gazebo에 중복 연결된 것입니다. 두 곳에서 `/clock`이 동시 발행되면 시간 타임스탬프가 꼬여 모든 노드의 TF 버퍼가 지속적으로 초기화되고, Nav2 및 RTAB-Map의 좌표 조회가 모두 실패하게 됩니다. 이 현상은 "간헐적 정지" 증상으로 나타나 원인 파악이 어렵습니다.

기존 잔여 프로세스 강제 정리 명령 (Python으로 실행된 노드까지 확실히 정리하기 위해 **명령줄 경로 전체를 매칭**합니다. 실행 파일명만 검색하면 `vodom_tf_relay.py` 같은 프로세스를 놓칠 수 있습니다):

```bash
ps -eo pid,args | grep -E '/opt/ros/jazzy|ros2_ws/install|[g]z sim' \
  | grep -v shell-snapshots | awk -v me=$$ '$1 != me {print $1}' | xargs -r kill -9
ros2 daemon stop      # 종료된 노드가 `ros2 node list`에 잔재로 남아있을 때 실행
```

URDF 모델이나 월드 파일을 수정한 후에는 반드시 위 정리 작업을 거친 뒤 재기동하세요. `gz sim` 서버가 남아 있으면 변경 이전의 모델을 그대로 재사용합니다.

### 시스템 구성

```
map --(rtabmap)--> vodom --(rgbd_odometry + vodom_tf_relay)--> odom --(EKF)--> base_footprint
```

`odom -> base_footprint` 는 **EKF 가 냅니다.** `diff_drive_controller` 쪽은 `enable_odom_tf: false` 로 꺼 두었습니다 — 둘 다 켜면 같은 변환을 두 노드가 발행해 좌표계가 꼬입니다.

- **시각 오도메트리 (Visual Odometry)**: `rtabmap_odom/rgbd_odometry`. 휠 오도메트리 데이터를 모션 추정의 초기 예측값(Guess)으로 활용하여 시각적 특징점이 부족한 구간에서도 안정적으로 동작합니다.
- **SLAM / 루프 클로저 / 점유 격자**: `rtabmap_slam/rtabmap` 노드가 `/map` 및 `map -> vodom` TF를 발행합니다. AMCL 및 map_server는 사용하지 않습니다.
- **Nav2 코스트맵 관측원 이원화**:
  - `obstacle_layer` — RPLIDAR A2M12의 `/scan` 데이터 활용. 360도 전방위 센서이므로 레이 트레이싱을 통한 장애물 해제(Ray clearing)가 정상 동작합니다.
  - `stvl_layer` (3D 복셀) — D435i의 `/camera/depth/points` 데이터 활용. 화각이 좁아(87°×58°) 로봇이 회전할 때 기존 장애물을 레이로 지울 기회가 없으므로 시간 감쇠(Time decay) 방식으로 장애물을 자동 소거합니다.
- **속도 제어 명령 경로**: `controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed` 및 수동 조종 `/cmd_vel_teleop`. 두 입력은 `twist_mux` 노드가 우선순위에 따라 중재하여 `/cmd_vel`로 최종 출력합니다. 이 중재 구성이 없으면 자율 주행 중 수동 개입이 불가능합니다.

### 경로 계획과 제어 — 좁은 공간에서 멈추던 원인

"로봇이 이유 없이 멈춰 있다가 몇 분 뒤 갑자기 지나간다"를 추적해 세 가지를 찾았고, 셋 다 **로그는 멀쩡한** 부류였습니다.

**1. MPPI 비평자가 꺼지고 있었습니다.** `threshold_to_consider` 의 판정이 **목표까지의 직선거리**이고 남은 경로 길이를 보지 않습니다(`mppi/tools/utils.hpp:242`). 그래서 벽 하나 너머에 목표가 있으면 — 직선은 가까운데 경로는 크게 우회하는 상황 — 경로를 따라가게 하는 비평자가 전부 꺼지고 목표 쪽 직선 인력만 남아 로봇이 벽으로 밀립니다. 미로에서 심했던 것은 셀 간격이 0.85 m 라 옛 문턱 1.4 m 가 주변 8칸을 다 덮었기 때문이고, 개활지가 멀쩡했던 것은 목표가 대개 몇 m 밖이라 문턱에 안 걸렸기 때문입니다.

| 비평자 | 이전 | 현재 |
|---|---|---|
| `GoalCritic` | 1.4 | **0.5** |
| `PathAlignCritic` | 0.5 | **0.25** |
| `PathFollowCritic` | 1.4 | **0.3** |

**2. NavFn 은 벽에서 떨어뜨릴 수단이 없습니다.** 비용 변환이 컴파일 상수(`COST_NEUTRAL 50`, `COST_FACTOR 0.8`)라 팽창 파라미터로 우회되지 않습니다 — 통로 0.85 m 에서 중앙과 10 cm 편심의 비용 차이가 `cost_scaling_factor` 10 일 때 45.7, 3 일 때 45.0 으로 사실상 같습니다. `SmacPlanner2D` 의 `cost_travel_multiplier` 가 그 손잡이라 전역 플래너를 바꿨습니다 (`4.0`).

| | NavFn | **SmacPlanner2D** |
|---|---|---|
| 미로 완주 (통로 0.85) | 378초 | **156초** |
| 미로 완주 (통로 0.75) | 304초 | **174초** |
| 컨트롤러가 0 을 낸 시간 | 있음 | **0초** |
| 기어가는 구간 | 88초 | **11초** |
| BT 복구 | 있음 | **0초** |

`NavFn` 은 `planner_plugins` 에 남겨 두었습니다. BT 의 `PlannerSelector` 로 실패가 반복될 때 전혀 다른 탐색으로 재시도할 수 있게 하기 위해서입니다.

**3. `memory_thr: 300` 이 `/map` 을 지우고 있었습니다.** 장기 기억으로 내려간 노드의 격자는 발행되는 지도에서도 사라집니다. 1733개 중 301개만 남아 **이미 그린 구역이 미탐색으로 되돌아갔고**, 그 가짜 경계를 프론티어로 잡아 탐사가 지도 가장자리를 오갔습니다(541초 → `3000` 에서 158초). 겉으로는 "이미 간 곳을 자꾸 다시 간다"로만 보입니다.

**측정으로 기각한 가설들** (전부 실측 후 버렸습니다): CPU 굶주림(코어 하나의 14%, 부하 2.2, GUI 를 꺼도 변화 없음), RTF 저하(1.000), 명령 전달 유실(20 Hz 무손실), 초기 헤딩 오차(출발 지연과 무상관), 경로 뒤집힘(방향 변화 중앙값 4.4도, 각속도 부호 반전 0.06회/초), 팽창 비용 크기(`cost_scaling_factor` A/B 효과 0%).

> **`progress_checker` 를 8 → 3초로 줄이는 것은 역효과였습니다.** BT 복구 시간이 21초 → 160초로 늘어 되돌렸습니다.

#### 탐사 노드에 함께 반영한 것

- **목표를 보내기 전에 전역 코스트맵으로 검증합니다.** `/map` 기준으로 고른 지점이 코스트맵에서는 내접/치명일 수 있고, 그러면 BT 가 복구를 6회 돌며 약 60초를 태운 뒤에야 포기합니다.
- **끼임 판정을 이동 거리에서 "목표까지 남은 거리"로 바꿨습니다.** 이동 거리로 재면 복구의 `BackUp`(0.35 m)이 `min_progress`(0.15)를 매번 넘겨 **복구가 돌 때마다 타이머가 초기화**됩니다. 실측에서 59초짜리 복구 루프 동안 30초 감지가 한 번도 안 울렸습니다.
- **후보가 전부 블랙리스트일 때 제자리에서 기다리지 않습니다.** 안 움직이면 TTL 이 풀려도 상황이 같습니다. 실패가 가장 적은 곳으로라도 다시 갑니다.
- **복귀 목표에 재시도(`home_retries: 5`)를 붙였습니다.** 한 번 실패하면 영구 정차했습니다.

### 최소 통과 가능 통로 폭: 0.70 m

**기준 판정 요소는 단순 직진 가능 여부가 아닌 제자리 회전 능력입니다.** 로봇은 막다른 통로에서 회차하여 빠져나올 수 있어야 하며, 차동 구동 로봇은 후진보다 제자리 회전을 우선 시도합니다 (Nav2의 `Spin` 복구 동작도 동일).

제자리 회전에 필요한 최소 통로 폭 = 대각선 길이 = √2 × 0.40 = **0.566 m**.

| 통로 폭 | 단방향 여유 | 중앙 제자리 회전 | 직진 시 허용 편심(오프셋) |
|---|---|---|---|
| 0.55 m | −8 mm | **회전 실패** | ±75 mm |
| 0.60 m | 17 mm | 통과 가능 | ±100 mm (**통로 끝단에서는 회전 불가**) |
| 0.70 m | 67 mm | 통과 가능 | ±150 mm |

0.60 m 폭 통로가 탈락한 이유: 직진 통과 중 로봇 위치의 편심 허용 범위(±100 mm) 끝단에 위치할 경우 회전이 불가능해집니다. 즉, 정상 주행 중 임의 위치에 멈추는 것만으로도 복구 회전이 불가능한 상태가 됩니다. 또한 단방향 여유 17 mm는 본 시스템의 SLAM 위치 추정 오차(중앙값 21 mm)보다 작아 위험합니다.

**따라서 SLAM 위치 추정 정확도와 통로 폭은 연계된 리소스 예산 관계입니다.** `detection_rate:=1.0` 설정처럼 SLAM 정확도를 저하시키는 옵션은 회전 여유 공간을 직접적으로 축소시킵니다.

### 충전 도킹

액션 규격은 Nav2 순정입니다 — `/dock_robot`, `/undock_robot`, `config/docking.yaml` 의 `dock_database`. 실기 전환 시 도크 좌표만 바꾸면 되고 BT 연동도 그대로 살아 있습니다. **접근 방식만 `docking_mode` 로 고릅니다.**

| | `staged` (기본) | `smooth` |
|---|---|---|
| 구현 | `scripts/staged_dock.py` (자체) | Nav2 순정 `opennav_docking` |
| 방식 | 정지 → 측정 → 보정 → 재측정 반복 | 마커를 추종하는 곡선 접근 |
| 세로 오차 (중앙값/최대) | **2.6 / 2.6 mm** | 20.3 / 22.7 mm |
| 가로 오차 (중앙값/최대) | 1.2 / 5.5 mm | **0.8 / 2.4 mm** |
| 각도 오차 (중앙값/최대) | 0.6 / 1.3도 | 0.4 / 1.8도 |
| 소요 시간 (중앙값) | 25.6초 | **8.6초** |

`staged` 를 기본으로 둔 이유는 **세로 오차가 한 자릿수 이상 좋기 때문입니다.** 정지 상태에서 잰 정확한 값을 그대로 최종 자세로 가져가는 반면, 곡선 접근은 그 측정값을 움직이면서 소비합니다. 대신 3배 느립니다 — 그 대가로 "어느 단계에서 틀어졌는지 로그만 보면 아는" 검사 가능성을 얻습니다.

```
Nav2 --(staging pose, 도크 앞 0.7 m)--> staged_dock / docking_server --> 도크
카메라 --> dock_marker_board (마커 3장 보드) --> /detected_dock_pose --> SimpleChargingDock
battery_sim / 실기 BMS --> /battery_state --> auto_dock (저전압 복귀 판단)
```

**일반 Nav2 주행으로 도크에 붙일 수 없는 이유가 두 가지 있습니다.** 코스트맵 팽창(0.40 m)이 벽 앞을 통째로 막아 경로계획 자체가 되지 않고, SLAM 위치 오차(중앙값 21 mm / 90% 46 mm)가 충전 접점이 요구하는 정밀도보다 큽니다. 그래서 마지막 구간은 지도 좌표가 아니라 **지금 카메라에 보이는 마커**를 기준으로 붙습니다.

**마커 3장을 하나의 보드로 풉니다.** 스테이션에 가이드 벽이 없어 최종 각도를 인식이 전적으로 결정하는데, 평면 마커 한 장은 자세 모호성 때문에 각도가 1.3 m 에서 4.77도까지 튑니다. 좌우로 벌린 3장의 코너 12개를 함께 풀면(`estimatePoseBoard`) 같은 자리에서 각도 오차가 0.01도 수준으로 떨어집니다.

**`smooth` 에서만: 자세를 0.65 m 에서 확정하고 그 뒤로는 직진합니다.** 더 가까이 가면 마커는 커지지만 바깥 두 장이 화각을 벗어나기 시작하고, 마지막 프레임이 가장 부정확합니다. `docking_server` 는 마지막 검출값을 목표로 얼려서 들어가므로 그대로 두면 **가장 나쁜 관측으로 마무리**하게 됩니다.

**`staged` 의 정렬 완료 판정은 축별 허용치가 아니라 "접촉 시점의 예상 횡오차" 하나입니다.** 횡오차와 각도오차는 독립이 아니어서, 정렬을 마치고 직진하는 동안 남은 각도가 그대로 횡오차로 바뀝니다:

```
접촉 시점 횡오차 = 정렬시 횡오차 − 직진거리 × sin(각도오차)
```

0.52 m 직진 기준으로 0.5도가 4.5 mm 입니다. 그래서 **예전 설정(횡 3 mm / 각도 0.5도)은 서로 앞뒤가 안 맞았습니다** — 허용한 각도만으로 4.5 mm 가 생기는데 그보다 작은 3 mm 를 맞추려고 크랩 기동을 반복했습니다(회당 15~20초 손해). 지금은 `contact_lateral_budget` (동판 허용 ±34 mm 의 절반인 15 mm) 하나로 판정하고 크랩 보정량도 이 예상값을 씁니다. 각도에는 느슨한 상한만 두는데 이건 정밀도용이 아니라 측정 이상 감시용입니다. 예측 모델의 실측 검증(예상 → Gazebo 실제): **+10.4 → +5.5 mm, +0.9 → +0.9 mm, +0.1 → +1.2 mm** — 부호가 맞고 크기는 보수적입니다.

| 항목 | 값 | 근거 |
|---|---|---|
| 마커 | ArUco `DICT_4X4_50` id 1/0/2, 0.10 m, 0.16 m 간격 | 424×240 에서 4×4 는 24 px 이면 읽힘 (6×6 은 32 px 필요) |
| 마커 중심 높이 | 0.31 m | 화각 계산상 최적. 낮추면 근거리에서 화면 아래로 잘림 |
| 자세 확정 거리 | 0.65 m (로봇 중심 기준) | 0.55 m 부근에 검출 절벽. 그 앞에서 확정 |
| 확정 지점 정확도 | 거리 ~1 mm / 가로 ~0.6 mm / 각도 ~0.01도 | `tools/dock_calib.py` 실측 |
| 동판 / 포고핀 | 75×100 mm / 6핀 2×3 피치 2.54 mm ×2 | 접촉 허용 세로 ±48 mm, 가로 ±34 mm (동판 − 핀 배열 6.1×3.5 mm) |
| 포고핀 브래킷 높이 | 0.045 m | **0.030 ~ 0.060 m 사이에만 놓을 수 있음** (아래 참고) |

**브래킷 높이가 이 로봇에서 가장 빡빡한 치수입니다.** 접지고가 0.060 m 인데 코스트맵 `min_obstacle_height` 가 0.030 m 라, 로봇은 30 mm 미만 물체를 장애물로 보지 않고 그냥 밟고 지나갑니다. 브래킷을 그보다 낮추면 코스트맵이 무시한 문턱에 그대로 걸리고, 0.060 m 위로 올리면 섀시 안에 들어가 동판에 닿지 않습니다. 그래서 **동판을 바닥에 평평하게 깔 수 없고 0.040 m 높이로 올려야 합니다.** 좌우 위치도 바퀴(구동륜 ±0.17, 캐스터 ±0.14)를 피해 |y| < 0.11 띠 안에 들어가야 합니다 — 바퀴가 동판 위로 올라가면 로봇 전체가 같이 들려 간격이 그대로 유지됩니다.

접근 구간(마지막 0.7 m)에서는 코스트맵 충돌 검사를 끕니다 — 도크 패널과 뒷벽의 팽창이 그 구간 전체를 덮어 켜 두면 도킹이 실패합니다. **실기에서는 이 구간을 위해 범퍼 스위치나 모터 전류 제한 같은 소프트웨어 밖의 보호 수단이 필요합니다.**

측정 도구는 `tools/dock_calib.py`(회전 보정값·인식 정확도), `tools/dock_range.py`(검출 범위·편차), `tools/dock_test.py`(반복 도킹 성공률·정렬 오차)입니다. 실측 수치와 함정 사례는 `CLAUDE.md` 의 "충전 도킹" 절에 정리되어 있습니다.

### 주요 시스템 한계 사항

- **D435i 카메라의 측후방 사각지대**: 뎁스 화각이 좁아(87°×58°) 측면과 후방을 보지 못합니다. 360도 라이다가 이를 보완하지만, 라이다 스캔 평면 높이가 지면 기준 **0.49 m**에 위치하므로 이보다 낮은 장애물은 원리적으로 감지하지 못합니다. 낮은 장애물 감지는 카메라가 담당합니다.
- **VSLAM의 시각적 특징점 의존성**: VSLAM은 카메라 영상의 시각 특징점을 기반으로 위치를 추정합니다. 뎁스 센서는 특징점에 3D 좌표를 부여하는 역할을 하므로, 밋밋하고 무늬가 없는 벽면 앞에서는 뎁스가 아무리 정확해도 오도메트리가 작동하지 않습니다. `worlds/generate_room.py` 스크립트에서 벽 세그먼트마다 **고유한 텍스처**를 생성하는 이유가 여기에 있습니다. 동일한 텍스처를 재사용하면 서로 다른 위치를 동일한 장소로 오인하여 잘못된 루프 클로저가 형성되고 위치 추정 좌표가 수 미터 튀게 됩니다.
- **협소 공간 진입 시 지도 손상 위험**: 로봇이 여유 공간이 부족한 벽면에 부딪히며 회전하면 바퀴에 슬립(미끄러짐)이 발생하여 오도메트리 오차가 누적되고 포즈 그래프 전체가 왜곡됩니다.
  실측 데이터: 회전 여유 공간을 고려하지 않고 자동 탐사를 수행한 결과 완주에 4526초가 소요되었으며 지도가 몇 도 기울어지는 왜곡이 발생했습니다. 0.70 m 회전 여유 기준을 적용한 후에는 왜곡 없이 188초 만에 탐사를 완료했습니다.

더 상세한 실측 분석 기록과 시스템 함정 사항은 `CLAUDE.md` 파일에 정리되어 있습니다.
