# orinbot workspace — 작업 지침

ROS 2 **Jazzy** + **Gazebo Harmonic** (Ubuntu 24.04). 개발은 시뮬레이션에서 하고, 배포 타깃은 **Jetson Orin Nano Super** 입니다 (실기 자원 실측 완료, 카메라·라이다 실물은 아직 없음).

이 문서는 **지금 지켜야 할 규칙과 현재 설정값**만 담습니다.
각 결정이 왜 그렇게 됐는지, 어떤 원리로 그 해결법이 통했는지는 **`docs/ros2-lessons.md`** 에 있습니다. 설계를 바꾸려 하기 전에 그쪽을 먼저 보세요.

## 개발 환경 전제 조건

- ROS 배포판: Jazzy (`/opt/ros/jazzy`)
- Gazebo: Harmonic (`ros-jazzy-gz-*-vendor` 로 설치, osrfoundation 저장소 미사용)
- 명령 실행 전 항상: `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash`
- `sudo` 는 비밀번호 입력이 필요하므로 **apt 패키지 설치는 사용자에게 요청할 것**

## 핵심 아키텍처 원칙

- **Gazebo 기본 DiffDrive 플러그인을 쓰지 않습니다.** 구동은 `ros2_control` + `gz_ros2_control` + `diff_drive_controller` 로 처리합니다. 실물 전환 시 URDF 의 `<hardware>` 블록만 교체하면 상위 스택이 그대로 살아 있게 하기 위함입니다.
- **센서 토픽·TF 프레임 이름은 실제 하드웨어 규격에 맞춥니다.** D435i 는 `realsense2_camera` 표준(`camera_name:=camera`, `camera_namespace:=""`)을 따릅니다. 시뮬레이션 전용 이름을 새로 만들지 마세요.
- 속도 명령 규격은 **`geometry_msgs/TwistStamped`** 입니다 (Jazzy `diff_drive_controller` 기본).
- 로봇 치수는 `orinbot.urdf.xacro` 상단 파라미터 블록에만 정의합니다 (하드코딩 금지). 치수를 바꾸면 `orinbot_bringup/config/controllers.yaml` 의 `wheel_separation` / `wheel_radius` 도 함께 고쳐야 합니다.

## 빌드, 실행 및 검증

표준 `colcon` 빌드입니다. 상황별 실행 명령은 `README.md` 를 참고하세요.

**파라미터를 바꿨으면 `tools/` 로 측정해 검증해야 합니다.** 측정 방법과 왜곡 주의사항은 `tools/README.md` 에 있고, 거기 있는 기준선과 직접 비교하세요.

## 월드 4개 — 서로를 대체하지 않습니다

| 월드 | 크기 | 목적 | 텍스처 |
|---|---|---|---|
| `room.sdf` | 10×8 m | **회귀 기준선.** 문서의 수치는 전부 여기서 나옵니다 | 커밋됨 |
| `maze.sdf` | 6×6 m | 좁은 통로 스트레스 (통로 0.75 m, 7×7 땋은 미로) | **생성 필요** |
| `hall.sdf` | 24×18 m | 배포 규모. 긴 주행 누적 오차, 넓은 곳의 탐사 수렴 | **생성 필요** |
| `office.sdf` | 20×14 m | 실전형 최종 시험. 무늬 없는 벽 + 가구 + 보행자 3명 | 커밋됨 |

- 전부 생성기(`worlds/generate_*.py`)가 만듭니다. **SDF 를 직접 고치지 말고 생성기를 고친 뒤 다시 내보내세요** — 설계 근거가 생성기 주석에 있습니다.
- **미로/홀 텍스처는 저장소에 없습니다** (176장 × 435 KB). `generate_textures.py maze hall` 로 만드세요 (3.9초, 시드 고정이라 바이트 단위로 동일). 없으면 벽이 흰색으로 떠서 **시각 오도메트리가 그대로 실패합니다.**
- **월드를 바꾸면 `database_path` 도 바꾸세요.** 한 DB 에 이어 붙이면 RTAB-Map 이 이전 월드의 포즈 그래프 위에 매핑합니다.
- `hall`/`office` 생성기는 `docking.yaml` 의 `home_dock.pose` 를 읽어 대조하고 어긋나면 `sys.exit(1)` 합니다. 남쪽 벽을 `room.sdf` 와 같은 `y = -4.0` 에 둔 것이 그래서입니다.
- **`office.sdf` 의 보행자는 `people_sim.py` 가 움직입니다** (별도 실행). 셋의 속도가 서로 달라야 마주치는 상황이 하나로 고정되지 않습니다 (1.20 / 0.55 / 0.85 m/s, 로봇 0.40). 로봇이 진행 방향 ±75도 안 1.4 m 에 들어오면 멈추고 1.8 m 에서 재개하며, 12초 뒤에는 돌아섭니다 (양쪽이 마주 선 채 교착되는 것을 막습니다).
  - 사람 원기둥은 **중력을 꺼야 합니다.** r=0.22 / h=1.7 원기둥은 물리적으로 불안정해 넘어지고, 로봇 스폰 지점과 겹치면 70 kg 이 로봇을 밀어 같이 넘어뜨립니다. 생성기가 스폰 거리 1.5 m 를 assert 합니다.
  - SDF 안에서 원을 그리게 두면 안 됩니다 — 반지름(1.5~1.8 m)이 복도 폭(1.8 m)보다 커서 벽을 뚫고 다닙니다.

## Gazebo 주의사항

- **포인트클라우드 토픽(`/camera/points`)을 직접 쓰지 마세요.** gz-sim 은 좌표를 카메라 본체 규약(x=전방)으로 채우면서 `frame_id` 에는 광학 좌표계(z=전방) 이름을 붙입니다. `optical_frame_id` / `gz_frame_id` 를 바꿔도 이름만 바뀌고 축 변환은 일어나지 않습니다. `depth_image_proc::PointCloudXyzrgbNode` 로 뎁스 + camera_info 에서 합성하세요.
- 카메라 계열 센서는 `<camera><optical_frame_id>` 가 `<gz_frame_id>` 보다 우선합니다. IMU 등 다른 센서는 `<gz_frame_id>` 가 적용됩니다.
- `XML Element[gz_frame_id] ... not defined in SDF` 는 libsdformat 스키마 경고일 뿐이고 gz-sim 은 값을 정상적으로 읽습니다. 무시해도 됩니다.
- **XML 주석 안에 `--` 를 쓰면 xacro 파싱이 깨집니다.**
- **`IncludeLaunchDescription` 의 `launch_arguments` 는 부모 스코프로 샙니다.** 하위 런치에 `use_rviz:=false` 를 넘기면 상위의 `use_rviz` 까지 덮여 상위 RViz 가 안 뜹니다. include 는 `GroupAction` 으로 감싸 스코프를 격리하세요.
- URDF/월드를 고친 뒤에는 Gazebo 를 완전히 종료하고 재기동하세요. `gz sim` 서버가 남아 있으면 옛 모델이 재사용됩니다.
- 카메라 영상이 안 나오면 월드 SDF 에 `gz-sim-sensors-system` 플러그인이 있는지 확인하세요.

## 센서 역할 분담

치수·주기·프레임 같은 사양값은 출처 파일(`orinbot.urdf.xacro` 상단 블록, `d435i.urdf.xacro`, `rplidar_a2m12.urdf.xacro`, `generate_room.py`)을 참조합니다. 여기에는 설계 근거만 적습니다.

- **라이다 스캔 평면은 지면 0.49 m 이고, 그보다 낮은 장애물은 물리적으로 감지할 수 없습니다.** 낮은 장애물은 D435i 가 전담합니다.
- **월드의 다층 선반 3종은 센서 분담 시험 장치입니다** (`generate_room.py` 의 `SHELVES`):
  - `shelf_0`: 상판이 라이다 평면에 **걸려** 2D 만으로도 벽면 장애물로 인식됨
  - `shelf_1`: 상판이 라이다 평면을 **통과함**. 스캔에는 기둥 4개만 보여 "통행 가능"으로 읽히지만 실제로는 상판이 튀어나와 있음. **STVL 이 없으면 Nav2 가 이 아래로 경로를 냅니다.**
  - `shelf_2`: 측면이 막힌 구조 + 상단 물품 (시각 특징점이 풍부)

## Nav2 구성

- 컨트롤러는 **MPPI**, 가중치와 샘플 수는 **Nav2 순정 기본값**입니다. **가중치를 건드리지 마세요** — 비평자들이 서로 물려 있어 하나만 올리면 "정지"가 최적해가 됩니다.
- 순정이 아닌 것은 **`threshold_to_consider` 3개뿐**입니다:

  | 비평자 | 값 |
  |---|---|
  | `GoalCritic` | 0.5 |
  | `PathAlignCritic` | 0.25 |
  | `PathFollowCritic` | 0.3 |

  이 문턱은 목표까지의 **직선거리**로만 판정하며 남은 경로 길이를 보지 않습니다 (`nav2_mppi_controller/tools/utils.hpp:242`). 환경의 셀 간격보다 문턱이 크면 로봇이 벽으로 밀립니다.
- 전역 플래너는 **`SmacPlanner2D`** (`cost_travel_multiplier: 4.0`). `NavFn` 은 `planner_plugins` 에 예비로 남아 있습니다 — BT 의 `PlannerSelector` 로 다른 원리의 탐색을 재시도할 여지용입니다.
  - **`downsample_costmap` 은 꺼야 합니다.** 통로 0.85 m 에 격자 0.05 m 라 해상도를 낮추면 통로가 사라집니다.
  - `NavFn` 의 비용 변환은 컴파일 상수(`COST_NEUTRAL 50`, `COST_FACTOR 0.8`)라 팽창 파라미터로 경로를 벽에서 떼어놓을 수 없습니다. 그쪽으로 튜닝하지 마세요.
- 코스트맵 레이어:
  - `obstacle_layer` (2D): RPLIDAR `/scan` 전용. 360도라 광선 소거가 정상 동작합니다.
  - `stvl_layer` (3D 복셀): D435i `/camera/depth/points` 전용. 화각이 87×58도뿐이라 회전 시 광선으로 소거할 수 없어 시간 감쇠를 씁니다 (`voxel_decay: 10.0`, 실측 등 돌린 뒤 9초 내 100→0).
  - `inflation_layer`: `cost_scaling_factor: 10.0`, `inflation_radius: 0.40`
- **`footprint` 로 실제 사각형(0.4×0.4 m)을 명시합니다.** 원형 근사(`robot_radius`)를 쓰면 0.8 m 문을 통과 불가로 봅니다. MPPI 의 `CostCritic.consider_footprint: true` 도 함께 켜야 합니다 — 둘 중 하나만으로는 안 됩니다.
- **전역 코스트맵 해상도는 yaml 이 아니라 `/map` 이 정합니다.** `StaticLayer` 가 수신 지도 크기에 맞춰 재조정하므로 RTAB-Map 의 `Grid/CellSize`(0.05 m, `slam.launch.py`)가 실제 해상도입니다. 로컬 코스트맵만 yaml 값(0.02 m)을 씁니다.
- **`behavior_server` 에는 `enable_stamped_cmd_vel` 이 없습니다.** 항상 TwistStamped 를 냅니다. 이 파라미터는 `controller_server` 와 `velocity_smoother` 에만 있습니다.
- **`velocity_smoother` 는 `stamp_smoothed_velocity_with_smoothing_time: True` 가 필요합니다.** false 면 입력 명령의 스탬프를 재사용해 `diff_drive_controller` 가 "0.5초 넘게 지난 명령"으로 보고 무시합니다.
- 속도 명령 경로:
  `controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed`
  `수동 조종 → /cmd_vel_teleop`, 두 입력은 `twist_mux` 가 중재해 `/cmd_vel` 로 냅니다.
- **`progress_checker` 는 8초입니다. 줄이지 마세요** — 3초로 하면 복구가 자리 잡기 전에 다음 복구가 들어와 BT 복구 시간이 21 → 160초가 됩니다.
- **`bt_navigator` 의 `default_server_timeout` 은 200 ms 입니다** (순정 20 ms). 이것은 BT 액션 노드가 하위 서버의 **목표 수락 응답**을 기다리는 예산이라, 넘기면 그 자리에서 FAILURE 가 나 주행 목표 전체가 실패합니다 (`nav2_behavior_tree/bt_action_node.hpp:230`). 대기 중에도 BT 는 RUNNING 을 반환하며 계속 틱을 돌리므로 늘려도 막히지 않고, 실패 감지만 늦어집니다. **이 증상은 네트워크 인터페이스 개수나 CPU 부하와 무관합니다** — 인터페이스 3개(`lo`/`enp4s0`/`docker0`, `docker0` 은 DOWN), 28코어에 부하 5 인 환경에서도 났습니다.

## 공간 예산 — 최소 통과 폭 0.70 m

- **판정 기준은 직진 통과가 아니라 제자리 회전입니다.** 차동구동은 막다른 곳에서 후진보다 회전을 먼저 시도합니다 (Nav2 `Spin` 복구 포함).
- 기하학적 최소 = √2 × 0.40 = **0.566 m**. 운용 최소는 **0.70 m** 입니다.

  | 통로 폭 | 단방향 여유 | 중앙 제자리 회전 | 직진 중 허용 편심 | 회전 실패 |
  |---|---|---|---|---|
  | 0.55 m | −8 mm | **실패** | ±75 mm | 항상 |
  | 0.60 m | 17 mm | 통과 | ±100 mm | **100 mm 편심에서 실패** |
  | 0.70 m | 67 mm | 통과 | ±150 mm | 60 mm 편심에서 정상 |
  | 0.90 m | 167 mm | 통과 | ±250 mm | 100 mm 편심에서 정상 |

- 0.60 m 가 탈락한 이유: 주행 편심 범위(±100 mm) 끝에서 멈추면 돌 수 없고, 단방향 여유 17 mm 가 SLAM 오차(중앙값 21 mm / 90% 46 mm)보다 작습니다.
- **따라서 SLAM 정확도와 통로 폭은 하나의 예산입니다.** 정확도를 떨어뜨리는 변경(`Rtabmap/DetectionRate` 감소 등)은 통과 여유를 직접 깎습니다.
- 시험할 때 **로봇을 통로 안으로 순간이동시키지 마세요.** Gazebo 는 정적 물체와의 초기 관통을 밀어내지 않아 벽에 묻힌 채로 회전합니다. 반드시 밖에서 주행으로 진입시키세요.

## 임무 사이클

로봇청소기와 같은 모델입니다. **도크에서 시작해 임무가 없으면 계속 도크에서 대기하고**, 명령을 받으면 나갔다가 끝나면 스스로 돌아옵니다. 완충되어도 나가지 않습니다 — 실기 배터리는 내장 BMS 가 전류를 끊으므로 붙은 채로 두면 됩니다.

```
도크 대기 -> 명령 수신 -> 절전 해제 -> 언도킹 -> 임무 수행 -> 복귀 -> 도크 대기
```

시험 런처는 `mission.launch.py` 이고, 명령은 다른 터미널에서 보냅니다.

```bash
ros2 launch orinbot_navigation mission.launch.py            # 띄워 두고
ros2 service call /mission/start_mapping std_srvs/srv/Trigger   # 자동 매핑
ros2 service call /mission/cancel        std_srvs/srv/Trigger   # 중단 후 복귀
ros2 topic echo /mission/state                                   # 진행 상황
```

- **역할 분담**: `auto_dock.py` 는 도킹 스테이션 담당(배터리 감시·절전·도킹/언도킹 실행), `mission_manager.py` 는 임무 담당(명령 수신·순서 지휘). 임무 관리자는 `~/leave` / `~/return` 서비스로 요청만 하고 완료는 `/dock_state` 로 확인합니다.
- **`auto_undock` 은 false 입니다.** true 로 두면 완충될 때마다 임무와 무관하게 로봇이 나갑니다.
- **절전 해제가 끝난 것을 확인한 뒤에 언도킹하고, 언도킹이 끝난 뒤에 임무를 켭니다.** 도크에 있는 동안 Nav2 는 PAUSE 라 이 상태에서 낸 주행 목표는 전부 즉시 거절되고, 증상은 "명령을 넣었는데 아무 일도 안 일어남"으로만 보입니다.
- **탐사 노드는 `start_paused:=true` 로 띄웁니다.** 안 그러면 임무 명령 없이 부팅 직후 로봇이 나갑니다.
- **임무를 다시 시작하기 전에 `/frontier_explorer/reset` 을 부릅니다.** 완료 플래그는 한 번 서면 남아 있어, 지우지 않으면 두 번째 임무가 시작하자마자 끝납니다.
- **임무 중 배터리가 떨어지면 `auto_dock` 이 스스로 끌고 들어가고, 임무는 `SUSPENDED` 로 살아 있다가 충전 후 이어서 합니다.** 처음부터 다시 하면 이미 그린 곳을 또 돕니다.
- **임무 사이클에서는 탐사의 `return_home` 을 끕니다.** 도킹이 진입점까지 데려가므로 같은 길을 두 번 갑니다.
- **`mission.launch.py` 는 `navigation.launch.py` include 를 `GroupAction` 으로 감싸지 않습니다.** 하위 런치가 SLAM/Nav2/도킹을 `OnProcessExit` 으로 나중에 띄우는데, `GroupAction` 은 include 가 끝날 때 스코프를 닫아 버려 뒤늦게 실행되는 핸들러가 인자를 못 찾습니다 (`launch configuration 'localization' does not exist` 로 스택 전체가 내려갑니다). **이벤트 핸들러를 쓰는 런치를 include 할 때는 스코프를 살려 두세요.**

임무를 추가하려면 `mission_manager.py` 의 `MISSIONS` 에 항목과 실행 함수를 더하면 됩니다. 깨우기·언도킹·복귀는 공통 절차라 임무 쪽에서 다시 쓸 필요가 없습니다.

## 자동 탐사 (프론티어)

```bash
ros2 launch orinbot_navigation navigation.launch.py explore:=true
ros2 launch orinbot_navigation explore.launch.py     # 스택이 이미 떠 있을 때
```

구현: `orinbot_navigation/scripts/frontier_explorer.py`. `/map` 에서 탐색/미탐색 경계 셀을 군집화하고 `거리 − gain × 경계길이` 가 최소인 곳으로 `NavigateToPose` 를 보냅니다. 실측 10×8 m 방을 약 3분에 완료합니다.

외부 패키지를 쓰지 않는 이유: Jazzy 공식 목록에 탐사 패키지가 없고, `explore_lite` 등은 자체 `costmap_2d` 인스턴스를 새로 만들어 Orin 예산에 불리합니다.

**지켜야 할 규칙**

- **목표 선별 기준은 "진입 가능"이 아니라 "회차 가능"입니다.** `min_clearance: 0.33` — 내접 반지름(0.18)이나 외접 반지름(0.283)이 아닙니다. 0.283 을 쓰면 0.60 m 통로(중앙 여유 0.300)가 잘못 통과됩니다.
- **`/map` 을 구독하는 모듈은 QoS 를 TRANSIENT_LOCAL 로 맞춥니다.** RTAB-Map 은 지도 갱신 시에만 발행하므로, VOLATILE 로 받으면 로봇이 서 있을 때 영구 대기에 빠집니다.
- **목표를 보내기 전에 전역 코스트맵으로 검증합니다.** `/map` 에서 자유인 셀이 팽창·STVL 때문에 내접(99)/치명(100)일 수 있고, 그런 목표는 BT 복구를 6회 돌며 약 60초를 태운 뒤에야 포기합니다.
- **준비 완료 신호는 `global_costmap/costmap` 을 1회 이상 수신한 시점입니다.** "액션 서버 연결됨"은 `bt_navigator` 가 살아 있다는 뜻뿐이고, 그 시점의 목표는 전부 즉시 ABORT 되어 프론티어 전체가 블랙리스트에 오릅니다.
- **결과 코드를 그대로 실패로 세면 안 됩니다.**
  - `ABORTED(6)` 은 선점(preemption)일 수 있습니다 → 목표마다 시퀀스 번호를 붙여 옛 시퀀스의 결과는 무시
  - `not accepted` 는 위치 문제가 아니라 서버 준비 미완료 → 재시도
  - `STATUS_UNKNOWN(0)` 도 연결 이상 → 재시도
- **성공 응답은 좌표로 재검증합니다.** 플래너 `tolerance`(0.5 m) 때문에 목표가 진입 불가 지역이면 현재 위치까지의 경로가 반환되고 컨트롤러가 즉시 "Reached the goal!" 을 냅니다 (실측 0.70 m 떨어진 곳에 253회 연속).
- **끼임 판정은 이동 거리가 아니라 목표까지 남은 거리로 합니다.** 복구의 `BackUp`(0.35 m)이 `min_progress`(0.15)를 매번 넘겨 타이머가 초기화됩니다.
- **군집 크기는 원본 격자로 재고, 여유값은 군집 안의 목표 좌표를 고를 때만 씁니다.** 팽창시킨 격자로 군집을 세면 0.60 m 통로 입구가 2셀로 줄어 통로 전체가 후보에서 사라집니다.
- **진입 불가능한 미탐색 구역은 영구 블랙리스트입니다.** 일시 블랙리스트만 두면 탐사가 끝나지 않습니다.
- **후보가 전부 블랙리스트일 때 제자리에서 기다리지 않습니다.** 안 움직이면 TTL 이 풀려도 상황이 같습니다. 실패가 가장 적은 곳으로라도 다시 보냅니다.
- **복귀 목표에도 재시도가 필요합니다** (`home_retries`).

## VSLAM (RTAB-Map)

- **VSLAM 은 RGB 특징점으로 동작합니다.** 뎁스는 특징점에 3D 좌표를 붙이는 역할입니다. 무늬 없는 벽 앞에서는 뎁스가 정확해도 오도메트리가 서지 않습니다.
- **벽면 텍스처를 재사용하지 마세요.** 서로 다른 장소가 같은 위치로 오인되어 틀린 루프 클로저가 걸리고 자세가 수 m 튑니다. `generate_textures.py` 의 `N_WALL_SEGMENTS` 를 늘려 모든 벽면에 고유 텍스처를 주세요 (현재 28개 생성, 26개 사용).
- **RGB/뎁스 해상도는 424×240 / 15 Hz 입니다.** 848×480 / 30 Hz 로 올리면 Gazebo 가 두 스트림을 같은 틱에 못 내보내 최대 0.13초 어긋나고, 제자리 360도 회전에서 오차가 1.2 m / 87도까지 벌어집니다 (424×240 에서는 3 mm / 0.03도).
- **`rgbd_odometry` 의 TF 를 직접 쓰지 마세요.** 연산 지연으로 스탬프가 평균 0.2초(최악 0.85초) 과거라 Nav2 의 `map→odom` 조회가 미래 외삽이 되어 실패합니다. `publish_tf:=false` 로 두고 `vodom_tf_relay.py` 가 50 Hz 로 미래 스탬프를 붙여 재발행합니다.
- **`Grid/Sensor: 1`** (0=레이저, 1=뎁스). 0 이면 `/map` 이 전부 미탐색으로 나옵니다.
- **`Grid/3D: false`** — 켜면 rtabmap 단독 914 → 2656 MB, 전체 1.66 → 3.65 GB. Nav2 는 2D `/map` 만 보고 3D 장애물은 STVL 이 담당하므로 자율주행에 영향이 없습니다. 3D 지도를 눈으로 볼 때만 `map_3d:=true`.
- **`Rtabmap/MemoryThr: 3000`.** 이것은 메모리 손잡이가 **아닙니다** — 노드 개수만 제한하고 점유 격자 캐시는 계속 누적되며, 장기 기억으로 내려간 노드의 격자가 **발행되는 `/map` 에서도 사라집니다.** 300 으로 두면 이미 그린 구역이 미탐색으로 되돌아가 탐사가 지도 가장자리를 오갑니다 (방 탐사 541초 vs 158초). 메모리는 `Grid/3D` / `Grid/RangeMax` 로 잡으세요.
- **`Rtabmap/DetectionRate: 2.0`** (기본값 유지). 1.0 으로 내리면 CPU 25.1 → 15.5 %p, 메모리 552 → 446 MB 로 줄지만 SLAM 오차 90% 값이 46 → 52 mm, `map→odom` 최대 보정량이 0.110 → 0.188 m 로 늘어 통로 통과 여유를 직접 깎습니다. 실기가 주행 중 3.1/6 코어라 지금은 켤 이유가 없습니다.
- **`Reg/Strategy: 2`** (영상 + ICP). 3D 격자를 끄면 비용 부담 없이 쓸 수 있고, 벽 패턴이 비슷한 실내에서 오조합 루프 클로저를 막습니다 (`map→odom` 최대 보정량 1.317 → 0.141 m, 오차 90% 0.323 → 0.056 m).
- **컬러를 특징점 입력으로 쓰려면 정합된 뎁스가 필요합니다.** `camera_color_optical_frame` 은 depth/infra1 에서 **15 mm** 떨어져 있습니다 (실물 D435i 와 동일). 실기는 `align_depth.enable:=true`, 시뮬레이션은 컬러 프레임 위치에 뎁스 센서를 추가로 둡니다 (렌더링 4개).
- **실기 IR 프로젝터 주의**: D435i 의 IR 도트가 infra 영상에 찍히는데 로봇과 함께 움직이므로 고정 특징점이 되지 못합니다. infra 를 VSLAM 입력으로 쓸 때 `emitter_enabled:=false`(뎁스 품질 하락) 또는 `emitter_on_off:=true`(프레임률 절반) 를 검토하세요. 시뮬레이션에는 IR 프로젝터가 없어 관찰되지 않습니다.

## Nav2 라이프사이클 워치독

- 네트워크 인터페이스가 여러 개면 FastDDS 서비스 응답 매칭이 지연되어 `failed to send response to .../change_state` 와 `Failed to bring up all requested nodes` 로 기동이 멈춥니다. `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 가 필요하지만 그것만으로 완전히 없어지지 않습니다.
- 증상이 잘 안 보입니다 — 노드 프로세스도 액션 서버도 정상인데 상태가 `inactive`/`unconfigured` 라 **모든 목표가 즉시 ABORT** 됩니다.
- `scripts/nav2_startup_watchdog.py` 가 `is_active` 를 감시하다 실패 시 `manage_nodes` 로 **RESET 후 STARTUP** 을 재요청합니다 (감지 후 2초 내 5개 서버 복구). **RESET 을 먼저 불러야 합니다** — 일부가 active 인 상태에서 STARTUP 만 보내면 "already active" 로 실패합니다. 끄려면 `startup_watchdog:=false`.
- **워치독에는 `use_sim_time` 을 적용하지 않습니다.** `/clock` 에 의존하면 시뮬레이터가 시계를 내기 전에 대기 상태에 빠져 감시해야 할 구간에 잠들어 있게 됩니다.
- Orin 실기에서 이 현상이 더 잦은지는 아직 확인하지 못했습니다.
- **아래 두 가지는 여기에 해당하지 않습니다. 인터페이스 탓으로 돌리지 마세요.**
  - `Timed out while waiting for action server to acknowledge goal request for ...` → `bt_navigator` 의 `default_server_timeout` 초과입니다 (아래 Nav2 구성 참고).
  - `Action server is inactive. Rejecting the goal.` → 충전 중 절전으로 Nav2 가 PAUSE 된 정상 상태입니다. 절전은 언도킹으로 풀립니다.

## EKF (휠 엔코더 + IMU)

- 설정: `orinbot_bringup/config/ekf.yaml`. 휠 엔코더의 `vx`/`vyaw` 와 D435i 자이로의 `vyaw` 만 융합합니다. 엔코더 **절대 위치**, IMU **자세**, IMU **가속도** 는 드리프트 누적·자력계 부재·평면 주행 시 노이즈 우세로 의도적으로 제외했습니다.
- **`controllers.yaml` 의 `enable_odom_tf` 는 false 여야 합니다.** EKF 와 컨트롤러가 같은 `odom → base_footprint` 를 각자 발행하면 좌표계가 꼬이고, **로그에는 아무것도 남지 않습니다.**
- **EKF 는 차동 구동 컨트롤러가 뜬 뒤에 기동해야 합니다.** `robot_localization` 은 `use_sim_time:=true` 일 때 시계가 유효해질 때까지 `Waiting for clock to start...` 로 대기하는데, 시뮬레이터보다 먼저 뜨면 **그 대기에서 영영 못 빠져나옵니다.** 노드는 살아 있는데 `/odometry/filtered` 와 TF 가 안 나와 TF 체인이 끊깁니다. `sim.launch.py` 의 `diff_drive_then_ekf` 이벤트 핸들러가 이것을 막습니다.
- 외부 IMU 추가의 실익은 적습니다. 자이로 `vyaw` 한 축만 쓰는데 외부 센서도 동급 MEMS 이고, 차별점인 자력계는 실내·모터 자계 간섭으로 못 씁니다. 각속도는 강체 내 위치와 무관하므로 카메라 내장이라는 점도 불리하지 않습니다.
- **휠 오도메트리 보정**: Gazebo 물리 엔진이 원기둥 바퀴의 접촉점을 안쪽 가장자리로 잡아 회전 반경에 오차가 생깁니다. `controllers.yaml` 의 `wheel_separation_multiplier` 가 보정치이고 현재 40 cm 큐브 기준 **1.0863** 입니다. 섀시 치수나 센서 배치를 바꾸면 반드시 재측정하세요.

## 충전 도킹

설정: `orinbot_navigation/config/docking.yaml`, `launch/docking.launch.py`.
도크 형상은 `generate_room.py` 의 `DOCK_*`, 로봇 쪽 접점은 `orinbot.urdf.xacro` 의 `pogo_*` 상수입니다.

- **Nav2 순정 `opennav_docking` 규격을 씁니다.** 실기 전환 시 `dock_database` 좌표만 바꾸면 되고 `/dock_robot` 액션 규격이 유지되어 BT 연동이 살아 있습니다. **검출만 자체 구현했습니다** (`scripts/dock_marker_board.py`).
- **접근 방식은 `docking_mode` 로 고릅니다** — 기본 `staged`(`scripts/staged_dock.py`), 비교용 `smooth`(순정 `opennav_docking`). 액션 규격이 같아 상위 노드는 바꿀 것이 없습니다.
- **지도 좌표가 아니라 "지금 보이는 마커" 기준으로 붙습니다.** SLAM 오차(중앙값 21 mm / 90% 46 mm)가 접점 요구 정밀도보다 크고, 코스트맵 팽창(0.40 m)이 벽 앞을 막아 일반 주행으로는 도크 앞 0.7 m 까지밖에 못 갑니다.
- **`fixed_frame` 은 `map` 이 아니라 `odom` 입니다.** 접근하는 몇 초 동안 SLAM 이 `map→odom` 을 수십 mm 씩 고쳐 넣는데, 그 보정이 그대로 목표 좌표의 점프가 됩니다.

**마커 — ArUco `DICT_4X4_50`, 0.10 m 3장(id 1/0/2)을 좌우 0.16 m 간격**

- **좌우로 벌리는 것이 핵심입니다.** 정면에 가까운 평면 마커 한 장은 자세 모호성 때문에 각도가 크게 흔들립니다(1.3 m 에서 4.77도). 3장의 코너 12개를 하나의 보드로 함께 풀면(`cv2.aruco.estimatePoseBoard`) 사라집니다. **세로로 쌓으면 yaw 개선 효과가 거의 없습니다.**
- **4×4 사전인 이유는 424×240 해상도 때문입니다.** 테두리 포함 6칸이라 24 px 이면 읽히지만, 6×6 은 8칸이라 32 px 이 필요해 검출 거리가 크게 짧아집니다.
- **텍스처는 `generate_textures.py` 가 검출기와 같은 cv2 로 생성합니다.** 비트 패턴을 손으로 옮기면 어긋나도 증상이 "그냥 검출 안 됨"이라 원인 추적이 어렵습니다.
- **마커 높이 h = 0.31 입니다. 낮추지 마세요.** 카메라가 지면 0.36 m 에서 15도 아래를 보고 컬러 화각이 69.4×42.8도이므로 시야는 +6.4도 ~ −36.4도입니다. 마커(한 변 s)가 완전히 들어오는 최소 거리는 `max((h + s/2 − 0.36)/tan6.4도, (0.36 − h + s/2)/tan36.4도)` 이고 두 항이 같아지는 **h = 0.316 에서 최소**입니다. h = 0.20 이면 0.30 m 에서 이미 화면 아래로 잘립니다.
- **검출 절벽은 0.55 m 부근이고, 세로가 아니라 가로입니다** — 바깥 마커 바깥 모서리 0.210 m / tan(34.7도) = 카메라 0.303 m = 로봇 중심 0.516 m. 마커를 내려도 이 절벽은 움직이지 않습니다.

  | 로봇–마커 거리 | 거리 오차 | 가로 오차 | 도크 yaw 오차 |
  |---|---|---|---|
  | 1.29 m | −9.2 mm | +3.1 mm | 2.31도 |
  | 0.90 m | +2.1 mm | +1.2 mm | 0.06도 |
  | 0.70 m | +1.4 mm | +0.8 mm | 0.00도 |
  | 0.65 m | 약 +1 mm | 약 +0.6 mm | 약 0.01도 |
  | 0.50 m | 검출 실패 (바깥 마커가 화각 밖) | | |

- **거리 편차는 거리에 비례해 커집니다** (마커가 작게 보일수록 코너가 안쪽으로 잡힘). `external_detection_translation_x` 보정치는 **가장 가까운 검출 지점의 편차**로 잡으세요.

**후진 도킹 (기본)**

정렬은 도크를 마주 본 채 끝내고, 회전점에서 180도 돌아 뒤로 들어갑니다. **충전 내내 카메라가 벽이 아니라 방을 보므로 시각 오도메트리가 살아 있습니다** — 전진 도킹에서는 도킹 한 사이클에 SLAM 드리프트가 2 mm → 935 mm 까지 벌어졌습니다.

- **회전점은 `rotate_distance: 0.60` (마커면 기준)입니다.** 검출 절벽(0.516 m) 위라 회전 직전에 마커로 다시 잴 수 있고, 후진량은 고정값이 아니라 그때 잰 거리에서 `dock_distance` 를 뺀 값이라 **회전점 오차가 최종 자세로 전달되지 않습니다.**
- **회전점을 정하는 것은 섀시가 아니라 캐스터입니다.** 캐스터는 x=±0.14 / y=±0.14 라 회전 반지름이 0.198 m 인데 구 반지름이 0.030 m 뿐이라 높이 0.040 m 인 동판 턱을 못 넘고 걸립니다. **늘리는 쪽은 여유가 느는 방향이라 안전하고, 줄일 때만 이 여유를 다시 확인해야 합니다.**
- 회전 구간에는 `vodom_tf_relay` 의 `~/pause` 로 시각 오도메트리 보정을 얼립니다. **얼린 동안에도 TF 는 계속 나가야 합니다** — 끊으면 `map` 과 `base_footprint` 가 다른 트리로 갈라집니다.
- 월드 생성기의 `REVERSE_DOCK` 과 `docking.yaml` 의 `reverse_dock` 은 **함께** 바꿔야 합니다 (동판 위치가 따라 움직입니다).

**단계 분리 정렬 (`staged`)**

정지 → 측정 → 보정 → 정지 → 재측정 을 반복합니다. 차동구동은 옆으로 못 가므로 횡오차는 **크랩 기동**(회전 30도 → 직진 → 회전 −30도)으로 없앱니다.

- **크랩의 직진은 후진 방향입니다.** 전진으로 하면 도크 쪽으로 횡오차의 1.73배(= s·cos30도)만큼 다가가 보정할수록 여유가 줄어듭니다.
- **크랩 부호를 뒤집으면 오차가 두 배가 되고**(76 → 154 mm) 그 뒤 바깥 마커가 화각을 벗어나 검출까지 잃습니다. 오차가 커지면 즉시 중단하는 감시가 들어 있습니다.
- **정렬 판정은 축별 허용치가 아니라 "접촉 시점 예상 횡오차" 하나로 합니다.** 정렬 후 직진하는 동안 각도는 그대로 횡오차가 됩니다:

  ```
  접촉 시점 횡오차 = 정렬시 횡오차 − 직진거리 × sin(각도오차)
  ```

  0.52 m 직진 기준 0.5도 = 4.5 mm, 1도 = 9.1 mm, 2도 = 18 mm. 축별로 재면 서로 모순된 목표가 되어 루프가 수렴하지 않습니다.
- `contact_lateral_budget: 0.006` 하나로 판정하고, **크랩 보정량도 측정 `lat` 이 아니라 이 예상값을 씁니다** — 남겨 둔 각도가 만들 흘러감까지 상쇄됩니다. `yaw_tolerance: 0.0175`(1도)는 정밀도용이 아니라 측정 이상 감시용입니다.
- 크랩 기동의 분해능(직진 정지 허용 2 mm, 회전 0.23도)이 바닥이라 **가로는 3~5 mm 아래로 못 내려갑니다.** 허용치 ±34 mm 대비 무시할 수준입니다.

**현재 실측 (room.sdf, `staged` + 후진)**: 세로 −0.7 mm / 가로 +1.8 mm / 각도 −0.13도.

**도크 기하 — 접점 높이가 가장 빡빡합니다**

| 항목 | 높이 |
|---|---|
| 로봇 섀시 바닥 (= 접지고) | 0.060 m |
| **포고핀 브래킷 하단** | **0.045 m** |
| 동판 상면 | 0.040 m |
| 코스트맵 `min_obstacle_height` | 0.030 m |

- 브래킷은 **0.030 ~ 0.060 m 사이**에만 놓을 수 있습니다. 아래로 내리면 코스트맵이 안 보는 낮은 물체(월드의 3 cm 문턱이 그 경계 시험용)에 걸리고, 위로 올리면 섀시 안에 들어가 동판에 닿지 않습니다.
- 좌우로는 구동륜 ±0.17, 캐스터 ±0.14 라 바퀴가 지나가지 않는 띠는 **|y| < 0.11** 뿐입니다. 동판이 바퀴 경로에 걸리면 로봇이 그 위로 올라타 간격이 유지됩니다.
- **접촉 허용치는 핀 배열(6.1 × 3.5 mm)이 동판(75 × 100 mm) 위에 얹히는 범위로 계산해 세로 ±48 mm / 가로 ±34 mm 입니다.** 브래킷(30 mm 각)을 기준으로 잡으면 ±35 / ±22.5 mm 가 되는데, 그러면 실제로는 닿아 있는데 "충전 안 됨"이 되어 있지도 않은 정확도를 요구하게 됩니다.
- **가이드 벽은 두지 않습니다.** 실물 스테이션(MiR 형태 — 벽걸이 본체 + 경사 램프 위 동판 스트립)에 없고, 통로 폭 0.44 m 는 로봇 내접 반경 0.20 m 의 2배보다 좁아 **코스트맵에서 안쪽 전체가 `INSCRIBED_INFLATED_OBSTACLE` 로 칠해집니다** (일반화: 내접 반경 2배보다 좁은 통로에서는 코스트맵 충돌 검사를 원리적으로 쓸 수 없습니다).
- **실기 확인 필요**: 동판 상면 40 mm 에 브래킷 하단 45 mm 이므로 핀이 브래킷 아래로 5 mm 이상 나와야 합니다. 2.54 mm 피치 포고핀 스트로크가 보통 1.5~2.5 mm 라 자유 길이로 5 mm 를 메우고 **남는 만큼만 압축 여유**가 되며, 그것이 곧 바닥 높이 편차(램프 기울기, 동판 두께 공차) 허용치입니다.

**도킹 설정에서 반드시 지킬 것**

- **`docking_server` 의 `enable_stamped_cmd_vel: true`.** 기본값이 false 라 그냥 두면 `Twist` 를 내보내고 `diff_drive_controller` 가 조용히 무시합니다. 증상은 "도킹은 진행되는데 로봇이 제자리".
- **`use_collision_detection: false`.** 도크 앞 0.7 m 는 일부러 벽에 붙으러 가는 구간이라 충돌 검사의 전제가 성립하지 않습니다. `dock_collision_threshold` 로 도크 근처를 예외 처리하는 것은 동작하지 않습니다(0.45 → 매회 재시도 1회, 0.75 → 0.49 m 지점 충돌 보고 후 `error_code 905`). staging pose 까지는 Nav2 가 자기 코스트맵으로 회피하므로 보호가 없는 구간은 마지막 0.7 m 뿐입니다. **실기에서는 그 구간을 위해 범퍼 스위치나 모터 전류 제한 같은 소프트웨어 밖의 보호가 필요합니다.**
- **`use_battery_status: false`.** `docking_server` 의 접근 루프는 `isDocked()` 뿐 아니라 `isCharging()` 으로도 빠져나오는데, 접점 허용치가 넓어(세로 ±48 mm) 목표보다 한참 앞에서 "충전 시작"이 걸려 접근이 끝납니다. 역할을 나눕니다 — `docking_server` 는 목표까지 들어가는 것만, 충전 확인은 `auto_dock.py` 가 `/battery_state` 로 하고 실패 시 재도킹합니다.
- **접촉 판정에 SLAM 자세를 쓰지 마세요.** 접촉은 물리적 사실이므로 `battery_sim.py` 는 Gazebo 지상 진실(`/ground_truth/odom`)로 판정합니다. 그 허용치는 `docking_threshold` 보다 **커야** 합니다 — 작으면 "도착했는데 충전이 안 잡히는" 교착이 납니다.
- **`/world/<world>/dynamic_pose/info` 를 `tf2_msgs/TFMessage` 로 브리지하지 마세요.** 프레임 이름을 담는 `header.data` 가 없어 `frame_id`/`child_frame_id` 가 전부 빈 문자열로 나옵니다. URDF 에 `gz-sim-odometry-publisher-system` 을 붙여 `nav_msgs/Odometry` 로 받으세요. gz 쪽 토픽 이름은 `<odom_topic>` 값 그대로이며 모델 이름으로 네임스페이스가 붙지 않습니다.
- **자세를 고정할 때 영상 타임스탬프로 TF 를 조회하지 마세요.** 영상이 `odom` TF 보다 먼저 도착해 "extrapolation into the future" 로 매번 실패합니다. **`timeout` 을 줘도 소용없습니다** — 그 콜백이 단일 스레드 실행기를 잡고 있어 TF 리스너 콜백이 그 사이 돌 수 없습니다. 가장 최근 TF(stamp=0)로 조회하세요.

**도크 좌표 자동 등록** (`scripts/dock_register.py`, `dock_register:=true`)

도크에 붙은 채 부팅하면 그 자세를 도크 좌표로 등록합니다. 스테이션을 옮겨도 로봇을 한 번 밀어 넣고 재부팅하면 되고, 좌표를 손으로 읽어 옮길 필요가 없습니다. Nav2 `dock_database` 규격 파일(`~/.ros/orinbot_docks.yaml`)로 쓰고, 도는 `staged_dock` 에는 `SetParameters` 로 바로 반영합니다. 부팅 때가 아니어도 `~/register` 서비스로 부를 수 있습니다.

- **저장하는 것은 "도크 앞"이 아니라 "도킹 완료 자세"입니다.** 진입점은 `staged_dock` 이 `approach_distance` 로 계산하므로, 앞으로 빼서 저장하면 두 번 빠져 진입점이 두 배로 멀어집니다.
- **후진 도킹이면 yaw 에서 180도를 뺍니다.** `dock_yaw` 는 접근할 때 바라보는 방향인데 후진 도킹의 최종 자세는 도크를 등지고 있어 정반대입니다. 부호를 틀리면 로봇이 도크 반대편에 진입점을 잡고 마커를 한 장도 못 봐서, 증상이 "마커 검출 실패"로만 보입니다. `reverse_dock` 은 `staged_dock` 쪽과 **같아야** 합니다.
- **등록 좌표는 `map` 기준이고 그 원점은 로봇이 부팅한 자리입니다.** 도크에서 부팅하면 도크가 곧 원점이라 `(0, 0)` 이 나오는 것이 정상입니다.
- **도킹 여부는 위치 추정이 아니라 충전 전류로 판정합니다.** 다만 **첫 샘플로 판단하면 안 됩니다** — 기동 직후에는 접촉 판정 입력이 아직 없어 방전으로 나오는 구간이 있습니다 (`charge_wait: 20.0`).
- **등록 대기는 실행기를 블록하므로 `MultiThreadedExecutor` + `ReentrantCallbackGroup` 이어야 합니다.** 단일 스레드로 두면 그 대기 동안 TF 콜백과 서비스 응답이 못 돌아, 안정화 판정이 갱신 없는 버퍼에서 같은 값만 읽어 무조건 통과하고 파라미터 반영은 성공했는데도 실패로 보고됩니다.
- **`scripts/*.py` 에 실행 권한이 없으면** 런치가 `executable not found` 로 **스택 전체를 중단**시킵니다. 새 스크립트를 추가하면 `chmod +x` 를 확인하세요.

**충전 중 절전** (`auto_dock.py`, `power_save: true`)

- **죽이지 말고 멈춥니다.** `rtabmap` 을 재기동하면 DB 재적재와 재위치추정이 필요해 `localization` 모드가 CPU 25.1 → 33.9 %p / 메모리 552 → 604 MB 로 **오히려 더 씁니다.** `/rtabmap/pause` 는 포즈 그래프를 램에 둔 채 연산만 멈춰 복귀가 즉시입니다.
- Nav2 는 `lifecycle_manager` 의 **PAUSE(1) / RESUME(2)** 로 재웁니다.
- `rgbd_odometry` 에는 pause 서비스가 **없습니다.** 실기에서 재우려면 카메라 스트림 자체를 끊어야 하고, 그 서비스 이름을 `extra_pause_services` / `extra_resume_services` 에 넣으면 코드 수정 없이 호출됩니다. 센서 드라이버 정지도 같은 방법입니다.
- **해제 순서**: SLAM resume → Nav2 RESUME → **전역 코스트맵이 실제로 다시 나오는 것을 확인** → 언도킹. RESUME 완료와 코스트맵 복귀 사이에 1.6초가 있고 그 사이 나간 목표는 즉시 ABORT 됩니다. **코스트맵 확인 구독은 VOLATILE 이어야 합니다** — TRANSIENT_LOCAL 이면 절전 직전의 옛 코스트맵이 즉시 들어와 준비된 것처럼 보입니다.
- 절전 중에는 도킹 명령을 막습니다. 인지가 죽은 채로 주행 명령이 나가면 아무것도 못 보고 움직입니다.
- **멀티스레드 실행기에서 상태 플래그는 블록 이전에 세우고 `threading.Lock` 으로 감쌉니다.** 절전 해제가 서비스 응답을 수 초 기다리는 동안 1 Hz 타이머가 재진입해 `UndockRobot` 이 4번 나간 적이 있습니다.
- **시뮬레이터에서는 센서를 끌 수 없어 절감량을 잴 수 없습니다.** 확인 가능한 것은 "무엇이 멈추는가"(토픽 발행률)까지입니다.

## 잔여 장애물 소거

- **로컬 코스트맵**: `obstacle_layer` 는 라이다 광선으로 즉시 비워지고, `stvl_layer` 는 `voxel_decay: 10.0` 으로 약 9초에 100 → 0.
- **RTAB-Map `/map`**: 즉시 소거되지 않습니다. 포즈 그래프의 각 노드가 생성 당시의 격자를 보존하므로 소급 수정되지 않고, 그 자리를 다시 지나며 새 노드가 쌓여야 갱신됩니다.
- **`static_layer` 의 유령 장애물은 라이다 광선으로 지워지지 않습니다** (레이어가 최댓값으로 합쳐지므로). `localization:=true` 로 운용 중이면 지도가 고정되어 영구히 남습니다.

## 실기 (Jetson Orin Nano Super) 자원 예산

6코어 Cortex-A78AE 1.7 GHz + 8 GB **통합** 메모리 (CPU/GPU 공유). JetPack 7 / L4T R39.2.0 / Ubuntu 24.04.4.

- **유휴 2.8 / 6 코어, 주행 중(MPPI 가동) 3.1 / 6 코어. 메모리 최대 2.76 / 7.85 GB, 최고 온도 60도 (`MAXN_SUPER`).**
- **병목은 CPU 이고 메모리가 아닙니다** (CPU 51% 대 메모리 35%). 자원을 더 쓰는 선택은 메모리로는 감당되지만 CPU 에서 막힙니다.
- **기본 이미지에 스왑도 zram 도 없습니다.** rtabmap 이 튀면 곧바로 OOM 이라 16 GB 스왑파일을 추가했습니다 (`vm.swappiness=10` — 안전망 용도이지 상시 계층이 아닙니다).
- **시뮬레이터에서 잰 Python 노드 CPU 를 실기 추정에 쓰지 마세요.** 거의 전부 `/clock` 처리 비용이고 실기에는 `/clock` 자체가 없습니다. 실기 실측으로 `/clock` 100 Hz 기준 **rclpy 노드 1개당 11.3 %p** (`use_sim_time: true` 11.5% vs `false` 0.2%). C++ 노드의 세금은 아직 못 쟀습니다.

**노드별 실측표, 실기 측정 방법, 기각된 시도 항목은 `orin-resource-budget` 스킬 문서에 있습니다.** 자원 절감을 위해 파라미터를 고치기 전에 반드시 먼저 확인하세요 — `controller_frequency` 하향, 전역 코스트맵 광선 거리 축소, Nav2 컴포지션 등은 이미 실측 후 기각됐습니다.

## `/clock` 솎기

Gazebo 브리지는 `/clock` 을 물리 스텝마다(약 1000 Hz) 내보내고, `use_sim_time: true` 인 rclpy 노드는 그 메시지마다 파이썬 콜백을 돕니다. `scripts/clock_throttle.py` 가 `/clock_raw` 를 받아 `/clock` 을 100 Hz 로 다시 냅니다 (`clock_rate` 인자, `0` 이면 그대로 통과).

- **물리 스텝은 건드리지 않습니다.** 시뮬레이션 정확도는 그대로 두고 ROS 로 나가는 시계 해상도만 낮춥니다. 100 Hz 면 분해능 10 ms 인데 가장 빠른 주기가 컨트롤러 20 Hz(50 ms)이고 TF/센서 동기화는 노드 시계가 아니라 메시지 헤더 스탬프를 쓰므로 영향이 없습니다.
- **`clock_throttle` 노드만은 `use_sim_time` 을 쓰면 안 됩니다.** 시뮬 시각의 출처가 자기 자신이라 스스로를 기다리며 멈춥니다. 그리고 rclpy 가 이미 선언해 두므로 다시 선언하면 `ParameterAlreadyDeclaredException` 으로 죽습니다.
- **발행 QoS 는 브리지가 쓰던 것(RELIABLE + VOLATILE + KEEP_LAST 1)과 맞춰야 합니다.** 어긋나면 모든 노드가 시계를 아예 못 받습니다.

## 프로세스 정리

- **잔여 `parameter_bridge` 가 있으면 새 Gazebo 에 스스로 붙습니다.** gz-transport 가 새 서버를 자동 탐색하므로 `/clock` 이 두 곳에서 발행되고, 타임스탬프 순서가 꼬여 모든 노드에서 `Detected jump back in time. Clearing TF buffer` 가 반복되며 TF 버퍼가 계속 비워집니다. Nav2 와 RTAB-Map 조회가 전면 실패하는데 증상은 "간헐적 정지"로만 보입니다.
- **기동 직후 `ros2 topic info /clock` 의 `Publisher count` 가 1인지 확인하세요.** 2 이상이면 잔여 프로세스가 있습니다.
- **`pkill -f` 를 쓰지 마세요** — 자기 자신의 명령줄에도 매칭되어 스크립트를 중단시킵니다:
  ```bash
  ps -eo pid,args | grep -E '[p]arameter_bridge|[g]z sim|[r]tabmap|[_]server' \
    | awk -v me=$$ '$1 != me {print $1}' | xargs -r kill
  ```

## 개발 주의사항

- **파라미터만 고쳤으면 전체를 내리지 마세요.** 시뮬레이션과 SLAM 을 띄운 채 Nav2 만 종료하고 `ros2 launch orinbot_navigation nav2.launch.py` 로 단독 재기동하면 약 45초입니다 (전체 재기동은 3분 이상).
- 컨트롤러 기동이 실패하면 `ros2 control list_controllers` 로 상태부터 확인하세요.
- 도킹 파라미터를 스윕할 때는 `tools/dock_bench.py` 를 쓰세요 — 도킹만 떼어 낸 시험대를 `ROS_DOMAIN_ID` + `GZ_PARTITION` 으로 격리해 여러 개 동시에 돌립니다 (8케이스 114초, 순차로는 약 11분).
- **주석에는 현재 규칙과 근거만 적습니다.** 과거 실패, 수정 이력, 시도했다가 되돌린 값은 넣지 마세요 — 그런 서사가 필요하면 `docs/ros2-lessons.md` 에 있습니다.
