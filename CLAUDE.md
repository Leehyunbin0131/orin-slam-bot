# orinbot workspace — 작업 지침

ROS 2 **Jazzy** + **Gazebo Harmonic** (Ubuntu 24.04). 실제 로봇 없이 시뮬레이션으로 개발 중.

## 환경 전제

- ROS 배포판: Jazzy (`/opt/ros/jazzy`)
- Gazebo: Harmonic, `ros-jazzy-gz-*-vendor` 패키지로 설치됨 (osrfoundation 저장소 없음)
- 명령 실행 전 항상 `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash`
- `sudo` 는 비밀번호를 요구함 — apt 설치는 사용자에게 실행을 요청할 것

## 아키텍처 원칙

- **Gazebo 의 DiffDrive 플러그인을 쓰지 않는다.** 구동은 `ros2_control` +
  `gz_ros2_control` + `diff_drive_controller` 로 처리한다. 실기 이전 시
  `<hardware>` 블록만 교체하면 되도록 하기 위함.
- **센서 토픽/프레임 이름은 실제 하드웨어와 동일하게 맞춘다.**
  D435i 는 `realsense2_camera` (`camera_name:=camera`, `camera_namespace:=""`)
  기준. 시뮬 전용 이름을 새로 만들지 말 것.
- 속도 명령은 `geometry_msgs/TwistStamped` (Jazzy `diff_drive_controller` 기본).
- 치수는 `orinbot.urdf.xacro` 상단 파라미터 블록에만 둔다. 하드코딩 금지.
  변경 시 `orinbot_bringup/config/controllers.yaml` 의 `wheel_separation` /
  `wheel_radius` 도 반드시 같이 수정.

## 빌드 / 실행 / 측정

표준 `colcon` 호출입니다. 상황별 실행 명령은 `README.md` 의
"자율주행 (VSLAM + Nav2)" 절에 있습니다.

**파라미터를 바꿨으면 반드시 `tools/` 로 재고 판단할 것.** 무엇을 어떻게
재는지와, 측정이 거짓말하는 세 가지 경우는 `tools/README.md` 에 있습니다.
현재 기준선도 거기 있습니다 — 새 값을 그것과 비교하세요.

## 알려진 Gazebo 함정

- **Gazebo 의 포인트클라우드(`/camera/points`)는 쓰지 말 것.** gz-sim 은 좌표를
  카메라 본체 규약(x=전방)으로 채우면서 frame_id 는 광학 프레임(z=전방) 이름을
  붙여 내보낸다. `optical_frame_id` / `gz_frame_id` 를 어떻게 조합해도
  이름표만 바뀌고 데이터는 변환되지 않는다 (실측 확인). 대신
  `depth_image_proc::PointCloudXyzrgbNode` 로 depth + camera_info 에서 생성한다.
- 카메라 계열 센서에서는 `<camera><optical_frame_id>` 가 `<gz_frame_id>` 보다
  우선한다. IMU 등 다른 센서는 `<gz_frame_id>` 가 적용된다.
- `XML Element[gz_frame_id] ... not defined in SDF` 경고는 무시해도 된다.
  libsdformat 의 스키마 검사 경고일 뿐 gz-sim 은 값을 정상적으로 읽는다.
- XML 주석 안에 `--` 를 쓰면 xacro 파싱이 깨진다 (`--controller-ros-args` 등).
- **`IncludeLaunchDescription` 의 `launch_arguments` 는 부모 스코프로 샌다.**
  하위 런치에 `use_rviz:=false` 를 넘기면 상위 런치의 `use_rviz` 까지 덮여
  상위 RViz 노드가 조건 거짓으로 실행되지 않는다. include 를 `GroupAction`
  으로 감싸서 스코프를 격리할 것.

## VSLAM / Nav2 관련 함정 (모두 실측으로 확인)

- **Nav2 의 lifecycle 활성화가 종종 정지한다. 감시 노드가 자동으로 다시 건다.**
  이 PC 는 NIC 가 3개라 FastDDS 서비스 응답 매칭이 늦어져
  `failed to send response to .../change_state` 후
  `Failed to bring up all requested nodes. Aborting bringup.` 로 끝난다.
  - `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 는 여전히 필요하지만
    **이것만으로 없어지지 않는다** (2026-08-02 하루에 4~5회 발생).
  - 증상이 고약한 이유: 노드는 살아 있고 액션 서버도 보이는데 전부
    `inactive`/`unconfigured` 라 모든 목표를 즉시 ABORT 한다. 겉으로는
    "Nav2 는 떠 있는데 로봇이 안 움직인다" 로만 보인다.
  - `scripts/nav2_startup_watchdog.py` 가 `is_active` 를 지켜보다 실패하면
    `manage_nodes` 로 **RESET 후 STARTUP** 을 다시 건다 (RESET 을 먼저 걸어야
    한다. 일부만 활성인 상태에서 STARTUP 만 보내면 "already active" 로 실패).
    실측: 정지 감지 후 2초 만에 5개 서버 전부 active.
    끄려면 `startup_watchdog:=false`.
  - **실기(Orin)는 코어당 성능이 1/4~1/5 이라 더 잘 난다.** 이 감시는
    개발 편의가 아니라 실기 대비 항목이다.
  - 감시 노드에 `use_sim_time` 을 주지 않는다. 기동 감시가 `/clock` 에 묶이면
    시뮬레이터가 아직 시계를 안 내보낼 때 영원히 대기한다.
- **RGB 와 뎁스의 타임스탬프가 어긋나면 VSLAM 이 크게 드리프트한다.**
  848x480 30Hz 는 Gazebo 가 두 스트림을 같은 틱에 못 내보내 최대 0.13초
  벌어졌고, 제자리 360도 회전에서 자세 오차가 1.2m/87도까지 갔다.
  424x240 15Hz 로 낮추니 114/114 프레임이 정확히 일치하고 오차 3mm/0.03도.
- **VSLAM 은 RGB 특징점으로 움직인다. 뎁스만으로는 안 된다.**
  뎁스는 특징점에 3D 좌표를 주는 역할일 뿐이다. 벽 무늬가 모두 같으면
  서로 다른 장소를 같은 곳으로 오인해 잘못된 루프 클로저가 그래프에 들어가고
  자세가 3m 튄다. `worlds/generate_room.py` 가 벽 세그먼트마다 고유 텍스처를
  만드는 이유.
- **`rgbd_odometry` 의 TF 를 그대로 쓰면 Nav2 가 목표를 취소한다.**
  영상 시각으로 스탬프가 찍혀 평균 0.2초(최악 0.85초) 뒤처지고, Nav2 의
  map->odom 조회가 미래 외삽을 요구해 실패한다. `publish_tf:=false` 로 두고
  `vodom_tf_relay.py` 가 50Hz·미래 시각으로 재발행한다.
- **`Grid/Sensor` 는 0 이 아니라 1 이다.** 0=레이저스캔, 1=뎁스영상.
  0 으로 두면 `/map` 이 전부 미탐색으로 나온다.
- **휠 오도메트리는 캘리브레이션이 필요하다.** Gazebo 가 원기둥 바퀴의
  접촉점을 안쪽 가장자리로 잡아 실제 회전이 어긋난다.
  `controllers.yaml` 의 `wheel_separation_multiplier` 가 그 보정값이며,
  40cm 큐브 기준 현재 값은 **1.0863** 이다. 섀시 치수나 센서 배치를
  바꾸면 하중 분포가 달라지므로 반드시 다시 측정할 것.
- **`behavior_server` 에는 `enable_stamped_cmd_vel` 이 없다.**
  항상 TwistStamped 를 쓴다. controller_server 와 velocity_smoother 에만 있다.
- **`velocity_smoother` 는 `stamp_smoothed_velocity_with_smoothing_time: True`
  가 필요하다.** false 면 입력 명령의 시각을 재사용해서
  `diff_drive_controller` 가 "0.5초보다 오래된 명령"이라며 무시한다.

## 센서 담당 구역

치수·주기·좌표 같은 사양은 여기 적지 않습니다. 바뀌면 이 복사본만 낡습니다
(실제로 두 번 낡았습니다). 출처를 보세요 — `orinbot.urdf.xacro` 상단 파라미터
블록, `d435i.urdf.xacro`, `rplidar_a2m12.urdf.xacro`, `generate_room.py`.

여기에는 코드에서 못 읽는 **왜**만 둡니다.

- **라이다 스캔 평면은 지면 0.49 m 이고, 그보다 낮은 것은 원리적으로 못
  봅니다.** 낮은 장애물은 D435i 담당입니다. 이 높이가 여러 판단의 기준이라
  숫자를 남겨 둡니다.
- **월드의 다층 선반 3개는 시험 장치입니다** (`generate_room.py` 의 `SHELVES`).
  두 센서의 담당이 갈리는 것을 재현하려고 판 높이를 일부러 다르게 뒀습니다.
  - `shelf_0` : 판이 라이다 평면에 **걸려** 2D 만으로도 벽처럼 보임
  - `shelf_1` : 판이 라이다 평면을 **비켜감**. 라이다에는 얇은 기둥 4개뿐이라
    "거의 뚫린 곳"으로 보이지만 실제로는 판이 튀어나와 있습니다.
    **STVL 3D 복셀 레이어가 없으면 Nav2 가 여기로 경로를 냅니다.**
  - `shelf_2` : 옆판까지 막힌 형태 + 선반 위 물건들 (특징점 풍부)

## Nav2 구성 (실측으로 확정)

- 컨트롤러는 **MPPI**, 값은 **Nav2 순정**을 쓴다 (아래 함정 항목 참조).
- 코스트맵 레이어 분담:
  - `obstacle_layer` (2D) : 라이다 `/scan` 전용. 360도라 광선 소거가 정상 동작.
  - `stvl_layer` (3D 복셀) : 카메라 `/camera/depth/points` 전용.
    화각이 좁아(87x58도) 회전하면 지울 기회가 없으므로 시간 감쇠가 필요.
    `voxel_decay: 10.0` — 실측으로 등 돌린 뒤 9초에 비용 100 -> 0.
  - `inflation_layer` : `cost_scaling_factor 10.0`, `inflation_radius 0.40`.
- `footprint` 로 실제 사각(0.4x0.4)을 준다. `robot_radius` 원 근사를 쓰면
  0.8 m 문을 통과 불가로 판단한다. 여기에 더해 MPPI 의
  `CostCritic.consider_footprint: true` 까지 켜야 좁은 통로를 지난다.
- **전역 코스트맵 해상도는 `nav2_params.yaml` 이 정하지 않는다.**
  `StaticLayer` 가 들어오는 `/map` 에 맞춰 코스트맵을 다시 크기 조정하므로
  RTAB-Map 의 `Grid/CellSize`(현재 0.05, `slam.launch.py`)가 실제 값이다.
  `global_costmap.resolution` 에 뭘 써도 무시된다. 지역 코스트맵만
  이 값(현재 0.02)을 그대로 쓴다.
- **최소 통과 가능 폭은 0.70 m 다. 직진 통과 능력이 아니라 회전 능력이
  기준이다** (2026-08-02 변경, 그 전 기준은 0.60 m).
  - 직진만 보면 0.55 m 도 지나간다(실측). 하지만 로봇은 막다른 곳에서
    되돌아 나와야 하고, 차동구동은 후진보다 제자리 회전을 먼저 한다
    (Nav2 의 `Spin` 복구 행동도 마찬가지).
  - **제자리 회전에 필요한 폭 = 대각선 = √2 × 0.40 = 0.566 m.**
  - 실측 (`tools/turnaround_test.py`, Nav2 없이 순수 기하):

    | 통로 | 편측 여유 | 중앙 회전 | 직진 시 가능한 편심 | 회전 실패 |
    |---|---|---|---|---|
    | 0.55 m | −8 mm | **실패**(86도, 325 mm 밀림) | ±75 mm | 항상 |
    | 0.60 m | 17 mm | 통과 | ±100 mm | **100 mm 에서 21도** |
    | 0.70 m | 67 mm | 통과 | ±150 mm | 60 mm 까지 확인 |
    | 0.90 m | 167 mm | 통과 | ±250 mm | 100 mm 까지 확인 |

  - **0.60 m 가 탈락한 이유**: 직진 통과 중 로봇이 놓일 수 있는 위치 범위가
    ±100 mm 인데, 그 끝에서는 회전이 아예 안 된다. 정상 주행 중 멈추기만
    해도 돌아설 수 없는 자세가 된다. 편측 여유 17 mm 는 우리 SLAM 자세
    오차(중앙값 21 mm, 90%값 46 mm)보다도 작다.
  - **이 때문에 SLAM 정확도와 통로 폭은 한 예산이다.** 정확도를 깎는 설정
    (예: `Rtabmap/DetectionRate` 인하)은 통과 여유를 직접 갉아먹는다.
  - 시험 하네스 주의: **통로 안으로 순간이동시키면 안 된다.** Gazebo 는
    정적 물체와의 초기 관통을 밀어내지 않아 벽에 80 mm 박힌 채 그대로
    회전한다(실측). 반드시 바깥에서 몰고 들어가게 할 것.
- 속도 명령 경로:
  `controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed`
  `사람 -> /cmd_vel_teleop`, 둘을 `twist_mux` 가 중재해 `/cmd_vel` 로 낸다.

## VSLAM 추가 함정

- **렌더링 스트림 3개도 됩니다 (앞선 기록은 틀렸습니다).**
  424x240 / 15Hz 에서 color+infra1+depth 를 동시에 돌린 실측:
  **15.2 / 15.2 / 15.2 Hz, 타임스탬프 256/256 완전 일치, 시차 0**.
  예전에 infra1 6.9Hz / depth 10.6Hz 로 어긋난 것은 848x480 시절이거나
  `/clock` 중복 브리지 버그가 있던 때의 값으로 보입니다.
  Gazebo 에 "스트림 2개" 같은 구조적 한계는 없고, gz-sim 렌더링 스레드의
  처리량 문제일 뿐입니다. 해상도/주기를 낮추면 더 늘릴 수도 있습니다.
  단 대가는 있습니다: 실시간 배속(RTF)이 0.98 -> 0.84 로 떨어집니다.
  이건 시뮬만의 비용이고 실기 D435i 는 세 스트림을 하드웨어로 동시에 냅니다.
  `orinbot.urdf.xacro` 의 `camera_color_rate` 로 조절합니다.
- **Gazebo 는 GPU 로 렌더링합니다.** `nvidia-smi` 에 gz sim 프로세스가
  GPU 메모리를 잡고 있고 `gz-rendering-ogre2` 가 로드됩니다.
  `libEGL warning: egl: failed to create dri2 screen` 은 헤드리스 EGL 경로
  시도가 실패하고 GLX 로 넘어간 것으로 무해합니다.
- **rgbd_odometry 에 IMU 를 넣지 말 것 (`use_imu_in_odom:=false`).**
  `Reg/Force3DoF=true` 인 평면 로봇에서 켜면 회전이 전혀 추적되지 않습니다.
  실측: 제자리 360도 회전 후 각도 오차 켰을 때 89.62도 / 껐을 때 0.01도.
  평면 로봇에서 IMU 의 자리는 휠 엔코더와 융합하는 EKF 쪽입니다.
- **IMU 센서는 광학 규약 링크에 붙일 것.** 카메라는 링크의 +X 를 바라보므로
  본체 규약 링크에 붙이지만, IMU 는 링크 축 그대로 데이터를 내므로
  `camera_imu_optical_frame`(광학 규약)에 붙여야 실기와 축이 맞습니다.
  검증: +0.5rad/s yaw 회전 시 angular_velocity.y = -0.5.
- **rtabmap 파라미터는 전부 문자열 타입.** `LaunchConfiguration` 을 그대로
  넘기면 launch 가 bool 로 추론해 노드가 InvalidParameterTypeException 으로
  죽습니다. `ParameterValue(..., value_type=str)` 로 감쌀 것.
- **gz 의 camera_info 는 토픽 경로의 부모에 붙습니다.** `<topic>camera/color</topic>`
  로 두면 세 카메라가 `/camera/camera_info` 하나를 공유합니다.
  `camera/color/image` 처럼 한 단계 더 두어야 각자 갖습니다.

## Nav2 함정 (모두 실측)

- **MPPI 비평자 가중치를 임의로 만지지 말 것. Nav2 순정값을 쓴다.**
  진동을 잡겠다고 `PathAlign 14->10`, `PreferForward 5->8`,
  `TwirlingCritic 10.0` 추가, `batch_size 2000->1000`, `wz_std 0.4->0.3`,
  `ax_max 3.0->1.0` 로 "튜닝" 했더니 로봇이 200초 동안 2.7 m 밖에 못 갔다
  (평균 **0.014 m/s**, `vx_max` 는 0.40). 같은 문·같은 목표를 순정값으로
  주면 **13초, 0.40 m/s, 진동 없음**.
  - 비평자 가중치군 -> 개활지에서도 서행. 한 항을 올리면 다른 항이 상쇄되며
    "정지"가 최적해가 된다.
  - 샘플러군(batch/std/ax_max) -> 좁은 문 앞에서 정지. 표본이 적고 탐색 폭이
    좁으면 문틈처럼 좁은 저비용 통로를 못 찾는다.
  - **증상이 보여도 가중치부터 만지지 말고 코스트맵/footprint 를 먼저 볼 것.**
- **`twist_mux` 가 없으면 수동 개입이 불가능하다.** Nav2 의
  `velocity_smoother` 는 활성 상태이면 목표가 없어도 20Hz 로 0 을 계속
  발행한다. 이것이 `/cmd_vel` 에 직접 물려 있으면 조이스틱 명령과 섞여
  로봇이 즉시 멈춘다. `config/twist_mux.yaml` 참조.
  `use_stamped: true` 필수 (false 면 Twist 로 나가 로봇이 안 움직임).
- **좁은 통로를 지나려면 `CostCritic.consider_footprint: true` 가 필요하다.**
  false 면 Nav2 가 "비용 253(내접반경 이내) 셀에 로봇 중심이 닿으면 충돌"로
  판단한다. 즉 폭 W 통로에서 중심이 움직일 수 있는 폭이 `W - 2*내접반경`
  으로 줄어든다. 이 로봇(0.4x0.4, 내접반경 0.20)이면 폭 0.55 m 통로에서
  0.15 m 밴드 안을 지나야 해서 사실상 통과 불가다.
  실측 (폭 0.90/0.70/0.55 m, 깊이 0.60 m 복도):
  `false` 28/23/**실패**, `true` 28/26/**50초 통과**.
  연산 부담은 없었다 (controller_server 루프 초과 2회 -> 0회).
  `worlds/generate_room.py` 의 `PASSAGES` 가 이 시험용 복도들이다.
- **막힌 뒤 후진까지 60~100초가 걸리던 문제 (2026-08-02 수정).**
  두 가지가 겹쳐 있었다.
  - `progress_checker.movement_time_allowance` 가 **20초**였다. 이게 곧
    "막혔다고 깨닫는 시간"이고, 복구 항목마다 이 시간이 다시 흐른다.
    카메라만 쓰던 시절(360도 라이다가 없어 재계획이 잦던 때)의 값이라
    **8초**로 내렸다.
  - Nav2 기본 행동 트리의 복구 순서가
    `코스트맵 지우기 -> Spin(90도) -> Wait(5초) -> BackUp(0.30 m)` 라
    **후진이 네 번째**다. 그래서 `behavior_trees/navigate_w_fast_recovery.xml`
    로 `지우기 -> BackUp(0.35 m) -> Spin -> Wait(3초)` 순서로 바꿨다
    (`bt_xml` 런치 인자, 기본값이 이 트리).
  - 순서를 바꾼 이유는 속도만이 아니다. **`Spin` 은 폭 0.566 m 를 요구한다.**
    막히는 곳은 대개 좁은 곳이므로, 하필 가장 좁을 때 회전을 먼저 시도하는
    셈이었다. 후진은 폭을 요구하지 않는다.
- **`inflation_radius` 와 `cost_scaling_factor` 는 역할이 다르다.**
  반경은 "얼마나 멀리 퍼지나", 계수는 "얼마나 빨리 떨어지나"
  (`252*exp(-계수*(거리-내접반경))`). 좁은 통로에서는 반경을 줄이지 말고
  **계수를 올린다**. 반경을 줄이면 벽 회피 여유 자체가 사라진다.
  폭 0.80 m 문 중앙 비용 실측: 계수 2.0 -> **74**, 계수 10.0 -> **24**.
- **`min_obstacle_height` 는 곧 "탐지 가능한 최소 장애물 높이"다.**
  0.03 으로 두면 3 cm 미만은 원리적으로 못 본다. 낮출수록 작은 것을 보지만
  바닥 잡음이 새어 들어온다. 실기에서는 평평한 바닥을 보며 포인트클라우드의
  높이 분포를 재고 정할 것 (`obstacle_range` 를 2 m 로 묶어 둔 이유도
  뎁스 잡음이 거리 제곱으로 커지기 때문).
- **바닥 타일의 윗면이 정확히 z=0 이어야 한다.** 시각용 타일(두께 0.01)의
  중심을 +0.005 에 두면 윗면이 z=0.010 이 되는데, 바퀴는 `ground_plane`(z=0)에
  닿으므로 카메라가 보는 바닥이 로봇이 굴러가는 면보다 1 cm 높아진다.
  실측: 바닥 점이 전 거리에서 +0.012 m 로 일정하게 떠 보였고, 3 cm 장애물이
  2 cm 로 측정됐다. 중심을 -0.005 로 고친 뒤 바닥은 +0.001~0.003 m.

## 자동 탐사 (프론티어 익스플로레이션)

지도 없이 시작해 스스로 미탐색 구역을 돌며 지도를 만듭니다.

```bash
ros2 launch orinbot_navigation navigation.launch.py explore:=true
ros2 launch orinbot_navigation explore.launch.py     # 이미 떠 있을 때 탐사만 추가
```

구현: `orinbot_navigation/scripts/frontier_explorer.py`.
`/map` 에서 "빈 곳인데 옆이 미탐색"인 셀(=프론티어)을 찾아 묶고,
`거리 - gain*경계길이` 가 최소인 곳으로 `NavigateToPose` 를 보냅니다.
실측: 10x8 m 방 전체를 약 3분에 매핑.

외부 패키지를 쓰지 않은 이유: Jazzy apt 에 탐사 패키지가 없고(확인함),
`explore_lite` 계열은 자기 `costmap_2d` 를 하나 더 굴립니다. 필요한 건
이미 발행 중인 `/map` 뿐이라 Orin 예산에서 손해입니다.

**여기서 밟은 함정들 (전부 실측)**

- **`/map` 은 `RELIABLE + TRANSIENT_LOCAL` 이고 RTAB-Map 은 지도가 바뀔
  때만 발행한다.** 기본 QoS(VOLATILE)로 구독하면 로봇이 멈춰 있는 동안
  붙었을 때 마지막 지도를 못 받고 영영 기다린다. `/map` 을 구독하는
  도구는 전부 TRANSIENT_LOCAL 로 맞출 것.
- **새 목표를 보내면 Nav2 는 이전 목표를 preemption 으로 끝내면서
  `ABORTED(6)` 를 돌려준다.** 이건 실패가 아니다. 목표에 일련번호를 달아
  현재 번호가 아닌 결과는 무시해야 한다. 이 구분이 없으면 멀쩡한 지점이
  전부 "실패"로 기록돼 탐사가 즉시 끝난다.
- **목표 거절(`not accepted`)은 "나쁜 지점"이 아니라 "서버가 아직 준비
  안 됨"이다** (`bt_navigator` activate 전 등). 재시도하면 된다.
- **`NavfnPlanner` 의 `tolerance`(0.5 m) 때문에 Nav2 가 도착하지 않고도
  성공을 반환한다.** 목표가 도달 불가(장애물 팽창 영역 안)이면 플래너가
  허용 오차 안의 가장 가까운 지점으로 경로를 자르는데, 그 지점이 이미
  로봇 위치이면 컨트롤러가 즉시 "Reached the goal!" 을 낸다.
  실측: 0.70 m 떨어진 같은 지점에 **253회 "도착"**. 성공을 받았을 때
  실제 거리를 확인해야 한다.
- **여유값으로 프론티어를 걸러내면 안 된다.** 장애물을 여유값만큼 팽창시켜
  뺀 뒤 덩어리 크기를 재면, 폭 0.60 m 통로 입구에서 살아남는 띠가 2셀뿐이라
  최소 크기 기준에 미달해 통로가 통째로 사라진다(실측: 후보 6곳 -> 0곳).
  크기는 원본으로 재고, 여유값은 "덩어리 안 어디를 목표로 찍을까"에만 쓴다.
- **도달 불가능한 미탐색은 영구 제외해야 한다.** 장애물 뒤 그늘은 원리적으로
  못 가는데, 시한부 블랙리스트만 있으면 그 한 곳 때문에 탐사가 안 끝난다.
- **"액션 서버가 떴다"를 Nav2 준비 신호로 쓰면 안 된다.** 그건 `bt_navigator`
  가 활성이라는 뜻일 뿐이고, `planner_server` 가 아직 활성화 중이거나 전역
  코스트맵이 `/map` 크기로 자리잡기 전이면 **모든 목표가 즉시 ABORT** 된다.
  그 실패로 지점을 판단하면 프론티어가 통째로 블랙리스트에 들어가 탐사가
  시작조차 못 한다(실측 2회, 11~14곳 전멸). `global_costmap/costmap` 을
  한 번 받은 것을 준비 신호로 쓴다.
- `STATUS_UNKNOWN(0)` 도 지점의 문제가 아니다(서버가 사라진 경우). 재시도할 것.

**목표 선별은 "들어갈 수 있는가"가 아니라 "돌아 나올 수 있는가"로 (실측)**

이 하나가 탐사 품질을 통째로 갈랐다. `min_clearance` 를 내접반경 근처(0.18)
에서 **0.33** 으로 올린 것뿐이다. 같은 월드, 둘 다 새로 띄운 상태:

| | 0.18 (들어갈 수 있는가) | 0.33 (돌아 나올 수 있는가) |
|---|---|---|
| 완주 시간 | 4526 초 | **188 초** |
| 후진 / 회전 / 끼임 | 38 / 30 / 19 회 | **0 / 0 / 0** |
| 후진·회전 실패 | 30 회 / 26 회 | **0 회** |
| 지도 형상 | 몇 도 기울고 동쪽이 부챗살로 뭉개짐 | **직각 유지, 왜곡 없음** |
| 미탐색 | 없음 (대신 지도를 잃음) | 0.60/0.55 m 통로 너머만 |

- 0.18 일 때 끼임이 전부 `x 4.4~4.7, y 1.6~3.7`(통로 뱅크 너머 좁은 띠)에
  몰렸다. 벽에 갈리며 도는 동안 바퀴가 미끄러져 오도메트리가 깨졌고,
  그것이 그래프에 누적돼 지도 전체를 기울였다.
- **로봇을 기동할 수 없는 공간에 들여보내면 주행만 실패하는 게 아니라
  지도까지 잃는다.** 이것이 0.70 m 최소 통과 폭 기준의 가장 강한 근거다.
- 임계값을 0.283(외접반경)이 아니라 0.33 으로 잡은 이유는 위 파라미터
  주석의 표를 볼 것 — 0.283 을 그대로 쓰면 0.60 m 통로(중앙 여유 0.300)가
  통과해 버린다.

## 실기(Jetson Orin Nano Super) 대비 자원 절감

목표 보드: 6코어 Cortex-A78AE 1.7GHz + 8GB **통합** 메모리(CPU/GPU 공유).
코어당 성능이 이 PC(i7-14700K)의 1/4~1/5 수준이라 여유가 없습니다.

**실측표·측정 방법·기각된 시도 목록은 `orin-resource-budget` 스킬에 있습니다.**
자원을 줄이려고 파라미터를 만지기 전에 그 스킬을 먼저 읽으세요 — 이미 재보고
기각한 것들이 적혀 있습니다 (`controller_frequency` 인하, 전역 코스트맵 광선거리
축소, Nav2 컴포지션).

## RTAB-Map 메모리 / 루프 클로저 (실측)

- **`Grid/3D: true` 가 메모리의 주범이었다.** 이것만 끄면:
  메모리 **3.65 GB -> 1.66 GB** (rtabmap 단독 2656 -> 914 MB),
  CPU **1.34 -> 1.16 코어**. 그래서 `map_3d` 기본값을 **false** 로 두었다.
  Nav2 는 2D `/map` 만 쓰고 3D 장애물은 STVL 이 담당하므로 주행에는
  영향이 없다. 3D 지도를 눈으로 볼 때만 `map_3d:=true`.
- **`Rtabmap/DetectionRate` 가 가장 큰 CPU 손잡이다. 그래도 2.0 을 유지한다.**
  (`detection_rate` 런치 인자). 같은 월드·둘 다 새로 띄운 상태에서 실측:

  | | 2.0 (현행) | 1.0 |
  |---|---|---|
  | rtabmap CPU | 25.1 %p | **15.5 %p** (−38%) |
  | rtabmap 메모리 | 552 MB | **446 MB** (−106 MB) |
  | SLAM 자세오차 중앙값 | **0.021 m** | 0.027 m (+29%) |
  | 90%값 | **0.046 m** | 0.052 m |
  | map->odom 최대 보정 | **0.110 m** | 0.188 m (거의 2배) |

  절감(Orin 환산 0.5코어)은 크지만, **이 로봇의 통로 통과 여유가 바로 이
  자세 오차에서 나옵니다.** 폭 0.70 m 통로의 편측 여유가 67 mm 인데
  90%값이 46 -> 52 mm 로 오르면 그 예산을 직접 깎습니다.
  Orin 에서 CPU 가 정말 모자랄 때만 `detection_rate:=1.0` 을 쓰세요.
- **`Rtabmap/MemoryThr` 는 작업 메모리의 "노드 수"만 제한한다.**
  점유격자 캐시는 그와 별개로 지도 전체에 대해 쌓이므로, 300 으로 걸어도
  메모리가 계속 자랐다. 격자 쪽(`Grid/3D`, `Grid/RangeMax`)을 봐야 한다.
- **`Reg/Strategy: 2`(영상+ICP)는 3D 격자를 끄면 사실상 공짜로 얻는다.**
  루프 클로저를 영상으로 찾은 뒤 라이다 스캔 ICP 로 기하를 검증하므로,
  벽 무늬가 비슷한 실내에서 잘못된 루프 클로저를 크게 줄인다.
  실측 (같은 왕복 주행): map->odom 최대 보정 **1.317 m -> 0.141 m**,
  SLAM 오차 90%값 0.323 -> 0.056 m.
- **벽 텍스처는 절대 재사용하지 말 것.** 통로 뱅크를 추가하며 기존
  텍스처를 돌려 썼더니(`wall_05` 3곳, `wall_06~10` 2곳씩) 지도가 크게
  흔들렸다. `generate_textures.py` 의 `N_WALL_SEGMENTS` 를 필요한 만큼
  늘리고 모든 벽면에 고유 번호를 줄 것. 현재 28개 생성, 26개 사용.
- **컬러 영상을 rtabmap 특징점 입력으로 쓰려면 정합된 뎁스가 필요하다.**
  `camera_color_optical_frame` 은 depth/infra1 프레임에서 **15 mm** 떨어져
  있다(실기 D435i 와 동일). 정합 없이 컬러+뎁스를 넣으면 특징점에 엉뚱한
  깊이가 붙는다. 실기에서는 `align_depth.enable:=true` 로
  `/camera/aligned_depth_to_color/image_raw` 를 쓴다. 시뮬에서 하려면
  컬러 프레임 위치에 뎁스 센서를 하나 더 두어야 한다(렌더링 4개).
- **실기 IR 프로젝터 주의.** D435i 의 IR 도트 패턴이 infra 영상에 찍히는데,
  로봇에 붙어 같이 움직이므로 고정된 세계 특징점이 아니다. VSLAM 에
  infra 를 쓸 때 `emitter_enabled:=false`(뎁스 품질 저하) 또는
  `emitter_on_off:=true`(프레임 교대, 프레임률 절반)를 검토할 것.
  시뮬에는 프로젝터가 없어 이 문제가 보이지 않는다.

## EKF (휠 엔코더 + IMU) 함정

- 구성: `orinbot_bringup/config/ekf.yaml`. 엔코더의 `vx`/`vyaw` 와
  D435i 자이로의 `vyaw` 만 융합합니다. 엔코더 **위치**, IMU **자세**,
  IMU **가속도**는 일부러 쓰지 않습니다 (드리프트 누적 / 자력계 없음 /
  평면 주행에서 잡음 우세).
- **`controllers.yaml` 의 `enable_odom_tf` 는 false 여야 합니다.**
  EKF 와 둘 다 켜면 같은 `odom -> base_footprint` 를 두 노드가 발행합니다.
- **EKF 는 반드시 컨트롤러가 올라온 뒤에 띄울 것.**
  `robot_localization` 은 `use_sim_time` 일 때 시계가 유효해질 때까지
  `Waiting for clock to start...` 로 대기하는데, Gazebo 보다 먼저 뜨면
  **그 대기에서 영구히 빠져나오지 못합니다.** 노드는 살아 있는데
  `/odometry/filtered` 도 TF 도 안 나오고, TF 체인이 끊겨 RViz 에서
  로봇이 튀고 매핑이 망가집니다. `sim.launch.py` 의
  `diff_drive_then_ekf` 이벤트 핸들러가 그 방지책입니다.
- 별도 IMU(HandsFree A9 등)를 추가할 이유는 약합니다. 우리는 자이로의
  `vyaw` 하나만 쓰는데 A9 도 같은 급 MEMS 이고, 유일한 차별점인 자력계는
  실내 + 모터 옆이라 못 씁니다. 각속도는 강체에서 위치와 무관하므로
  카메라에 붙어 있다는 것도 불리하지 않습니다.

## 사라진 장애물은 언제 지워지나

- **지역 코스트맵**: 실시간. `obstacle_layer` 는 라이다 광선으로 즉시
  비우고, `stvl_layer` 는 `voxel_decay: 10.0` 초 뒤 스스로 사라집니다
  (실측: 비용 100 -> 0 까지 9초).
- **RTAB-Map 의 `/map`**: 즉시 지우지 않습니다. 그래프의 각 노드가 "그때
  본 격자"를 들고 있고 소급 수정되지 않습니다. 그 자리를 다시 지나가며
  새 노드가 쌓여야 갱신됩니다.
- **주의**: `static_layer` 의 유령 장애물은 라이다가 비었다고 해도 안
  지워집니다. 코스트맵 레이어가 최댓값으로 합쳐지기 때문입니다.
  `localization:=true` 로 운용 중이면 지도가 고정되어 영구히 남습니다.

## 프로세스 정리 함정

- **이전 실행의 `parameter_bridge` 가 살아 있으면 새 Gazebo 에 다시 붙는다.**
  gz-transport 가 알아서 새 서버를 찾아가므로, 브리지가 둘이 되어 `/clock` 을
  각자 재발행한다. 순서가 뒤섞이면서 모든 노드가
  `Detected jump back in time. Clearing TF buffer` 를 반복하고, TF 버퍼가
  비어 Nav2·RTAB-Map 의 조회가 전부 실패한다. 증상이 "가끔 멈춤"이라 원인을
  찾기 어렵다.
- 그래서 **재실행 직후 `ros2 topic info /clock` 의 `Publisher count` 가
  1 인지 반드시 확인할 것.** 2 이면 잔여 프로세스가 있다.
- PID 를 나열해 죽이면 빠뜨리기 쉽다. 패턴으로 훑되 자기 자신은 제외할 것
  (`pkill -f` 는 자기 명령줄에 걸려 스스로를 죽인다):
  ```bash
  ps -eo pid,args | grep -E '[p]arameter_bridge|[g]z sim|[r]tabmap|[_]server' \
    | awk -v me=$$ '$1 != me {print $1}' | xargs -r kill
  ```

## 주의할 점

- URDF 나 월드를 고친 뒤에는 Gazebo 를 완전히 종료했다가 다시 띄울 것
  (`gz sim` 서버가 남아 있으면 이전 모델이 재사용됨).
  `pkill -f` 는 자기 명령줄에도 걸려 스스로를 죽이므로 위의 `ps | awk` 방식을 쓸 것.
- 파라미터만 바꿨을 때는 전체를 내리지 말 것. 시뮬과 SLAM 은 켜 둔 채
  Nav2 노드만 죽였다 `ros2 launch orinbot_navigation nav2.launch.py` 로 다시
  올리면 된다 (약 45초). 전체 재시작은 3분 이상 걸린다.
- 컨트롤러가 안 올라오면 `ros2 control list_controllers` 로 상태부터 확인.
- 카메라가 안 나오면 월드 SDF 에 `gz-sim-sensors-system` 플러그인이 있는지 확인.
