# orinbot — ROS 2 Jazzy & Gazebo Harmonic 자율주행 시뮬레이션 워크스페이스

본 프로젝트는 가상 시뮬레이션 환경과 실물 로봇 환경 간의 **완전한 인터페이스 호환성(100% API Parity)**을 보장하도록 설계된 ROS 2 자율주행 워크스페이스입니다. 

주 배포 타깃은 초저전력 임베디드 AI 플랫폼인 **Jetson Orin Nano Super**(6코어 ARM Cortex-A78AE, 8GB Unified Memory)입니다.

```bash
ros2 launch orinbot_navigation mission.launch.py                # 자율주행 스택 및 임무 관리자 실행
ros2 service call /mission/start_mapping std_srvs/srv/Trigger   # 자동 자율 탐사 및 매핑 시작
```

![Gazebo Harmonic에서 실행 중인 room.sdf 월드](docs/images/gazebo-room.png)

*`room.sdf` (10 × 8 m) — 시각적 오도메트리(Visual Odometry)의 특징점 추출 성능을 극대화하기 위해 각 벽면에 고유 텍스처를 적용한 월드입니다. 내부에는 파티션, 복도, 3종의 가변 높이 선반 및 저상 장애물이 정교하게 배치되어 있습니다.*

![RViz — RTAB-Map 지도와 Nav2 코스트맵](docs/images/rviz-navigation.png)

*프론티어 기반 자율 탐사(Frontier Exploration)를 통해 실시간으로 방 전체를 매핑한 결과 화면 (206초 만에 임무 완료). 검은색 영역은 점유 격자 지도(Occupancy Grid), 빨간색 점은 라이다 360° 스캔 데이터, 청록/보라색 영역은 로컬 코스트맵 팽창 레이어(Costmap Inflation Area)이며, 좌측 하단 윈도우는 VSLAM 프론트엔드 입력용 RGB 칼라 영상 스트림입니다.*

---

## 목차

- [관련 문서 목록](#관련-문서-목록)
- [로봇 하드웨어 사양](#로봇-하드웨어-사양)
- [패키지 구조 및 역할](#패키지-구조-및-역할)
- [시스템 설치](#시스템-설치) · [빌드 가이드](#빌드-가이드)
- **[실행 가이드](#실행-가이드)** — **[QUICK START]**
  - [빠른 시작](#빠른-시작) — 원클릭 실행 커맨드
  - **[명령어 전체 레퍼런스](#명령어-전체-레퍼런스)** — 바로 복사해서 사용하는 종합 CLI 레퍼런스
  - [임무 생주기 (Mission Lifecycle)](#임무-생주기-mission-lifecycle) · [실행 파라미터 옵션](#실행-파라미터-옵션) · [트러블슈팅 및 주의사항](#트러블슈팅-및-주의사항)
  - [3가지 배포 아키텍처](#3가지-배포-아키텍처--컴퓨팅-노드-배치-전략) — PC 단독 / 분산 처리(PC+Orin) / 실무 노드 단독
- [시뮬레이터 · 월드 · 예제 스크립트](#시뮬레이터--월드--예제-스크립트)
  - [시뮬레이션 단독 실행](#시뮬레이션-단독-실행)
  - [제공 월드 라인업](#제공-월드-라인업)
  - [URDF 모델 검증 및 수동 텔레옵(Teleop)](#urdf-모델-검증-및-수동-텔레옵teleop)
  - [예제 노드](#예제-노드)
- [시스템 인터페이스 (Topic & Action)](#시스템-인터페이스-topic--action) — 토픽/액션 명세 및 [TF 프레임 트랙 구조](#tf-프레임-트랙-구조)
- [실물 로봇 배포 마이그레이션 가이드](#실물-로봇-배포-마이그레이션-가이드)
- [핵심 아키텍처 및 세부 기술 사양](#핵심-아키텍처-및-세부-기술-사양)
  - [위치 추정(Localization) 및 코스트맵 파이프라인](#위치-추정localization-및-코스트맵-파이프라인)
  - [경로 제어 및 플래닝 알고리즘](#경로-제어-및-플래닝-알고리즘)
  - [정밀 자동 충전 도킹 (OpenNav Docking)](#정밀-자동-충전-도킹-opennav-docking)

---

## 관련 문서 목록

| 문서명 | 주요 내용 및 목적 |
|---|---|
| `README.md` (본 문서) | 전체 시스템 설치, 실행 방법, 인터페이스 규격 및 주요 아키텍처 |
| **`docs/ros2-lessons.md`** | **실무 개발 중 직면했던 핵심 엔지니어링 문제와 해결 과정 분석** — 문제 원인 추적 및 검증된 대응책 기술 |
| `tools/README.md` | 시스템 성능 Benchmarking 툴 및 베이스라인 성능 데이터 |
| `CLAUDE.md` | 개발 및 코딩 컨벤션 지침 (규칙, 제약조건 및 파라미터 세팅값) |

> **개발자 노트 (`docs/ros2-lessons.md`를 정리하며)**
>
> ROS 2 기반 자율주행 스택을 설계하고 최적화하는 과정에서 마주했던 수많은 실패와 치열한 고찰의 기록들을 [`docs/ros2-lessons.md`](docs/ros2-lessons.md)에 가감 없이 담았습니다. 자율주행 로봇을 직접 제작하고 정밀 제어 체계를 구축하려는 엔지니어 분들께 깊이 있는 영감과 기술적 이정표가 되기를 진심으로 바랍니다.

---

## 로봇 하드웨어 사양

- **구동 방식**: Differential Drive (2륜 차동 구동 방식)
- **캐스터 구조**: 전방 2개 / 후방 2개 (마찰 계수 0의 무마찰 구체로 시뮬레이션 모델링)
- **섀시 규격**: 0.40 m 정육면체 (최대 외접 반지름 **0.283 m**)
- **비전 센서**: Intel RealSense D435i (전방 탑재, 하향 15° 피치 틸트) — RGB + Depth + IMU 융합
- **라이다(LiDAR)**: RPLIDAR A2M12 (상단 중앙 탑재) — 360° 전방위 스캔 (지면 기준 측정 평면 높이 0.49 m)

로봇의 주요 폼팩터 치수는 `orinbot_description/urdf/orinbot.urdf.xacro` 파일 상단의 매개변수 블록에 명시되어 있습니다.
실물 하드웨어 제작 시 섀시 규격이 변경되면 해당 파일 및 `orinbot_bringup/config/controllers.yaml` 내부의 `wheel_separation`, `wheel_radius` 파라미터를 동일하게 동기화해야 합니다.

---

## 패키지 구조 및 역할

| 패키지명 | 역할 및 주요 기능 |
|---|---|
| `orinbot_description` | URDF/Xacro 기반 로봇 kinematic 키네마틱 모델링, RViz 시각화 레이아웃 및 URDF 검증 런치 파일 |
| `orinbot_bringup` | Gazebo 물리 월드, controllers/bridge/EKF 하드웨어 설정 파일 및 시뮬레이션 통합 런치 파이프라인 |
| `orinbot_examples_py` | `rclpy` 기반 예제 알고리즘 노드 (`square_driver`) |
| `orinbot_examples_cpp` | `rclcpp` 기반 고성능 C++ 예제 노드 (`depth_safety_filter`) |
| `orinbot_navigation` | RTAB-Map VSLAM + Nav2 자율주행 스택 + 프론티어 자동 탐사 및 자율 도킹 파이프라인 |
| `tools/` | 시스템 성능 측정, 벤치마킹 및 회귀 테스트 스크립트 모음 (`tools/README.md` 참고) |

---

## 시스템 설치

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

의존성 패키지는 소스 트리를 기준으로 다음과 같이 일괄 설치할 수 있습니다:
```bash
rosdep install --from-paths src -y --ignore-src
```

### 필수 런타임 패키지 체크리스트

| 의존 패키지명 | 미설치 시 치명적 오류 현상 |
|---|---|
| `spatio-temporal-voxel-layer` | 3D 코스트맵 복셀 레이어 플러그인 로드 실패로 입체 장애물 회피 불가능 |
| `twist-mux` | 속도 명령 멀티플렉싱 불능으로 인한 `/cmd_vel` 토픽 미발행 (로봇 동작 중단) |
| `robot-localization` | EKF 노드 미실행으로 `odom -> base_footprint` TF 릴레이 트랜스폼 단절 |

### 필수 네트워크 환경변수 설정

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

복수의 네트워크 인터페이스 카드(NIC)가 활성화된 환경에서는 FastDDS의 멀티캐스트 디스커버리 트래픽으로 인해 Nav2 Lifecycle Manager의 상태 전이 응답 서비스 타임아웃이 발생할 수 있습니다. 디스커버리 바운더리를 `LOCALHOST`로 고정하여 네트워크 병목을 완벽히 방지하십시오.

> **참고**: ROS 2 Jazzy 환경에서는 Gazebo Harmonic 버전을 `ros-jazzy-gz-*-vendor` 공식 벤더 패키지로 기본 제공하므로, OSRF 서드파티 저장소를 추가할 필요가 없습니다.

---

## 빌드 가이드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 실행 가이드

### 빠른 시작

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 launch orinbot_navigation mission.launch.py          # [터미널 1] 시스템 메인 스택 기동
ros2 service call /mission/start_mapping std_srvs/srv/Trigger   # [터미널 2] 매핑 및 자동 탐사 트리거
```

시스템이 초기화되면 로봇이 도크에 접속된 상태로 스폰되며, 약 40초 후 `임무 상태 메세지: DOCKED`가 출력되면 모든 명령 수신 준비가 완료됩니다. 매핑 명령을 전달하면 로봇이 스스로 언도킹한 후 전 공간을 자율 매핑하고 원래 도크 위치로 완벽히 자동 복귀합니다.

| 목적 및 유즈케이스 | 실행 런치 파일 |
|---|---|
| **전체 미션 자동화 (기본 실행)** | `ros2 launch orinbot_navigation mission.launch.py` |
| 자율주행 스택 단독 기동 (미션 매니저 비활성화) | `ros2 launch orinbot_navigation navigation.launch.py` |
| 시뮬레이션 엔진 단독 기동 (월드 및 센서 검증) | `ros2 launch orinbot_bringup sim.launch.py` |
| URDF 기네마틱 모델 검증 (Gazebo 오프로드) | `ros2 launch orinbot_description display.launch.py` |

---

### 명령어 전체 레퍼런스

`mission.launch.py` 기동 후 독립된 제2 터미널에서 제어하는 제어 커맨드 집합입니다.

#### 임무 제어 (Mission Control)

| 제어 목적 | CLI 실행 명령어 |
|---|---|
| 자율 탐사 매핑 시작 | `ros2 service call /mission/start_mapping std_srvs/srv/Trigger` |
| 현재 미션 즉시 중단 및 도크 복귀 | `ros2 service call /mission/cancel std_srvs/srv/Trigger` |

#### 도킹 제어 (Docking Control) — 미션 매니저 개입 없이 수동 도킹 수행 시

| 제어 목적 | CLI 실행 명령어 |
|---|---|
| 웨이크업 및 자동 언도킹 | `ros2 service call /auto_dock/leave std_srvs/srv/Trigger` |
| 도크 자동 복귀 및 접안 | `ros2 service call /auto_dock/return std_srvs/srv/Trigger` |
| 현재 로봇 좌표를 도크 원점으로 등록 | `ros2 service call /dock_register/register std_srvs/srv/Trigger` |
| DockRobot 액션 직접 호출 | `ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{use_dock_id: true, dock_id: home_dock}"` |
| UndockRobot 액션 직접 호출 | `ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot "{}"` |

#### 상태 모니터링 (Observation)

| 관측 대상 | CLI 실행 명령어 |
|---|---|
| 미션 상태 머신 모니터링 | `ros2 topic echo /mission/state` |
| 도킹 모듈 릴레이 상태 | `ros2 topic echo /dock_state` |
| 충전 전류 모니터링 (양수: 충전 중) | `ros2 topic echo /battery_state \| grep current` |
| 로봇 정밀 참값 포즈 (Simulator Ground Truth) | `ros2 topic echo /ground_truth/odom --field pose.pose.position` |

#### 시스템 진단 및 수동 오버라이드 (Diagnostics & Manual Override)

| 제어 목적 | CLI 실행 명령어 |
|---|---|
| 프론티어 탐사 노드 재설정 | `ros2 service call /frontier_explorer/reset std_srvs/srv/Trigger` |
| SLAM 맵 업데이트 일시 정지 / 재개 | `ros2 service call /rtabmap/pause std_srvs/srv/Empty` <br> `ros2 service call /rtabmap/resume std_srvs/srv/Empty` |
| 마커 검출 알고리즘 일시 정지 / 재개 | `ros2 service call /dock_marker_board/pause std_srvs/srv/Empty` <br> `ros2 service call /dock_marker_board/resume std_srvs/srv/Empty` |
| VSLAM TF 릴레이 보정 동결 / 재개 | `ros2 service call /vodom_tf_relay/pause std_srvs/srv/Empty` <br> `ros2 service call /vodom_tf_relay/resume std_srvs/srv/Empty` |
| Nav2 Lifecycle 초기화 (RESET -> STARTUP) | `ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"` <br> (재시작 시 동일 명령에 `{command: 0}` 전달) |

---

### 임무 생주기 (Mission Lifecycle)

본 로봇은 지능형 로봇 청소기의 시퀀스 아키텍처와 동일하게 동작합니다. 충전 도크에서 대기하다 명령이 전달되면 정밀 언도킹 후 탐사/복귀를 수행합니다.

```
[도크 대기] ──► [명령 수신] ──► [절전 해제] ──► [언도킹] ──► [임무 수행] ──► [복귀] ──► [도크 접안 & 충전]
```

`/mission/state` 토픽을 통해 실시간 FSM(Finite State Machine) 상태를 추적할 수 있습니다:

| FSM 상태명 | 묘사 및 상세 동작 |
|---|---|
| `DOCKED` | 도크에 완전 접안하여 대기/충전 중. **본 상태에서만 신규 미션 명령을 수용합니다.** |
| `WAKING` -> `UNDOCKING` | 서보 및 노드 절전 모드를 해제하고 후진/회전 언도킹 수행 중 |
| `RUNNING:mapping` | 프론티어 자동 탐사 및 자율주행 매핑 임무 수행 중 |
| `SUSPENDED:mapping` | 임계 배터리 전압 하강으로 인해 임무 일시 중단 후 충전 도킹. 충전 완료 시 **중단 지점부터 재개**합니다. |
| `RETURNING` | 매핑 완료 또는 취소 명령으로 인해 도크 원점으로 자율 주행 복귀 중 |

새로운 임무 커스텀 정의는 `mission_manager.py` 파일의 `MISSIONS` 구조체 단 한 곳에서 관리됩니다.

---

### 실행 파라미터 옵션

`navigation.launch.py` 및 `mission.launch.py`에서 제공하는 매개변수 명세입니다:

| 인자명 | 기본값 | 상세 설명 |
|---|---|---|
| `world` | `room.sdf` | 로드할 Gazebo 물리 월드 파일 지정 |
| `database_path` | `~/.ros/orinbot_rtabmap.db` | RTAB-Map 3D 공간 맵 데이터베이스 저장 경로 |
| `x` `y` `yaw` | `1.0` `-3.64` `1.5708` | 로봇 초기 생성 위치 좌표 및 요(Yaw) 회전각 |
| `explore` | `false` | 자율 프론티어 탐사 노드 자동 기동 여부 |
| `explore_paused` | `false` | 탐사 노드를 일시 정지 상태로 시작 (미션 매니저 시그널 대기) |
| `localization` | `false` | 위치 추정 전용 모드 (신규 지도 갱신 비활성화, 기존 DB 참조) |
| `use_sim` | `true` | Gazebo 시뮬레이터 동시 기동 여부 |
| `use_rviz` / `gui` | `true` | RViz2 및 Gazebo GUI 그래픽 인터페이스 표시 여부 |
| `use_vslam` | `true` | Visual Odometry(비전 기반 오도메트리) 활성화 여부 |
| `detection_rate` | `2.0` | RTAB-Map Graph Node 업데이트 주기 [Hz] |
| `memory_thr` | `3000` | RTAB-Map Working Memory 최대 노드 수 제어 (0 = 무제한) |
| `map_3d` | `false` | 3D Pointcloud Octomap 점유 격자 생성 활성화 여부 |
| `reg_strategy` | `2` | 루프 클로저(Loop Closure) 검증 제어 (0=Visual, 2=Visual+ICP) |
| `dock` | `true` | 충전 도킹 하위 스택 로드 여부 |
| `docking_mode` | `staged` | 도킹 알고리즘 선택 (`staged` 단계별 제어 \| `smooth` 연속 궤적 제어) |
| `auto_dock` | `true` | 배터리 잔량 모니터링 기반 자동 복귀 알고리즘 활성화 |
| `dock_register` | `true` | 도크 접안 부팅 시 현재 포즈를 도크 원점으로 자동 등록 |
| `clock_rate` | `100.0` | ROS `/clock` 시간 퍼블리시 주기 [Hz] |
| `battery_speedup` | `1.0` | 시뮬레이션 배터리 방전/충전 가속 속도 비율 |
| `initial_soc` | `0.85` | 초기 로봇 부팅 시 배터리 충전 잔량 (SOC: 0.0 ~ 1.0) |

#### 대표적인 파라미터 조합 예시

```bash
# 미션 매니저 없이 즉시 자율 탐사 시작
ros2 launch orinbot_navigation navigation.launch.py explore:=true

# 기존 매핑 지도를 로드하여 순수 위치 추정(Localization) 모드로 실행
ros2 launch orinbot_navigation navigation.launch.py localization:=true

# GUI 없는 헤드리스(Headless) 고성능 실행
ros2 launch orinbot_navigation navigation.launch.py gui:=false use_rviz:=false

# 시뮬레이터가 별도 실행 중일 때 순수 항법 스택만 바인딩
ros2 launch orinbot_navigation navigation.launch.py use_sim:=false
```

---

### 트러블슈팅 및 주의사항

| 장애 증상 | 원인 분석 및 팁 |
|---|---|
| 서비스 명령을 전송해도 로봇이 반응하지 않음 | 시스템 상태가 `DOCKED`에 완전히 도달했는지 확인하십시오. 초기화 이전 명령은 차단됩니다. |
| `이미 ... 작업 중입니다` 오류 메시지 반환 | `/auto_dock/*` 명령은 `RETURNING` 또는 `UNDOCKING` 상태에서 거부됩니다. `CHARGING` 또는 `IDLE` 상태에서 호출하십시오. |
| 간헐적 위치 점프 및 TF 조회 타임아웃 발생 | 이전 세션의 잔여 `parameter_bridge`가 좀비 프로세스로 남아 `/clock`을 중복 퍼블리시하는 현상입니다. 아래 정리 스크립트를 실행하십시오. |
| `Nav2 PAUSE failed` 또는 `bt_navigator: unconfigured` | Lifecycle 노드 기동 실패 상태입니다. 프로세스를 종료하고 깨끗하게 재시작하십시오. |
| 비전 인지가 정지된 상태로 외딴 주행 수행 | `pause` 서비스 호출 후 `resume`을 수행하지 않은 상태입니다. 절전 제어는 `auto_dock` 노드가 제어하므로 명시적으로 조작하지 마십시오. |

#### 런타임 필수 점검: 단일 `/clock` 퍼블리셔 검증

```bash
ros2 topic info /clock | grep Publisher      # 반드시 Publisher count: 1 이어야 함

# 퍼블리셔가 2 이상일 경우 잔여 프로세스 강제 정리
ps -eo pid,args | grep -E '/opt/ros/jazzy|ros2_ws/install|[g]z sim' \
  | grep -v shell-snapshots | awk -v me=$$ '$1 != me {print $1}' | xargs -r kill -9
ros2 daemon stop
```

모든 지도 DB 및 등록된 도크 위치를 완전히 초기화하고 clean start 하려면 아래 파일들을 제거하십시오:
```bash
rm -f ~/.ros/orinbot_rtabmap.db ~/.ros/orinbot_docks.yaml
```

---

### 3가지 배포 아키텍처 — 컴퓨팅 노드 배치 전략

| 아키텍처 구분 | 시뮬레이터(Gazebo) | 시각화(RViz) | 주 연산 노드 | 상태 및 검증 |
|---|---|---|---|---|
| **[구성 1] PC 단독** | PC | PC | PC | **완전 검증 완료** |
| **[구성 2] 분산 처리 (PC + Orin)** | PC | PC | **Jetson Orin** | **완전 검증 완료** |
| **[구성 3] 실물 로봇 단독** | 없음 (실물 센서) | 없음 | **로봇 임베디드** | **배포 준비 중** |

#### [구성 1] 단일 개발 PC 실행
[빠른 시작](#빠른-시작)에 명시된 기본 환경입니다. 단일 PC에서 모든 시뮬레이션과 항법 연산을 수행하며, 알고리즘 개발 및 회귀 테스트에 적합합니다.

#### [구성 2] 엣지 컴퓨팅 분산 처리 (PC + Jetson Orin)
실물 배포에 가장 근접한 테스트 형태입니다. **Jetson Orin 보드가 VSLAM 및 Nav2 연산 부하를 실시간으로 처리할 수 있는지** 검증합니다.

```
 [개발 PC]                                        [Jetson Orin Nano Super]
 ─────────────────────────────                   ─────────────────────────────────────────
 Gazebo (물리 엔진 & 센서 렌더링)   ◄──────────►   카메라 전처리 (Depth -> PointCloud 복원)
 RViz2 (상태 및 맵 시각화)                        VSLAM (rgbd_odometry + rtabmap)
                                                 Nav2 / 프론티어 탐사 / 도킹 스택
```

**① 네트워크 디스커버리 동기화** — 두 기기의 `ROS_DOMAIN_ID`를 명시적으로 일치시키고, `ROS_AUTOMATIC_DISCOVERY_RANGE`를 `SUBNET`으로 전환합니다.

```bash
# PC 및 Orin 양쪽 터미널 모두 적용
export ROS_DOMAIN_ID=0
unset ROS_AUTOMATIC_DISCOVERY_RANGE
```

**② 개발 PC — 시뮬레이터 및 시각화 전담 실행**

```bash
ros2 launch orinbot_bringup sim.launch.py use_rviz:=false use_pointcloud:=false
ros2 run rviz2 rviz2 -d src/orinbot_navigation/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

> **핵심 최적화: `use_pointcloud:=false`**
>
> 3D 포인트클라우드 raw 데이터를 PC에서 생성하여 네트워크로 전송할 경우 네트워크 트래픽이 **589 Mbps**까지 치솟지만, PC는 뎁스 영상만 송신하고 Orin 내부에서 포인트클라우드를 복원하게 설계하면 트래픽이 **134 Mbps**로 대폭 감소합니다 (1 Gbps 유선 대역폭 기준 네트워크 점유율 59% -> 13% 절감). **실물 로봇 배포 시에도 이 아키텍처가 필수적으로 적용됩니다.**

**③ Jetson Orin — 카메라 전처리 및 자율주행 메인 스택 기동**

```bash
ros2 launch orinbot_navigation navigation.launch.py \
  use_sim:=false use_rviz:=false explore:=true
```

**벤치마크 실측 성적** (`room.sdf` 월드, 1 Gbps 유선 환경):

| 측정 항목 | 실측 성능 수치 |
|---|---|
| Orin CPU 점유율 | 평균 3.2 / 6 코어 |
| 탐사 매핑 완주 시간 | 138 ~ 170 초 |
| SLAM 위치 추정 오차 | 중앙값(Median) 5 mm |
| PC -> Orin 전송 대역폭 | 134 Mbps |

> **주의: 탐사(Explore)와 도킹(Docking)을 동시에 활성화하지 마십시오.**
>
> 두 모듈을 동시에 구동할 경우 Orin의 CPU 부하가 4.9/6 코어까지 급증하여 `rtabmap`의 실시간 처리 타임 바운더리(0.5s)를 초과하게 되며, 이로 인해 Pose Graph가 붕괴되고 지도가 파손될 수 있습니다. 실제 운용 환경에서도 두 모듈은 상호 배타적으로 실행되며, `auto_dock`의 노드 절전 기능과 `dock_marker_board`의 `~/pause` 서비스가 이를 안전하게 관리합니다.

---

#### [구성 3] 실물 로봇 단독 실행 (배포 준비 중)

시뮬레이터와 RViz2 오버헤드 없이 로봇 임베디드 컴퓨터 내에서 항법 자율주행 스택만 구동하는 최종 배포 아키텍처입니다.

##### 실물 하드웨어 전환 시 요구사항:

| 관련 항목 | 현재 시뮬레이션 설정 | 실물 로봇 배포 시 전환 필요사항 |
|---|---|---|
| **시계 계통** | `use_sim_time: true` 하드코딩 | 런치 파라미터 분리 및 `false` 전환 (실기에는 `/clock`이 없으므로 노드가 멈춤) |
| 센서 드라이버 | Gazebo Bridge 노드 | `realsense2_camera` (`align_depth.enable:=true`), `rplidar_ros` 실물 드라이버 바인딩 |
| 구동 제어 | `gz_ros2_control` | URDF 내 `<hardware>` 블록을 실물 모터 컨트롤러 `hardware_interface`로 교체 |
| 배터리 관리 | `battery_sim` (Gazebo 위치 참값 기반) | 실물 BMS 스마트 배터리 센서 바인딩 (충전 전류 기반 접안 판정) |
| 하드웨어 안전 | 도크 전방 0.7m 코스트맵 검사 해제 | **범퍼 스위치, 모터 과전류 차단 등 펌웨어/하드웨어 차원의 물리적 보호 수단 배치** |

---

## 시뮬레이터 · 월드 · 예제 스크립트

시뮬레이션 환경 단독 기동 및 모델 검증 방법입니다.

### 시뮬레이션 단독 실행

```bash
ros2 launch orinbot_bringup sim.launch.py
```

| 파라미터 인자 | 기본값 | 묘사 및 역할 |
|---|---|---|
| `world` | `room.sdf` | `orinbot_bringup/worlds/` 디렉터리 내 월드 파일 |
| `x` `y` `z` `yaw` | `0 0 0.15 0` | 로봇 스폰 초기 좌표 및 회전각 |
| `use_rviz` | `true` | RViz2 시각화 동시 기동 여부 |
| `gui` | `true` | `false` 설정 시 Gazebo GUI 렌더링을 끄고 Headless 실행 |
| `use_pointcloud` | `true` | `depth_image_proc` 기반 포인트 클라우드 파이프라인 기동 여부 |
| `clock_rate` | `100.0` | `/clock` 퍼블리시 빈도 [Hz] (0 지정 시 Gazebo 원본 클락 사용) |

---

### 제공 월드 라인업

| 월드 파일명 | 공간 폼팩터 | 주요 벤치마크 목적 | 도크 배치 여부 |
|---|---|---|---|
| `room.sdf` (기본) | 10 × 8 m | 알고리즘 회귀 테스트 및 표준 성능 베이스라인 측정 | 기본 배치 |
| `maze.sdf` | 6 × 6 m | 극소 협소 통로(0.75 m) 통과 및 회차 한계 검증 | 미배치 |
| `hall.sdf` | 24 × 18 m | 광역 공간 주행 시 SLAM 누적 오차 및 탐사 수렴성 측정 | 기본 배치 |
| `office.sdf` | 20 × 14 m | 복합 환경 및 동적 보행자(3명) 회피 주행 성능 검증 | 기본 배치 |

```bash
ros2 launch orinbot_navigation navigation.launch.py world:=office.sdf explore:=true
ros2 launch orinbot_navigation navigation.launch.py world:=maze.sdf dock:=false
```

> 월드를 전환하여 실행할 때는 RTAB-Map 포즈 그래프 맵 붕괴를 방지하기 위해 DB 저장 경로를 반드시 분리하여 실행하십시오 (`database_path:=~/.ros/office.db`).

#### 텍스처 절차적 생성 스크립트

```bash
python3 src/orinbot_bringup/models/room_materials/generate_textures.py maze hall
```

#### 동적 보행자 장애물 시뮬레이션

`office.sdf` 월드 내의 동적 보행자는 아래 헬퍼 노드로 제어합니다:

```bash
ros2 run orinbot_bringup people_sim.py --ros-args -p use_sim_time:=true
```

| 동적 객체 ID | 이동 속도 | 행동 패턴 및 특성 |
|---|---|---|
| `person_0` | 1.20 m/s | 고속 직진 보행 |
| `person_1` | 0.55 m/s | 저속 보행 (경로 단부 도달 시 6초간 웨이팅) |
| `person_2` | 0.85 m/s | 일반 표준 속도 보행 |

보행자는 로봇이 진행 방향 ±75° 이내 1.4 m 거리로 접근하면 안전을 위해 일시 정지하며, 1.8 m 이상 이격되면 보행을 재개합니다. 12초 이상 대기 상황이 지속되면 반대 방향으로 회전하여 주행합니다.

---

### URDF 모델 검증 및 수동 텔레옵(Teleop)

```bash
# Gazebo 물리 엔진 없이 URDF 키네마틱스 모델만 렌더링 확인
ros2 launch orinbot_description display.launch.py

# 키보드 수동 텔레옵 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_footprint
```

---

### 예제 노드

```bash
# Python 기반 정사각형 궤적 드라이버 노드
ros2 run orinbot_examples_py square_driver --ros-args -p use_sim_time:=true

# C++ 기반 Depth 이미지 비상 정지 안전 필터 노드
ros2 run orinbot_examples_cpp depth_safety_filter --ros-args -p use_sim_time:=true
```

---

## 시스템 인터페이스 (Topic & Action)

### ROS 2 토픽 명세

| 토픽 경로 | 메시지 타입 | 방향 | 비고 및 상세 설명 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Input | `twist_mux`를 거친 최종 구동 속도 출력 토픽 |
| `/cmd_vel_teleop` | `geometry_msgs/TwistStamped` | Input | 수동 원격 조종 속도 (최우선 제어권 부여) |
| `/odom` | `nav_msgs/Odometry` | Output | 휠 엔코더 기반 휠 오도메트리 |
| `/odometry/filtered` | `nav_msgs/Odometry` | Output | EKF 노드가 융합한 보정 오도메트리 (Wheel + IMU) |
| `/joint_states` | `sensor_msgs/JointState` | Output | 로봇 관절 상태 정보 |
| `/scan` | `sensor_msgs/LaserScan` | Output | RPLIDAR 360° 2D 라이다 스캔 데이터 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Output | VSLAM 특징점 추적용 RGB 영상 스트림 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Output | 렌즈 왜곡 보정 뎁스 영상 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Output | RGB 프레임 좌표계로 정렬된 뎁스 영상 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | Output | STVL 복셀 레이어 입력용 3D 포인트클라우드 |
| `/camera/imu` | `sensor_msgs/Imu` | Output | IMU 6축 융합 센서 데이터 |
| `/map` | `nav_msgs/OccupancyGrid` | Output | RTAB-Map 점유 격자 지도 (QoS: RELIABLE + TRANSIENT_LOCAL) |
| `/battery_state` | `sensor_msgs/BatteryState` | Output | 배터리 상태 (전압, 전류, 잔량 SOC) |
| `/detected_dock_pose` | `geometry_msgs/PoseStamped` | Output | 비전 알루코 마커 기반 도크 추정 포즈 |
| `/cmd_vel_dock` | `geometry_msgs/TwistStamped` | Internal | 정밀 도킹 제어 속도 토픽 (우선순위: 50) |
| `/exploration_enabled` | `std_msgs/Bool` | Input | 자율 탐사 스위칭 파이프라인 |
| `/ground_truth/odom` | `nav_msgs/Odometry` | Output | Gazebo 참값 좌표 포즈 (시뮬레이션 전용) |

### ROS 2 액션 명세

| 액션 경로 | 액션 타입 | 목적 및 비고 |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | 목표 지점(Goal Pose) 자율주행 이동 |
| `/dock_robot` | `nav2_msgs/DockRobot` | 충전 도크 정밀 자동 접안 |
| `/undock_robot` | `nav2_msgs/UndockRobot` | 도크 안전 이탈 및 언도킹 |

> `/map` 토픽을 커스텀 노드에서 구독할 때는 반드시 QoS 프로필을 **RELIABLE + TRANSIENT_LOCAL** 조합으로 지정해야 데이터를 정상 수신할 수 있습니다.

---

### TF 프레임 트랙 구조

```
map ──(rtabmap)──► vodom ──(rgbd_odometry)──► odom ──(EKF)──► base_footprint
                                                               ├──► base_link
                                                               │     ├──► 휠 / 캐스터
                                                               │     ├──► laser
                                                               │     └──► camera_*
```

---

## 실물 로봇 배포 마이그레이션 가이드

`orinbot_description/urdf/orinbot.ros2_control.xacro` 파일 내부의 `<hardware>` 태그 블록을 실물 모터 드라이버용 `hardware_interface` C++ 플러그인으로 교체하기만 하면, 상위 알고리즘 스택(Nav2, RTAB-Map, 미션 매니저 등)의 수정 없이 100% 즉시 실물 로봇으로 심리스 이식(Seamless Migration)할 수 있습니다.

---

## 핵심 아키텍처 및 세부 기술 사양

### 위치 추정(Localization) 및 코스트맵 파이프라인

- **Visual Odometry**: `rtabmap_odom/rgbd_odometry` (휠 오도메트리를 motion prediction 초기 모션 예측값으로 활용)
- **SLAM / Mapping**: `rtabmap_slam/rtabmap` (QoS: RELIABLE + TRANSIENT_LOCAL 적용)
- **Obstacle Layer**: RPLIDAR `/scan` 기반 2D 동적 장애물 소거 (Ray Clearing)
- **STVL Layer**: RealSense D435i `/camera/depth/points` 기반 3D 복셀 지형 감쇄 소거 (`voxel_decay`: 로컬 5.0 / 전역 10.0)
- **속도 파이프라인**: `controller_server` -> `/cmd_vel_nav` -> `velocity_smoother` -> `/cmd_vel_smoothed` -> `twist_mux` -> `/cmd_vel`

---

### 경로 제어 및 플래닝 알고리즘

- **전역 플래너 (Global Planner)**: `SmacPlanner2D` (`cost_travel_multiplier: 4.0`)
- **지역 제어기 (Local Controller)**: `MPPI` (Model Predictive Path Integral) 제어기
  - `GoalCritic`: 0.5
  - `PathAlignCritic`: 0.25
  - `PathFollowCritic`: 0.3
- **통로 통과 최소 폭 규격**: **0.70 m** (로봇 제자리 회전 시 외접 반지름 0.566 m 기준 안전 마진 반영)

---

### 정밀 자동 충전 도킹 (OpenNav Docking)

| 도킹 방식 | 제어 알고리즘 핵심 | 특징 및 성능 |
|---|---|---|
| `staged` (기본 추천 방식) | `scripts/staged_dock.py` | 정지 -> 비전 측정 -> 각도 보정 단계 분리 제어, 후진 정밀 접안 |
| `smooth` | Nav2 순정 `opennav_docking` | 마커 실시간 트래킹 기반 순차 곡선 궤적 접안 |

- **도크 비전 인식 파이프라인**: ArUco `DICT_4X4_50` 마커 3장을 좌우 0.16 m 간격으로 배치한 싱글 타깃 보드를 통해 정밀 포즈 추정 (`dock_marker_board.py`)
- **후진 접안 시퀀스**: 마커를 전방 정면으로 인식하여 정렬을 완료한 후, 회전점(마커면 0.60 m 전방)에서 180° 피봇 회전 후 후진으로 접안합니다. 이로 인해 충전 중 카메라가 벽면이 아닌 오픈된 장소를 향하므로 VSLAM 시각 오도메트리 추적 상태가 완벽히 유지됩니다.
- **접안 물리 허용 오차**: 세로 ±48 mm / 가로 ±34 mm (포고핀 핀 헤드가 동판 전면 도금에 안정적으로 얹히는 오차 마진 범위)
- **도킹 절전 파이프라인**: 도킹 접안 성공 시 인지 및 항법 파이프라인을 즉시 일시 정지(Lifecycle PAUSE / RTAB-Map pause)시켜 불필요한 전력 소모 및 발열 최소화

```bash
# 도킹 파라미터 스윕 벤치마킹 (다양한 환경 조건 동시 테스트)
python3 tools/dock_bench.py --jobs 4
```
