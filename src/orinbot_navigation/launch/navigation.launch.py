"""시뮬레이션 + VSLAM + Nav2 를 한 번에.

    ros2 launch orinbot_navigation navigation.launch.py

    # 이미 만들어 둔 지도로 자율주행만
    ros2 launch orinbot_navigation navigation.launch.py localization:=true

    # 시뮬레이터를 따로 띄워 놓고 SLAM+Nav2 만
    ros2 launch orinbot_navigation navigation.launch.py use_sim:=false

    # 지도 없이 시작해 스스로 미탐색 구역을 돌며 지도를 만듦
    ros2 launch orinbot_navigation navigation.launch.py explore:=true

    # 도킹 없이 (실기에서 도크를 안 쓸 때 / 자원을 아낄 때)
    ros2 launch orinbot_navigation navigation.launch.py dock:=false

    # 충전 복귀 시나리오를 몇 분 안에 보기 (배터리 60배속, 30% 에서 시작)
    ros2 launch orinbot_navigation navigation.launch.py \
        battery_speedup:=60 initial_soc:=0.3

RViz 에서 "2D Goal Pose" 로 목표를 찍으면 주행합니다.
지도가 아직 비어 있으면 경로계획이 실패하니, explore:=true 로 자동 탐사를
시키거나 텔레오퍼레이션으로 한 바퀴 돌아 지도를 만든 뒤 목표를 주세요.

기동 순서에 대해
-----------------
세 스택을 동시에 띄우면 CPU 경합으로 Nav2 의 lifecycle 전이가 자주
실패합니다 (planner_server 가 코스트맵을 만드는 동안 change_state 응답이
유실됨). 그래서 고정 시간 지연 대신, 선행 조건 토픽이 실제로 나올 때까지
기다렸다가 다음 단계를 띄웁니다.

    시뮬 --(/odom 대기)--> VSLAM --(/map 대기)--> Nav2
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def wait_for_topic(topic, timeout_s, label):
    """해당 토픽에 첫 메시지가 올 때까지 블록하는 프로세스.

    타임아웃되어도 (비정상 종료해도) 다음 단계는 진행됩니다. 무한정
    멈춰 있는 것보다 일단 띄우고 로그로 원인을 보는 편이 낫습니다.
    """
    return ExecuteProcess(
        cmd=['bash', '-c',
             f'echo "[{label}] {topic} 대기 중..."; '
             f'timeout {timeout_s} ros2 topic echo {topic} --once > /dev/null '
             f'&& echo "[{label}] {topic} 확인" '
             f'|| echo "[{label}] {topic} 대기 시간 초과 — 그대로 진행합니다"'],
        output='screen',
        name=label,
    )


def generate_launch_description():
    nav_share = FindPackageShare('orinbot_navigation')
    bringup_share = FindPackageShare('orinbot_bringup')

    args = [
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='Gazebo 시뮬레이터도 함께 실행'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true 면 기존 지도로 위치추정만'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'map_3d', default_value='false',
            description='RTAB-Map 3D 점유격자. 기본 false. '
                        'true 로 켜면 메모리가 2.7배, CPU 가 16% 늘어납니다'),
        DeclareLaunchArgument(
            'reg_strategy', default_value='2',
            description='루프 클로저 검증. 0=영상만, 2=영상+ICP'),
        DeclareLaunchArgument(
            'use_vslam', default_value='true',
            description='시각 오도메트리 사용. false 면 EKF(휠+IMU) 만. '
                        '실기에서 CPU 를 아끼려면 false'),
        DeclareLaunchArgument(
            'detection_rate', default_value='2.0',
            description='rtabmap 이 지도 노드를 추가하는 최대 주기 [Hz]. '
                        'Orin 에서 CPU 가 모자랄 때 효과가 가장 큰 손잡이입니다 '
                        '(1.0 이면 rtabmap CPU -38%, 메모리 -106MB). '
                        '대가는 정확도 — slam.launch.py 의 실측표 참고'),
        DeclareLaunchArgument(
            'memory_thr', default_value='3000',
            description='RTAB-Map 작업 메모리에 유지할 노드 수 상한. '
                        '0 이면 무제한. 낮추면 장기 기억으로 내려간 노드의 '
                        '격자가 /map 에서 사라집니다 — slam.launch.py 주석 참고'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI'),
        DeclareLaunchArgument(
            'world', default_value='room.sdf',
            description='orinbot_bringup/worlds/ 아래의 월드 파일. '
                        'room(기준선) / maze(협소) / hall(대형) / office(실전형)'),
        # 월드를 바꿀 때는 이것도 같이 바꾸세요. 한 DB 에 서로 다른 월드를
        # 이어 붙이면 RTAB-Map 이 이전 월드의 포즈 그래프 위에 매핑합니다.
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/orinbot_rtabmap.db',
            description='RTAB-Map DB 경로. 월드마다 다르게 두세요'),
        # 카메라가 15도 아래를 보므로 뎁스를 LaserScan 으로 눌러 쓸 수 없습니다
        # (바닥이 통째로 장애물이 됩니다). 코스트맵이 높이 필터와 함께
        # 포인트클라우드를 직접 씁니다.
        DeclareLaunchArgument('use_pointcloud', default_value='true',
                              description='뎁스 포인트클라우드 (코스트맵 장애물 관측원)'),
        DeclareLaunchArgument(
            'explore', default_value='false',
            description='지도 없이 시작해 스스로 미탐색 구역을 탐사. '
                        'localization:=true 와 함께 쓰면 의미가 없습니다'),
        DeclareLaunchArgument(
            'dock', default_value='true',
            description='충전 도킹 (마커 검출 + docking_server + 배터리)'),
        DeclareLaunchArgument(
            'auto_dock', default_value='true',
            description='잔량이 떨어지면 스스로 도크로 복귀. false 로 두면 '
                        '도킹 기능은 살아 있고 사람이 /dock_robot 을 직접 겁니다'),
        DeclareLaunchArgument(
            'battery_speedup', default_value='1.0',
            description='배터리 방전/충전 시간 배속. 도킹 시나리오를 몇 분 '
                        '안에 보려면 60 정도로 올리세요'),
        DeclareLaunchArgument(
            'initial_soc', default_value='0.85',
            description='시작 잔량 (0~1)'),
    ]

    # GroupAction 으로 감싸는 이유
    # ------------------------------------------------------------------
    # IncludeLaunchDescription 의 launch_arguments 는 부모 스코프로 새어
    # 나갑니다. sim 에 use_rviz:=false 를 넘기면 그 값이 이 런치의
    # use_rviz 까지 덮어써서, 아래 rviz 노드가 조건 거짓으로 아예 실행되지
    # 않습니다(실제로 그렇게 동작했습니다). GroupAction 이 스코프를 격리합니다.
    sim = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'sim.launch.py'])]),
        launch_arguments={
            'use_rviz': 'false',        # RViz 는 아래 네비게이션용 설정으로 하나만
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            # 포인트클라우드는 시각화 전용인데 120 MB/s 를 먹습니다.
            # SLAM/Nav2 는 뎁스 영상과 /scan 만 쓰므로 꺼 둡니다.
            'use_pointcloud': LaunchConfiguration('use_pointcloud'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )])

    slam = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav_share, 'launch', 'slam.launch.py'])]),
        launch_arguments={
            'localization': LaunchConfiguration('localization'),
            'use_vslam': LaunchConfiguration('use_vslam'),
            'map_3d': LaunchConfiguration('map_3d'),
            'reg_strategy': LaunchConfiguration('reg_strategy'),
            # 자원 절감용 손잡이 둘. 여기서 넘기지 않으면 slam.launch.py 를
            # 직접 띄울 때만 쓸 수 있어서, 정작 실기 운용 진입점인 이 런치에서는
            # detection_rate:=1.0 을 줘도 조용히 무시됩니다.
            'detection_rate': LaunchConfiguration('detection_rate'),
            'memory_thr': LaunchConfiguration('memory_thr'),
            'database_path': LaunchConfiguration('database_path'),
        }.items(),
    )])

    nav2 = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav_share, 'launch', 'nav2.launch.py'])]),
    )])

    # 탐사 노드는 /map 과 navigate_to_pose 를 스스로 기다리므로 순차 기동
    # 체인에 끼울 필요가 없습니다. 처음부터 띄워 두면 준비되는 대로 시작합니다.
    explore = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav_share, 'launch', 'explore.launch.py'])]),
    )], condition=IfCondition(LaunchConfiguration('explore')))

    # 도킹은 Nav2 뒤에 띄웁니다. docking_server 의 충돌 검사기가
    # /local_costmap/costmap_raw 를 구독하고, staging pose 이동은
    # navigate_to_pose 를 씁니다. 둘 다 Nav2 가 올라와야 생깁니다.
    dock = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav_share, 'launch', 'docking.launch.py'])]),
        launch_arguments={
            'auto_dock': LaunchConfiguration('auto_dock'),
            'battery_speedup': LaunchConfiguration('battery_speedup'),
            'initial_soc': LaunchConfiguration('initial_soc'),
        }.items(),
    )], condition=IfCondition(LaunchConfiguration('dock')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([nav_share, 'rviz', 'navigation.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ------------------------------------------------------------------
    # 순차 기동
    # ------------------------------------------------------------------
    # /odom 이 나온다 = diff_drive_controller 활성화 완료.
    # 이게 있어야 rgbd_odometry 가 휠 오도메트리를 guess 로 쓸 수 있습니다.
    wait_odom = wait_for_topic('/odom', 120, 'wait_odom')
    start_slam = RegisterEventHandler(
        OnProcessExit(target_action=wait_odom, on_exit=[slam]))

    # /map 이 나온다 = RTAB-Map 이 첫 점유격자를 발행함.
    # 이게 있어야 global_costmap 의 StaticLayer 가 바로 초기화됩니다.
    wait_map = wait_for_topic('/map', 120, 'wait_map')
    start_wait_map = RegisterEventHandler(
        OnProcessExit(target_action=wait_odom, on_exit=[wait_map]))
    start_nav2 = RegisterEventHandler(
        OnProcessExit(target_action=wait_map, on_exit=[nav2, dock]))

    return LaunchDescription(args + [
        sim,
        rviz,
        explore,
        wait_odom,
        start_slam,
        start_wait_map,
        start_nav2,
    ])
