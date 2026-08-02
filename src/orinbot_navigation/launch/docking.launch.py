"""충전 도킹 일체 (마커 검출 + docking_server + 배터리 + 자동 복귀).

    ros2 launch orinbot_navigation docking.launch.py

Nav2 와 따로 떼어 둔 이유
-------------------------
- 도킹을 안 쓰는 구성에서는 통째로 빼기 위해서입니다. 실기(Orin)는
  코어가 6개뿐이라 안 쓰는 노드를 띄워 둘 여유가 없습니다.
- 파라미터를 만지면서 도킹만 다시 올릴 일이 많은데, Nav2 전체를
  내렸다 올리면 45초가 더 듭니다.

구성
----
    /camera/color/image_raw --> dock_marker_board --> /detected_dock_pose
                                                            |
    /battery_state <-- battery_sim.py                       v
            |                                        docking_server
            v                                         /dock_robot
        auto_dock.py  --(DockRobot)--------------------->  |
                                                           v
                                              /cmd_vel_dock --> twist_mux

토픽 이름에 대하여
------------------
docking_server 의 속도 출력은 `cmd_vel` 이라 그대로 두면 twist_mux 의
출력과 이름이 겹칩니다. `cmd_vel_dock` 으로 돌려 twist_mux 입력으로
넣습니다 (config/twist_mux.yaml 에 우선순위 50 으로 등록되어 있습니다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('orinbot_navigation'), 'config', 'docking.yaml']),
            description='도킹 파라미터 파일'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='docking_server 를 자동으로 activate'),
        DeclareLaunchArgument(
            'startup_watchdog', default_value='true',
            description='lifecycle 활성화가 실패하면 자동으로 다시 겁니다'),
        DeclareLaunchArgument(
            'use_battery_sim', default_value='true',
            description='배터리 시뮬레이터. 실기에서는 BMS 드라이버가 '
                        '같은 토픽을 내므로 false 로 두세요'),
        DeclareLaunchArgument(
            'battery_speedup', default_value='1.0',
            description='방전/충전 시간 배속. 1.0 이 실제 속도이고, '
                        '도킹 시나리오를 시험할 때만 올려 씁니다'),
        DeclareLaunchArgument(
            'initial_soc', default_value='0.85',
            description='시작 잔량 (0~1)'),
        DeclareLaunchArgument(
            'auto_dock', default_value='true',
            description='잔량이 떨어지면 스스로 도크로 복귀'),
        DeclareLaunchArgument(
            'docking_mode', default_value='staged',
            description="'staged' = 정지·측정·정렬을 끊어서 수행 "
                        "(staged_dock.py). 'smooth' = Nav2 순정 "
                        'opennav_docking 의 곡선 접근. 액션 규격은 같습니다'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/color/image_raw',
            description='마커를 찾을 영상. 컬러가 이미 15 Hz 로 돌고 있어 '
                        '추가 렌더링 부담이 없습니다. infra1 로 바꾸려면 '
                        'orinbot.urdf.xacro 에서 그 스트림부터 켜야 합니다'),
    ]

    common = {'use_sim_time': use_sim_time}

    # ArUco 마커 3장을 하나의 보드로 풀어 도크 자세를 냅니다.
    # 0.65 m 안으로 들어오면 그 자세를 odom 기준으로 고정하고, 이후에는
    # 같은 값을 계속 내보냅니다 — 로봇 입장에서는 "확정된 목표로 직진".
    # SimpleChargingDock 이 이 값을 받아 도크 자세로 환산합니다
    # (docking.yaml 의 external_detection_* 참고).
    detector = Node(
        package='orinbot_navigation',
        executable='dock_marker_board.py',
        name='dock_marker_board',
        output='screen',
        parameters=[params_file, common, {
            # 단계 도킹은 정지할 때마다 **새로** 재야 하므로 자세 고정을
            # 끕니다(0 = 안 함). 고정해 두면 회전 뒤의 "재측정"이 실은
            # 옛 값을 다시 투영한 것이 되어 검증이 되지 않습니다.
            'lock_distance': ParameterValue(PythonExpression([
                "0.0 if '", LaunchConfiguration('docking_mode'),
                "' == 'staged' else 0.45"]), value_type=float),
        }],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera_info', '/camera/color/camera_info'),
            ('detected_dock_pose', '/detected_dock_pose'),
        ],
    )

    staged = LaunchConfiguration('docking_mode')
    is_staged = PythonExpression(["'", staged, "' == 'staged'"])
    is_smooth = PythonExpression(["'", staged, "' != 'staged'"])

    # 단계 도킹 — 정지 상태 인식 정확도를 그대로 최종 자세로 가져갑니다.
    staged_server = Node(
        package='orinbot_navigation',
        executable='staged_dock.py',
        name='staged_dock',
        output='screen',
        parameters=[params_file, common],
        condition=IfCondition(is_staged),
    )

    # Nav2 순정 곡선 접근 (비교용).
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file, common],
        remappings=[('cmd_vel', 'cmd_vel_dock')],
        condition=IfCondition(is_smooth),
    )

    # lifecycle 관리자와 워치독은 opennav_docking(lifecycle 노드)에만
    # 필요합니다. staged_dock 은 평범한 노드라 관리 대상이 아닙니다.
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['docking_server'],
        }],
        condition=IfCondition(is_smooth),
    )

    # Nav2 쪽과 같은 이유로 둡니다 (nav2_startup_watchdog.py 설명 참고).
    # use_sim_time 을 주지 않습니다 — 기동 감시가 /clock 에 묶이면
    # 시뮬레이터가 시계를 내기 전에 영원히 대기합니다.
    watchdog = Node(
        package='orinbot_navigation',
        executable='nav2_startup_watchdog.py',
        name='docking_startup_watchdog',
        output='screen',
        parameters=[{'manager': '/lifecycle_manager_docking'}],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('startup_watchdog'),
            "' == 'true' and '", staged, "' != 'staged'"])),
    )

    battery = Node(
        package='orinbot_bringup',
        executable='battery_sim.py',
        name='battery_sim',
        output='screen',
        parameters=[common, {
            'speedup': LaunchConfiguration('battery_speedup'),
            'initial_soc': LaunchConfiguration('initial_soc'),
        }],
        condition=IfCondition(LaunchConfiguration('use_battery_sim')),
    )

    supervisor = Node(
        package='orinbot_navigation',
        executable='auto_dock.py',
        name='auto_dock',
        output='screen',
        parameters=[params_file, common],
        condition=IfCondition(LaunchConfiguration('auto_dock')),
    )

    return LaunchDescription(
        args + [detector, staged_server, docking_server, lifecycle,
                watchdog, battery, supervisor])
