"""임무 시험용 런처 — 로봇을 도크에 세워 두고 명령을 기다립니다.

    ros2 launch orinbot_navigation mission.launch.py

이것을 띄워 두고, 다른 터미널에서 임무 명령을 보냅니다.

    ros2 service call /mission/start_mapping std_srvs/srv/Trigger   # 자동 매핑
    ros2 service call /mission/cancel        std_srvs/srv/Trigger   # 중단 후 복귀
    ros2 topic echo /mission/state                                  # 진행 상황

로봇은 도크에서 시작해 임무가 없으면 계속 충전하며 대기하고, 명령을 받으면
절전 해제 -> 언도킹 -> 임무 수행 -> 복귀 -> 대기 순으로 한 사이클을 돕니다.

navigation.launch.py 를 그대로 쓰되 임무 사이클에 맞게 세 가지를 바꿉니다.

- 탐사를 **멈춘 채로** 띄웁니다. 안 그러면 임무 명령 없이 로봇이 나갑니다.
- 탐사 완료 후 시작 지점 복귀를 **끕니다**. 도킹이 진입점까지 데려가므로
  같은 길을 두 번 갑니다.
- 로봇을 **도킹 완료 자세**에 스폰합니다. 그래야 dock_register 가 부팅 때
  도크 좌표를 잡고, auto_dock 이 충전을 보고 대기 상태로 들어갑니다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = FindPackageShare('orinbot_navigation')

    args = [
        DeclareLaunchArgument('world', default_value='room.sdf'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI'),
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='false 면 시뮬레이터 없이 임무 스택만 띄웁니다'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/orinbot_rtabmap.db',
            description='RTAB-Map 데이터베이스. 월드를 바꾸면 함께 바꾸세요'),
        DeclareLaunchArgument('initial_soc', default_value='0.95'),
        DeclareLaunchArgument(
            'battery_speedup', default_value='1.0',
            description='배터리 방전/충전 배속'),
        DeclareLaunchArgument(
            'mission_timeout', default_value='0.0',
            description='임무 한도 [s]. 0 이면 무제한'),
    ]

    # **이 include 는 GroupAction 으로 감싸지 않습니다.** navigation.launch.py 는
    # SLAM/Nav2/도킹을 OnProcessExit 이벤트 핸들러로 **나중에** 띄우는데,
    # GroupAction 은 스코프를 만들었다가 include 가 끝날 때 닫습니다. 그러면
    # 뒤늦게 실행되는 핸들러가 인자를 못 찾아 `launch configuration
    # 'localization' does not exist` 로 스택 전체가 내려갑니다.
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav_share, 'launch', 'navigation.launch.py'])]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'database_path': LaunchConfiguration('database_path'),
            'initial_soc': LaunchConfiguration('initial_soc'),
            'battery_speedup': LaunchConfiguration('battery_speedup'),
            'dock': 'true',
            'auto_dock': 'true',
            'explore': 'true',
            'explore_paused': 'true',
            'explore_return_home': 'false',
        }.items(),
    )

    # 노드 이름이 곧 명령 이름입니다 — `mission` 이라 서비스가
    # /mission/start_mapping, /mission/cancel 이고 상태가 /mission/state 입니다.
    manager = Node(
        package='orinbot_navigation',
        executable='mission_manager.py',
        name='mission',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mission_timeout': LaunchConfiguration('mission_timeout'),
        }],
    )

    return LaunchDescription(args + [stack, manager])
