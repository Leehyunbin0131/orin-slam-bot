"""프론티어 탐사 노드만 띄웁니다.

이미 시뮬 + SLAM + Nav2 가 돌고 있을 때:

    ros2 launch mybot_navigation explore.launch.py

노드가 /map 과 navigate_to_pose 액션 서버를 스스로 기다리므로 순서는
상관없습니다. 언제 띄워도 준비되면 알아서 시작합니다.

탐사를 멈추려면 이 런치만 Ctrl-C 하면 됩니다. 진행 중이던 목표는 Nav2 가
그대로 수행하니, 즉시 세우려면 RViz 의 Cancel 을 함께 누르세요.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'return_home', default_value='true',
            description='탐사가 끝나면 출발점으로 복귀'),
        DeclareLaunchArgument(
            'explore_timeout', default_value='0.0',
            description='전체 제한 시간 [s]. 0 이면 무제한'),
        DeclareLaunchArgument(
            'gain', default_value='1.5',
            description='클수록 "멀어도 큰 미탐색 구역"을 선호합니다'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true'),
    ]

    node = Node(
        package='mybot_navigation',
        executable='frontier_explorer.py',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'return_home': LaunchConfiguration('return_home'),
            'explore_timeout': LaunchConfiguration('explore_timeout'),
            'gain': LaunchConfiguration('gain'),
        }],
    )

    return LaunchDescription(args + [node])
