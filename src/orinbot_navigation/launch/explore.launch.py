"""프론티어 자동 탐사 노드 단독 런치 파일.

    ros2 launch orinbot_navigation explore.launch.py
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
        package='orinbot_navigation',
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
