"""Standalone launch file for autonomous frontier exploration node.

    ros2 launch orinbot_navigation explore.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'start_paused', default_value='false',
            description='멈춘 채로 시작 (임무 관리자가 켤 때까지 대기)'),
        DeclareLaunchArgument(
            'return_home', default_value='true',
            description='Return to home pose after exploration completes'),
        DeclareLaunchArgument(
            'explore_timeout', default_value='0.0',
            description='Overall timeout [s] (0 for unlimited)'),
        DeclareLaunchArgument(
            'gain', default_value='1.5',
            description='Unexplored area gain weight'),
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
            'start_paused': LaunchConfiguration('start_paused'),
            'return_home': LaunchConfiguration('return_home'),
            'explore_timeout': LaunchConfiguration('explore_timeout'),
            'gain': LaunchConfiguration('gain'),
        }],
    )

    return LaunchDescription(args + [node])
