"""URDF 모델 검증 및 시각화 전용 런치 파일 (robot_state_publisher + RViz2).

    ros2 launch orinbot_description display.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('orinbot_description')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'orinbot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'orinbot.rviz'])

    # sim_mode:=false 로 두어야 Gazebo 전용 플러그인 없이 파싱됩니다.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_mode:=false']),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='joint_state_publisher_gui 사용 여부',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('use_gui')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
