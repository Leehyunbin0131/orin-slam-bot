"""URDF 확인용 런치: Gazebo 없이 robot_state_publisher + RViz 만 띄웁니다.

    ros2 launch mybot_description display.launch.py

joint_state_publisher_gui 슬라이더로 바퀴를 돌려보며 링크/조인트가
의도대로 붙었는지 확인하는 용도입니다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('mybot_description')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'mybot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'mybot.rviz'])

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
