"""Docking-only test bench, built to run several instances in parallel.

    ros2 launch orinbot_navigation dock_bench.launch.py

No SLAM and no Nav2: measuring docking accuracy only needs the simulator,
camera, marker detection and staged_dock, and the rest costs 2-3 cores per
instance.

Two behaviours change without them (both only warn):
  - `_goto_entry` needs Nav2, so the test script sends the goal with
    `navigate_to_staging_pose: false` and spawns the robot near the
    alignment point.
  - `_face_dock`, `_restore_standoff` and the SLAM freeze need the map frame
    and rtabmap services, and are skipped.

There is no twist_mux either, so staged_dock publishes straight to /cmd_vel.
Parallel runs are isolated per instance by ROS_DOMAIN_ID and GZ_PARTITION.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = FindPackageShare('orinbot_navigation')
    bringup_share = FindPackageShare('orinbot_bringup')
    params_file = LaunchConfiguration('params_file')
    common = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([nav_share, 'config', 'docking.yaml'])),
        DeclareLaunchArgument('world', default_value='room.sdf'),
        # Spawn pose, varied per case to create the initial error.
        DeclareLaunchArgument('x', default_value='1.0'),
        DeclareLaunchArgument('y', default_value='-3.10'),
        DeclareLaunchArgument('yaw', default_value='-1.5708'),
        DeclareLaunchArgument('initial_soc', default_value='0.85'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'sim.launch.py'])]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': 'false',
            'use_rviz': 'false',
            # No costmap here, so skip pointcloud synthesis.
            'use_pointcloud': 'false',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    detector = Node(
        package='orinbot_navigation', executable='dock_marker_board.py',
        name='dock_marker_board', output='screen',
        parameters=[params_file, common, {'lock_distance': 0.0}],
        remappings=[('image', '/camera/color/image_raw'),
                    ('camera_info', '/camera/color/camera_info'),
                    ('detected_dock_pose', '/detected_dock_pose')],
    )

    dock = Node(
        package='orinbot_navigation', executable='staged_dock.py',
        name='staged_dock', output='screen',
        parameters=[params_file, common],
        # No twist_mux: publish straight to the controller.
        remappings=[('cmd_vel_dock', '/cmd_vel')],
    )

    battery = Node(
        package='orinbot_bringup', executable='battery_sim.py',
        name='battery_sim', output='screen',
        parameters=[common, {'initial_soc': LaunchConfiguration('initial_soc')}],
    )

    return LaunchDescription(args + [sim, detector, dock, battery])
