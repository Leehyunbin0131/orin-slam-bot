"""Simulation, VSLAM, and Nav2 integrated autonomous navigation launch file.

    ros2 launch orinbot_navigation navigation.launch.py
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
    """Block until the first message arrives on a topic.

    Continues on timeout: starting anyway and reading the log beats hanging.

    Default (VOLATILE) QoS -- /odom is VOLATILE and /map is TRANSIENT_LOCAL,
    and a VOLATILE subscriber matches both.
    """
    return ExecuteProcess(
        cmd=['bash', '-c',
             f'timeout {timeout_s} ros2 topic echo --once {topic} > /dev/null '
             f'&& echo "[{label}] {topic} ok" '
             f'|| echo "[{label}] {topic} timed out, continuing"'],
        output='screen',
        name=label,
    )


def generate_launch_description():
    nav_share = FindPackageShare('orinbot_navigation')
    bringup_share = FindPackageShare('orinbot_bringup')

    args = [
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='Launch Gazebo simulator'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localization-only mode'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'map_3d', default_value='false',
            description='RTAB-Map 3D occupancy grid toggle'),
        DeclareLaunchArgument(
            'reg_strategy', default_value='2',
            description='Loop closure strategy (0=Vis, 2=Vis+ICP)'),
        DeclareLaunchArgument(
            'use_vslam', default_value='true',
            description='Use visual odometry'),
        DeclareLaunchArgument(
            'detection_rate', default_value='2.0',
            description='RTAB-Map node detection rate [Hz]'),
        DeclareLaunchArgument(
            'memory_thr', default_value='3000',
            description='RTAB-Map working memory node limit'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch Gazebo GUI'),
        DeclareLaunchArgument(
            'world', default_value='room.sdf',
            description='World file name in orinbot_bringup/worlds/'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/orinbot_rtabmap.db',
            description='RTAB-Map database path'),
        DeclareLaunchArgument('use_pointcloud', default_value='true',
                              description='Depth pointcloud generation'),
        DeclareLaunchArgument(
            'explore', default_value='false',
            description='Autonomous frontier exploration toggle'),
        DeclareLaunchArgument(
            'explore_paused', default_value='false',
            description='Start exploration paused until mission_manager enables it'),
        DeclareLaunchArgument(
            'explore_return_home', default_value='true',
            description='Return to the start pose after exploring; false for '
                        'the mission cycle, where docking drives to staging'),
        DeclareLaunchArgument(
            'dock', default_value='true',
            description='Charging docking stack toggle'),
        DeclareLaunchArgument(
            'auto_dock', default_value='true',
            description='Autonomous low battery docking return'),
        DeclareLaunchArgument(
            'battery_speedup', default_value='1.0',
            description='Battery discharge/charge speedup factor'),
        DeclareLaunchArgument(
            'initial_soc', default_value='0.85',
            description='Initial battery state of charge (0 to 1)'),
        # Spawn pose. Defaults to the docked pose (docking.yaml dock_x/dock_y,
        # yaw opposite dock_yaw because docking is reversed). Move these
        # together with dock_distance, or dock_register stores a pose the
        # robot never actually reaches.
        DeclareLaunchArgument('x', default_value='1.0'),
        DeclareLaunchArgument('y', default_value='-3.64'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
    ]

    sim = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'sim.launch.py'])]),
        launch_arguments={
            'use_rviz': 'false',
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'use_pointcloud': LaunchConfiguration('use_pointcloud'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
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
            'detection_rate': LaunchConfiguration('detection_rate'),
            'memory_thr': LaunchConfiguration('memory_thr'),
            'database_path': LaunchConfiguration('database_path'),
        }.items(),
    )])

    nav2 = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav_share, 'launch', 'nav2.launch.py'])]),
    )])

    explore = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav_share, 'launch', 'explore.launch.py'])]),
        launch_arguments={
            'start_paused': LaunchConfiguration('explore_paused'),
            'return_home': LaunchConfiguration('explore_return_home'),
        }.items(),
    )], condition=IfCondition(LaunchConfiguration('explore')))

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

    # Sequential bringup chain
    wait_odom = wait_for_topic('/odom', 120, 'wait_odom')
    start_slam = RegisterEventHandler(
        OnProcessExit(target_action=wait_odom, on_exit=[slam]))

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
