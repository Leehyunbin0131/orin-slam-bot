"""Mission test launcher: spawns the robot docked and waits for commands.

    ros2 launch orinbot_navigation mission.launch.py

Leave this running and send commands from another terminal.

    ros2 service call /mission/start_mapping std_srvs/srv/Trigger
    ros2 service call /mission/cancel        std_srvs/srv/Trigger
    ros2 topic echo /mission/state

Same stack as navigation.launch.py with three changes for the mission cycle:
exploration starts paused (otherwise the robot leaves with no command), its
return-home is off (docking already drives to staging), and the robot spawns
at the docked pose so dock_register and auto_dock see a charging robot.
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
            description='false runs the mission stack without the simulator'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/orinbot_rtabmap.db',
            description='RTAB-Map database; change it when changing worlds'),
        DeclareLaunchArgument('initial_soc', default_value='0.95'),
        DeclareLaunchArgument(
            'battery_speedup', default_value='1.0',
            description='Battery discharge/charge speedup'),
        DeclareLaunchArgument(
            'mission_timeout', default_value='0.0',
            description='Mission timeout [s], 0 = unlimited'),
    ]

    # Not wrapped in a GroupAction: navigation.launch.py starts SLAM/Nav2/
    # docking from OnProcessExit handlers, and a group closes its scope when
    # the include returns, so those handlers lose their arguments.
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

    # The node name sets the command namespace: /mission/start_mapping,
    # /mission/cancel, /mission/state.
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
