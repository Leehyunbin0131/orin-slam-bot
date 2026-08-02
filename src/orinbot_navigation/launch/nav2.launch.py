"""Nav2 자율주행 스택 런치 파일.

    ros2 launch orinbot_navigation nav2.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# lifecycle_manager 가 순서대로 configure/activate 시킬 노드들.
# 코스트맵을 쓰는 서버가 먼저 올라와야 하므로 순서가 의미 있습니다.
# 기본 행동 트리(navigate_to_pose_w_replanning_and_recovery.xml)가 실제로
# 호출하는 액션은 ComputePathToPose / FollowPath / Spin / BackUp / Wait 뿐입니다.
# smoother_server(SmoothPath) 와 waypoint_follower(FollowWaypoints) 는
# 호출되지 않는데도 각각 CPU 5% / 3.6% 를 쓰고 있어 뺐습니다.
# 나중에 웨이포인트 주행이나 경로 평활화가 필요해지면 되살리세요.
LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    twist_mux_file = LaunchConfiguration('twist_mux_file')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('orinbot_navigation'), 'config', 'nav2_params.yaml']),
            description='Nav2 파라미터 파일'),
        DeclareLaunchArgument(
            'twist_mux_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('orinbot_navigation'), 'config', 'twist_mux.yaml']),
            description='twist_mux 파라미터 파일'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='lifecycle 노드를 자동으로 activate'),
        DeclareLaunchArgument(
            'use_composition', default_value='false',
            description='[동작 안 함 — 위 주석 참고] 서버 6개를 한 프로세스에 '
                        '모읍니다. 코스트맵이 설정을 못 받아 컨테이너가 죽습니다'),
        DeclareLaunchArgument(
            'startup_watchdog', default_value='true',
            description='lifecycle 활성화가 실패하면 자동으로 다시 겁니다'),
        DeclareLaunchArgument(
            'bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('orinbot_navigation'), 'behavior_trees',
                'navigate_w_fast_recovery.xml']),
            description='navigate_to_pose 행동 트리. 기본은 후진을 Spin 보다 '
                        '앞세운 전용 트리입니다. Nav2 순정과 비교하려면 '
                        '$(ros2 pkg prefix nav2_bt_navigator)/share/.../'
                        'navigate_to_pose_w_replanning_and_recovery.xml'),
    ]

    common = [params_file, {'use_sim_time': True}]
    # bt_navigator 에만 주는 추가 파라미터.
    bt_extra = {'default_nav_to_pose_bt_xml': LaunchConfiguration('bt_xml')}
    composed = IfCondition(LaunchConfiguration('use_composition'))
    separate = UnlessCondition(LaunchConfiguration('use_composition'))

    # (패키지, 실행파일, 컴포넌트 플러그인, 노드이름, 리매핑, 추가 파라미터)
    # 순서는 lifecycle_manager 가 configure/activate 하는 순서와 맞춥니다.
    CMD_VEL_NAV = [('cmd_vel', 'cmd_vel_nav')]
    SERVERS = [
        # 컨트롤러 출력은 velocity_smoother 를 거쳐 /cmd_vel 로 나갑니다
        ('nav2_controller', 'controller_server',
         'nav2_controller::ControllerServer', 'controller_server', CMD_VEL_NAV, None),
        ('nav2_planner', 'planner_server',
         'nav2_planner::PlannerServer', 'planner_server', [], None),
        ('nav2_behaviors', 'behavior_server',
         'behavior_server::BehaviorServer', 'behavior_server', CMD_VEL_NAV, None),
        ('nav2_bt_navigator', 'bt_navigator',
         'nav2_bt_navigator::BtNavigator', 'bt_navigator', [], bt_extra),
        # 출력은 기본 이름(/cmd_vel_smoothed) 그대로 두고 twist_mux 가 받습니다.
        # 예전처럼 여기서 바로 /cmd_vel 로 내보내면, 활성 상태의 smoother 가
        # 목표가 없을 때도 20Hz 로 0 을 발행해 수동 조작을 덮어씁니다.
        ('nav2_velocity_smoother', 'velocity_smoother',
         'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother',
         CMD_VEL_NAV, None),
    ]

    lifecycle_params = [{
        'use_sim_time': True,
        'autostart': LaunchConfiguration('autostart'),
        'node_names': LIFECYCLE_NODES,
    }]

    # --- 프로세스를 따로 쓰는 기존 방식 ---
    separate_nodes = [
        Node(package=pkg, executable=exe, name=name, output='screen',
             parameters=common + ([extra] if extra else []),
             remappings=remap, condition=separate)
        for pkg, exe, _plugin, name, remap, extra in SERVERS
    ] + [
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=lifecycle_params, condition=separate),
    ]

    # --- 한 프로세스에 모으는 방식 ---
    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        # 서버들이 각자 여러 스레드를 쓰므로 멀티스레드 실행기가 필요합니다.
        # component_container(단일 스레드)를 쓰면 코스트맵 갱신이 액션 처리를
        # 막아 제어 주기를 놓칩니다.
        executable='component_container_isolated',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg, plugin=plugin, name=name,
                parameters=common + ([extra] if extra else []), remappings=remap,
                extra_arguments=[{'use_intra_process_comms': True}])
            for pkg, _exe, plugin, name, remap, extra in SERVERS
        ] + [
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=lifecycle_params,
                extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        condition=composed,
    )

    # 자율주행 명령과 사람의 명령을 우선순위로 중재합니다.
    # lifecycle 노드가 아니라서 lifecycle_manager 목록에 넣지 않습니다.
    # 컴포넌트로 등록돼 있지 않아 항상 별도 프로세스입니다.
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_file, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'cmd_vel')],
    )

    # lifecycle 활성화가 실패하면 다시 걸어 주고 스스로 끝납니다.
    # 이 PC 에서 하루에 여러 번 났고, 실기(Orin)는 더 잘 납니다.
    # 자세한 사정은 scripts/nav2_startup_watchdog.py 의 설명 참고.
    watchdog = Node(
        package='orinbot_navigation',
        executable='nav2_startup_watchdog.py',
        name='nav2_startup_watchdog',
        output='screen',
        # use_sim_time 을 주지 않습니다. 기동 감시가 /clock 에 묶이면
        # 시뮬레이터가 아직 시계를 안 내보낼 때 영원히 대기합니다.
        parameters=[],
        condition=IfCondition(LaunchConfiguration('startup_watchdog')),
    )

    return LaunchDescription(
        args + separate_nodes + [container, twist_mux, watchdog])
