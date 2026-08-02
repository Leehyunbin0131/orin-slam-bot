"""orinbot 시뮬레이션 전체 구동.

    ros2 launch orinbot_bringup sim.launch.py

띄우는 것:
  1. Gazebo Harmonic (worlds/room.sdf)
  2. robot_state_publisher  (URDF -> /robot_description, TF)
  3. Gazebo 에 로봇 스폰
  4. ros_gz_bridge  (카메라/IMU/clock)
  5. ros2_control 컨트롤러 (joint_state_broadcaster, diff_drive_controller)
  6. RViz2 (옵션)

컨트롤러 스포너는 로봇 스폰이 끝난 뒤에 순차 실행되도록 이벤트로 묶여 있습니다.
"""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = FindPackageShare('orinbot_description')
    bringup_share = FindPackageShare('orinbot_bringup')

    xacro_file = PathJoinSubstitution([desc_share, 'urdf', 'orinbot.urdf.xacro'])
    controllers_file = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])
    bridge_config = PathJoinSubstitution([bringup_share, 'config', 'gz_bridge.yaml'])
    rviz_config = PathJoinSubstitution([bringup_share, 'config', 'sim.rviz'])
    world_file = PathJoinSubstitution([bringup_share, 'worlds', LaunchConfiguration('world')])

    # ---------------- launch arguments ----------------
    args = [
        DeclareLaunchArgument('world', default_value='room.sdf',
                              description='worlds/ 아래의 월드 파일 이름'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.15'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI 사용 여부 (false 면 headless)'),
        DeclareLaunchArgument('use_pointcloud', default_value='true',
                              description='depth_image_proc 로 포인트클라우드 생성 '
                                          '(Nav2 코스트맵이 이걸 장애물 관측원으로 씁니다)'),
        DeclareLaunchArgument('clock_rate', default_value='100.0',
                              description='ROS 로 내보낼 /clock 주파수 [Hz]. 0 이면 '
                                          'gz 원본(997 Hz)을 그대로 통과시킵니다. '
                                          'scripts/clock_throttle.py 주석 참고'),
    ]

    # ---------------- robot_description ----------------
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' sim_mode:=true',
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ---------------- Gazebo ----------------
    # 월드 SDF 안의 model:// URI (텍스처) 를 찾을 수 있도록 경로 등록
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([bringup_share, 'models']),
    )

    # gui:=false 이면 -s (server only) 로 headless 실행
    headless_flag = PythonExpression([
        "'' if '", LaunchConfiguration('gui'), "' == 'true' else '-s '"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            # -r: 시작과 동시에 물리 엔진 실행, -v4: 로그 레벨
            'gz_args': ['-r -v4 ', headless_flag, world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------------- 로봇 스폰 ----------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'orinbot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # ---------------- 브리지 ----------------
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
    )

    # ---------------- 포인트클라우드 생성 ----------------
    # Gazebo 가 내보내는 클라우드는 좌표 규약이 frame_id 와 어긋나 있어
    # 쓰지 않습니다 (config/gz_bridge.yaml 주석 참고).
    # 대신 depth 영상 + camera_info 로 직접 생성합니다 — 실제
    # realsense2_camera 의 pointcloud 필터와 동일한 방식입니다.
    pointcloud_container = ComposableNodeContainer(
        name='camera_pointcloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                # 장애물 코스트맵용 포인트클라우드.
                # 여기에는 원본 뎁스(화각 87도)를 씁니다. 아래 정합 뎁스는
                # 컬러 화각(69도)으로 잘리므로 장애물 탐지에는 손해입니다.
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image_rect', '/camera/depth/image_rect_raw'),
                    ('camera_info', '/camera/depth/camera_info'),
                    ('points', '/camera/depth/points'),
                ],
            ),
            # 뎁스를 컬러 카메라 시점으로 재투영합니다.
            #
            # 실기의 realsense2_camera `align_depth.enable:=true` 와 같은 일을
            # 합니다. 실기에서도 이것은 하드웨어가 아니라 소프트웨어 재투영이라
            # 렌더링 스트림이 늘지 않습니다.
            #
            # 왜 필요한가: RTAB-Map 은 RGB 영상에서 찾은 특징점 픽셀에 뎁스 값을
            # 대입합니다. camera_color_optical_frame 은 뎁스 프레임에서 15 mm
            # 떨어져 있어(실기 D435i 와 동일) 정합 없이 넣으면 특징점에 엉뚱한
            # 깊이가 붙습니다.
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::RegisterNode',
                name='depth_register',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('rgb/camera_info', '/camera/color/camera_info'),
                    ('depth/camera_info', '/camera/depth/camera_info'),
                    ('depth/image_rect', '/camera/depth/image_rect_raw'),
                    ('depth_registered/camera_info',
                     '/camera/aligned_depth_to_color/camera_info'),
                    ('depth_registered/image_rect',
                     '/camera/aligned_depth_to_color/image_raw'),
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('use_pointcloud')),
    )

    # gz 의 /clock(997 Hz)을 솎아 /clock 으로 다시 냅니다.
    # 그대로 두면 use_sim_time 인 rclpy 노드가 시계 갱신에만 CPU 를 태웁니다
    # (Orin 실측 환산 노드당 약 113%p). 자세한 근거는 스크립트 주석 참고.
    clock_throttle = Node(
        package='orinbot_bringup',
        executable='clock_throttle.py',
        name='clock_throttle',
        output='screen',
        parameters=[{'rate': LaunchConfiguration('clock_rate'),
                     'use_sim_time': False}],
    )

    # ---------------- 컨트롤러 스포너 ----------------
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # 컨트롤러 기본 토픽(~/cmd_vel, ~/odom)을 표준 이름으로 리맵.
    # --controller-ros-args 는 컨트롤러 노드에만 적용되므로
    # controller_manager 에 직접 리맵을 거는 (deprecated) 방식보다 안전합니다.
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-ros-args', '-r /diff_drive_controller/cmd_vel:=/cmd_vel',
            '--controller-ros-args', '-r /diff_drive_controller/odom:=/odom',
        ],
    )

    # 스폰 완료 -> joint_state_broadcaster -> diff_drive_controller 순서 보장
    spawn_then_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])
    )
    jsb_then_diff_drive = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[diff_drive_spawner])
    )
    # ------------------------------------------------------------------
    # 이 노드가 odom -> base_footprint TF 를 담당합니다.
    # controllers.yaml 의 enable_odom_tf 가 false 여야 합니다 (둘 다 켜면
    # 같은 TF 를 두 노드가 발행해 TF 가 흔들립니다).
    #
    # 여기(bringup)에 두는 이유: EKF 는 SLAM 이 아니라 로봇 베이스의
    # 오도메트리에 속합니다. 실기로 옮길 때 이 런치만 실기용으로 바꾸면
    # navigation 쪽은 손대지 않아도 됩니다.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[PathJoinSubstitution([bringup_share, 'config', 'ekf.yaml'])],
    )

    # EKF 는 반드시 컨트롤러가 올라온 뒤에 띄웁니다.
    #
    # robot_localization 은 use_sim_time 일 때 시계가 유효해질 때까지
    # "Waiting for clock to start..." 를 찍으며 대기하는데, Gazebo 보다 먼저
    # 뜨면 그 대기에서 영구히 빠져나오지 못합니다 (실측: 노드는 살아 있는데
    # /odometry/filtered 도 odom->base_footprint TF 도 전혀 나오지 않음).
    # 그러면 TF 체인이 끊겨 RViz 에서 로봇이 튀고 매핑이 망가집니다.
    #
    # diff_drive_controller 스포너가 끝난 뒤면 /clock 과 /odom 이 모두
    # 살아 있으므로 안전합니다.
    diff_drive_then_ekf = RegisterEventHandler(
        OnProcessExit(target_action=diff_drive_spawner, on_exit=[ekf])
    )

    # ---------------- RViz ----------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ------------------------------------------------------------------
    # 휠 엔코더 + IMU 융합 EKF (odom -> base_footprint)

    return LaunchDescription(args + [
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        clock_throttle,
        spawn_robot,
        gz_bridge,
        pointcloud_container,
        spawn_then_jsb,
        jsb_then_diff_drive,
        diff_drive_then_ekf,
        rviz,
    ])
