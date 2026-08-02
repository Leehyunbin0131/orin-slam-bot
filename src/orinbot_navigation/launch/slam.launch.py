"""RTAB-Map RGB-D VSLAM.

    ros2 launch orinbot_navigation slam.launch.py                # 지도 작성
    ros2 launch orinbot_navigation slam.launch.py localization:=true  # 기존 지도로 위치추정

TF 구성:

    map --(rtabmap)--> vodom --(rgbd_odometry)--> odom --(diff_drive_controller)--> base_footprint

`rgbd_odometry` 는 휠 오도메트리(`odom` 프레임)를 모션 추정 초기값(guess)으로
받습니다. 이 때 rtabmap 은 자기 오도메트리 프레임(`vodom`)에서 guess 프레임
(`odom`)으로 가는 TF 를 내보내므로, `odom -> base_footprint` 를 이미
퍼블리시하는 diff_drive_controller 와 충돌하지 않습니다.

이 구조의 장점: 시각 특징이 부족한 구간(흰 벽 정면 등)에서 시각 오도메트리가
흔들려도 휠 오도메트리가 바닥을 받쳐 줍니다. 실제 로봇에서도 그대로 씁니다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# 센서 토픽 설정 (realsense2_camera 규격)
GRAY_TOPIC = '/camera/color/image_raw'
INFO_TOPIC = '/camera/color/camera_info'
DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'

# 원시 IMU (자세 없음) 와 필터가 자세를 채워 넣은 IMU
RAW_IMU_TOPIC = '/camera/imu'
IMU_TOPIC = '/camera/imu/data'

BASE_FRAME = 'base_footprint'
WHEEL_ODOM_FRAME = 'odom'
VISUAL_ODOM_FRAME = 'vodom'

# rtabmap 이 받을 오도메트리 토픽.
#   use_vslam=true  -> /vodom            (rgbd_odometry 의 시각 오도메트리)
#   use_vslam=false -> /odometry/filtered (EKF 의 휠+IMU 융합)
ODOM_TOPIC_FOR_RTABMAP = PythonExpression([
    "'/vodom' if '", LaunchConfiguration('use_vslam'), "' == 'true' "
    "else '/odometry/filtered'"])

RTABMAP_REMAPPINGS = [
    # 흑백 영상이지만 rtabmap 의 입력 이름은 rgb/* 그대로입니다.
    ('rgb/image', GRAY_TOPIC),
    ('rgb/camera_info', INFO_TOPIC),
    ('depth/image', DEPTH_TOPIC),
    ('odom', ODOM_TOPIC_FOR_RTABMAP),
    ('imu', RAW_IMU_TOPIC),
    ('scan', '/scan'),
]


def generate_launch_description():
    localization = LaunchConfiguration('localization')

    args = [
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='true 면 기존 DB 로 위치추정만 (지도 갱신 안 함)'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/orinbot_rtabmap.db',
            description='RTAB-Map 데이터베이스 경로'),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='false',
            description='RTAB-Map 전용 시각화 창 (특징점/루프클로저 디버깅용)'),
        DeclareLaunchArgument(
            'reg_strategy', default_value='2',
            description='루프 클로저 검증 방식. 0=영상만, 1=ICP만, 2=영상+ICP'),
        DeclareLaunchArgument(
            'memory_thr', default_value='3000',
            description='RTAB-Map 작업 메모리에 유지할 노드 수 상한. '
                        '0 이면 무제한. 300 은 /map 을 파괴합니다 — '
                        '아래 주석 참고'),
        DeclareLaunchArgument(
            'detection_rate', default_value='2.0',
            description='rtabmap 이 지도 노드를 추가하는 최대 주기 [Hz]. '
                        'Orin 에서 CPU 가 모자랄 때 가장 효과가 큰 손잡이입니다 '
                        '(1.0 으로 내리면 rtabmap CPU -38%, 메모리 -106MB). '
                        '대가는 정확도입니다 — 아래 실측표 참고'),
        DeclareLaunchArgument(
            'use_vslam', default_value='true',
            description='시각 오도메트리(rgbd_odometry) 사용. false 면 EKF(휠+IMU) 만 씁니다. '
                        '실기에서 CPU 를 아끼려면 false'),
        DeclareLaunchArgument(
            'use_imu_in_odom', default_value='false',
            description='시각 오도메트리에 IMU 사용 (아래 주석 참고 — 기본 꺼짐)'),
        DeclareLaunchArgument(
            'map_3d', default_value='false',
            description='3D 점유격자/포인트클라우드 생성 (/cloud_map, /octomap_*)'),
        DeclareLaunchArgument(
            'use_lidar_in_slam', default_value='true',
            description='RTAB-Map 에 라이다 스캔도 입력 (지도 품질 향상)'),
    ]

    # ------------------------------------------------------------------
    # 시각 오도메트리
    # ------------------------------------------------------------------
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': BASE_FRAME,
            'odom_frame_id': VISUAL_ODOM_FRAME,
            # 휠 오도메트리를 모션 예측 초기값으로 사용.
            # 이 값이 설정되면 vodom -> odom TF 를 발행한다.
            'guess_frame_id': WHEEL_ODOM_FRAME,
            # TF 는 직접 내보내지 않습니다. 영상 시각으로 스탬프가 찍히고
            # 영상 속도로만 나가서 Nav2 의 map->odom 조회가 실패합니다.
            # 대신 vodom_tf_relay 가 50 Hz 로 재발행합니다.
            'publish_tf': False,
            'approx_sync': True,
            # 컬러/뎁스 시각이 이보다 더 벌어진 쌍은 버립니다.
            # 어긋난 쌍을 쓰면 특징점에 엉뚱한 깊이가 붙어 드리프트합니다.
            'approx_sync_max_interval': 0.02,
            'wait_for_transform': 0.3,
            'publish_null_when_lost': False,
            # 주의: 기본값은 false 입니다.
            #
            # Reg/Force3DoF=true 인 평면 로봇에서 이 옵션을 켜면 rtabmap 이
            # IMU 자세로 오도메트리 방향을 잡으려 하는데, Madgwick 필터의
            # yaw 기준(ENU 초기화 시점)이 vodom 프레임과 무관해서 회전이
            # 전혀 추적되지 않습니다.
            # 실측: 제자리 360도 회전 후 각도 오차가
            #   켰을 때 89.62도 / 껐을 때 0.01도.
            #
            # 평면 주행 로봇에서 IMU 의 올바른 자리는 시각 오도메트리 내부가
            # 아니라, 휠 엔코더와 자이로 각속도를 융합하는 EKF
            # (robot_localization) 입니다.
            'wait_imu_to_init': LaunchConfiguration('use_imu_in_odom'),

            # --- RTAB-Map 내부 파라미터 ---
            # F2M(Frame-to-Map): 지역 특징점 지도를 유지해 프레임 간 방식보다 드리프트가 적다
            'Odom/Strategy': '0',
            'Odom/ResetCountdown': '1',      # 손실되면 즉시 리셋 후 재시작
            'Odom/GuessMotion': 'true',
            'OdomF2M/MaxSize': '1000',
            'Vis/MaxFeatures': '1000',
            'Vis/MinInliers': '15',
            'Vis/EstimationType': '1',       # 3D->2D (PnP)
            # 평면을 달리는 차동구동 로봇이므로 3자유도로 구속 -> 훨씬 안정적
            'Reg/Force3DoF': 'true',
        }],
        remappings=RTABMAP_REMAPPINGS,
        condition=IfCondition(LaunchConfiguration('use_vslam')),
    )

    # 시각 오도메트리 보정 TF 를 50 Hz 로 재발행 (자세한 이유는 스크립트 주석 참고)
    vodom_tf_relay = Node(
        package='orinbot_navigation',
        executable='vodom_tf_relay.py',
        name='vodom_tf_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'visual_odom_topic': '/vodom',
            'visual_odom_frame': VISUAL_ODOM_FRAME,
            'wheel_odom_frame': WHEEL_ODOM_FRAME,
            'base_frame': BASE_FRAME,
            'publish_rate': 50.0,
            'tf_tolerance': 0.2,
        }],
        condition=IfCondition(LaunchConfiguration('use_vslam')),
    )

    # ------------------------------------------------------------------
    # SLAM (지도 작성 + 루프 클로저)
    # ------------------------------------------------------------------
    rtabmap_parameters = {
        'use_sim_time': True,
        'frame_id': BASE_FRAME,
        'map_frame_id': 'map',
        'odom_frame_id': '',          # 빈 값 = /vodom 토픽에서 오도메트리를 받음
        'subscribe_depth': True,
        'subscribe_rgb': True,
        # 라이다 스캔을 함께 넣으면 점유격자가 360도로 채워지고
        # 루프 클로저 검증에도 기하 정보가 더해집니다.
        'subscribe_scan': True,
        'approx_sync': True,
        'approx_sync_max_interval': 0.02,
        'publish_tf': True,
        'database_path': LaunchConfiguration('database_path'),
        'wait_for_transform': 0.3,
        # map->vodom TF 를 20 Hz 로, 현재보다 조금 앞선 시각으로 찍어 발행합니다.
        # 그래야 Nav2 가 "지금" 시점으로 조회할 때 외삽 오류가 나지 않습니다.
        'tf_delay': 0.05,
        'tf_tolerance': 0.2,

        # --- 2D 로봇 구속 ---
        # 평면 주행이라 roll/pitch 를 0 으로 묶습니다. 이 상태에서는 IMU 의
        # 중력 제약이 크게 기여하지 않으므로 GravitySigma 는 꺼 둡니다.
        # (IMU 의 실질적인 기여는 rgbd_odometry 의 회전 예측 쪽입니다)
        'Optimizer/GravitySigma': '0.0',
        'Reg/Force3DoF': 'true',
        'RGBD/OptimizeMaxError': '3.0',

        # --- 작업 메모리 노드 수 상한 ---
        # 초과분은 장기 기억(디스크 DB)으로 내려갑니다. 상한이 없으면 계속
        # 자랍니다 (실측 834 -> 1491 MB, 장시간 뒤 5 GB).
        #
        # **낮추면 `/map` 이 깨집니다.** 장기 기억으로 내려간 노드의 격자는
        # 발행되는 지도에서도 사라집니다. 300 에서 1733 개 중 301 개만 남아
        # **이미 그린 구역이 미탐색으로 되돌아갔고**, 그 가짜 경계를 프론티어로
        # 잡아 탐사가 끝나지 않았습니다 (방 탐사 541초, 3000 에서는 158초).
        # 겉으로는 "이미 간 곳을 자꾸 다시 간다"로만 보입니다.
        #
        # 3000 은 방 규모에서는 사실상 무제한이고 홀/사무실에서만 걸립니다.
        # 메모리를 줄여야 하면 여기가 아니라 Grid/3D 를 보세요 — 그쪽이
        # 2656 -> 914 MB 로 훨씬 크고 지도를 망가뜨리지 않습니다.
        'Rtabmap/MemoryThr': ParameterValue(
            LaunchConfiguration('memory_thr'), value_type=str),
        # 지도 노드 추가 주기 [Hz]. 자원 절감 손잡이 중 효과가 가장 큽니다.
        # 실측 (같은 월드, 각각 새로 띄움):
        #
        #            rtabmap CPU  메모리   자세오차 중앙값  90%값   map->odom 최대보정
        #   2.0 (현행)   25.1 %p   552 MB     0.021 m    0.046 m      0.110 m
        #   1.0          15.5 %p   446 MB     0.027 m    0.052 m      0.188 m
        #
        # 2.0 을 유지하는 이유: 통로 통과 여유가 SLAM 자세 오차에서 나옵니다.
        # 폭 0.70 m 통로의 편측 여유 67 mm 인데 90%값이 46 -> 52 mm 로 오르면
        # 그 예산을 직접 깎습니다. CPU 가 정말 모자랄 때만 내리세요.
        'Rtabmap/DetectionRate': ParameterValue(
            LaunchConfiguration('detection_rate'), value_type=str),
        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/ProximityBySpace': 'true',
        # --- 루프 클로저 기하 검증 ---
        # 0=영상만, 1=ICP만, 2=영상+ICP.
        # 영상으로 "여기 와본 것 같다"고 판단한 뒤 라이다 스캔을 ICP 로
        # 맞춰 봐서 기하가 실제로 일치하는지 확인합니다.
        # 실내는 벽 무늬가 비슷한 곳이 많아 영상만으로는 다른 장소를 같은
        # 곳으로 오인하기 쉬운데(perceptual aliasing), 방 모양·기둥 위치·
        # 통로 폭은 다르므로 여기서 걸러집니다.
        # 360도 라이다를 달아 두고 이 검증에 안 쓰는 것은 낭비입니다.
        'Reg/Strategy': ParameterValue(
            LaunchConfiguration('reg_strategy'), value_type=str),
        # 2D 라이다 스캔용 ICP 설정
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/PointToPlane': 'false',
        'Icp/Iterations': '10',
        'Icp/Epsilon': '0.001',
        'Icp/CorrespondenceRatio': '0.3',

        # --- Nav2 용 2D 점유격자 (/map) ---
        # 0=레이저스캔, 1=뎁스영상, 2=둘 다. 라이다와 카메라를 모두 씁니다.
        # (1 로 두면 카메라 화각 밖은 영원히 미탐색으로 남습니다)
        'Grid/Sensor': '2',
        # 3D 점유격자를 유지합니다. Nav2 가 쓰는 2D /map 은 이 3D 격자를
        # xy 평면에 투영해 그대로 만들어지므로 둘 다 나옵니다.
        # 대신 메모리와 계산이 늘어나므로, 3D 시각화가 필요 없으면
        # map_3d:=false 로 끄세요.
        # rtabmap 의 내부 파라미터는 전부 문자열 타입입니다.
        # LaunchConfiguration 을 그대로 넘기면 launch 가 "true" 를 bool 로
        # 추론해 넣어 노드가 InvalidParameterTypeException 으로 죽습니다.
        'Grid/3D': ParameterValue(LaunchConfiguration('map_3d'), value_type=str),
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '5.0',
        'Grid/RayTracing': 'true',          # 빈 공간을 free 로 표시
        'Grid/NormalsSegmentation': 'false',  # 아래 높이 기준으로 바닥/장애물 분리
        'Grid/MaxGroundHeight': '0.08',
        'Grid/MaxObstacleHeight': '1.0',
        'GridGlobal/MinSize': '10.0',
    }

    # 지도 작성 모드: 매번 새 DB 로 시작
    rtabmap_mapping = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters, {'Mem/IncrementalMemory': 'true'}],
        remappings=RTABMAP_REMAPPINGS,
        arguments=['--delete_db_on_start'],
        condition=UnlessCondition(localization),
    )

    # 위치추정 모드: 기존 DB 를 읽기 전용으로 사용
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters, {
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
        }],
        remappings=RTABMAP_REMAPPINGS,
        condition=IfCondition(localization),
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': BASE_FRAME,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
        }],
        remappings=RTABMAP_REMAPPINGS,
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
    )

    # 뎁스 -> 2D LaserScan 변환 노드는 제거했습니다 (2026-08-02).
    # RPLIDAR 가 들어오기 전, 카메라 뎁스를 한 줄로 눌러 코스트맵 관측원으로
    # 쓰던 것입니다. 지금은 역할이 둘로 나뉘어 있습니다:
    #   라이다 /scan       -> obstacle_layer (2D, 360도 광선 소거)
    #   카메라 포인트클라우드 -> stvl_layer (3D 복셀, 시간 감쇠)
    # 눌러 만든 한 줄 스캔은 높이 정보를 버리므로 STVL 보다 나쁘고,
    # 실제로 그 토픽(/scan_camera)을 구독하는 곳이 하나도 없었습니다.

    return LaunchDescription(args + [
        rgbd_odometry,
        vodom_tf_relay,
        rtabmap_mapping,
        rtabmap_localization,
        rtabmap_viz,
    ])
