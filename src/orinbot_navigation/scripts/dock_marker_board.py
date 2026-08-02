#!/usr/bin/env python3
"""ArUco 마커 보드 기반 도크 자세 추정 노드.

    /camera/color/image_raw + camera_info  -->  /detected_dock_pose

복수의 ArUco 마커(3장) 코너 정보를 보드 단위(estimatePoseBoard)로 정합하여 자세 추정 정확도를 향상시킵니다.
근접 거리(lock_distance 이내) 진입 시 고정 프레임(fixed_frame) 상의 자세를 확정하고 시각만 갱신하여 근거리 검출 오차를 방지합니다.
"""

import math

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (PoseStamped 변환 등록)
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def rvec_to_quat(rvec):
    """회전 벡터 -> (x, y, z, w)."""
    R, _ = cv2.Rodrigues(rvec)
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


class DockMarkerBoard(Node):

    def __init__(self):
        super().__init__('dock_marker_board')
        p = self.declare_parameter

        # ArUco 딕셔너리 설정 (0 = DICT_4X4_50)
        p('dictionary', 0)
        # 마커 ID 및 도크 중심 기준 X축 offset 배치 [m]
        p('marker_ids', [1, 0, 2])
        p('marker_dx', [-0.16, 0.0, 0.16])
        # 마커 단일 변 크기 [m]
        p('marker_size', 0.10)
        # 최소 검출 마커 수
        p('min_markers', 2)

        # 근거리 자세 확정 거리 [m] (카메라 기준)
        p('lock_distance', 0.40)
        # 자세 확정 해제 거리 [m]
        p('unlock_distance', 0.60)
        p('fixed_frame', 'odom')
        # 자세 확정 후 재발행 주기 [Hz]
        p('lock_publish_rate', 15.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        ids = [int(v) for v in g('marker_ids')]
        dxs = [float(v) for v in g('marker_dx')]
        self.msize = g('marker_size')
        self.min_markers = int(g('min_markers'))
        self.lock_d = g('lock_distance')
        self.unlock_d = g('unlock_distance')
        self.fixed_frame = g('fixed_frame')

        self.dict = cv2.aruco.Dictionary_get(int(g('dictionary')))
        self.params = cv2.aruco.DetectorParameters_create()
        # 서브픽셀 코너 정밀화 설정
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.board = self._make_board(ids, dxs)
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.locked = None            # 고정된 PoseStamped (fixed_frame)

        # 충전 중 절전용 서비스 핸들러
        self.paused = False
        self.create_service(Empty, '~/pause', self._srv_pause)
        self.create_service(Empty, '~/resume', self._srv_resume)

        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.create_subscription(CameraInfo, 'camera_info', self._info, 10)
        self.create_subscription(Image, 'image', self._image, 10)
        self.create_timer(1.0 / g('lock_publish_rate'), self._republish)

        self.get_logger().info(
            '도크 마커 보드 검출 시작: id %s, 간격 %s, 한 변 %.3f m / '
            '%.2f m 안에서 자세 고정'
            % (ids, dxs, self.msize, self.lock_d))

    def _make_board(self, ids, dxs):
        """마커 3장의 3D 배치를 만든다.

        좌표 규약을 OpenCV 의 estimatePoseSingleMarkers 와 똑같이 맞춥니다
        (마커 평면이 z=0, x 오른쪽, y 위, z 는 마커에서 카메라 쪽).
        그래야 docking.yaml 의 external_detection_rotation_* 을 한 장짜리
        때 구한 값 그대로 쓸 수 있습니다.

        주의 — 좌우 부호: 카메라 광학 프레임의 +x(영상에서 오른쪽)는
        이 배치에서 월드 -x 입니다(로봇이 도크를 마주 보므로).
        따라서 월드에서 오른쪽(+dx)에 있는 마커가 보드 좌표에서는
        -dx 에 옵니다. 부호를 반대로 두면 좌우가 뒤집힌 보드가 되어
        자세가 엉뚱하게 나옵니다.
        """
        h = self.msize / 2.0
        obj = []
        for dx in dxs:
            ox = -dx
            obj.append(np.array([
                [ox - h,  h, 0.0],
                [ox + h,  h, 0.0],
                [ox + h, -h, 0.0],
                [ox - h, -h, 0.0]], dtype=np.float32))
        return cv2.aruco.Board_create(
            obj, self.dict, np.array(ids, dtype=np.int32).reshape(-1, 1))

    def _info(self, m):
        if self.K is None:
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(m.d, dtype=np.float64).reshape(1, -1)

    def _srv_pause(self, _req, res):
        if not self.paused:
            self.paused = True
            self.get_logger().info('마커 검출 일시정지')
        return res

    def _srv_resume(self, _req, res):
        if self.paused:
            self.paused = False
            self.locked = None
            self.get_logger().info('마커 검출 재개')
        return res

    def _image(self, msg):
        if self.paused or self.K is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        corners, ids, _ = cv2.aruco.detectMarkers(
            img, self.dict, parameters=self.params)
        if ids is None or len(ids) < self.min_markers:
            return
        ok, rvec, tvec = cv2.aruco.estimatePoseBoard(
            corners, ids, self.board, self.K, self.D, None, None)
        if not ok:
            return
        dist = float(tvec[2])

        if self.locked is not None:
            # 물러났으면 고정 해제
            if dist > self.unlock_d:
                self.get_logger().info('%.2f m 로 물러남 — 자세 고정 해제' % dist)
                self.locked = None
            else:
                return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = dist
        q = rvec_to_quat(rvec)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = q

        if dist <= self.lock_d:
            # 고정 프레임으로 옮겨 둡니다. 카메라 프레임 그대로 얼리면
            # 목표가 로봇을 따라다녀 영원히 도달하지 못합니다.
            #
            # **영상 타임스탬프로 조회하면 안 됩니다.** 영상이 odom TF 보다
            # 앞서 도착해 "Lookup would require extrapolation into the
            # future" 로 매번 실패합니다. timeout 을 줘도 소용없습니다 —
            # 이 콜백이 실행기를 잡고 있어 TF 리스너 콜백이 그 사이에
            # 돌지 못하기 때문입니다(단일 스레드 실행기의 전형적인 교착).
            # 실제로 이것 때문에 고정이 한 번도 걸리지 않았고, 예전처럼
            # 마지막 프레임으로 마무리해 세로 오차가 +55 mm 까지 났습니다.
            #
            # 그래서 "가장 최근 TF"(stamp=0)로 조회합니다. 영상 시각과
            # 수십 ms 어긋나지만 접근 속도가 0.1 m/s 라 수 mm 수준이고,
            # 접촉 허용치 ±48 mm 에 비하면 무시할 수 있습니다.
            latest = PoseStamped()
            latest.header.frame_id = pose.header.frame_id
            latest.pose = pose.pose
            try:
                self.locked = self.buf.transform(latest, self.fixed_frame)
                self.get_logger().info(
                    '%.2f m 에서 자세 확정 (마커 %d장) — 이후 직진'
                    % (dist, len(ids)))
            except Exception as e:                       # noqa: BLE001
                self.get_logger().warning('고정 변환 실패: %s' % e)
                self.pub.publish(pose)
            return

        self.pub.publish(pose)

    def _republish(self):
        if self.paused:
            return

        """고정된 자세를 시각만 갱신하며 계속 내보낸다.

        내보내기를 멈추면 SimpleChargingDock 이
        external_detection_timeout 후 FAILED_TO_DETECT_DOCK 을 냅니다.
        """
        if self.locked is None:
            return
        self.locked.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.locked)


def main():
    rclpy.init()
    n = DockMarkerBoard()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
