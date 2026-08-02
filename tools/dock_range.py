#!/usr/bin/env python3
"""도크 마커의 검출 거리·정확도·사각지대를 실측한다.

    python3 tools/dock_range.py            # 시뮬레이터만 떠 있으면 됩니다

로봇을 도크 정면 축을 따라 여러 거리에 순간이동시키면서, 그 자리에서
`/detected_dock_pose` 를 받아 **기하학적 참값과 비교**합니다. 알고 싶은 것:

  1. 몇 m 까지 검출되는가          -> staging_x_offset 을 정할 근거
  2. 가까이서 언제 화각 밖으로 나가는가 -> external_detection_timeout 의 하한
  3. 거리별 편차는 얼마인가        -> external_detection_translation_x 보정

왜 편차가 생기는가: 424x240 에서 마커는 1 m 거리에 몇십 픽셀뿐이라,
검은 사각형 경계가 배경과 섞이면서 코너가 안쪽으로 치우쳐 잡힙니다.
그러면 마커가 실제보다 작아 보이고 = 실제보다 멀다고 나옵니다.
거리가 가까워질수록 픽셀이 커져 편차가 줄어듭니다.

현재 구성(마커 3장 보드)에서는 `dock_marker_board` 가 0.65 m 안에서
자세를 고정하므로, 그보다 가까운 행은 고정된 값이 그대로 나옵니다.
검출 한계를 재려면 노드의 `lock_distance` 를 0 으로 두고 돌리세요.

주의: 도크 안쪽(로봇 중심 y > 도킹 자세)으로는 보내지 않습니다. Gazebo 는
정적 물체와의 초기 관통을 밀어내지 않아, 도크에 박힌 상태로 측정하게
됩니다.
"""

import math
import subprocess
import sys
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

# generate_room.py 와 맞춘 값
DOCK_X = 1.0
MARKER_Y = -3.898 + 0.008 / 2.0     # 마커판 앞면 = -3.894
MARKER_Z = 0.31
DOCKED_Y = -3.60                    # 도킹 완료 시 로봇 중심
ROBOT_YAW = -1.5708                 # 도크를 마주 본 방향

# 로봇 중심에서 마커면까지의 거리 [m]
OFFSETS = [2.4, 2.2, 2.0, 1.6, 1.3, 1.0, 0.8, 0.6, 0.50, 0.46, 0.44,
           0.42, 0.40, 0.38, 0.36, 0.34, 0.32, 0.30, 0.28,
           MARKER_Y * -1 + DOCKED_Y]  # 마지막 = 도킹 자세 (0.264)


def teleport(y):
    # z 는 0.15. 0 에 가깝게 두면 바퀴가 지면에 파묻힌 상태로 놓여
    # 물리가 로봇을 튕겨내면서 자세가 흐트러집니다.
    subprocess.run(
        ['gz', 'service', '-s', '/world/room/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000',
         '--req', 'name: "orinbot", position: {x: %.4f, y: %.4f, z: 0.15}, '
                  'orientation: {x: 0, y: 0, z: -0.7071068, w: 0.7071068}'
                  % (DOCK_X, y)],
        capture_output=True, timeout=15)


def gt_y():
    """Gazebo 가 말하는 실제 로봇 y. 명령한 값을 믿지 않습니다 —
    순간이동 뒤 물리가 로봇을 조금 밀어낼 수 있습니다."""
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/room/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=20).stdout
    i = out.find('name: "orinbot"')
    if i < 0:
        return None
    blk = out[i:i + 700]
    j = blk.find('y: ', blk.find('position'))
    return float(blk[j + 3:blk.find('\n', j)])


class Probe(Node):

    def __init__(self):
        super().__init__('dock_range')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.samples = []
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.create_subscription(PoseStamped, '/detected_dock_pose', self._cb, 10)

    def _cb(self, msg):
        try:
            out = self.buf.transform(
                msg, 'base_footprint',
                timeout=rclpy.duration.Duration(seconds=0.3))
        except Exception:
            return
        p, o = out.pose.position, out.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        # 마커 좌표계 -> 도크 좌표계 회전 (pitch +90도, dock_calib.py 로 구함)
        # 여기서는 yaw 만 필요하므로 전체 회전 대신 결과 yaw 를 직접 씁니다.
        self.samples.append((p.x, p.y, p.z, yaw, o))

    def settle(self, seconds):
        """대기하면서 **계속 spin 합니다.**

        그냥 sleep 하면 이전 위치에서 찍힌 메시지가 구독 큐(깊이 10)에
        쌓였다가 다음 collect 때 배달됩니다. 그러면 마커가 화각 밖으로
        나간 자리에서도 "검출됨" 으로 나오고, 값은 이전 위치의 것이라
        탈락 거리를 실제보다 가깝게 잘못 재게 됩니다 (실제로 그랬습니다:
        0.36 m 행이 0.42 m 행과 소수점 4자리까지 같았습니다).
        """
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def collect(self, seconds=1.5):
        self.samples = []
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self.samples)


def dock_yaw_from(o):
    """external_detection_rotation = (0, +pi/2, 0) 을 적용한 뒤의 yaw."""
    # q_ext = pitch +90도
    ex, ey, ez, ew = 0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)
    ax, ay, az, aw = o.x, o.y, o.z, o.w
    x = aw * ex + ax * ew + ay * ez - az * ey
    y = aw * ey - ax * ez + ay * ew + az * ex
    z = aw * ez + ax * ey - ay * ex + az * ew
    w = aw * ew - ax * ex - ay * ey - az * ez
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    rclpy.init()
    n = Probe()
    print('거리[m]  검출률   측정거리   편차[mm]  가로[mm]  도크yaw[도]')
    print('-------  ------  ---------  --------  --------  -----------')
    last_ok = None
    first_blind = None
    rows = []
    for d in OFFSETS:
        y = MARKER_Y + d
        # 도크 안쪽(도킹 자세보다 벽에 가까운 쪽)으로는 보내지 않습니다.
        # y 가 작을수록 벽에 가깝습니다.
        if y < DOCKED_Y - 1e-6:
            continue
        teleport(y)
        n.settle(2.0)                   # 물리 안정화 + 렌더 갱신 (큐도 비웁니다)
        actual = gt_y()
        if actual is not None:
            d = actual - MARKER_Y       # 명령값이 아니라 실제 거리로 평가
        s = n.collect(2.0)
        # 이 자리에서 기대되는 프레임 수 (컬러 15 Hz)
        rate = min(1.0, len(s) / 30.0)
        if not s:
            print('%7.2f  %5.0f%%   (검출 없음)' % (d, 0))
            if last_ok is not None and first_blind is None:
                first_blind = d
            continue
        mx = sum(v[0] for v in s) / len(s)
        my = sum(v[1] for v in s) / len(s)
        yaws = [dock_yaw_from(v[4]) for v in s]
        myaw = math.atan2(sum(math.sin(v) for v in yaws) / len(yaws),
                          sum(math.cos(v) for v in yaws) / len(yaws))
        print('%7.2f  %5.0f%%  %9.4f  %+8.1f  %+8.1f  %+11.2f'
              % (d, rate * 100, mx, (mx - d) * 1000, my * 1000,
                 math.degrees(myaw)))
        rows.append((d, mx, my, myaw))
        last_ok = d

    print()
    if rows:
        near = min(rows, key=lambda r: r[0])
        print('가장 가까운 검출: %.2f m (측정 %.4f, 편차 %+.1f mm, yaw %+.2f도)'
              % (near[0], near[1], (near[1] - near[0]) * 1000,
                 math.degrees(near[3])))
        far = max(rows, key=lambda r: r[0])
        print('가장 먼 검출: %.2f m' % far[0])
        # 플러그인은 dock_pose = 측정된_마커_위치 + R(yaw)*(tx, ty) 로 목표를
        # 만듭니다. 목표가 도킹 자세(마커에서 0.264 m 앞)가 되려면
        #   tx = (참거리 - 0.264) - 측정거리 = -(0.264 + 편차)
        # 이고, 마지막까지 보이는 가까운 거리의 편차를 써야 합니다.
        # 그 시점의 검출값이 최종 정렬을 결정하기 때문입니다.
        target = DOCKED_Y - MARKER_Y
        bias = near[1] - near[0]
        print()
        print('external_detection_translation_x 권장값: %+.4f' % -(target + bias))
        print('  = -(도킹 목표 거리 %.3f + 근거리 편차 %+.3f)' % (target, bias))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
