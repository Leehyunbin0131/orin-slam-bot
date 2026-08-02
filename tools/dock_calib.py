#!/usr/bin/env python3
"""도크 마커 비전 자세 추정 회전 보정 파라미터(external_detection_rotation_*) 산출 스크립트.

    python3 tools/dock_calib.py
"""

import argparse
import itertools
import math
import sys
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  (PoseStamped 용 do_transform 등록)
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

HALF = math.pi / 2.0


def quat_mul(a, b):
    """(x, y, z, w) 순서. a 다음 b 를 적용 = a * b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_to_yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Calib(Node):

    def __init__(self, frame):
        super().__init__('dock_calib')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.frame = frame
        self.samples = []
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.create_subscription(PoseStamped, '/detected_dock_pose', self._cb, 10)

    def _cb(self, msg):
        try:
            out = self.buf.transform(msg, self.frame, timeout=rclpy.duration.Duration(
                seconds=0.3))
        except Exception:
            return
        p, o = out.pose.position, out.pose.orientation
        self.samples.append(((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default='base_footprint',
                    help='이 프레임 기준으로 풉니다')
    ap.add_argument('--expect', type=float, default=0.0,
                    help='이 프레임에서 본 도크의 yaw [rad]. 정면이면 0')
    ap.add_argument('--samples', type=int, default=30)
    a = ap.parse_args()

    rclpy.init()
    n = Calib(a.frame)
    t0 = time.time()
    while len(n.samples) < a.samples and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.samples:
        print('/detected_dock_pose 를 %s 로 변환하지 못했습니다. '
              '검출기와 TF 가 살아 있는지 확인하세요.' % a.frame)
        rclpy.shutdown()
        return 1

    pos = [s[0] for s in n.samples]
    quats = [s[1] for s in n.samples]
    print('표본 %d개, 기준 프레임 %s' % (len(n.samples), a.frame))
    print('마커 위치 (평균): x=%.4f y=%.4f z=%.4f'
          % tuple(sum(c[i] for c in pos) / len(pos) for i in range(3)))

    # 축 정렬 회전 24가지를 전부 넣어 보고, 결과 yaw 가 기대값에
    # 맞는 것을 고릅니다. 사람이 부호를 헤아리는 것보다 확실합니다.
    print('\n기대 도크 yaw = %+.4f rad. 맞는 회전:' % a.expect)
    hits = []
    for r, p, y in itertools.product(range(-1, 3), repeat=3):
        ext = rpy_to_quat(r * HALF, p * HALF, y * HALF)
        yaws = [quat_to_yaw(quat_mul(q, ext)) for q in quats]
        # 원형 평균
        my = math.atan2(sum(math.sin(v) for v in yaws) / len(yaws),
                        sum(math.cos(v) for v in yaws) / len(yaws))
        spread = max(abs(math.atan2(math.sin(v - my), math.cos(v - my))) for v in yaws)
        err = abs(math.atan2(math.sin(my - a.expect), math.cos(my - a.expect)))
        if err < math.radians(8):
            hits.append((err, spread, r, p, y, my))
    if not hits:
        print('  없음 — 로봇이 도크 정면에 있는지, --expect 가 맞는지 확인하세요.')
    for err, spread, r, p, y, my in sorted(hits):
        print('  roll=%+.4f pitch=%+.4f yaw=%+.4f  ->  도크 yaw %+.4f '
              '(오차 %.2f도, 표본 흔들림 %.2f도)'
              % (r * HALF, p * HALF, y * HALF, my,
                 math.degrees(err), math.degrees(spread)))

    print('\ndocking.yaml 에 넣을 값 (오차가 가장 작은 것):')
    if hits:
        _e, _s, r, p, y, _m = sorted(hits)[0]
        print('      external_detection_rotation_roll: %.4f' % (r * HALF))
        print('      external_detection_rotation_pitch: %.4f' % (p * HALF))
        print('      external_detection_rotation_yaw: %.4f' % (y * HALF))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
