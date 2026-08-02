#!/usr/bin/env python3
"""경로 요구 방향 대비 헤딩 역방향 회전 감지 스크립트.

    python3 tools/wrongway.py [측정초]
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

LOOK = 0.5        # 경로 방향을 재는 앞거리 [m]
THRESH = 20.0     # 이 각도[도] 이상 틀어졌을 때만 판정
WZ_MIN = 0.05     # 이 이상 돌고 있을 때만 판정 [rad/s]


class WW(Node):

    def __init__(self):
        super().__init__('wrongway')
        self.xy = None
        self.yaw = 0.0
        self.path = None
        self.events = []
        self.cur = None
        self.samples = 0
        self.bad = 0
        self.create_subscription(Odometry, '/ground_truth/odom', self._od, 20)
        self.create_subscription(Path, '/plan', lambda m: setattr(self, 'path', m), 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cmd, 20)

    def _od(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.xy = (p.x, p.y)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y ** 2 + q.z ** 2))

    def _look_ahead(self):
        """경로를 따라 LOOK m 앞 지점과, 근처에서 경로가 접히는지."""
        if self.path is None or self.xy is None or len(self.path.poses) < 2:
            return None, False
        pts = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
        d = [math.dist(self.xy, p) for p in pts]
        i0 = min(range(len(d)), key=lambda i: d[i])
        acc = 0.0
        tgt = pts[-1]
        for i in range(i0, len(pts) - 1):
            acc += math.dist(pts[i], pts[i + 1])
            if acc >= LOOK:
                tgt = pts[i + 1]
                break
        # 접힘: 반경 0.6 m 안 경로점의 인덱스가 끊겨 있으면 되돌아온 것
        idx = [i for i, v in enumerate(d) if v < 0.6]
        folded = any(idx[k + 1] - idx[k] > 4 for k in range(len(idx) - 1))
        return tgt, folded

    def _cmd(self, m):
        wz = m.twist.angular.z
        vx = m.twist.linear.x
        if self.xy is None:
            return
        tgt, folded = self._look_ahead()
        if tgt is None:
            return
        want = math.atan2(tgt[1] - self.xy[1], tgt[0] - self.xy[0]) - self.yaw
        want = math.atan2(math.sin(want), math.cos(want))
        deg = math.degrees(want)
        self.samples += 1
        wrong = (abs(deg) > THRESH and abs(wz) > WZ_MIN
                 and (deg > 0) != (wz > 0))
        if wrong:
            self.bad += 1
            if self.cur is None:
                self.cur = {'t': time.time(), 'xy': self.xy, 'deg': deg,
                            'wz': wz, 'vx': vx, 'folded': folded, 'n': 1}
            else:
                self.cur['n'] += 1
                self.cur['deg'] = deg
                self.cur['folded'] = self.cur['folded'] or folded
        elif self.cur is not None:
            self.cur['dur'] = time.time() - self.cur['t']
            self.events.append(self.cur)
            self.cur = None

    def report(self):
        if self.cur is not None:
            self.cur['dur'] = time.time() - self.cur['t']
            self.events.append(self.cur)
        print()
        print('=' * 70)
        print('표본 %d개 중 역방향 회전 %d개 (%.1f%%), 사건 %d건'
              % (self.samples, self.bad,
                 100 * self.bad / max(self.samples, 1), len(self.events)))
        print('=' * 70)
        ev = [e for e in self.events if e.get('dur', 0) > 0.3]
        if not ev:
            print('0.3초 이상 이어진 역방향 회전이 없습니다.')
            return
        print('%7s  %-18s %9s %8s %8s  %s'
              % ('길이', '위치', '요구각도', '명령wz', '전진vx', '경로접힘'))
        for e in sorted(ev, key=lambda x: -x['dur'])[:15]:
            print('%6.1f초  (%6.2f, %6.2f)  %7.0f도 %8.2f %8.2f  %s'
                  % (e['dur'], e['xy'][0], e['xy'][1], e['deg'], e['wz'],
                     e['vx'], '예' if e['folded'] else '아니오'))
        tot = sum(e['dur'] for e in ev)
        fold = sum(e['dur'] for e in ev if e['folded'])
        print()
        print('역방향 회전 총 %.1f 초 중 경로가 접힌 상태였던 시간 %.1f 초 (%.0f%%)'
              % (tot, fold, 100 * fold / max(tot, 1e-9)))


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    rclpy.init()
    n = WW()
    t0 = time.time()
    try:
        while time.time() - t0 < dur:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    n.report()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
