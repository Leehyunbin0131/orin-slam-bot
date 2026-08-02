#!/usr/bin/env python3
"""전역 경로가 재계획될 때마다 뒤집히는지, 컨트롤러가 진동하는지 잰다.

    python3 tools/path_stability.py [측정초]

왜 필요한가
-----------
`stall_attribution.py` 로 "명령은 나가는데 구역을 못 벗어난다(CREEPING)"가
갇힘 시간의 절반이라는 것까지 좁혔습니다. 복구 중도 아니고 주변이 막힌
것도 아닌데 제자리에서 왔다 갔다 합니다. 남은 후보는 둘입니다:

  (a) 전역 경로가 재계획마다 좌우로 뒤집히고 MPPI 가 번갈아 쫓는다
  (b) 경로는 안정적인데 MPPI 자체가 두 국소해 사이에서 진동한다

둘은 측정으로 갈립니다:
  - 경로 초반 방향(로봇 앞 `LOOK` m 지점)의 **변화량**  -> (a)
  - 각속도 명령의 **부호 반전 빈도**                    -> (b)
(a)면 플래너/경로 쪽(스무딩, 팽창, 재계획 주기)을, (b)면 컨트롤러 쪽
(비평자, 회전 쉼)을 봐야 합니다.

전제: 시뮬레이터와 Nav2 가 실행 중이어야 합니다.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

LOOK = 0.5          # 경로 초반 방향을 재는 거리 [m]
FLIP = 0.35         # 이 각속도 이상에서 부호가 바뀌면 "반전" [rad/s]


class PS(Node):

    def __init__(self):
        super().__init__('path_stability')
        self.xy = None
        self.prev_head = None
        self.turns = []          # 연속 경로 사이의 초반 방향 변화 [deg]
        self.plan_t = []
        self.wz = []             # (t, wz)
        self.flips = 0
        self.last_sign = 0
        self.dist = 0.0
        self.last_xy = None
        self.create_subscription(Odometry, '/ground_truth/odom', self._odom, 10)
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cmd, 10)

    def _odom(self, m):
        p = m.pose.pose.position
        if self.last_xy is not None:
            self.dist += math.dist((p.x, p.y), self.last_xy)
        self.last_xy = (p.x, p.y)
        self.xy = (p.x, p.y)

    def _plan(self, m):
        if self.xy is None or len(m.poses) < 2:
            return
        # 로봇에서 경로를 따라 LOOK 만큼 간 지점의 방향
        acc, prev = 0.0, None
        target = None
        for ps in m.poses:
            q = (ps.pose.position.x, ps.pose.position.y)
            if prev is not None:
                acc += math.dist(q, prev)
                if acc >= LOOK:
                    target = q
                    break
            prev = q
        if target is None:
            target = (m.poses[-1].pose.position.x, m.poses[-1].pose.position.y)
        head = math.atan2(target[1] - self.xy[1], target[0] - self.xy[0])
        self.plan_t.append(time.time())
        if self.prev_head is not None:
            d = math.degrees(abs(math.atan2(math.sin(head - self.prev_head),
                                            math.cos(head - self.prev_head))))
            self.turns.append(d)
        self.prev_head = head

    def _cmd(self, m):
        w = m.twist.angular.z
        self.wz.append((time.time(), w))
        s = 1 if w > FLIP else (-1 if w < -FLIP else 0)
        if s != 0:
            if self.last_sign != 0 and s != self.last_sign:
                self.flips += 1
            self.last_sign = s


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    rclpy.init()
    n = PS()
    t0 = time.time()
    try:
        while time.time() - t0 < dur:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    el = time.time() - t0

    print()
    print('=' * 66)
    print('%.0f 초 관측 / 실제 이동 거리 %.1f m (%.3f m/s)' % (el, n.dist, n.dist / el))
    print('=' * 66)

    if n.turns:
        t = sorted(n.turns)
        big = sum(1 for x in n.turns if x > 60)
        rev = sum(1 for x in n.turns if x > 120)
        rate = len(n.plan_t) / el
        print('(a) 전역 경로 초반 방향 (%.2f m 앞) 변화 — 재계획 %d회, %.2f Hz'
              % (LOOK, len(n.plan_t), rate))
        print('    중앙값 %.1f도 / 90%% %.1f도 / 최대 %.1f도'
              % (t[len(t) // 2], t[int(len(t) * 0.9)], t[-1]))
        print('    60도 초과 %d회 (%.0f%%),  120도 초과(사실상 반전) %d회 (%.0f%%)'
              % (big, 100 * big / len(t), rev, 100 * rev / len(t)))
    else:
        print('(a) /plan 을 충분히 못 받았습니다.')

    print()
    if n.wz:
        turning = [w for _, w in n.wz if abs(w) > FLIP]
        print('(b) 각속도 명령 진동 — 표본 %d개' % len(n.wz))
        print('    |wz| > %.2f 인 구간 %d개 (%.0f%%), 부호 반전 %d회 (%.2f 회/초)'
              % (FLIP, len(turning), 100 * len(turning) / len(n.wz),
                 n.flips, n.flips / el))
    print()
    print('해석: 경로 방향 변화가 크고 잦으면 (a) 경로 뒤집힘.')
    print('      경로는 안정적인데 부호 반전만 잦으면 (b) 컨트롤러 진동.')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
