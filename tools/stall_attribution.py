#!/usr/bin/env python3
"""로봇이 **한 구역을 벗어나지 못한 시간**을 원인별로 배분한다.

    python3 tools/stall_attribution.py [측정초] [반경m] [시간s]

왜 이 정의인가
--------------
"속도 0" 으로 판정하면 제자리에서 꼼지락거리는 구간을 놓칩니다. 답답한 것은
**일정 반경을 오래 벗어나지 못하는 것**이므로, 최근 `시간` 동안의 위치가 전부
`반경` 안이면 갇힘으로 봅니다 (기본 5초 / 0.25 m — 외접 반경 0.283 보다 작음).

"멈춰서 오래 생각한다"는 관찰만으로는 고칠 수 없습니다. 원인이 여럿이고
고치는 곳이 각각 다릅니다:

    목표가 없어서               -> 탐사 노드 (블랙리스트 대기 등)
    경로가 없어서               -> planner
    컨트롤러가 0 을 내서        -> MPPI 비평자 / 좁은 공간
    컨트롤러가 침묵해서         -> 제어 루프 굶주림
    복구 동작 중이라서          -> BT (BackUp/Spin/Wait)
    명령은 나가는데 안 움직여서 -> 물리적 끼임 또는 twist_mux

갇힘 구간마다 그 순간의 액션 상태 / BT 노드 / cmd_vel 각 단계를 보고
원인을 붙인 뒤, 구역 좌표와 함께 보고합니다.

전제: 시뮬레이터(`/ground_truth/odom`)와 Nav2 가 실행 중이어야 합니다.
"""

import math
import sys
import time
from collections import defaultdict, deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

TL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

CMD_EPS_V, CMD_EPS_W = 0.005, 0.01
SILENT = 0.5            # 이 시간 이상 새 명령이 없으면 "침묵"
STALE_PLAN = 2.0

RECOVERY_NODES = ('BackUp', 'Spin', 'Wait', 'ClearEntireCostmap',
                  'ClearLocalCostmap', 'ClearGlobalCostmap', 'RecoveryActions',
                  'ClearingActions')

CAUSES = [
    ('NO_GOAL', '목표 없음 (탐사 노드가 목표를 안 보냄)'),
    ('BT_RECOVERY', '복구 동작 중 (BackUp/Spin/Wait/코스트맵 지우기)'),
    ('NO_PLAN', '경로 없음 (planner 실패 또는 경로가 낡음)'),
    ('CTRL_SILENT', '컨트롤러 침묵 (명령이 아예 안 나옴)'),
    ('CTRL_ZERO', '컨트롤러가 0 을 냄 (유효 궤적을 못 찾음)'),
    ('CMD_BLOCKED', '명령이 중간에 막힘 (smoother/twist_mux 단계)'),
    ('CREEPING', '명령은 나가는데 구역을 못 벗어남 (헛돌기/끼임)'),
]


class Attrib(Node):

    def __init__(self, radius, window):
        super().__init__('stall_attribution')
        self.R, self.W = radius, window
        self.xy = None
        self.hist = deque()                 # (t, x, y)
        self.goal_active = False
        self.bt_running = ''
        self.t_plan = 0.0
        self.plan_len = 0
        self.t_nav = self.t_out = 0.0
        self.nav = self.out = (0.0, 0.0)

        self.create_subscription(Odometry, '/ground_truth/odom', self._odom, 10)
        self.create_subscription(GoalStatusArray,
                                 '/navigate_to_pose/_action/status', self._st, 10)
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self._bt, 10)
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_nav', self._cnav, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cout, 10)
        # "몇 분 기다리면 결국 통과한다" 를 설명하려면 주변 코스트맵이
        # 시간이 지나며 열리는지 봐야 합니다 (STVL voxel_decay 10초,
        # BT 의 ClearEntireCostmap, SLAM 보정 등).
        self.lc = None
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: setattr(self, 'lc', m), TL)

        self.tally = defaultdict(float)     # 전체 원인별 초
        self.episodes = []                  # dict(center, dur, causes)
        self.cur = None
        self.free_time = 0.0
        self.last = time.time()
        self.create_timer(0.1, self._tick)

    def _odom(self, m):
        p = m.pose.pose.position
        self.xy = (p.x, p.y)

    def _st(self, m):
        self.goal_active = any(s.status == GoalStatus.STATUS_EXECUTING
                               for s in m.status_list)

    def _bt(self, m):
        for e in m.event_log:
            if e.current_status == 'RUNNING':
                self.bt_running = e.node_name

    def _plan(self, m):
        self.t_plan, self.plan_len = time.time(), len(m.poses)

    def _cnav(self, m):
        self.t_nav = time.time()
        self.nav = (m.twist.linear.x, m.twist.angular.z)

    def _cout(self, m):
        self.t_out = time.time()
        self.out = (m.twist.linear.x, m.twist.angular.z)

    def _classify(self, now):
        if not self.goal_active:
            return 'NO_GOAL'
        if any(k in self.bt_running for k in RECOVERY_NODES):
            return 'BT_RECOVERY'
        if now - self.t_plan > STALE_PLAN or self.plan_len == 0:
            return 'NO_PLAN'
        if now - self.t_nav > SILENT:
            return 'CTRL_SILENT'
        if abs(self.nav[0]) < CMD_EPS_V and abs(self.nav[1]) < CMD_EPS_W:
            return 'CTRL_ZERO'
        if (now - self.t_out > SILENT
                or (abs(self.out[0]) < CMD_EPS_V and abs(self.out[1]) < CMD_EPS_W)):
            return 'CMD_BLOCKED'
        return 'CREEPING'

    def _free_frac(self):
        """반경 1 m 안에서 로봇 중심이 놓일 수 있는 셀 비율.

        `/local_costmap/costmap` 은 0~100 척도입니다 (99=내접, 100=치명).
        이 값이 갇힘 구간 동안 올라가면 "기다리니 길이 열린" 것입니다.
        """
        if self.lc is None or self.xy is None:
            return float('nan')
        i = self.lc.info
        a = np.array(self.lc.data, dtype=np.int16).reshape(i.height, i.width)
        r = int((self.xy[1] - i.origin.position.y) / i.resolution)
        c = int((self.xy[0] - i.origin.position.x) / i.resolution)
        n = int(1.0 / i.resolution)
        sub = a[max(0, r - n):min(a.shape[0], r + n),
                max(0, c - n):min(a.shape[1], c + n)]
        if sub.size == 0:
            return float('nan')
        return float(np.sum((sub >= 0) & (sub < 99)) / sub.size)

    def _confined(self, now):
        """최근 W 초의 위치가 전부 반경 R 안이면 갇힘. (중심, True/False)"""
        while self.hist and now - self.hist[0][0] > self.W:
            self.hist.popleft()
        if not self.hist or now - self.hist[0][0] < self.W * 0.9:
            return None, False          # 창이 아직 안 참
        cx = sum(h[1] for h in self.hist) / len(self.hist)
        cy = sum(h[2] for h in self.hist) / len(self.hist)
        far = max(math.dist((cx, cy), (h[1], h[2])) for h in self.hist)
        return (cx, cy), far < self.R

    def _tick(self):
        now = time.time()
        dt, self.last = now - self.last, now
        if self.xy is None:
            return
        self.hist.append((now, self.xy[0], self.xy[1]))
        center, stuck = self._confined(now)
        if not stuck:
            self.free_time += dt
            if self.cur:
                self.episodes.append(self.cur)
                self.cur = None
            return
        cause = self._classify(now)
        self.tally[cause] += dt
        ff = self._free_frac()
        if self.cur is None:
            self.cur = {'center': center, 'dur': 0.0,
                        'causes': defaultdict(float), 'nodes': set(),
                        'ff0': ff, 'ff1': ff}
        self.cur['center'] = center
        self.cur['dur'] += dt
        self.cur['causes'][cause] += dt
        self.cur['ff1'] = ff
        if self.bt_running:
            self.cur['nodes'].add(self.bt_running)

    def report(self):
        if self.cur:
            self.episodes.append(self.cur)
        stuck_t = sum(self.tally.values())
        total = stuck_t + self.free_time
        print()
        print('=' * 72)
        print('갇힘 판정: 최근 %.0f 초의 위치가 모두 반경 %.2f m 안' % (self.W, self.R))
        print('총 %.0f 초 중  정상 이동 %.0f 초 (%.0f%%) / 갇힘 %.0f 초 (%.0f%%)'
              % (total, self.free_time, 100 * self.free_time / max(total, 1e-9),
                 stuck_t, 100 * stuck_t / max(total, 1e-9)))
        print('=' * 72)
        if stuck_t <= 0:
            print('갇힌 구간이 없습니다.')
            return
        print('%-13s %8s %7s   %s' % ('원인', '초', '갇힘중%', '설명'))
        for key, desc in CAUSES:
            s = self.tally.get(key, 0.0)
            if s > 0:
                print('%-13s %8.1f %6.0f%%   %s'
                      % (key, s, 100 * s / stuck_t, desc))
        print()
        print('가장 오래 갇힌 구역 10곳')
        print('  자유비율 = 반경 1 m 안에서 로봇 중심이 놓일 수 있는 셀 비율.')
        print('  갇힘 중에 이 값이 올라갔다면 "기다리니 길이 열린" 것입니다.')
        print('%8s  %-18s %-13s %-13s %s'
              % ('길이', '구역 중심', '주원인', '자유비율 처음->끝', 'BT 노드'))
        for e in sorted(self.episodes, key=lambda x: -x['dur'])[:10]:
            if e['dur'] < 2.0:
                continue
            top = max(e['causes'].items(), key=lambda kv: kv[1])
            nodes = ','.join(sorted(n for n in e['nodes'] if n))[:28]
            print('%6.1f 초  (%6.2f, %6.2f)   %-13s %.2f -> %.2f     %s'
                  % (e['dur'], e['center'][0], e['center'][1], top[0],
                     e['ff0'], e['ff1'], nodes or '-'))


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    radius = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
    window = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    rclpy.init()
    n = Attrib(radius, window)
    t0 = time.time()
    try:
        while time.time() - t0 < dur:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.report()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
