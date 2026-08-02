"""폭이 다른 좁은 복도를 차례로 통과시켜 한계 폭을 찾는다.

x=3.2 에 깊이 0.60 m 의 벽이 있고, 폭 0.90 / 0.70 / 0.55 m 개구부가
각각 y=+2.6 / +0.6 / -1.6 에 뚫려 있다. 로봇 폭은 0.40 m.

각 복도마다
  - 통과했는가 (x > 3.6 에 도달)
  - 몇 초 걸렸는가
  - 각속도 부호가 몇 번 바뀌었는가 (좌우 헤맴)
를 재고, 매번 시작점으로 돌아온다.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# (라벨, 개구부 중심 y, 폭)
_ALL = [('0.90m', 2.6, 0.90), ('0.70m', 0.6, 0.70),
        ('0.60m', -3.0, 0.60), ('0.55m', -1.6, 0.55)]
CORRIDORS = [c for c in _ALL if len(sys.argv) < 2 or c[0] in sys.argv[1:]]
HOME = (0.0, 0.0)
TIMEOUT = 120.0


class C(Node):
    def __init__(self):
        super().__init__('corridor_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.signs = []
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 10)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

    def on_cmd(self, m):
        wz = m.twist.angular.z
        if abs(wz) > 0.05:
            self.signs.append(1 if wz > 0 else -1)

    def pose(self):
        t = self.buf.lookup_transform('map', 'base_footprint', Time())
        return t.transform.translation.x, t.transform.translation.y

    def goto(self, gx, gy, timeout=TIMEOUT):
        self.signs = []
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = float(gx), float(gy)
        g.pose.pose.orientation.w = 1.0
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, 0.0, 0
        res = gh.get_result_async()
        t0 = time.time()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        st = res.result().status if res.done() else 0
        flips = sum(1 for a, b in zip(self.signs, self.signs[1:]) if a != b)
        return st, time.time() - t0, flips


rclpy.init()
n = C()
n.ac.wait_for_server(timeout_sec=60.0)
t = time.time()
while time.time() - t < 4:
    rclpy.spin_once(n, timeout_sec=0.1)

print('로봇 폭 0.40 m. 복도 깊이 0.60 m.')
print('%-8s %-7s %-9s %-7s %-6s %s' % ('복도폭', '편측여유', '결과', '소요', '반전', '최종위치'))
for label, gy, wdt in CORRIDORS:
    st, dt, fl = n.goto(4.3, gy)
    x, y = n.pose()
    ok = x > 3.6
    print('%-8s %-7s %-9s %5.0f초 %5d회  (%.2f, %.2f)'
          % (label, '%.3f m' % ((wdt - 0.40) / 2),
             '통과' if ok else ('실패(status=%s)' % st), dt, fl, x, y))
    st2, dt2, fl2 = n.goto(*HOME)
    x2, y2 = n.pose()
    if math.hypot(x2 - HOME[0], y2 - HOME[1]) > 0.5:
        print('         (복귀 실패 — 다음 시험이 부정확할 수 있음: %.2f, %.2f)' % (x2, y2))
rclpy.shutdown()
sys.exit(0)
