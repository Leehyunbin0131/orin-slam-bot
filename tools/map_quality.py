"""주행 중 SLAM 추정 자세와 Gazebo 참값 간 오차 연속 측정 스크립트.

    python3 map_quality.py <라벨>
"""
import math
import subprocess
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

LABEL = sys.argv[1] if len(sys.argv) > 1 else ''
# 서로 다른 두 통로(c070 0.70 m, c090 0.90 m)로 동쪽을 왕복합니다.
# 다른 길로 같은 곳에 돌아오므로 루프 클로저가 걸릴 기회가 생깁니다.
#
# 예전 첫 경유지는 c060 (폭 0.60 m) 이었는데, 최소 통과 가능 폭 0.70 m
# 기준 밖이라 뺐습니다. 기준 밖 통로에서 로봇이 끼여 벽에 갈리며 돌면
# 바퀴가 미끄러져 오도메트리가 깨지는데, 그러면 이 스크립트가 재는 것이
# "SLAM 정확도" 가 아니라 "그날 얼마나 끼였나" 가 됩니다.
WAYPOINTS = [(4.3, 0.6), (0.0, 0.0), (4.3, 2.6), (0.0, 0.0)]


def gt():
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/room/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=20).stdout
    i = out.find('name: "orinbot"')
    blk = out[i:i + 700]

    def num(f, a):
        j = blk.find(f, a)
        k = blk.find('\n', j)
        return float(blk[j + len(f):k]), k

    x, p = num('x: ', blk.find('position'))
    y, p = num('y: ', p)
    return x, y


class Q(Node):
    def __init__(self):
        super().__init__('map_quality')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

    def xy(self, parent, child):
        t = self.buf.lookup_transform(parent, child, Time())
        return t.transform.translation.x, t.transform.translation.y

    def goto(self, x, y, timeout=140.0, sampler=None):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.w = 1.0
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None
        res = gh.get_result_async()
        t0 = last = time.time()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if sampler and time.time() - last > 1.0:
                last = time.time()
                sampler()
        return res.result().status if res.done() else 0


rclpy.init()
n = Q()
n.ac.wait_for_server(timeout_sec=60.0)
t = time.time()
while time.time() - t < 4:
    rclpy.spin_once(n, timeout_sec=0.1)

errs = []
corr = []
prev_mo = [None]


def sample():
    try:
        sx, sy = n.xy('map', 'base_footprint')
        mo = n.xy('map', 'odom')
    except Exception:
        return
    gx, gy = gt()
    errs.append(math.hypot(sx - gx, sy - gy))
    if prev_mo[0] is not None:
        corr.append(math.hypot(mo[0] - prev_mo[0][0], mo[1] - prev_mo[0][1]))
    prev_mo[0] = mo


print('===== 지도 품질 %s =====' % LABEL)
sts = []
for wx, wy in WAYPOINTS:
    sts.append(n.goto(wx, wy, sampler=sample))

print('경유지 결과 %s (4=성공), 표본 %d개' % (sts, len(errs)))
if errs:
    errs.sort()
    print('SLAM 자세 오차 (실제 대비):')
    print('   중앙값 %.3f m | 90%%값 %.3f m | 최대 %.3f m'
          % (errs[len(errs) // 2], errs[int(len(errs) * 0.9)], errs[-1]))
if corr:
    big = [c for c in corr if c > 0.05]
    print('map->odom 보정 (지도가 흔들리는 양):')
    print('   1초당 평균 %.4f m | 최대 %.3f m | 5cm 넘는 보정 %d 회 / %d'
          % (sum(corr) / len(corr), max(corr), len(big), len(corr)))
rclpy.shutdown()
sys.exit(0)
