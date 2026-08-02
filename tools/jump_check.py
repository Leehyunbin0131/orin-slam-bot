"""로봇이 RViz 에서 튀는 원인을 가른다.

  1) odom -> base_footprint 를 두 노드가 발행하고 있는가 (TF 충돌)
  2) map -> odom 이 얼마나 자주, 얼마나 크게 점프하는가 (루프 클로저 보정)
  3) odom -> base_footprint 자체가 튀는가 (EKF/오도메트리 문제)

map->odom 이 튄다 = SLAM 이 뒤늦게 위치를 고치는 것 (오도메트리가 나쁠수록 큼)
odom->base 가 튄다 = 오도메트리 자체가 망가진 것
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


class J(Node):
    def __init__(self):
        super().__init__('jump_check')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        # 어떤 노드가 어떤 TF 를 발행하는지 센다
        self.pubs = {}
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)

    def on_tf(self, msg):
        for t in msg.transforms:
            key = '%s -> %s' % (t.header.frame_id, t.child_frame_id)
            self.pubs[key] = self.pubs.get(key, 0) + 1

    def xyq(self, parent, child):
        t = self.buf.lookup_transform(parent, child, Time())
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))


rclpy.init()
n = J()
t = time.time()
while time.time() - t < 4:
    rclpy.spin_once(n, timeout_sec=0.1)

print('=== /tf 에 실린 변환들 (4초간 메시지 수) ===')
for k, v in sorted(n.pubs.items(), key=lambda x: -x[1]):
    print('   %-34s %5d 회  (%.0f Hz)' % (k, v, v / 4.0))

print('\n=== %.0f 초간 점프 관측 ===' % DUR)
prev_mo = prev_ob = None
jumps_mo, jumps_ob = [], []
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        mo = n.xyq('map', 'odom')
        ob = n.xyq('odom', 'base_footprint')
    except Exception:
        continue
    if prev_mo is not None:
        d = math.hypot(mo[0] - prev_mo[0], mo[1] - prev_mo[1])
        da = abs(math.atan2(math.sin(mo[2] - prev_mo[2]), math.cos(mo[2] - prev_mo[2])))
        if d > 0.02 or da > 0.02:
            jumps_mo.append((d, math.degrees(da)))
        d2 = math.hypot(ob[0] - prev_ob[0], ob[1] - prev_ob[1])
        if d2 > 0.05:      # 정지 상태에서 5cm 이상 = 비정상
            jumps_ob.append(d2)
    prev_mo, prev_ob = mo, ob

print('map -> odom  (SLAM 보정): 점프 %d 회' % len(jumps_mo))
if jumps_mo:
    print('   최대 %.3f m / %.2f 도,  합계 이동 %.3f m'
          % (max(j[0] for j in jumps_mo), max(j[1] for j in jumps_mo),
             sum(j[0] for j in jumps_mo)))
print('odom -> base (오도메트리): 비정상 점프 %d 회%s'
      % (len(jumps_ob),
         ('  최대 %.3f m' % max(jumps_ob)) if jumps_ob else '  (정상)'))
rclpy.shutdown()
sys.exit(0)
