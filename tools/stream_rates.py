"""카메라 스트림들의 실제 주기와 타임스탬프 일치도를 잰다.

"Gazebo 는 렌더링 스트림 2개까지" 라는 것이 정말인지 확인하기 위한 것.
요청 주기 대비 실제로 몇 Hz 가 나오는지, 그리고 infra1 과 depth 의
타임스탬프가 같은 틱에 찍히는지를 본다.
"""
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
TOPICS = {
    'infra1': '/camera/infra1/image_rect_raw',
    'depth': '/camera/depth/image_rect_raw',
    'color': '/camera/color/image_raw',
}


class R(Node):
    def __init__(self):
        super().__init__('stream_rates')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.stamps = defaultdict(list)
        for name, topic in TOPICS.items():
            self.create_subscription(
                Image, topic,
                lambda m, n=name: self.stamps[n].append(
                    m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                qos_profile_sensor_data)


rclpy.init()
n = R()
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)

print('=== %.0f 초간 스트림별 실제 주기 ===' % DUR)
for name in TOPICS:
    s = n.stamps[name]
    if len(s) < 2:
        print('  %-8s 수신 %d 개 (없음/비활성)' % (name, len(s)))
        continue
    span = s[-1] - s[0]
    print('  %-8s 수신 %4d 개, 시뮬시간 %.1f 초 -> %.1f Hz'
          % (name, len(s), span, (len(s) - 1) / span if span > 0 else 0))

a, b = n.stamps['infra1'], n.stamps['depth']
if a and b:
    matched = sum(1 for x in a if any(abs(x - y) < 1e-4 for y in b))
    print('\ninfra1 - depth 타임스탬프 완전 일치: %d / %d (%.0f%%)'
          % (matched, len(a), 100.0 * matched / len(a)))
    worst = 0.0
    for x in a:
        if b:
            worst = max(worst, min(abs(x - y) for y in b))
    print('최악 시차: %.4f 초' % worst)
rclpy.shutdown()
sys.exit(0)
