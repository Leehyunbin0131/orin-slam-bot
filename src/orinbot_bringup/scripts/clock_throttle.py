#!/usr/bin/env python3
"""Gazebo 의 `/clock` 을 솎아서 다시 낸다.

    /clock_raw (브리지, 997 Hz) --> [이 노드] --> /clock (기본 100 Hz)

왜 필요한가 (실측)
------------------
`use_sim_time: true` 인 **rclpy** 노드는 /clock 메시지마다 파이썬 콜백을
돌며 시계를 갱신합니다. 비용이 메시지당 고정이라 발행 주파수에 비례합니다.

  개발 PC : 997 Hz 에서 노드당 24.7%   (use_sim_time=false 는 0~2.3%)
  Orin    : 100 Hz 에서 노드당 11.3%p  (false 는 0.2%)

실기에는 /clock 자체가 없으므로 이 부하는 **순수한 시뮬레이션 인공물**인데,
그것 때문에 시나리오가 못 돌아갑니다. (시뮬에서 잰 rclpy 노드 CPU 를 실기
추정에 쓰면 안 되는 이유이기도 합니다.)

물리 스텝(`max_step_size` 1 ms)은 건드리지 않습니다 — 시뮬레이션 정확도는
그대로 두고 **ROS 로 나가는 시계의 해상도만** 낮춥니다. 100 Hz 면 분해능이
10 ms 인데, 가장 빠른 주기가 컨트롤러 20 Hz(50 ms)이고 TF/센서 동기화는 노드
시계가 아니라 메시지 헤더 스탬프를 쓰므로 영향이 없습니다.

`clock_rate:=0` 이면 솎지 않고 그대로 통과시킵니다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

# ros_gz_bridge 가 /clock 을 내보내는 QoS 와 맞춥니다. 구독자(모든 노드)가
# 기대하는 것과 어긋나면 시계를 아예 못 받습니다.
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ClockThrottle(Node):

    def __init__(self):
        # **이 노드만은 use_sim_time 을 쓰면 안 됩니다.** 시뮬 시각의
        # 출처가 자기 자신이라 스스로를 기다리며 멈춥니다.
        super().__init__('clock_throttle')
        self.declare_parameter('rate', 100.0)
        # use_sim_time 은 rclpy 가 이미 선언해 둡니다 (다시 선언하면 예외).
        rate = float(self.get_parameter('rate').value)

        self.latest = None
        self.pub = self.create_publisher(Clock, 'clock', QOS)
        self.create_subscription(Clock, 'clock_raw', self._in, QOS)

        self.passthrough = rate <= 0.0
        if rate > 0.0:
            # 벽시계 타이머입니다 (use_sim_time=False 이므로).
            self.create_timer(1.0 / rate, self._tick)
            self.get_logger().info('/clock_raw -> /clock 을 %.0f Hz 로 솎습니다' % rate)
        else:
            self.get_logger().info('/clock 솎기 없음 (그대로 통과)')

    def _in(self, msg):
        self.latest = msg
        # rate<=0 이면 타이머가 없으므로 받은 즉시 그대로 내보냅니다.
        if self.passthrough:
            self.pub.publish(msg)

    def _tick(self):
        if self.latest is not None:
            self.pub.publish(self.latest)


def main():
    rclpy.init()
    n = ClockThrottle()
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
