#!/usr/bin/env python3
"""Gazebo 시뮬레이션 `/clock` 토픽 발행 주기를 조정하는 다운샘플링 노드.

    /clock_raw (브리지) --> [ ClockThrottle ] --> /clock (기본 100 Hz)

rclpy 노드의 시계 콜백 부하를 관리하기 위해 물리 스텝을 유지하면서 ROS `/clock` 메시지 발행 주기를 감소시킵니다.
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
