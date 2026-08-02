#!/usr/bin/env python3
"""odom 피드백으로 정사각형 경로를 도는 예제 노드 (rclpy).

퍼블리시: /cmd_vel  (geometry_msgs/TwistStamped)
서브스크라이브: /odom (nav_msgs/Odometry)

실행:
    ros2 run mybot_examples_py square_driver --ros-args -p use_sim_time:=true

파라미터:
    side_length   정사각형 한 변 길이 [m]        (기본 1.5)
    linear_speed  직진 속도 [m/s]                (기본 0.3)
    angular_speed 회전 속도 [rad/s]              (기본 0.6)
    position_tol  직진 종료 판정 오차 [m]        (기본 0.03)
    yaw_tol       회전 종료 판정 오차 [rad]      (기본 0.02)
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_from_quaternion(q) -> float:
    """쿼터니언에서 yaw(z축 회전)만 추출."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    """각도를 [-pi, pi) 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


class SquareDriver(Node):

    DRIVE = 'drive'
    TURN = 'turn'

    def __init__(self):
        super().__init__('square_driver')

        self.declare_parameter('side_length', 1.5)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('position_tol', 0.03)
        self.declare_parameter('yaw_tol', 0.02)

        self.side_length = self.get_parameter('side_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.position_tol = self.get_parameter('position_tol').value
        self.yaw_tol = self.get_parameter('yaw_tol').value

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # odom 은 센서성 토픽이 아니므로 reliable 이 기본이지만,
        # 컨트롤러 설정에 따라 best effort 인 경우도 있어 명시해 둔다.
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Odometry, '/odom', self.on_odom, odom_qos)

        self.state = self.DRIVE
        self.leg = 0            # 완료한 변의 개수
        self.pose = None        # (x, y, yaw)
        self.segment_start = None

        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz
        self.get_logger().info('square_driver 시작 — /odom 수신 대기 중')

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def publish_cmd(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    def on_timer(self):
        if self.pose is None:
            return

        if self.segment_start is None:
            self.segment_start = self.pose
            self.get_logger().info(f'{self.leg + 1}번째 변 주행 시작')

        x, y, yaw = self.pose
        sx, sy, syaw = self.segment_start

        if self.state == self.DRIVE:
            traveled = math.hypot(x - sx, y - sy)
            remaining = self.side_length - traveled
            if remaining <= self.position_tol:
                self.publish_cmd(0.0, 0.0)
                self.state = self.TURN
                self.segment_start = self.pose
                self.get_logger().info(f'직진 완료 ({traveled:.3f} m) — 90도 회전')
                return
            # 목표에 가까워지면 감속 (오버슈트 방지)
            speed = min(self.linear_speed, max(0.05, remaining * 1.5))
            self.publish_cmd(speed, 0.0)

        elif self.state == self.TURN:
            turned = abs(normalize_angle(yaw - syaw))
            remaining = (math.pi / 2.0) - turned
            if remaining <= self.yaw_tol:
                self.publish_cmd(0.0, 0.0)
                self.state = self.DRIVE
                self.segment_start = self.pose
                self.leg += 1
                self.get_logger().info(
                    f'회전 완료 ({math.degrees(turned):.1f} deg) — 총 {self.leg}변 완료')
                return
            speed = min(self.angular_speed, max(0.1, remaining * 1.5))
            self.publish_cmd(0.0, speed)


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 정지 명령
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
