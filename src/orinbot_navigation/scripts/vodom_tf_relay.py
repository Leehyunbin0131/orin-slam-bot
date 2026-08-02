#!/usr/bin/env python3
"""Visual Odometry 보정 변환(vodom -> odom)을 일정한 주기 및 최신 타임스탬프 기반으로 재발행하는 노드.

Visual Odometry 연산 지연 및 타임스탬프 비동기화로 인한 Nav2 TF 조회 실패를 방지하기 위해
T(vodom->odom) = T(vodom->base) * T(odom->base)^-1 관계식을 연산하여 주기적으로 재발행합니다.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def q_mult(a, b):
    """쿼터니언 곱 (x, y, z, w 순서)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def q_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_rotate(q, v):
    """벡터 v 를 쿼터니언 q 로 회전."""
    qv = np.array([v[0], v[1], v[2], 0.0])
    return q_mult(q_mult(q, qv), q_conj(q))[:3]


class VodomTfRelay(Node):

    def __init__(self):
        super().__init__('vodom_tf_relay')

        self.declare_parameter('visual_odom_topic', '/vodom')
        self.declare_parameter('visual_odom_frame', 'vodom')
        self.declare_parameter('wheel_odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)
        # 타임스탬프 예측 오프셋 [초] (TF 조회 시 미래 외삽 오류 방지)
        self.declare_parameter('tf_tolerance', 0.2)

        self.visual_frame = self.get_parameter('visual_odom_frame').value
        self.wheel_frame = self.get_parameter('wheel_odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_tolerance = float(self.get_parameter('tf_tolerance').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        # 초기 항등 변환 (TF 체인 유지)
        self.correction_t = np.zeros(3)
        self.correction_q = np.array([0.0, 0.0, 0.0, 1.0])
        self.have_correction = False

        self.create_subscription(
            Odometry, self.get_parameter('visual_odom_topic').value,
            self.on_visual_odom, 10)
        self.create_timer(1.0 / rate, self.publish)

        self.get_logger().info(
            '%s -> %s 보정 TF 를 %.0f Hz, %+.2fs 오프셋으로 재발행합니다.'
            % (self.visual_frame, self.wheel_frame, rate, self.tf_tolerance))

    # ------------------------------------------------------------------
    def on_visual_odom(self, msg: Odometry):
        """T(vodom->odom) = T(vodom->base) * T(odom->base)^-1 을 갱신."""
        try:
            wheel = self.buffer.lookup_transform(
                self.wheel_frame, self.base_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.1))
        except Exception as exc:  # TF 아직 없음 / 시각 불일치
            self.get_logger().warn(
                '휠 오도메트리 TF 조회 실패: %s' % exc, throttle_duration_sec=5.0)
            return

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t1 = np.array([p.x, p.y, p.z])
        q1 = np.array([o.x, o.y, o.z, o.w])

        wt = wheel.transform.translation
        wr = wheel.transform.rotation
        t2 = np.array([wt.x, wt.y, wt.z])
        q2 = np.array([wr.x, wr.y, wr.z, wr.w])

        q = q_mult(q1, q_conj(q2))
        t = t1 - q_rotate(q, t2)

        self.correction_q = q / np.linalg.norm(q)
        self.correction_t = t
        self.have_correction = True

    # ------------------------------------------------------------------
    def publish(self):
        stamp = self.get_clock().now() + Duration(seconds=self.tf_tolerance)

        msg = TransformStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.visual_frame
        msg.child_frame_id = self.wheel_frame
        msg.transform.translation.x = float(self.correction_t[0])
        msg.transform.translation.y = float(self.correction_t[1])
        msg.transform.translation.z = float(self.correction_t[2])
        msg.transform.rotation.x = float(self.correction_q[0])
        msg.transform.rotation.y = float(self.correction_q[1])
        msg.transform.rotation.z = float(self.correction_q[2])
        msg.transform.rotation.w = float(self.correction_q[3])
        self.broadcaster.sendTransform(msg)


def main():
    rclpy.init()
    node = VodomTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
