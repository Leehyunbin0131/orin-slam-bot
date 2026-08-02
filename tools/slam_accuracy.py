"""SLAM 자세 정확도를 Gazebo 실제 자세와 비교.

  1) 제자리 360도 회전 후 각도/위치 오차
  2) 왕복 주행 후 원점 복귀 오차

시각 오도메트리를 끄면(use_vslam:=false) 여기가 나빠지는지 보는 것이 목적.
기준(시각 오도메트리 사용 시): 360도 회전 오차 3mm / 0.03도.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

LABEL = sys.argv[1] if len(sys.argv) > 1 else ''


def gt_pose():
    """Gazebo 의 실제 로봇 자세 (x, y, yaw)."""
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/room/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=20).stdout
    i = out.find('name: "orinbot"')
    blk = out[i:i + 700]

    def num(field, after):
        j = blk.find(field, after)
        k = blk.find('\n', j)
        return float(blk[j + len(field):k]), k

    pi = blk.find('position')
    x, p = num('x: ', pi)
    y, p = num('y: ', p)
    oi = blk.find('orientation')
    qx, p = num('x: ', oi)
    qy, p = num('y: ', p)
    qz, p = num('z: ', p)
    qw, p = num('w: ', p)
    return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


class A(Node):
    def __init__(self):
        super().__init__('slam_accuracy')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

    def slam_pose(self):
        t = self.buf.lookup_transform('map', 'base_footprint', Time())
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))

    def cmd(self, vx, wz, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_footprint'
            m.twist.linear.x, m.twist.angular.z = vx, wz
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)


def ang_err(a, b):
    return abs(math.degrees(math.atan2(math.sin(a - b), math.cos(a - b))))


rclpy.init()
n = A()
t = time.time()
while time.time() - t < 5:
    rclpy.spin_once(n, timeout_sec=0.1)

print('===== SLAM 정확도 %s =====' % LABEL)

s0, g0 = n.slam_pose(), gt_pose()
print('시작  SLAM (%.3f, %.3f, %.2f도) | 실제 (%.3f, %.3f, %.2f도)'
      % (s0[0], s0[1], math.degrees(s0[2]), g0[0], g0[1], math.degrees(g0[2])))

# 1) 제자리 360도 회전 (0.5 rad/s 로 4pi/0.5... 2바퀴는 과하니 1바퀴)
print('\n[1] 제자리 360도 회전')
n.cmd(0.0, 0.5, 2 * math.pi / 0.5)
time.sleep(1.0)
for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.1)
s1, g1 = n.slam_pose(), gt_pose()
print('    SLAM (%.3f, %.3f, %.2f도) | 실제 (%.3f, %.3f, %.2f도)'
      % (s1[0], s1[1], math.degrees(s1[2]), g1[0], g1[1], math.degrees(g1[2])))
print('    >>> 위치 오차 %.3f m,  각도 오차 %.2f 도'
      % (math.hypot(s1[0] - g1[0], s1[1] - g1[1]), ang_err(s1[2], g1[2])))

# 2) 1.5m 전진 후 후진 복귀
print('\n[2] 1.5 m 전진 -> 제자리 180도 -> 1.5 m 전진 (원점 근처 복귀)')
n.cmd(0.25, 0.0, 6.0)
n.cmd(0.0, 0.5, math.pi / 0.5)
n.cmd(0.25, 0.0, 6.0)
time.sleep(1.0)
for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.1)
s2, g2 = n.slam_pose(), gt_pose()
print('    SLAM (%.3f, %.3f, %.2f도) | 실제 (%.3f, %.3f, %.2f도)'
      % (s2[0], s2[1], math.degrees(s2[2]), g2[0], g2[1], math.degrees(g2[2])))
print('    >>> 위치 오차 %.3f m,  각도 오차 %.2f 도'
      % (math.hypot(s2[0] - g2[0], s2[1] - g2[1]), ang_err(s2[2], g2[2])))
rclpy.shutdown()
sys.exit(0)
