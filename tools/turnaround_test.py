"""통로 안에서 제자리로 돌아설 수 있는가를 순수 기하로 잰다.

  python3 turnaround_test.py            # 0.90 / 0.70 / 0.60 / 0.55 전부
  python3 turnaround_test.py 0.70m 0.60m

왜 필요한가
-----------
지금까지의 통로 시험은 "직진해서 빠져나가는가"만 봤다. 하지만 실제 운용에서
로봇은 막다른 곳에서 되돌아 나와야 하고, 차동구동은 후진보다 제자리 회전을
먼저 시도한다 (Nav2 의 spin 복구 행동도 마찬가지다).

  로봇 대각선 = sqrt(2) * 0.40 = 0.5657 m  <- 제자리 회전에 필요한 최소 폭
  폭 0.60 m 통로의 편측 여유 = (0.60 - 0.5657)/2 = 17 mm
  폭 0.70 m 통로의 편측 여유 = 67 mm

17 mm 는 우리가 실측한 SLAM 자세 오차(중앙값 24 mm)보다도 작다.

시험 방법
---------
Nav2 를 거치지 않는다. Gazebo 로 로봇을 통로 한가운데에 직접 놓고
/cmd_vel_teleop 으로 제자리 회전만 시킨 뒤, Gazebo 실제 자세로

  - 360도를 다 돌았는가
  - 도는 동안 중심이 얼마나 밀렸는가 (벽에 닿으면 밀리거나 멈춘다)

를 잰다. sim.launch.py 만 떠 있으면 되고 SLAM/Nav2 는 필요 없다.
(오히려 SLAM 이 떠 있으면 순간이동 때문에 지도가 망가지므로 띄우지 말 것)
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

WORLD = 'room'
WALL_X = 3.2            # 통로 뱅크 벽의 중심 x
# (라벨, 개구부 중심 y, 폭)
_ALL = [('0.90m', 2.6, 0.90), ('0.70m', 0.6, 0.70),
        ('0.60m', -3.0, 0.60), ('0.55m', -1.6, 0.55)]
CASES = [c for c in _ALL if len(sys.argv) < 2 or c[0] in sys.argv[1:]]

WZ = 0.5                # 회전 각속도 [rad/s]
SPIN_T = 2 * math.pi / WZ * 1.35     # 360도 + 여유


def set_pose(x, y, yaw):
    req = ('name: "orinbot", position: {z: 0.06}, '
           'orientation: {x: 0, y: 0, z: %.6f, w: %.6f}' % (
               math.sin(yaw / 2), math.cos(yaw / 2)))
    req = req.replace('position: {z: 0.06}',
                      'position: {x: %.4f, y: %.4f, z: 0.06}' % (x, y))
    subprocess.run(
        ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req', req],
        capture_output=True, text=True, timeout=15)


def ground_truth():
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/%s/dynamic_pose/info' % WORLD, '-n', '1'],
        capture_output=True, text=True, timeout=20).stdout
    i = out.find('name: "orinbot"')
    if i < 0:
        return None
    blk = out[i:i + 800]

    def num(f, a):
        j = blk.find(f, a)
        k = blk.find('\n', j)
        return float(blk[j + len(f):k]), k

    p = blk.find('position')
    x, p = num('x: ', p)
    y, p = num('y: ', p)
    o = blk.find('orientation')
    qx, p = num('x: ', o)
    qy, p = num('y: ', p)
    qz, p = num('z: ', p)
    qw, p = num('w: ', p)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
    return x, y, yaw


class T(Node):
    def __init__(self):
        super().__init__('turnaround_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        # sim.launch.py 만 띄우고 시험하므로 twist_mux 가 없습니다.
        # diff_drive_controller 가 구독하는 /cmd_vel 로 직접 보냅니다.
        # (Nav2 까지 떠 있다면 /cmd_vel_teleop 으로 바꿔야 섞이지 않습니다)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def spin_cmd(self, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.angular.z = wz
        self.pub.publish(m)


rclpy.init()
n = T()
# 통로 중앙에서 옆으로 얼마나 벗어난 채 회전을 시도하는가 [m].
# 실기에서 로봇이 통로 정중앙에 있을 리 없습니다. 우리가 실측한 SLAM 자세
# 오차는 중앙값 24 mm / 90%값 56 mm 이므로, 이만큼 벗어나도 도는지가
# "실제로 쓸 수 있는 폭"의 기준입니다.
OFFSETS = [0.06, 0.10, 0.14, 0.20]


def try_spin(x, y):
    """통로 바깥에 놓고 몰고 들어간 뒤 제자리 회전. (회전각[도], 최대 밀림[m])

    순간이동으로 통로 안에 바로 놓으면 안 됩니다. Gazebo 는 정적 물체와의
    초기 관통을 밀어내지 않아서, 벽에 80 mm 박힌 채로 가만히 있다가 그대로
    돌아 버립니다 (실측). 그러면 "폭이 모자라도 통과"라는 거짓 결과가 나옵니다.
    반드시 빈 곳에서 출발해 접촉을 물리적으로 만들어야 합니다.
    """
    set_pose(x - 0.9, y, 0.0)          # 통로 바깥 (벽은 x 2.9~3.5)
    time.sleep(2.5)
    # 통로 안으로 전진
    t0 = time.time()
    while time.time() - t0 < 9.0:
        m = TwistStamped()
        m.header.stamp = n.get_clock().now().to_msg()
        m.twist.linear.x = 0.15
        n.pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.02)
        time.sleep(0.05)
        g = ground_truth()
        if g and g[0] >= x:
            break
    n.spin_cmd(0.0)
    time.sleep(1.0)
    start = ground_truth()
    if start is None:
        return None
    if start[0] < x - 0.25:
        return -1.0, 0.0               # 진입 자체를 못 함
    t0 = time.time()
    maxd, total_yaw, prev_yaw = 0.0, 0.0, start[2]
    while time.time() - t0 < SPIN_T:
        n.spin_cmd(WZ)
        rclpy.spin_once(n, timeout_sec=0.02)
        time.sleep(0.05)
        g = ground_truth()
        if g:
            maxd = max(maxd, math.hypot(g[0] - start[0], g[1] - start[1]))
            total_yaw += math.atan2(math.sin(g[2] - prev_yaw), math.cos(g[2] - prev_yaw))
            prev_yaw = g[2]
    n.spin_cmd(0.0)
    rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(0.8)
    return abs(math.degrees(total_yaw)), maxd


print('=== 통로 안 제자리 회전 시험 (로봇 대각선 0.566 m) ===')
print('SLAM 자세 오차 실측: 중앙값 24 mm, 90%값 56 mm\n')
header = '%-7s %9s' % ('통로', '편측여유')
for o in OFFSETS:
    header += ' %9s' % ('%.0fmm 편심' % (o * 1000))
print(header + '   견딜 수 있는 편심')

for label, gy, wid in CASES:
    margin = (wid - 0.4 * math.sqrt(2)) / 2 * 1000
    row = '%-7s %6.0f mm' % (label, margin)
    worst_ok = -1.0
    for off in OFFSETS:
        r = try_spin(WALL_X, gy + off)
        if r is None:
            row += ' %9s' % '?'
            continue
        deg, maxd = r
        if deg < 0:
            row += ' %9s' % '진입실패'
            continue
        ok = deg > 350 and maxd < 0.08
        if ok:
            worst_ok = off
        row += ' %6.0f도%s' % (deg, 'o' if ok else 'x')
    print(row + '   %s' % ('%.0f mm' % (worst_ok * 1000) if worst_ok >= 0 else '없음'))

n.spin_cmd(0.0)
rclpy.shutdown()
sys.exit(0)
