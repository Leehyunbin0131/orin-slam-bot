#!/usr/bin/env python3
"""사무실 월드(office.sdf)의 보행자를 복도를 따라 왕복시킨다.

    ros2 run orinbot_bringup people_sim.py

SDF 안에서 원을 그리게 두면 반지름(1.5~1.8 m)이 복도 폭(1.8 m)보다 커서 벽을
뚫고 다니고, 무엇보다 의미 없는 움직임이라 시험이 되지 않습니다. 복도를 따라
왕복시키면 로봇과 **정면으로 마주치므로** 코스트맵에 갑자기 장애물이 생기고
경로가 막히는 상황이 실제로 만들어집니다.

자세는 **추측항법**으로 들고 있습니다 — `velocity-control` 이 사실상 기구학
제어라 명령 속도를 적분하면 실제와 거의 같아서, 자세 피드백을 따로 브리지하지
않습니다.

경로는 `ROUTES` 에 있고 **generate_office.py 의 보행자 시작 위치와 맞아야
합니다.** 어긋나면 첫 구간에서 벽으로 걸어갑니다.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

ALIGN = math.radians(15)   # 이 각도 안에 들면 전진
REACH = 0.35           # 경유점 도달 판정 [m]
RATE = 20.0

# ---- 로봇 회피 ----
# 사람이 로봇을 무시하고 밀고 지나가면 **시험으로서 무의미합니다** — 로봇이
# 무엇을 하든 밀려 버려 회피 성능을 관찰할 수 없습니다.
YIELD_DIST = 1.4       # 이 안에 로봇이 들어오면 멈춤 [m]
YIELD_FOV = math.radians(75)   # 진행 방향 기준 이 각도 안일 때만 (뒤는 무시)
YIELD_CLEAR = 1.8      # 이만큼 멀어지면 다시 걷습니다 (히스테리시스)
YIELD_GIVEUP = 12.0    # 이만큼 기다려도 안 비키면 돌아섭니다 [s]

# (이름, 시작자세(x, y, yaw), 왕복 경유점, 속도[m/s], 회전[rad/s], 끝점 정지[s])
#
# **셋이 서로 다르게 움직여야 합니다.** 같은 속도로 다니면 마주치는 상황이
# 하나로 고정되어, 정작 보고 싶은 것(갑자기 나타남 / 앞을 오래 막음)이
# 안 만들어집니다. 실제 보행은 0.6~1.4 m/s, 로봇 최대는 0.40 m/s 입니다.
#
# person_2 의 y=-1.6 은 로비를 가로지르되 도크 최종 접근(x=1.0, y -2.9~-3.6)
# 을 침범하지 않는 자리입니다 — 도킹까지 망가뜨리면 원인 분리가 어렵습니다.
ROUTES = [
    # 빠른 걸음. 로봇의 3배라 뒤에서 따라잡고 앞을 스쳐 갑니다.
    ('person_0', (-8.0, 0.0, 0.0), [(8.0, 0.0), (-8.0, 0.0)], 1.20, 1.6, 0.0),
    # 느린 걸음 + 끝에서 오래 섬. 로봇 앞을 오래 막는 역할.
    ('person_1', (8.0, 6.0, math.pi), [(-8.0, 6.0), (8.0, 6.0)], 0.55, 0.9, 6.0),
    ('person_2', (-8.0, -1.6, 0.0), [(8.0, -1.6), (-8.0, -1.6)], 0.85, 1.2, 2.0),
]


class Walker:

    def __init__(self, node, name, start, waypoints, speed, turn, pause):
        self.name = name
        self.x, self.y, self.yaw = start
        self.wps = waypoints
        self.speed, self.turn, self.pause = speed, turn, pause
        self.i = 0
        self.rest = 0.0          # 남은 정지 시간 [s]
        self.yielding = False    # 로봇에 길을 내주는 중
        self.waited = 0.0        # 그 상태로 기다린 시간 [s]
        self.pub = node.create_publisher(Twist, '/%s/cmd_vel' % name, 10)

    def _robot_in_way(self, robot):
        """진행 방향 앞쪽 가까이에 로봇이 있는가. (거리, 막힘 여부)"""
        if robot is None:
            return None, False
        dx, dy = robot[0] - self.x, robot[1] - self.y
        d = math.hypot(dx, dy)
        b = math.atan2(math.sin(math.atan2(dy, dx) - self.yaw),
                       math.cos(math.atan2(dy, dx) - self.yaw))
        # 한 번 멈추면 조금 더 멀어질 때까지 유지합니다 (경계에서 떨림 방지).
        limit = YIELD_CLEAR if self.yielding else YIELD_DIST
        return d, (d < limit and abs(b) < YIELD_FOV)

    def step(self, dt, robot=None):
        t = Twist()

        # 로봇이 앞을 막고 있으면 멈춰 섭니다.
        d, blocked = self._robot_in_way(robot)
        if blocked:
            self.yielding = True
            self.waited += dt
            if self.waited > YIELD_GIVEUP:
                # 로봇도 사람을 피해 서 있을 수 있으므로 누군가는 양보를
                # 끝내야 합니다. 안 그러면 둘 다 서서 교착됩니다.
                self.wps = list(reversed(self.wps))
                self.i = 0
                self.waited = 0.0
                self.yielding = False
            self.pub.publish(t)
            return
        self.yielding = False
        self.waited = 0.0

        if self.rest > 0.0:
            # 끝점에서 잠시 섭니다 — 서 있는 사람은 코스트맵에 계속 남습니다.
            self.rest -= dt
            self.pub.publish(t)
            return

        tx, ty = self.wps[self.i]
        dx, dy = tx - self.x, ty - self.y
        if math.hypot(dx, dy) < REACH:
            self.i = (self.i + 1) % len(self.wps)
            self.rest = self.pause
            tx, ty = self.wps[self.i]
            dx, dy = tx - self.x, ty - self.y

        err = math.atan2(math.sin(math.atan2(dy, dx) - self.yaw),
                         math.cos(math.atan2(dy, dx) - self.yaw))
        if abs(err) > ALIGN:
            t.angular.z = self.turn if err > 0 else -self.turn
        else:
            t.linear.x = self.speed
            t.angular.z = max(-self.turn, min(self.turn, 2.0 * err))
        self.pub.publish(t)
        # 추측항법 적분 (velocity-control 은 명령 속도를 그대로 씁니다)
        self.yaw += t.angular.z * dt
        self.x += t.linear.x * math.cos(self.yaw) * dt
        self.y += t.linear.x * math.sin(self.yaw) * dt


class People(Node):

    def __init__(self):
        super().__init__('people_sim')
        self.walkers = [Walker(self, *r) for r in ROUTES]
        # 시뮬레이션 전용 노드라 SLAM 추정값 대신 지상 진실을 씁니다.
        self.robot = None
        self.create_subscription(
            Odometry, '/ground_truth/odom',
            lambda m: setattr(self, 'robot',
                              (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_timer(1.0 / RATE, self._tick)
        for wk in self.walkers:
            self.get_logger().info(
                '%s: %.2f m/s, 회전 %.1f rad/s, 끝점 정지 %.0f 초'
                % (wk.name, wk.speed, wk.turn, wk.pause))

    def _tick(self):
        for wk in self.walkers:
            wk.step(1.0 / RATE, self.robot)


def main():
    rclpy.init()
    n = People()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        # 멈춰 세우고 나갑니다. 안 그러면 마지막 속도로 계속 걸어갑니다.
        stop = Twist()
        for wk in n.walkers:
            wk.pub.publish(stop)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
