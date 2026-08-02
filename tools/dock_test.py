#!/usr/bin/env python3
"""도킹 반복 수행 성공률 및 정렬 오차 검증 스크립트.

    python3 tools/dock_test.py [반복횟수]

도킹 성공률, 최종 물리 정렬 오차(Gazebo 참값 기준), 무검출 구간 시간 및 단계별 소요 시간을 측정합니다.
"""

import math
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from rclpy.action import ActionClient
from rclpy.node import Node

DOCK_ID = 'home_dock'
DOCK_POSE = (1.0, -3.60, -1.5708)      # docking.yaml 의 home_dock.pose
# 도킹을 걸기 전 로봇을 세워 둘 자리. staging pose(도크에서 0.7 m) 보다
# 더 멀리 두어, staging 이동까지 포함해 시험합니다.
START_POSE = (1.0, -2.0, -1.5708)
# 지도를 먼저 만들어야 Nav2 가 도크 앞까지 경로를 냅니다.
WARMUP = [(1.0, -1.2), (1.0, -2.0)]

# 판정 허용치 [m], [rad] — 위 설명의 유도 결과
TOL_LON = 0.048
TOL_LAT = 0.034
TOL_YAW = math.radians(5.0)


def gt_pose():
    """Gazebo 가 말하는 실제 로봇 자세 (x, y, yaw)."""
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/room/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=20).stdout
    i = out.find('name: "orinbot"')
    if i < 0:
        return None
    blk = out[i:i + 900]

    def num(field, start):
        j = blk.find(field, start)
        return float(blk[j + len(field):blk.find('\n', j)]), j + 1

    pi = blk.find('position')
    x, p = num('x: ', pi)
    y, p = num('y: ', p)
    oi = blk.find('orientation')
    qx, p = num('x: ', oi)
    qy, p = num('y: ', p)
    qz, p = num('z: ', p)
    qw, p = num('w: ', p)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return x, y, yaw


class Tester(Node):

    def __init__(self):
        super().__init__('dock_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.dock = ActionClient(self, DockRobot, 'dock_robot')
        self.undock = ActionClient(self, UndockRobot, 'undock_robot')
        self.last_detect = None
        self.states = []
        self.create_subscription(PoseStamped, '/detected_dock_pose',
                                 self._detect, 10)

    def _detect(self, _msg):
        self.last_detect = time.time()

    # --- 액션 공통 ---
    def _run(self, client, goal, timeout, feedback=None):
        if not client.wait_for_server(timeout_sec=30.0):
            return None, '서버 없음'
        fut = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, '거절됨'
        res = gh.get_result_async()
        t0 = time.time()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res.done():
            gh.cancel_goal_async()
            return None, '시간 초과'
        return res.result(), None

    def goto(self, x, y, yaw=None, timeout=180.0):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        if yaw is None:
            g.pose.pose.orientation.w = 1.0
        else:
            g.pose.pose.orientation.z = math.sin(yaw / 2)
            g.pose.pose.orientation.w = math.cos(yaw / 2)
        r, err = self._run(self.nav, g, timeout)
        return err is None and r is not None and r.status == GoalStatus.STATUS_SUCCEEDED

    def _dock_fb(self, fb):
        s = fb.feedback.state
        if not self.states or self.states[-1][0] != s:
            self.states.append((s, time.time()))

    def do_dock(self, timeout=240.0):
        self.states = []
        self.last_detect = None
        g = DockRobot.Goal()
        g.use_dock_id = True
        g.dock_id = DOCK_ID
        g.navigate_to_staging_pose = True
        t0 = time.time()
        r, err = self._run(self.dock, g, timeout, feedback=self._dock_fb)
        dt = time.time() - t0
        blind = (time.time() - self.last_detect) if self.last_detect else None
        return r, err, dt, blind

    def do_undock(self, timeout=90.0):
        r, err = self._run(self.undock, UndockRobot.Goal(), timeout)
        return r, err


STATE_NAME = {0: '-', 1: 'staging이동', 2: '초기인식', 3: '접근', 4: '충전대기', 5: '재시도'}


def main():
    n_iter = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rclpy.init()
    t = Tester()

    print('지도 준비 — 도크 앞을 먼저 돌아 봅니다')
    for wx, wy in WARMUP:
        ok = t.goto(wx, wy)
        print('  (%.1f, %.1f) %s' % (wx, wy, '도착' if ok else '실패'))

    rows = []
    for i in range(n_iter):
        print('\n===== %d/%d 회 =====' % (i + 1, n_iter))
        if i > 0:
            print('  시작 위치로 이동...')
            t.goto(*START_POSE)

        r, err, dt, blind = t.do_dock()
        if err or r is None:
            print('  도킹 실패: %s' % err)
            rows.append((False, None, None, None, dt, blind, err))
            continue
        ok = r.result.success
        code = r.result.error_code
        seq = ' -> '.join('%s' % STATE_NAME.get(s, s) for s, _ in t.states)
        print('  결과: %s (error_code=%d, 재시도 %d회, %.1f초)'
              % ('성공' if ok else '실패', code, r.result.num_retries, dt))
        print('  단계: %s' % seq)
        if blind is not None:
            print('  무검출 구간: %.2f 초 '
                  '(external_detection_timeout 이 이보다 커야 합니다)' % blind)

        g = gt_pose()
        if g and ok:
            # 도크 좌표계로 옮겨 세로/가로 성분을 분리합니다.
            dx, dy = g[0] - DOCK_POSE[0], g[1] - DOCK_POSE[1]
            c, s = math.cos(DOCK_POSE[2]), math.sin(DOCK_POSE[2])
            lon = dx * c + dy * s          # 도크 진행 방향 (+ 는 덜 들어감)
            lat = -dx * s + dy * c         # 좌우
            dyaw = math.atan2(math.sin(g[2] - DOCK_POSE[2]),
                              math.cos(g[2] - DOCK_POSE[2]))
            print('  실제 자세 오차 — 세로 %+.1f mm / 가로 %+.1f mm / 각도 %+.2f도'
                  % (lon * 1000, lat * 1000, math.degrees(dyaw)))
            verdict = []
            verdict.append('세로 OK' if abs(lon) <= TOL_LON else '세로 초과')
            verdict.append('가로 OK' if abs(lat) <= TOL_LAT else '가로 초과(단락 위험)')
            verdict.append('각도 OK' if abs(dyaw) <= TOL_YAW else '각도 초과')
            print('  판정: %s' % ', '.join(verdict))
            rows.append((ok, lon, lat, dyaw, dt, blind, None))
        else:
            rows.append((ok, None, None, None, dt, blind, None))

        print('  언도킹...')
        ur, uerr = t.do_undock()
        print('  언도킹 %s' % ('성공' if (ur and ur.result.success) else
                             ('실패: %s' % uerr)))

    print('\n===== 종합 =====')
    good = [r for r in rows if r[0]]
    print('성공 %d / %d' % (len(good), len(rows)))
    if good:
        def stat(idx, scale, unit):
            v = [abs(r[idx]) * scale for r in good if r[idx] is not None]
            if not v:
                return '-'
            return '중앙값 %.1f%s 최대 %.1f%s' % (sorted(v)[len(v) // 2], unit,
                                              max(v), unit)
        print('세로 오차: %s' % stat(1, 1000, 'mm'))
        print('가로 오차: %s' % stat(2, 1000, 'mm'))
        print('각도 오차: %s' % stat(3, 180 / math.pi, '도'))
        dts = sorted(r[4] for r in good)
        print('소요 시간: 중앙값 %.1f초 최대 %.1f초' % (dts[len(dts) // 2], dts[-1]))
        bl = [r[5] for r in good if r[5] is not None]
        if bl:
            print('무검출 구간: 최대 %.2f초' % max(bl))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
