"""고정 시나리오 실행 중 노드별 CPU 및 메모리 사용량 측정 스크립트.

    python3 measure_load.py <라벨>

시나리오: c070 통로 주행 및 복귀. `/proc/<pid>/stat` 기반 차분 샘플링 수행.
"""
import os
import subprocess
import sys
import threading
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

LABEL = sys.argv[1] if len(sys.argv) > 1 else 'baseline'
# c070 (폭 0.70 m, y=+0.6) 을 지나 동쪽 띠까지 갔다가 복귀.
#
# 예전에는 c060 (폭 0.60 m, y=-3.0) 을 지났습니다. 최소 통과 가능 폭을
# 0.70 m 로 정한 뒤로는 기준 밖 통로입니다. 거기서는 로봇이 자주 끼여
# 복구 행동에 시간을 쓰는데, 그 시간이 CPU 표본에 섞이면 "파라미터를 바꿔서
# 느려졌는지" 와 "그날 유난히 끼였는지" 를 구분할 수 없습니다.
WAYPOINTS = [(4.3, 0.6), (0.0, 0.0)]
INTERVAL = 0.5

# 실기에 남는 노드 (합계에 포함)
ROBOT_NODES = ['rtabmap', 'rgbd_odometry', 'controller_serv', 'planner_server',
               'bt_navigator', 'behavior_server', 'smoother_server',
               'waypoint_follow', 'velocity_smooth', 'lifecycle_mana',
               'robot_state_pub', 'twist_mux', 'imu_filter_madg',
               'component_conta', 'vodom_tf_relay', 'ekf_node',
               'frontier_explor']
# 시뮬/시각화 전용 (따로 표시)
SIM_NODES = ['gz', 'parameter_bridg', 'rviz2', 'ruby']
WATCH = set(ROBOT_NODES) | set(SIM_NODES)

HZ = os.sysconf('SC_CLK_TCK')
PAGE_KB = os.sysconf('SC_PAGE_SIZE') / 1024.0

samples = []          # [{comm: (cpu_percent, rss_mb)}]
stop = threading.Event()


def scan():
    """{pid: (comm, ticks, rss_kb)} — 관심 있는 프로세스만."""
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open('/proc/%s/stat' % pid, 'rb') as f:
                raw = f.read().decode('utf-8', 'replace')
        except OSError:
            continue
        # comm 은 괄호 안에 있고 공백을 포함할 수 있으므로 마지막 ')' 로 자른다
        i, j = raw.find('('), raw.rfind(')')
        if i < 0 or j < 0:
            continue
        comm = raw[i + 1:j]
        if comm not in WATCH:
            continue
        f = raw[j + 2:].split()
        # stat 필드는 pid,comm 을 뺀 뒤 0-기반: utime=11, stime=12, rss=21
        try:
            ticks = int(f[11]) + int(f[12])
            rss_kb = int(f[21]) * PAGE_KB
        except (IndexError, ValueError):
            continue
        out[int(pid)] = (comm, ticks, rss_kb)
    return out


def sampler():
    prev, prev_t = scan(), time.time()
    while not stop.is_set():
        time.sleep(INTERVAL)
        cur, cur_t = scan(), time.time()
        dt = cur_t - prev_t
        row = {}
        for pid, (comm, ticks, rss) in cur.items():
            if pid not in prev or prev[pid][0] != comm:
                continue          # 새로 뜬 프로세스는 다음 샘플부터
            cpu = (ticks - prev[pid][1]) / HZ / dt * 100.0
            c, r = row.get(comm, (0.0, 0.0))
            row[comm] = (c + cpu, r + rss)
        if row:
            samples.append(row)
        prev, prev_t = cur, cur_t


class M(Node):
    def __init__(self):
        super().__init__('measure_load')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goto(self, x, y, timeout=150.0):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.w = 1.0
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None
        res = gh.get_result_async()
        t0 = time.time()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return res.result().status if res.done() else 0


rclpy.init()
n = M()
n.ac.wait_for_server(timeout_sec=60.0)
t = time.time()
while time.time() - t < 3:
    rclpy.spin_once(n, timeout_sec=0.1)

th = threading.Thread(target=sampler, daemon=True)
th.start()
t0 = time.time()
results = [n.goto(x, y) for x, y in WAYPOINTS]
elapsed = time.time() - t0
stop.set()
th.join(timeout=3)

names = sorted({k for s in samples for k in s})
print('===== %s =====' % LABEL)
print('시나리오 %.0f 초, 표본 %d개, 목표 결과 %s (4=성공)'
      % (elapsed, len(samples), results))
print('%-18s %9s %9s %10s' % ('노드', '평균CPU%', '최대CPU%', '최대RSS MB'))


def stat(name):
    vals = [s[name][0] for s in samples if name in s]
    rss = [s[name][1] for s in samples if name in s]
    if not vals:
        return None
    return sum(vals) / len(vals), max(vals), max(rss) / 1024.0


tot_cpu = tot_rss = 0.0
for nm in names:
    if nm not in ROBOT_NODES:
        continue
    st = stat(nm)
    if not st or st[0] < 0.05:
        continue
    print('%-18s %9.1f %9.1f %10.0f' % (nm, st[0], st[1], st[2]))
    tot_cpu += st[0]
    tot_rss += st[2]
print('%-18s %9.1f %9s %10.0f' % ('--- 실기 합계 ---', tot_cpu, '', tot_rss))
print('   = %.2f 코어,  Orin(코어당 4.5배 느림 가정) 환산 %.1f 코어 / 6코어'
      % (tot_cpu / 100.0, tot_cpu / 100.0 * 4.5))
print()
for nm in names:
    if nm not in SIM_NODES:
        continue
    st = stat(nm)
    if not st or st[0] < 0.05:
        continue
    print('(실기 제외) %-12s %8.1f %9.1f %10.0f' % (nm, st[0], st[1], st[2]))
rclpy.shutdown()
sys.exit(0)
