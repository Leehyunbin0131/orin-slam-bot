#!/usr/bin/env python3
"""Run docking from several initial conditions in parallel and tabulate.

    python3 tools/dock_bench.py                 # default case set
    python3 tools/dock_bench.py --jobs 4        # concurrent instances
    python3 tools/dock_bench.py --repeat 3      # repeats per case

Bringing the whole stack up and down for every parameter change costs three
minutes a run. Gazebo cannot vectorise environments in one process, but N
isolated instances do work, isolated on two axes:

    ROS_DOMAIN_ID   ROS 2 discovery
    GZ_PARTITION    gz-transport

Each domain then sees exactly one /clock publisher. Roughly 1 core and 1 GB
per instance.

The bench runs without SLAM or Nav2 (see dock_bench.launch.py), so instead of
_goto_entry the robot spawns near the alignment point and each case perturbs
that spawn pose to create the initial error.

Pass/fail uses the contact tolerance: +-48 mm long, +-34 mm lat, +-6.3 deg.
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCK = (1.0, -3.64, 1.5708)          # docked pose; faces away when reversed
TOL_LON, TOL_LAT, TOL_YAW = 0.048, 0.034, math.radians(6.3)
# Reference spawn for alignment, ~0.79 m from the marker face (-3.894).
BASE = (1.0, -3.10, -1.5708)

# (name, spawn dx [m], dy [m], dyaw [deg], parameter overrides)
CASES = [
    ('baseline',        0.00,  0.00,  0.0, {}),
    ('lat +5cm',        0.05,  0.00,  0.0, {}),
    ('lat -5cm',       -0.05,  0.00,  0.0, {}),
    ('yaw +8deg',       0.00,  0.00,  8.0, {}),
    ('yaw -8deg',       0.00,  0.00, -8.0, {}),
    ('far +10cm',       0.00,  0.10,  0.0, {}),
    ('turn 0.70',       0.00,  0.00,  0.0, {'rotate_distance': 0.70}),
    ('budget 15mm',     0.00,  0.00,  0.0, {'contact_lateral_budget': 0.015}),
]


def env_for(idx):
    e = dict(os.environ)
    e['ROS_DOMAIN_ID'] = str(40 + idx)
    e['GZ_PARTITION'] = 'bench%d' % idx
    e['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    return e


def overlay(base_yaml, overrides, workdir):
    """Write a temporary yaml with only staged_dock parameters overridden."""
    if not overrides:
        return base_yaml
    txt = open(base_yaml, encoding='utf-8').read()
    for k, v in overrides.items():
        # Only "    key: value" lines; comments are left alone.
        out, done = [], False
        for line in txt.split('\n'):
            s = line.strip()
            if not done and s.startswith(k + ':') and not s.startswith('#'):
                out.append('%s%s: %s' % (line[:len(line) - len(line.lstrip())], k, v))
                done = True
            else:
                out.append(line)
        if not done:
            raise SystemExit('parameter %s not found in %s' % (k, base_yaml))
        txt = '\n'.join(out)
    path = os.path.join(workdir, 'docking.yaml')
    open(path, 'w', encoding='utf-8').write(txt)
    return path


# The probe runs as its own process: one Python process cannot watch several
# ROS_DOMAIN_IDs at once. The dock pose is prepended as a line below, since
# the body contains % formatting.
PROBE_BODY = r'''
import json, math, sys, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import DockRobot
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
rclpy.init(); n = Node('bench')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
s = {'o': None, 'm': 0}
n.create_subscription(Odometry, '/ground_truth/odom',
                      lambda m: s.__setitem__('o', m), 10)
n.create_subscription(PoseStamped, '/detected_dock_pose',
                      lambda m: s.__setitem__('m', s['m'] + 1), 10)
ac = ActionClient(n, DockRobot, 'dock_robot')
out = {'ok': False, 'why': ''}
try:
    if not ac.wait_for_server(timeout_sec=120):
        out['why'] = 'no dock_robot action server'; raise SystemExit
    t0 = time.time()
    while s['m'] == 0 and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.2)
    if s['m'] == 0:
        out['why'] = 'no marker detected from the spawn pose'; raise SystemExit
    g = DockRobot.Goal(); g.use_dock_id = True; g.dock_id = 'home_dock'
    g.navigate_to_staging_pose = False      # no Nav2 on the bench
    t0 = time.time()
    f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
    gh = f.result()
    if gh is None or not gh.accepted:
        out['why'] = 'goal rejected'; raise SystemExit
    rf = gh.get_result_async()
    while not rf.done() and time.time() - t0 < 300:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not rf.done():
        out['why'] = 'timed out'; raise SystemExit
    res = rf.result()
    out['secs'] = time.time() - t0
    for _ in range(40): rclpy.spin_once(n, timeout_sec=0.05)
    p = s['o'].pose.pose.position; q = s['o'].pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    dx, dy = p.x-DX, p.y-DY
    c, sn = math.cos(DYAW), math.sin(DYAW)
    out['lon'] = dx*c + dy*sn
    out['lat'] = -dx*sn + dy*c
    out['yaw'] = math.atan2(math.sin(yaw-DYAW), math.cos(yaw-DYAW))
    out['status'] = res.status
    out['ok'] = res.status == 4
    if not out['ok']:
        out['why'] = 'error_code %d' % res.result.error_code
except SystemExit:
    pass
finally:
    print('BENCH' + json.dumps(out))
    rclpy.shutdown()
'''

PROBE = 'DX, DY, DYAW = %.6f, %.6f, %.6f\n' % DOCK + PROBE_BODY


def run_case(idx, name, dx, dy, dyaw, overrides, repeat):
    env = env_for(idx)
    work = tempfile.mkdtemp(prefix='dockbench%d_' % idx)
    results = []
    try:
        yml = overlay(os.path.join(WS, 'src/orinbot_navigation/config/docking.yaml'),
                      overrides, work)
        for _ in range(repeat):
            x, y = BASE[0] + dx, BASE[1] + dy
            yaw = BASE[2] + math.radians(dyaw)
            log = open(os.path.join(work, 'launch.log'), 'w')
            proc = subprocess.Popen(
                ['ros2', 'launch', 'orinbot_navigation', 'dock_bench.launch.py',
                 'params_file:=' + yml,
                 'x:=%.4f' % x, 'y:=%.4f' % y, 'yaw:=%.4f' % yaw],
                cwd=WS, env=env, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            try:
                probe = subprocess.run(
                    [sys.executable, '-c', PROBE], cwd=WS, env=env,
                    capture_output=True, text=True, timeout=480)
                line = next((l for l in probe.stdout.split('\n')
                             if l.startswith('BENCH')), None)
                results.append(json.loads(line[5:]) if line
                               else {'ok': False, 'why': 'no probe output'})
            except subprocess.TimeoutExpired:
                results.append({'ok': False, 'why': 'probe timed out'})
            finally:
                log.close()
                try:
                    os.killpg(proc.pid, 9)
                except Exception:                             # noqa: BLE001
                    pass
                proc.wait(timeout=30)
                time.sleep(3)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return name, overrides, results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--jobs', type=int, default=4, help='concurrent instances')
    ap.add_argument('--repeat', type=int, default=1, help='repeats per case')
    a = ap.parse_args()

    print('%d cases x %d runs, %d concurrent' % (len(CASES), a.repeat, a.jobs))
    print('~1 core / 1 GB per instance; nproc %d, check free memory\n'
          % os.cpu_count())
    t0 = time.time()
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = [ex.submit(run_case, i, *c, a.repeat) for i, c in enumerate(CASES)]
        rows = [f.result() for f in futs]

    print('%-14s %7s %9s %9s %8s %7s  %s'
          % ('case', 'ok', 'lon mm', 'lat mm', 'yaw deg', 's', 'note'))
    print('-' * 78)
    for name, ov, rs in rows:
        good = [r for r in rs if r.get('ok')]
        if not good:
            why = rs[0].get('why', '?') if rs else '?'
            print('%-14s %7s %9s %9s %8s %7s  %s'
                  % (name, '0/%d' % len(rs), '-', '-', '-', '-', why))
            continue
        f = lambda k: sum(r[k] for r in good) / len(good)      # noqa: E731
        print('%-14s %7s %9.1f %9.1f %8.2f %7.0f  %s'
              % (name, '%d/%d' % (len(good), len(rs)),
                 f('lon') * 1000, f('lat') * 1000,
                 math.degrees(f('yaw')), f('secs'),
                 ' '.join('%s=%s' % kv for kv in ov.items())))
    print('\ntolerance: lon +-%.0f mm / lat +-%.0f mm / yaw +-%.1f deg'
          % (TOL_LON * 1000, TOL_LAT * 1000, math.degrees(TOL_YAW)))
    print('total %.0f s' % (time.time() - t0))


if __name__ == '__main__':
    main()
