#!/usr/bin/env python3
"""Runs one mission cycle on command.

    dock idle -> command -> wake -> undock -> mission -> return -> dock idle

The robot stays docked while there is no mission; this node decides when it
leaves. Power save must be fully released before undocking, and undocking must
finish before the mission starts -- goals issued while Nav2 is paused are
rejected outright and look like "nothing happened".

To add a mission, add an entry to MISSIONS with its run function. Wake, undock
and return are shared.
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

# Published on /mission/state. RUNNING and SUSPENDED carry the mission name.
DOCKED = 'DOCKED'
IDLE = 'IDLE'
WAKING = 'WAKING'
UNDOCKING = 'UNDOCKING'
RUNNING = 'RUNNING'
SUSPENDED = 'SUSPENDED'
RETURNING = 'RETURNING'
FAILED = 'FAILED'

# Values auto_dock publishes on /dock_state.
D_IDLE, D_RETURNING, D_CHARGING, D_UNDOCKING = (
    'IDLE', 'RETURNING', 'CHARGING', 'UNDOCKING')


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        p = self.declare_parameter

        p('leave_service', '/auto_dock/leave')
        p('return_service', '/auto_dock/return')
        p('explorer_reset_service', '/frontier_explorer/reset')
        p('dock_state_topic', '/dock_state')
        p('exploration_state_topic', '/exploration_state')
        p('exploration_enable_topic', '/exploration_enabled')
        p('leave_timeout', 180.0)         # wake + undock [s]
        p('return_timeout', 300.0)        # staging drive + docking [s]
        p('mission_timeout', 0.0)         # 0 = unlimited
        # Must match auto_dock's resume_soc.
        p('resume_soc', 0.90)
        p('return_retries', 3)

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.leave_srv_name = g('leave_service')
        self.return_srv_name = g('return_service')
        self.reset_srv_name = g('explorer_reset_service')
        self.leave_timeout = g('leave_timeout')
        self.return_timeout = g('return_timeout')
        self.mission_timeout = g('mission_timeout')
        self.resume_soc = g('resume_soc')
        self.return_retries = int(g('return_retries'))

        self.cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.dock_state = None
        self.explore_state = None
        self.soc = None
        self.create_subscription(String, g('dock_state_topic'),
                                 lambda m: setattr(self, 'dock_state', m.data),
                                 latched, callback_group=self.cb)
        self.create_subscription(String, g('exploration_state_topic'),
                                 lambda m: setattr(self, 'explore_state', m.data),
                                 latched, callback_group=self.cb)
        self.create_subscription(BatteryState, '/battery_state',
                                 lambda m: setattr(self, 'soc', m.percentage),
                                 10, callback_group=self.cb)
        self.explore_pub = self.create_publisher(
            Bool, g('exploration_enable_topic'), 10)
        self.state_pub = self.create_publisher(String, '~/state', latched)

        # The cycle runs for minutes in its own thread. Claim the slot under
        # the lock before blocking, or a second command overlaps undock/dock.
        self.lock = threading.Lock()
        self.running = None
        self.cancelled = False
        self.worker = None

        self.state = IDLE
        self._set_state(IDLE)
        self.create_timer(1.0, self._sync_idle_state, callback_group=self.cb)

        self.MISSIONS = {
            'mapping': self._run_mapping,
        }
        for name in self.MISSIONS:
            self.create_service(
                Trigger, '~/start_%s' % name,
                (lambda n: lambda req, res: self._srv_start(n, req, res))(name),
                callback_group=self.cb)
        self.create_service(Trigger, '~/cancel', self._srv_cancel,
                            callback_group=self.cb)

        self.get_logger().info(
            '임무 관리자 시작 — 가능한 임무: %s. 시작은 '
            'ros2 service call /mission/start_<임무> std_srvs/srv/Trigger'
            % ', '.join(sorted(self.MISSIONS)))

    # ---------------- State ----------------

    def _set_state(self, s, detail=None):
        self.state = '%s:%s' % (s, detail) if detail else s
        self.state_pub.publish(String(data=self.state))
        self.get_logger().info('임무 상태 → %s' % self.state)

    def _set_exploration(self, on):
        self.explore_pub.publish(Bool(data=bool(on)))

    def _sync_idle_state(self):
        """Track the dock state while no mission is running."""
        if self.running is not None or self.state.startswith(FAILED):
            return
        want = DOCKED if self.dock_state == D_CHARGING else IDLE
        if want != self.state:
            self._set_state(want)

    # ---------------- Commands ----------------

    def _srv_start(self, name, _req, res):
        with self.lock:
            if self.running is not None:
                res.success = False
                res.message = '이미 "%s" 임무를 수행 중입니다' % self.running
                return res
            self.running = name
            self.cancelled = False
        self.worker = threading.Thread(
            target=self._cycle, args=(name,), daemon=True)
        self.worker.start()
        res.success, res.message = True, '"%s" 임무를 시작합니다' % name
        return res

    def _srv_cancel(self, _req, res):
        with self.lock:
            if self.running is None:
                res.success, res.message = False, '수행 중인 임무가 없습니다'
                return res
            self.cancelled = True
            name = self.running
        self._set_exploration(False)
        res.success, res.message = True, '"%s" 임무 중단을 요청했습니다' % name
        return res

    # ---------------- Shared steps ----------------

    def _call(self, name, timeout=10.0):
        cli = self.create_client(Trigger, name, callback_group=self.cb)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                self.get_logger().error('서비스 %s 없음' % name)
                return False
            fut = cli.call_async(Trigger.Request())
            t0 = time.time()
            while not fut.done() and time.time() - t0 < timeout:
                time.sleep(0.05)
            r = fut.result()
            if r is None:
                self.get_logger().error('서비스 %s 응답 없음' % name)
                return False
            if not r.success:
                self.get_logger().error('%s: %s' % (name, r.message))
            return r.success
        finally:
            self.destroy_client(cli)

    def _wait_dock_state(self, targets, timeout, why, give_up=()):
        """Wait for auto_dock to reach one of `targets`.

        A state in `give_up` fails immediately -- a failed docking falls back
        to IDLE, and not watching for it burns the whole timeout first.
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.dock_state in targets:
                return True
            if self.dock_state in give_up:
                self.get_logger().warning(
                    '%s 실패 (auto_dock 이 %s 로 돌아갔습니다)'
                    % (why, self.dock_state))
                return False
            time.sleep(0.2)
        self.get_logger().error(
            '%s 대기 시간 초과 (%.0f s, 현재 %s)'
            % (why, timeout, self.dock_state))
        return False

    def _leave_dock(self):
        """Release power save and undock. Returns once the robot is out."""
        if self.dock_state == D_IDLE:
            return True
        self._set_state(WAKING)
        if not self._call(self.leave_srv_name):
            return False
        # auto_dock releases power save inside UNDOCKING, then lands on IDLE.
        self._set_state(UNDOCKING)
        return self._wait_dock_state({D_IDLE}, self.leave_timeout, '언도킹')

    def _return_dock(self):
        self._set_state(RETURNING)
        self._set_exploration(False)
        for i in range(1, self.return_retries + 1):
            if self.dock_state == D_CHARGING:
                return True
            if not self._call(self.return_srv_name):
                time.sleep(2.0)
                continue
            # Confirm it actually started, then split arrival from failure.
            if not self._wait_dock_state({D_RETURNING}, 15.0, '복귀 시작'):
                continue
            if self._wait_dock_state({D_CHARGING}, self.return_timeout, '복귀',
                                     give_up={D_IDLE}):
                return True
            self.get_logger().warning('복귀 실패 — 재시도 %d/%d'
                                      % (i, self.return_retries))
            time.sleep(2.0)
        return False

    # ---------------- Cycle ----------------

    def _cycle(self, name):
        ok = False
        try:
            if not self._leave_dock():
                self._set_state(FAILED, '언도킹')
                return
            ok = self.MISSIONS[name]()
            if not self._return_dock():
                self._set_state(FAILED, '복귀')
                return
            self._set_state(DOCKED)
            self.get_logger().info(
                '임무 "%s" %s — 도크에서 대기합니다'
                % (name, '완료' if ok else '중단'))
        except Exception as exc:                           # noqa: BLE001
            self.get_logger().error('임무 "%s" 예외: %s' % (name, exc))
            self._set_state(FAILED, name)
        finally:
            self._set_exploration(False)
            with self.lock:
                self.running = None
                self.cancelled = False

    def _wait_recharge(self, name):
        """Hold the mission while auto_dock recharges, then resume it.

        Restarting from scratch would re-cover ground already mapped.
        """
        self._set_state(SUSPENDED, name)
        self._set_exploration(False)
        while not self.cancelled:
            if self.dock_state == D_CHARGING and \
                    self.soc is not None and self.soc >= self.resume_soc:
                self.get_logger().info('충전 완료 — 임무 "%s" 를 이어서 합니다' % name)
                return self._leave_dock()
            time.sleep(1.0)
        return False

    # ---------------- Mission: auto mapping ----------------

    def _run_mapping(self):
        """Explore frontiers until none remain."""
        if not self._call(self.reset_srv_name):
            self.get_logger().warning(
                '탐사 초기화 실패 — 이전 임무 상태가 남아 있을 수 있습니다')
        self._set_state(RUNNING, 'mapping')
        self._set_exploration(True)

        t0 = time.time()
        while not self.cancelled:
            if self.explore_state == 'COMPLETE':
                self.get_logger().info('자동 매핑 완료')
                return True
            if self.mission_timeout > 0 and time.time() - t0 > self.mission_timeout:
                self.get_logger().warning('임무 시간 초과 — 복귀합니다')
                return False
            # auto_dock pulls the robot back on its own when the battery drops.
            if self.dock_state in (D_RETURNING, D_CHARGING):
                self.get_logger().warning('배터리 부족으로 임무가 중단됐습니다')
                if not self._wait_recharge('mapping'):
                    return False
                self._set_state(RUNNING, 'mapping')
                self._set_exploration(True)
            time.sleep(0.5)
        self.get_logger().info('자동 매핑 중단 요청을 받았습니다')
        return False


def main():
    rclpy.init()
    n = MissionManager()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
