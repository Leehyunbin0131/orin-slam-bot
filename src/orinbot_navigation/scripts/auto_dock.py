#!/usr/bin/env python3
"""Node that monitors battery state to manage autonomous docking and undocking.

    /battery_state --> [ AutoDock ] --> /dock_robot   (opennav_docking)
                                    --> /undock_robot

State Architecture
------------------
    IDLE      : 도크 밖에 있고 배터리를 감시하는 중.
    RETURNING : 도킹 액션(DockRobot) 진행 중.
    CHARGING  : 도크에 붙어 대기 중. **완충되어도 스스로 나가지 않습니다** —
                나가는 것은 임무 관리자가 정합니다. 실기 배터리는 내장 BMS 가
                전류를 끊으므로 붙은 채로 두어도 됩니다.
    UNDOCKING : 언도킹 액션(UndockRobot) 진행 중.

현재 상태를 `/dock_state` 로 발행하고, 임무 관리자는 `~/leave` / `~/return`
서비스로 전환을 요청합니다. 두 서비스는 **요청만 걸어 두고 즉시 돌아옵니다** —
실제 완료는 `/dock_state` 로 확인하세요. 서비스 안에서 도킹이 끝나기를 기다리면
그 몇십 초 동안 응답이 묶입니다.

Exploration Control
-------------------
To prevent frontier explorer from sending conflicting navigation goals during docking,
`/exploration_enabled` is set to false when docking starts and restored upon completion.
"""

import threading
import time

import rclpy
from nav2_msgs.action import DockRobot, UndockRobot
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger

IDLE, RETURNING, CHARGING, UNDOCKING = 'IDLE', 'RETURNING', 'CHARGING', 'UNDOCKING'


class AutoDock(Node):

    def __init__(self):
        super().__init__('auto_dock')
        p = self.declare_parameter

        p('enabled', True)
        p('dock_id', 'home_dock')
        # Threshold battery SOC to initiate return (20%)
        p('low_soc', 0.20)
        # Threshold battery SOC to resume operation (90%)
        p('resume_soc', 0.90)
        # Retry delay after a failed docking attempt [s]
        p('retry_delay', 30.0)
        p('max_attempts', 5)
        # SOC hysteresis to prevent state oscillation
        p('soc_hysteresis', 0.03)
        # 완충되면 스스로 나갈지 여부. **기본은 false 입니다** — 임무가 없으면
        # 도크에서 대기하는 것이 정상 동작이고, 나가는 시점은 임무 관리자가
        # 정합니다. true 로 두면 임무와 무관하게 로봇이 돌아다닙니다.
        p('auto_undock', False)
        p('pause_exploration', True)
        # Grace ticks before confirming disconnected charging state
        p('charge_grace_ticks', 3)

        # --- Power Saving Mode During Charging ---
        # Suspend perception and navigation nodes while docked to reduce power consumption.
        # Uses Lifecycle PAUSE and Service Pause rather than killing process instances.
        p('power_save', True)
        p('nav2_manager', '/lifecycle_manager_navigation')
        p('slam_pause_service', '/rtabmap/pause')
        p('slam_resume_service', '/rtabmap/resume')
        # Optional pause/resume services for camera or lidar drivers
        p('extra_pause_services', [''])
        p('extra_resume_services', [''])
        # Timeout waiting for costmap recovery after resume [s]
        p('resume_timeout', 60.0)
        p('costmap_topic', '/global_costmap/costmap')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.enabled = g('enabled')
        self.dock_id = g('dock_id')
        self.low_soc = g('low_soc')
        self.resume_soc = g('resume_soc')
        self.retry_delay = g('retry_delay')
        self.max_attempts = int(g('max_attempts'))
        self.hyst = g('soc_hysteresis')
        self.auto_undock = g('auto_undock')
        self.pause_exploration = g('pause_exploration')
        self.charge_grace = int(g('charge_grace_ticks'))
        self.power_save = g('power_save')
        self.nav2_manager = g('nav2_manager')
        self.slam_pause = g('slam_pause_service')
        self.slam_resume = g('slam_resume_service')
        self.extra_pause = [x for x in g('extra_pause_services') if x]
        self.extra_resume = [x for x in g('extra_resume_services') if x]
        self.resume_timeout = g('resume_timeout')
        self.costmap_topic = g('costmap_topic')

        self.state = IDLE
        self.soc = None
        self.charging = False
        self.attempts = 0
        self.next_try = None
        self.goal_handle = None
        self.not_charging_ticks = 0
        self.charge_done_logged = False
        self.saving = False

        # Mutex to prevent state machine re-entry in multi-threaded executor
        self._busy = threading.Lock()
        self.costmap_seen = False

        self.cbg = ReentrantCallbackGroup()
        self.manage = self.create_client(
            ManageLifecycleNodes, self.nav2_manager + '/manage_nodes',
            callback_group=self.cbg)

        self.dock_ac = ActionClient(self, DockRobot, 'dock_robot')
        self.undock_ac = ActionClient(self, UndockRobot, 'undock_robot')
        self.create_subscription(BatteryState, '/battery_state', self._battery, 10)
        self.explore_pub = self.create_publisher(Bool, '/exploration_enabled', 10)

        # 늦게 뜬 임무 관리자도 현재 상태를 바로 받도록 TRANSIENT_LOCAL 입니다.
        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(String, '/dock_state', latched)

        # 임무 관리자의 요청. 서비스 안에서 처리하지 않고 플래그만 세워
        # 다음 틱에서 실행합니다 — 상태 전이의 단일 진입점을 유지합니다.
        self.request = None
        self.create_service(Trigger, '~/leave', self._srv_leave,
                            callback_group=self.cbg)
        self.create_service(Trigger, '~/return', self._srv_return,
                            callback_group=self.cbg)

        self.create_timer(1.0, self._tick, callback_group=self.cbg)
        self._publish_state()
        self.get_logger().info(
            'Auto dock monitor started: return <= %.0f%%, resume >= %.0f%%%s'
            % (self.low_soc * 100, self.resume_soc * 100,
               '' if self.enabled else ' (disabled)'))

    # ---------------- Inputs ----------------

    def _battery(self, m):
        self.soc = m.percentage
        self.charging = m.power_supply_status in (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            BatteryState.POWER_SUPPLY_STATUS_FULL)

    def _set_exploration(self, on):
        if self.pause_exploration:
            self.explore_pub.publish(Bool(data=bool(on)))

    def _publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def _set_state(self, s):
        if s != self.state:
            self.state = s
            self._publish_state()

    # ---------------- 임무 관리자 요청 ----------------

    def _srv_leave(self, _req, res):
        if self.state in (RETURNING, UNDOCKING):
            res.success, res.message = False, '이미 %s 중입니다' % self.state
            return res
        self.request = 'leave'
        res.success, res.message = True, '언도킹을 요청했습니다 (/dock_state 확인)'
        return res

    def _srv_return(self, _req, res):
        if self.state in (RETURNING, UNDOCKING):
            res.success, res.message = False, '이미 %s 중입니다' % self.state
            return res
        if self.state == CHARGING:
            res.success, res.message = True, '이미 도크에 있습니다'
            return res
        # 임무 복귀는 배터리와 무관하므로 실패 이력과 대기 시간을 지웁니다.
        self.attempts = 0
        self.next_try = None
        self.request = 'return'
        res.success, res.message = True, '복귀를 요청했습니다 (/dock_state 확인)'
        return res

    # ---------------- State Machine ----------------

    def _tick(self):
        if not self._busy.acquire(blocking=False):
            return
        try:
            self._tick_locked()
        finally:
            self._busy.release()

    def _tick_locked(self):
        if not self.enabled or self.soc is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9

        # 임무 관리자의 요청이 배터리 판단보다 우선합니다.
        req, self.request = self.request, None
        if req == 'leave' and self.state in (IDLE, CHARGING):
            self._start_undocking('임무 시작')
            return
        if req == 'return' and self.state == IDLE:
            self._start_docking('임무 종료')
            return

        if self.state == IDLE:
            if self.charging:
                # 충전이 잡히면 완충 여부와 상관없이 대기 상태로 들어갑니다.
                self.get_logger().info('충전이 감지되어 도크 대기 상태로 들어갑니다')
                self._set_state(CHARGING)
                self.not_charging_ticks = 0
                self.charge_done_logged = False
                self._enter_power_save()
                return
            if self.soc <= self.low_soc:
                if self.saving:
                    self.get_logger().warning(
                        'Power save mode active, skipping docking attempt', once=True)
                    return
                if self.next_try is not None and now < self.next_try:
                    return
                if self.attempts >= self.max_attempts:
                    return
                self._start_docking('배터리 %.0f%%' % (self.soc * 100))

        elif self.state == CHARGING:
            if not self.charging:
                self.not_charging_ticks += 1
                if self.not_charging_ticks >= self.charge_grace:
                    self.get_logger().warning('Charging disconnected -- retrying docking')
                    self._exit_power_save()
                    self._set_state(IDLE)
                    self.attempts = 0
                return
            self.not_charging_ticks = 0
            if self.soc >= self.resume_soc + self.hyst:
                if self.auto_undock:
                    self._start_undocking('완충')
                elif not self.charge_done_logged:
                    self.charge_done_logged = True
                    self.get_logger().info(
                        '충전 완료 (%.0f%%) — 임무 명령이 올 때까지 도크에서 대기합니다'
                        % (self.soc * 100))

    # ---------------- Power Saving ----------------

    def _call_empty(self, name):
        """Call std_srvs/Empty service."""
        cli = self.create_client(Empty, name, callback_group=self.cbg)
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('Service %s unavailable -- skipping' % name)
            return False
        fut = cli.call_async(Empty.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 10.0:
            time.sleep(0.05)
        return fut.done()

    def _manage_nav2(self, command, label):
        if not self.manage.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service %s unavailable' % self.nav2_manager)
            return False
        fut = self.manage.call_async(
            ManageLifecycleNodes.Request(command=command))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 30.0:
            time.sleep(0.05)
        ok = fut.done() and fut.result() is not None and fut.result().success
        self.get_logger().info('Nav2 %s %s' % (label, 'completed' if ok else 'failed'))
        return ok

    def _wait_costmap(self, timeout):
        """Check for fresh Nav2 global costmap publication."""
        self.costmap_seen = False
        sub = self.create_subscription(
            OccupancyGrid, self.costmap_topic,
            lambda _m: setattr(self, 'costmap_seen', True), 1,
            callback_group=self.cbg)
        t0 = time.time()
        while not self.costmap_seen and time.time() - t0 < timeout:
            time.sleep(0.1)
        self.destroy_subscription(sub)
        return self.costmap_seen

    def _enter_power_save(self):
        if not self.power_save or self.saving:
            return
        # 플래그를 **블록하기 전에** 세웁니다. 아래 서비스 호출이 몇 초 동안
        # 막혀 있는 사이 다른 콜백이 들어와 언도킹을 걸면, Nav2 가 켜진 채로
        # PAUSE 되어 그대로 멈춥니다.
        self.saving = True
        self.get_logger().info('Charging started -- entering power save mode')
        self._call_empty(self.slam_pause)
        for name in self.extra_pause:
            self._call_empty(name)
        self._manage_nav2(ManageLifecycleNodes.Request.PAUSE, 'PAUSE')

    def _exit_power_save(self):
        """Exit power save mode and restore SLAM, Nav2, and costmap in sequence."""
        if not self.power_save or not self.saving:
            return True
        self.get_logger().info('Exiting power save mode -- resuming perception and navigation')
        self._call_empty(self.slam_resume)
        for name in self.extra_resume:
            self._call_empty(name)
        self._manage_nav2(ManageLifecycleNodes.Request.RESUME, 'RESUME')
        if not self._wait_costmap(self.resume_timeout):
            self.get_logger().error(
                'Global costmap not received within %.0f s' % self.resume_timeout)
            return False
        self.saving = False
        self.get_logger().info('Resume completed')
        return True

    # ---------------- Docking ----------------

    def _start_docking(self, why):
        if not self.dock_ac.server_is_ready():
            self.dock_ac.wait_for_server(timeout_sec=0.1)
            if not self.dock_ac.server_is_ready():
                self.get_logger().warning('Waiting for docking_server', once=True)
                return
        self.attempts += 1
        self._set_state(RETURNING)
        self._set_exploration(False)
        self.get_logger().info(
            '%s — 도크 "%s" 로 복귀합니다 (%d/%d)'
            % (why, self.dock_id, self.attempts, self.max_attempts))

        g = DockRobot.Goal()
        g.use_dock_id = True
        g.dock_id = self.dock_id
        g.navigate_to_staging_pose = True
        self.dock_ac.send_goal_async(g).add_done_callback(self._accepted)

    def _start_undocking(self, why):
        # 절전을 먼저 풀어야 합니다. 인지가 죽은 채로 나가면 아무것도 못 보고
        # 움직입니다.
        if not self._exit_power_save():
            self.get_logger().error('절전 해제 실패 — 언도킹을 중단합니다')
            return
        if not self.undock_ac.server_is_ready():
            self.undock_ac.wait_for_server(timeout_sec=0.1)
            if not self.undock_ac.server_is_ready():
                self.get_logger().warning('Waiting for docking_server', once=True)
                return
        self._set_state(UNDOCKING)
        self.get_logger().info('%s — 언도킹합니다 (배터리 %.0f%%)'
                               % (why, self.soc * 100))
        self.undock_ac.send_goal_async(UndockRobot.Goal()).add_done_callback(
            self._accepted)

    def _accepted(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warning('Goal rejected -- retrying later')
            self._failed()
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self._result)

    def _result(self, fut):
        res = fut.result()
        ok = res is not None and getattr(res.result, 'success', False)
        if self.state == UNDOCKING:
            if ok:
                self.get_logger().info('언도킹 완료')
            else:
                self.get_logger().error('언도킹 실패 — 그대로 진행합니다')
            self._set_state(IDLE)
            self.attempts = 0
            self.next_try = None
            return

        if ok:
            self.get_logger().info('도킹 완료 — 충전을 시작합니다')
            self._set_state(CHARGING)
            self.not_charging_ticks = 0
            self.charge_done_logged = False
            self._enter_power_save()
            self.attempts = 0
            self.next_try = None
            return

        code = getattr(res.result, 'error_code', 0) if res else 0
        msg = getattr(res.result, 'error_msg', '') if res else ''
        self.get_logger().error('Docking failed (error_code=%d) %s' % (code, msg))
        self._failed()

    def _failed(self):
        self._set_state(IDLE)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.next_try = now + self.retry_delay
        if self.attempts >= self.max_attempts:
            self.get_logger().error(
                'Docking failed %d times. Stopping auto return.' % self.attempts)
        self._set_exploration(True)


def main():
    rclpy.init()
    n = AutoDock()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
