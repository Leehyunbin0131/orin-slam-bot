#!/usr/bin/env python3
"""배터리 상태를 모니터링하여 자동 충전 도킹 및 언도킹을 수행하는 노드.

    /battery_state --> [ AutoDock ] --> /dock_robot   (opennav_docking)
                                    --> /undock_robot

상태 구조
---------
    IDLE      : 배터리 잔량 모니터링 및 대기 상태
    RETURNING : 충전 도크 복귀 액션(DockRobot) 수행 중
    CHARGING  : 도킹 완료 및 충전 수행 중 (resume_soc 도달 시까지 대기)
    UNDOCKING : 언도킹 액션(UndockRobot) 수행 중

탐사 제어
---------
도킹 동작 수행 중 프론티어 탐사가 목표 지점을 갱신하는 것을 방지하기 위해
도킹 시작 시 `/exploration_enabled`를 false로 전환하고 충전/언도킹 완료 시 원복합니다.
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
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Empty

IDLE, RETURNING, CHARGING, UNDOCKING = 'IDLE', 'RETURNING', 'CHARGING', 'UNDOCKING'


class AutoDock(Node):

    def __init__(self):
        super().__init__('auto_dock')
        p = self.declare_parameter

        p('enabled', True)
        p('dock_id', 'home_dock')
        # 복귀 시작 배터리 잔량 임계값 (20%)
        p('low_soc', 0.20)
        # 작업 재개 배터리 잔량 임계값 (90%)
        p('resume_soc', 0.90)
        # 도킹 실패 시 재시도 대기 시간 [초]
        p('retry_delay', 30.0)
        p('max_attempts', 5)
        # 잔량 히스테리시스 (상태 진동 방지)
        p('soc_hysteresis', 0.03)
        # 충전 완료 시 자동 언도킹 수행 여부
        p('auto_undock', True)
        p('pause_exploration', True)
        # 충전 해제 판정 전 연속 확인 틱 수 (/battery_state 1 Hz 고려)
        p('charge_grace_ticks', 3)

        # --- 충전 중 절전 관리 ---
        # 충전 중 인지 및 항법 노드를 정지시켜 리소스 사용량을 절감합니다.
        # 노드를 파괴하지 않고 Lifecycle PAUSE / Service Pause로 제어합니다.
        p('power_save', True)
        p('nav2_manager', '/lifecycle_manager_navigation')
        p('slam_pause_service', '/rtabmap/pause')
        p('slam_resume_service', '/rtabmap/resume')
        p('extra_pause_services', [''])
        p('extra_resume_services', [''])
        # 복귀 후 Nav2 코스트맵 갱신 대기 타임아웃 [초]
        p('resume_timeout', 60.0)
        p('costmap_topic', '/global_costmap/costmap')
        p('power_save', True)
        p('nav2_manager', '/lifecycle_manager_navigation')
        p('slam_pause_service', '/rtabmap/pause')
        p('slam_resume_service', '/rtabmap/resume')
        # 실기에서 카메라/라이다 드라이버를 재우는 서비스가 있으면 여기에
        # 이름만 추가하면 됩니다 (std_srvs/Empty 규격). 코드 수정 불필요.
        p('extra_pause_services', [''])
        p('extra_resume_services', [''])
        # 복귀 후 Nav2 가 실제로 목표를 받을 수 있게 될 때까지의 대기 [s]
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
        self.saving = False           # 절전 중인가
        # 멀티스레드 환경 상태 기계 중복 실행 방지 락
        self._busy = threading.Lock()
        self.costmap_seen = False

        self.cbg = ReentrantCallbackGroup()
        self.manage = self.create_client(
            ManageLifecycleNodes, self.nav2_manager + '/manage_nodes',
            callback_group=self.cbg)

        self.dock_ac = ActionClient(self, DockRobot, 'dock_robot')
        self.undock_ac = ActionClient(self, UndockRobot, 'undock_robot')
        self.create_subscription(BatteryState, '/battery_state', self._battery, 10)
        # 탐사 노드 제어용 발행자
        self.explore_pub = self.create_publisher(Bool, '/exploration_enabled', 10)

        self.create_timer(1.0, self._tick, callback_group=self.cbg)
        self.get_logger().info(
            '자동 충전 감시 시작: %.0f%% 이하면 복귀, %.0f%% 이상이면 출발%s'
            % (self.low_soc * 100, self.resume_soc * 100,
               '' if self.enabled else ' (현재 비활성)'))

    # ---------------- 입력 ----------------

    def _battery(self, m):
        self.soc = m.percentage
        self.charging = m.power_supply_status in (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            BatteryState.POWER_SUPPLY_STATUS_FULL)

    def _set_exploration(self, on):
        if self.pause_exploration:
            self.explore_pub.publish(Bool(data=bool(on)))

    # ---------------- 상태 기계 ----------------

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

        if self.state == IDLE:
            if self.charging:
                # 사람이 손으로 붙여 놨거나, 도크 위에서 시작했습니다.
                #
                # 잔량 조건을 반드시 같이 봐야 합니다. `이미 충전 중`
                # 하나만 보고 CHARGING 으로 넘기면, auto_undock 이 꺼져
                # 있을 때 CHARGING 이 다시 IDLE 로 돌아오고 IDLE 이 또
                # CHARGING 으로 넘기면서 1초마다 로그를 뱉는 무한 왕복이
                # 생깁니다.
                if self.soc >= self.resume_soc + self.hyst:
                    if self.auto_undock:
                        self._start_undocking()
                    return
                self.get_logger().info('이미 충전 중입니다 — 충전 상태로 전환')
                self.state = CHARGING
                self.not_charging_ticks = 0
                # 이 경로에서도 절전에 들어가야 합니다. 도킹 액션 성공
                # 시에만 부르면, 사람이 손으로 붙여 놨거나 다른 노드가
                # 도킹시킨 경우 충전 내내 Nav2 와 SLAM 이 그대로 돕니다
                # (실측으로 그 상태를 확인했습니다).
                self._enter_power_save()
                return
            if self.soc <= self.low_soc:
                # 절전 중이면 도킹 명령을 내리지 않습니다. 인지가 죽어 있는
                # 상태에서 주행 명령이 나가면 아무것도 못 보고 움직입니다.
                if self.saving:
                    self.get_logger().warning(
                        '절전 중이라 도킹을 걸지 않습니다', once=True)
                    return
                if self.next_try is not None and now < self.next_try:
                    return
                if self.attempts >= self.max_attempts:
                    return
                self._start_docking()

        elif self.state == CHARGING:
            if self.soc >= self.resume_soc + self.hyst:
                if self.auto_undock:
                    self._start_undocking()
                else:
                    self.get_logger().info(
                        '충전 완료 (%.0f%%). auto_undock 이 꺼져 있어 대기합니다'
                        % (self.soc * 100))
                    self.state = IDLE
                    self._set_exploration(True)
            elif not self.charging:
                # 붙어 있다가 떨어졌습니다 (밀렸거나 접점이 빠짐).
                #
                # 한 틱만 보고 판단하면 안 됩니다. 도킹이 끝난 직후에는
                # /battery_state (1 Hz) 가 아직 충전 상태를 안 실어 보낸
                # 시점이라, 곧바로 "끊겼다"로 판정해 IDLE 로 튕겼다가
                # 다음 틱에 다시 CHARGING 으로 돌아오는 왕복이 생깁니다
                # (실측 확인). 그 사이에 재도킹이 걸릴 수도 있습니다.
                self.not_charging_ticks += 1
                if self.not_charging_ticks >= self.charge_grace:
                    self.get_logger().warning('충전이 끊겼습니다 — 다시 붙습니다')
                    # 다시 붙으려면 눈이 필요합니다.
                    self._exit_power_save()
                    self.state = IDLE
                    self.attempts = 0
            else:
                self.not_charging_ticks = 0

    # ---------------- 충전 중 절전 ----------------

    def _call_empty(self, name):
        """std_srvs/Empty 서비스를 부른다. 없으면 조용히 넘어갑니다."""
        cli = self.create_client(Empty, name, callback_group=self.cbg)
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('%s 서비스가 없습니다 — 건너뜁니다' % name)
            return False
        fut = cli.call_async(Empty.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 10.0:
            time.sleep(0.05)
        return fut.done()

    def _manage_nav2(self, command, label):
        if not self.manage.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('%s 를 찾지 못했습니다' % self.nav2_manager)
            return False
        fut = self.manage.call_async(
            ManageLifecycleNodes.Request(command=command))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 30.0:
            time.sleep(0.05)
        ok = fut.done() and fut.result() is not None and fut.result().success
        self.get_logger().info('Nav2 %s %s' % (label, '완료' if ok else '실패'))
        return ok

        """Nav2 전역 코스트맵의 신규 발행 여부를 확인합니다."""
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
        self.get_logger().info('충전 중 — 인지/항법을 재웁니다')
        # SLAM 을 먼저 재웁니다. Nav2 를 먼저 재우면 지도만 갱신되는
        # 어정쩡한 구간이 생깁니다.
        self._call_empty(self.slam_pause)
        for name in self.extra_pause:
            self._call_empty(name)
        self._manage_nav2(ManageLifecycleNodes.Request.PAUSE, 'PAUSE')
        self.saving = True

        """절전 모드를 해제하고 SLAM, Nav2 및 코스트맵 복구를 순차적으로 진행합니다."""
        if not self.power_save or not self.saving:
            return True
        self.get_logger().info('절전 해제 — 인지/항법을 되살립니다')
        self._call_empty(self.slam_resume)
        for name in self.extra_resume:
            self._call_empty(name)
        self._manage_nav2(ManageLifecycleNodes.Request.RESUME, 'RESUME')
        if not self._wait_costmap(self.resume_timeout):
            self.get_logger().error(
                '전역 코스트맵이 %.0f초 안에 돌아오지 않았습니다 — '
                '이 상태로 움직이면 목표가 즉시 ABORT 됩니다' % self.resume_timeout)
            return False
        self.saving = False
        self.get_logger().info('복귀 완료')
        return True

    # ---------------- 도킹 ----------------

    def _start_docking(self):
        if not self.dock_ac.server_is_ready():
            self.dock_ac.wait_for_server(timeout_sec=0.1)
            if not self.dock_ac.server_is_ready():
                self.get_logger().warning('docking_server 를 기다리는 중', once=True)
                return
        self.attempts += 1
        self.state = RETURNING
        self._set_exploration(False)
        self.get_logger().info(
            '잔량 %.0f%% — 도크 "%s" 로 복귀합니다 (%d/%d 회)'
            % (self.soc * 100, self.dock_id, self.attempts, self.max_attempts))

        g = DockRobot.Goal()
        g.use_dock_id = True
        g.dock_id = self.dock_id
        g.navigate_to_staging_pose = True
        self.dock_ac.send_goal_async(g).add_done_callback(self._accepted)

    def _start_undocking(self):
        # 센서와 항법이 살아난 것을 확인하기 전에는 절대 움직이지 않습니다.
        if not self._exit_power_save():
            return
        if not self.undock_ac.server_is_ready():
            self.undock_ac.wait_for_server(timeout_sec=0.1)
            if not self.undock_ac.server_is_ready():
                return
        self.state = UNDOCKING
        self.get_logger().info('충전 완료 (%.0f%%) — 도크에서 나갑니다'
                               % (self.soc * 100))
        self.undock_ac.send_goal_async(UndockRobot.Goal()).add_done_callback(
            self._accepted)

    # 액션 콜백은 절대 여기서 spin 하지 않습니다. 타이머 콜백 안에서
    # spin_until_future_complete 를 부르면 rclpy 가
    # "Executor is already spinning" 으로 죽습니다.

    def _accepted(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warning('목표가 거절되었습니다 — 나중에 다시 겁니다')
            self._failed()
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self._result)

    def _result(self, fut):
        res = fut.result()
        ok = res is not None and getattr(res.result, 'success', False)
        if self.state == UNDOCKING:
            if ok:
                self.get_logger().info('도크에서 나왔습니다 — 작업을 재개합니다')
            else:
                self.get_logger().error('언도킹 실패 — 그대로 재개합니다')
            self.state = IDLE
            self.attempts = 0
            self.next_try = None
            self._set_exploration(True)
            return

        if ok:
            self.get_logger().info('도킹 완료 — 충전을 시작합니다')
            self.state = CHARGING
            self.not_charging_ticks = 0
            self._enter_power_save()
            self.attempts = 0
            self.next_try = None
            return

        code = getattr(res.result, 'error_code', 0) if res else 0
        msg = getattr(res.result, 'error_msg', '') if res else ''
        self.get_logger().error('도킹 실패 (error_code=%d) %s' % (code, msg))
        self._failed()

    def _failed(self):
        self.state = IDLE
        now = self.get_clock().now().nanoseconds * 1e-9
        self.next_try = now + self.retry_delay
        if self.attempts >= self.max_attempts:
            self.get_logger().error(
                '도킹을 %d 회 실패했습니다. 자동 복귀를 멈춥니다 — '
                '사람이 확인해야 합니다' % self.attempts)
            # 포기하더라도 탐사는 되살립니다. 잔량이 남아 있는 동안
            # 아무것도 안 하고 서 있는 것보다는 낫습니다.
        self._set_exploration(True)


def main():
    rclpy.init()
    n = AutoDock()
    # 절전 진입/해제 서비스 호출 대기를 위해 MultiThreadedExecutor 사용 (스레드 2개 지정)
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
