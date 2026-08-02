#!/usr/bin/env python3
"""잔량이 떨어지면 스스로 충전 도크로 돌아가고, 다 차면 다시 나갑니다.

    /battery_state --> [ 감시 ] --> /dock_robot   (opennav_docking)
                                --> /undock_robot

상태
----
    IDLE      : 잔량만 봅니다. 주행은 다른 노드(탐사/사용자)가 합니다.
    RETURNING : DockRobot 진행 중. staging pose 까지의 이동도 이 액션이
                직접 합니다.
    CHARGING  : 붙었습니다. resume_soc 까지 기다립니다.
    UNDOCKING : UndockRobot 진행 중.

탐사 노드와의 충돌
------------------
DockRobot 은 내부적으로 NavigateToPose 로 staging pose 에 갑니다. 그런데
frontier_explorer 가 2초마다 자기 목표를 보내 그 이동을 계속 밀어냅니다
(증상: "도킹을 시작했는데 로봇이 엉뚱한 데로 간다"). 그래서 도킹 전에
`/exploration_enabled` 로 false 를 보내고 충전이 끝나면 되돌립니다.

주의: 잔량이 실제로 떨어져야 움직입니다. 바로 시험하려면 직접 낮추세요.

    ros2 topic pub --once /battery_sim/set_soc std_msgs/Float32 '{data: 0.1}'
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
        # 이 잔량 밑으로 내려가면 복귀를 시작합니다. 0.20 으로 둔 이유:
        # 방 대각선이 약 13 m 이고 복귀에 넉넉히 2분이 걸린다고 보면
        # 5% 면 충분하지만, 도킹 실패 후 재시도(최대 3회 x 60초)까지
        # 감당해야 해서 여유를 크게 잡았습니다.
        p('low_soc', 0.20)
        # 이 잔량이 되면 다시 나갑니다. 만충(1.0)을 기다리면 충전 전류가
        # taper 구간에서 줄어 마지막 5% 에 전체 시간의 3분의 1을 씁니다.
        p('resume_soc', 0.90)
        # 도킹이 실패했을 때 다시 걸기까지의 간격 [s]
        p('retry_delay', 30.0)
        p('max_attempts', 5)
        # 잔량이 임계값 근처에서 떨릴 때 상태가 왔다 갔다 하지 않도록
        p('soc_hysteresis', 0.03)
        # 충전이 끝나면 자동으로 도크에서 빠져나올지
        p('auto_undock', True)
        p('pause_exploration', True)
        # 충전이 끊겼다고 판정하기 전에 연속으로 확인할 틱 수.
        # /battery_state 가 1 Hz 라 도킹 직후 한두 틱은 아직 충전
        # 상태가 안 실려 옵니다.
        p('charge_grace_ticks', 3)

        # --- 충전 중 절전 ---
        # 도크에 붙어 있는 동안은 인지·항법이 필요 없습니다. 실기(Orin)는
        # 6코어뿐이고 충전 전력도 유한하므로, 놀고 있는 노드를 재우면
        # 충전이 그만큼 빨라지고 발열도 줄어듭니다.
        #
        # **죽이지 말고 멈춰야 합니다.** rtabmap 을 죽였다 살리면 DB 를
        # 다시 읽고 재위치추정을 해야 하는데, 실측상 localization 모드는
        # CPU 25.1 -> 33.9 %p / 메모리 552 -> 604 MB 로 오히려 더 씁니다.
        # pause 는 포즈 그래프를 램에 둔 채 연산만 멈춥니다
        # (그래서 CPU 만 벌고 메모리는 그대로입니다).
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
        # 상태 기계 재진입 방지.
        # 멀티스레드 실행기에서는 타이머 콜백이 겹쳐 돕니다. 절전 해제는
        # 서비스 응답을 기다리며 수 초 블록하는데, 그 사이 타이머가 다시
        # 들어와 언도킹을 여러 번 걸었습니다 (실측: UndockRobot 4회 발행,
        # 로봇이 그만큼 더 후진). 한 번에 하나만 돌게 잠급니다.
        self._busy = threading.Lock()
        self.costmap_seen = False

        self.cbg = ReentrantCallbackGroup()
        self.manage = self.create_client(
            ManageLifecycleNodes, self.nav2_manager + '/manage_nodes',
            callback_group=self.cbg)

        self.dock_ac = ActionClient(self, DockRobot, 'dock_robot')
        self.undock_ac = ActionClient(self, UndockRobot, 'undock_robot')
        self.create_subscription(BatteryState, '/battery_state', self._battery, 10)
        # 탐사 노드가 구독합니다. TRANSIENT_LOCAL 이 아니라 그냥
        # 상태가 바뀔 때마다 보냅니다 — 늦게 뜬 탐사 노드는 기본값
        # (활성)으로 시작하므로 놓쳐도 안전한 쪽으로 틀립니다.
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

    def _wait_costmap(self, timeout):
        """Nav2 가 정말 목표를 받을 수 있는 상태인지 확인한다.

        "액션 서버가 살아 있다"는 준비 완료가 아닙니다 — planner_server 가
        아직 초기화 중이면 모든 목표가 즉시 ABORT 됩니다. 전역 코스트맵이
        새로 발행됐다는 것이 직접 증거입니다.

        QoS 를 기본(VOLATILE)으로 두는 것이 중요합니다. TRANSIENT_LOCAL 로
        받으면 **절전 직전에 발행된 옛 코스트맵**이 즉시 들어와 준비된
        것처럼 보입니다.
        """
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

    def _exit_power_save(self):
        """절전 해제. **순서가 중요합니다.**

        SLAM 을 먼저 살리고, Nav2 를 살린 뒤, 전역 코스트맵이 실제로
        나오는 것까지 확인한 다음에야 로봇을 움직여야 합니다. 거꾸로 하면
        복귀 직후 첫 목표가 즉시 ABORT 됩니다.
        """
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
    # 절전 진입/해제가 서비스 호출을 기다리며 블록하므로 멀티스레드
    # 실행기가 필요합니다. 단일 스레드에서는 그 사이 응답 콜백이 돌지
    # 못해 영원히 기다립니다.
    # **스레드 수를 반드시 지정해야 합니다.** 인자 없이 만들면 rclpy 가
    # CPU 코어 수만큼(이 PC 는 28개) 스레드를 만들고, 그 스레드들이
    # 대기셋을 두고 경합하며 놀고 있어도 CPU 를 태웁니다.
    # 실측: 지정 없이 두었을 때 idle 상태에서 86 %p.
    # 여기서 동시에 필요한 것은 (상태 기계 + 액션/서비스 응답) 둘뿐입니다.
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
