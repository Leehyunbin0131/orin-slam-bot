#!/usr/bin/env python3
"""임무 명령을 받아 로봇의 한 사이클을 지휘합니다.

    /mission/start_mapping  --> [ MissionManager ] --> /auto_dock/leave
    /mission/cancel                               --> /exploration_enabled
                                                  --> /auto_dock/return
                            <--  /dock_state
                            <--  /exploration_state

임무 사이클
-----------
    도크 대기 -> 명령 수신 -> 절전 해제 -> 언도킹 -> 임무 수행 -> 복귀 -> 도크 대기

로봇은 임무가 없으면 계속 도크에서 대기합니다. 완충되어도 나가지 않습니다 —
나가는 시점을 정하는 것은 여기입니다.

**절전이 임무 시작을 막으면 안 됩니다.** 도크에 있는 동안 Nav2 는 PAUSE 이고
SLAM 은 정지 상태라, 이 상태에서 주행 목표를 내면 전부 즉시 거절됩니다.
그래서 순서가 고정되어 있습니다 — 절전 해제가 **끝난 것을 확인한 뒤에** 언도킹하고,
언도킹이 끝난 뒤에 임무를 켭니다. 확인 없이 다음 단계로 넘어가면 증상이
"명령을 넣었는데 아무 일도 안 일어남"으로만 보입니다.

임무를 추가하려면
-----------------
`MISSIONS` 에 항목을 하나 더하고 실행 함수를 씁니다. 깨우기·언도킹·복귀는
공통 절차라 임무 쪽에서 다시 쓸 필요가 없습니다.
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

# /mission/state 로 나가는 값
DOCKED = 'DOCKED'            # 도크에서 대기 중 (임무 없음)
IDLE = 'IDLE'                # 도크 밖에서 대기 중 (임무 없음)
WAKING = 'WAKING'            # 절전 해제 중
UNDOCKING = 'UNDOCKING'
RUNNING = 'RUNNING'          # 'RUNNING:mapping' 처럼 임무 이름이 붙습니다
SUSPENDED = 'SUSPENDED'      # 배터리 부족으로 중단, 충전 후 이어서 합니다
RETURNING = 'RETURNING'
FAILED = 'FAILED'

# auto_dock 이 /dock_state 로 내는 값
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
        # 절전 해제 + 언도킹이 끝나기를 기다리는 한도 [s]
        p('leave_timeout', 180.0)
        # 복귀(진입점 주행 + 도킹)를 기다리는 한도 [s]
        p('return_timeout', 300.0)
        # 임무 자체의 한도 [s]. 0 이면 무제한.
        p('mission_timeout', 0.0)
        # 배터리 부족으로 중단된 임무를 다시 시작할 충전량.
        # auto_dock 의 resume_soc 와 같은 값을 쓰세요.
        p('resume_soc', 0.90)
        # 복귀 실패 시 재시도 횟수. 도크 앞까지 갔다가 실패하면 그 자리에
        # 서 있게 되므로 재시도가 필요합니다.
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

        # 상태 플래그는 블록에 들어가기 전에 잠금 안에서 세웁니다. 임무는
        # 별도 스레드에서 몇 분씩 도는데, 그 사이 들어온 두 번째 명령이
        # 같은 절차를 겹쳐 실행하면 언도킹과 도킹이 서로를 밟습니다.
        self.lock = threading.Lock()
        self.running = None
        self.cancelled = False
        self.worker = None

        self.state = IDLE
        self._set_state(IDLE)
        # 임무가 없을 때는 /mission/state 가 실제 도크 상태를 따라가게 합니다.
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

    # ---------------- 상태 ----------------

    def _set_state(self, s, detail=None):
        self.state = '%s:%s' % (s, detail) if detail else s
        self.state_pub.publish(String(data=self.state))
        self.get_logger().info('임무 상태 → %s' % self.state)

    def _set_exploration(self, on):
        self.explore_pub.publish(Bool(data=bool(on)))

    def _sync_idle_state(self):
        if self.running is not None or self.state.startswith(FAILED):
            return
        want = DOCKED if self.dock_state == D_CHARGING else IDLE
        if want != self.state:
            self._set_state(want)

    # ---------------- 명령 ----------------

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

    # ---------------- 공통 절차 ----------------

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
        """auto_dock 이 목표 상태에 도달할 때까지 기다립니다.

        `give_up` 에 든 상태로 가면 곧바로 실패로 봅니다. 도킹이 실패하면
        auto_dock 은 IDLE 로 되돌아가는데, 그것을 안 보면 성공을 기다리며
        한도 전체를 태운 뒤에야 재시도하게 됩니다.
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
        """절전을 풀고 도크에서 나옵니다. 나온 것을 확인하고 돌아옵니다."""
        if self.dock_state == D_IDLE:
            return True                       # 이미 도크 밖입니다
        self._set_state(WAKING)
        if not self._call(self.leave_srv_name):
            return False
        # auto_dock 이 UNDOCKING 을 거쳐 IDLE 로 갑니다. 절전 해제는 그
        # 안에서 먼저 끝나므로 여기서 따로 확인할 것이 없습니다.
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
            # 먼저 실제로 출발했는지 보고, 그다음 도착과 실패를 갈라 봅니다.
            if not self._wait_dock_state({D_RETURNING}, 15.0, '복귀 시작'):
                continue
            if self._wait_dock_state({D_CHARGING}, self.return_timeout, '복귀',
                                     give_up={D_IDLE}):
                return True
            self.get_logger().warning('복귀 실패 — 재시도 %d/%d'
                                      % (i, self.return_retries))
            time.sleep(2.0)
        return False

    # ---------------- 사이클 ----------------

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
        """배터리 부족으로 auto_dock 이 끌고 들어갔을 때 충전을 기다립니다.

        임무는 살아 있습니다. 충전이 끝나면 이어서 합니다 — 매핑은 한 번에
        끝나지 않을 수 있고, 처음부터 다시 하면 이미 그린 곳을 또 돕니다.
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

    # ---------------- 임무: 자동 매핑 ----------------

    def _run_mapping(self):
        """미탐색 경계를 다 채울 때까지 자동 탐사합니다."""
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
            # 배터리가 떨어지면 auto_dock 이 스스로 끌고 들어갑니다.
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
