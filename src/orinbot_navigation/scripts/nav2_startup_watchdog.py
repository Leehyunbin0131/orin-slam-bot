#!/usr/bin/env python3
"""Nav2 가 활성화에 실패하면 다시 시도시키고, 되면 스스로 끝난다.

왜 필요한가
-----------
`lifecycle_manager` 의 `change_state` 응답이 유실되면 기동이 이렇게 끝납니다.

    [controller_server.rclcpp] failed to send response to
        /controller_server/change_state (timeout)
    [lifecycle_manager_navigation] Failed to bring up all requested nodes.

노드는 살아 있는데 전부 `inactive` 라 액션 서버는 보이면서 모든 목표를 즉시
ABORT 합니다. 겉으로는 "Nav2 는 떠 있는데 로봇이 안 움직인다"로만 보입니다.
이 개발 PC 는 NIC 가 3개라 FastDDS 서비스 매칭이 늦어 자주 납니다
(`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 로도 완전히 없어지지 않습니다).
실기에서 더 잦은지는 아직 확인하지 못했습니다 — 2026-08-02 실행에서는
재현되지 않았습니다.

무엇을 하는가
-------------
`is_active` 를 물어보다가 `startup_timeout` 안에 활성이 안 되면 `manage_nodes`
로 **RESET 후 STARTUP** 을 다시 겁니다. RESET 이 먼저인 이유는 일부 노드가
이미 활성이면 STARTUP 만으로는 "already active" 로 실패하기 때문입니다.
활성이 확인되면 이 노드는 종료합니다.

구현 주의
---------
- **타이머 콜백 안에서 `spin_until_future_complete` 를 부르면 안 됩니다**
  (`Executor is already spinning` 으로 죽습니다). main 에서 순차 루프로 돕니다.
- **시계는 벽시계입니다.** `use_sim_time` 에 묶으면 시뮬레이터가 `/clock` 을
  내보내기 전까지 시간이 흐르지 않아 영원히 대기합니다.
"""

import sys
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from std_srvs.srv import Trigger

DEFAULT_MANAGER = '/lifecycle_manager_navigation'


class Watchdog(Node):

    def __init__(self):
        super().__init__('nav2_startup_watchdog')
        p = self.declare_parameter
        # 감시할 lifecycle_manager. Nav2 말고 도킹 서버용 관리자
        # (lifecycle_manager_docking) 에도 같은 증상이 나므로
        # 인스턴스를 하나 더 띄워 쓸 수 있게 열어 둡니다.
        p('manager', DEFAULT_MANAGER)
        # 처음 활성화를 이만큼 [s] 기다립니다. 코스트맵 초기화까지 포함해야
        # 하므로 넉넉히. 이 PC 에서 정상일 때 약 12초 걸립니다.
        p('startup_timeout', 60.0)
        # 재시도 후 이만큼 [s] 안에 활성이 되어야 합니다.
        p('retry_timeout', 45.0)
        p('max_retries', 3)
        p('poll_period', 2.0)

        g = self.get_parameter
        self.startup_timeout = g('startup_timeout').value
        self.retry_timeout = g('retry_timeout').value
        self.max_retries = int(g('max_retries').value)
        self.period = g('poll_period').value

        self.manager = g('manager').value
        self.is_active = self.create_client(Trigger, self.manager + '/is_active')
        self.manage = self.create_client(
            ManageLifecycleNodes, self.manager + '/manage_nodes')

    def call(self, client, req, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def active(self):
        res = self.call(self.is_active, Trigger.Request(), timeout=3.0)
        return res is not None and res.success

    def restart(self):
        # RESET 을 먼저. 일부만 활성인 상태에서 STARTUP 만 보내면
        # "already active" 로 실패합니다.
        self.call(self.manage, ManageLifecycleNodes.Request(
            command=ManageLifecycleNodes.Request.RESET), timeout=30.0)
        self.call(self.manage, ManageLifecycleNodes.Request(
            command=ManageLifecycleNodes.Request.STARTUP), timeout=60.0)

    def run(self):
        self.get_logger().info('%s 활성화 감시 시작' % self.manager)
        # lifecycle_manager 가 뜨기 전에는 시계를 시작하지 않습니다.
        if not self.is_active.wait_for_service(timeout_sec=120.0):
            self.get_logger().error('%s 를 찾지 못했습니다' % self.manager)
            return 1

        deadline = time.monotonic() + self.startup_timeout
        retries = 0
        while rclpy.ok():
            if self.active():
                self.get_logger().info(
                    '%s 활성 확인 (재시도 %d 회) — 감시 종료'
                    % (self.manager, retries))
                return 0
            if time.monotonic() < deadline:
                time.sleep(self.period)
                continue
            if retries >= self.max_retries:
                self.get_logger().error(
                    '%s 활성화가 %d 회 재시도 후에도 실패했습니다. 남은 '
                    '프로세스가 있는지(ros2 topic info /clock 의 Publisher '
                    'count 가 1인지) 확인하세요.' % (self.manager, retries))
                return 1
            retries += 1
            self.get_logger().warn(
                '%s 활성화 실패 — RESET 후 STARTUP 재시도 (%d/%d)'
                % (self.manager, retries, self.max_retries))
            self.restart()
            deadline = time.monotonic() + self.retry_timeout
        return 0


def main():
    rclpy.init()
    node = Watchdog()
    code = 0
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
