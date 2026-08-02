#!/usr/bin/env python3
"""배터리 상태 시뮬레이터 노드 (sensor_msgs/BatteryState).

시뮬레이션 환경에서 배터리 방전, 충전 및 Gazebo 참값 기준 도크 접촉 판정을 수행합니다.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


class BatterySim(Node):

    def __init__(self):
        super().__init__('battery_sim')
        p = self.declare_parameter

        p('publish_rate', 1.0)
        p('capacity_ah', 10.0)          # 24 V 10 Ah = 240 Wh
        p('voltage_full', 29.4)         # 7S 리튬이온 만충
        p('voltage_empty', 21.0)        # 방전 하한
        p('initial_soc', 0.85)

        # 소비 전류 [A]
        p('idle_current', 1.25)              # 컴퓨트 + 센서 (약 30 W)
        p('current_per_mps', 3.0)            # 직진 속도에 비례
        p('current_per_radps', 0.6)          # 회전 속도에 비례
        p('charge_current', 3.0)             # 충전 (약 3.3 시간)
        # 만충 근처에서 전류를 줄입니다. 이걸 두지 않으면 100% 에
        # 도달한 뒤에도 전류가 그대로라 SimpleChargingDock 이
        # 계속 "충전 중" 으로 보고, auto_dock 이 출발 시점을
        # 판단하지 못합니다.
        p('taper_from_soc', 0.95)

        # 시간 배속. 실제 방전은 몇 시간이 걸려 시뮬레이션 검증에
        # 쓸 수 없습니다. 1.0 이 물리적으로 정직한 값이고, 도킹
        # 시나리오를 돌릴 때만 올려 씁니다 (tools/dock_test.py).
        p('speedup', 1.0)

        # 도크 위치 — docking.yaml 의 home_dock.pose 와 같아야 합니다
        p('dock_x', 1.0)
        p('dock_y', -3.60)
        p('dock_yaw', -1.5708)
        # 접촉 허용치 (위 설명의 유도 결과)
        p('contact_tolerance_lon', 0.048)     # (동판 길이 100 - 핀 3.5)/2
        p('contact_tolerance_lat', 0.034)     # (동판 폭 75 - 핀 6.1)/2
        p('contact_tolerance_yaw', 0.11)      # 6.3도
        # Gazebo 지상 진실. URDF 의 OdometryPublisher + gz_bridge.yaml.
        p('ground_truth_topic', '/ground_truth/odom')
        p('base_frame', 'base_footprint')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cap_ah = g('capacity_ah')
        self.v_full, self.v_empty = g('voltage_full'), g('voltage_empty')
        self.i_idle = g('idle_current')
        self.i_mps, self.i_radps = g('current_per_mps'), g('current_per_radps')
        self.i_charge = g('charge_current')
        self.taper = g('taper_from_soc')
        self.speedup = g('speedup')
        self.dock = (g('dock_x'), g('dock_y'), g('dock_yaw'))
        self.tol_lon = g('contact_tolerance_lon')
        self.tol_lat = g('contact_tolerance_lat')
        self.tol_yaw = g('contact_tolerance_yaw')
        self.base_frame = g('base_frame')

        self.soc = max(0.0, min(1.0, g('initial_soc')))
        self.v, self.w = 0.0, 0.0
        self.last = None
        self.truth = None            # (x, y, yaw)
        self.warned_no_truth = False

        self.create_subscription(Odometry, '/odometry/filtered', self._odom, 10)
        self.create_subscription(Odometry, g('ground_truth_topic'),
                                 self._truth, 10)
        # 시험용 주입구. 방전을 몇 시간 기다리지 않고 원하는 잔량에서
        # 시나리오를 시작할 수 있게 합니다.
        self.create_subscription(Float32, '~/set_soc', self._set_soc, 1)
        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)

        self.create_timer(1.0 / g('publish_rate'), self._tick)
        self.get_logger().info(
            '배터리 시뮬레이터 시작: %.0f%% / %.1f Ah / 배속 %.0fx / '
            '도크 (%.2f, %.2f) / 접촉 허용 세로 %.0fmm 가로 %.0fmm 각도 %.1f도'
            % (self.soc * 100, self.cap_ah, self.speedup, self.dock[0],
               self.dock[1], self.tol_lon * 1000, self.tol_lat * 1000,
               math.degrees(self.tol_yaw)))

    def _odom(self, m):
        self.v = m.twist.twist.linear.x
        self.w = m.twist.twist.angular.z

    def _set_soc(self, m):
        self.soc = max(0.0, min(1.0, float(m.data)))
        self.get_logger().warning('잔량을 %.0f%% 로 강제 설정' % (self.soc * 100))

    def _truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.truth = (p.x, p.y,
                      math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _on_dock(self):
        if self.truth is None:
            if not self.warned_no_truth:
                self.warned_no_truth = True
                self.get_logger().warning(
                    '지상 진실 자세(%s)를 못 받고 있습니다 — 충전 판정이 '
                    '안 됩니다. URDF 의 OdometryPublisher 플러그인과 '
                    'gz_bridge.yaml 항목을 확인하세요.'
                    % self.get_parameter('ground_truth_topic').value)
            return False
        x, y, yaw = self.truth
        # 도크 좌표계로 옮겨 세로/가로를 분리합니다. 둘의 허용치가 다릅니다
        # (세로는 접점 깊이, 가로는 +/- 동판 단락 한계).
        dx, dy = x - self.dock[0], y - self.dock[1]
        c, s = math.cos(self.dock[2]), math.sin(self.dock[2])
        lon = dx * c + dy * s
        lat = -dx * s + dy * c
        dyaw = math.atan2(math.sin(yaw - self.dock[2]),
                          math.cos(yaw - self.dock[2]))
        return (abs(lon) <= self.tol_lon and abs(lat) <= self.tol_lat
                and abs(dyaw) <= self.tol_yaw)

    def _tick(self):
        now = self.get_clock().now()
        if self.last is None:
            self.last = now
            return
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        # 시뮬레이터가 되감기면(재기동) 음수가 나올 수 있습니다
        if dt <= 0.0 or dt > 5.0:
            return

        docked = self._on_dock()
        if docked:
            # 만충에 가까워지면 전류를 줄입니다 (CV 구간 흉내)
            k = 1.0
            if self.soc > self.taper:
                k = max(0.02, (1.0 - self.soc) / (1.0 - self.taper))
            current = self.i_charge * k
        else:
            current = -(self.i_idle
                        + self.i_mps * abs(self.v)
                        + self.i_radps * abs(self.w))

        # Ah 적산. current [A] * dt [s] / 3600 = Ah
        self.soc += current * dt * self.speedup / 3600.0 / self.cap_ah
        self.soc = max(0.0, min(1.0, self.soc))

        m = BatteryState()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.base_frame
        m.voltage = self.v_empty + (self.v_full - self.v_empty) * self.soc
        m.temperature = 27.0
        # ROS 규약: 방전이 음수, 충전이 양수입니다.
        # SimpleChargingDock 은 current > charging_threshold 로 판정합니다.
        m.current = float(current)
        m.charge = float(self.cap_ah * self.soc)
        m.capacity = float(self.cap_ah)
        m.design_capacity = float(self.cap_ah)
        m.percentage = float(self.soc)
        if docked:
            m.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_FULL
                                     if self.soc >= 0.999
                                     else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        else:
            m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        m.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        m.present = True
        self.pub.publish(m)


def main():
    rclpy.init()
    n = BatterySim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
