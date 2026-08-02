#!/usr/bin/env python3
"""배터리 시뮬레이터 — 실기 BMS 드라이버의 자리를 메웁니다.

이 노드는 **시뮬레이션 전용**입니다. 실기에서는 BMS 가 같은 토픽
(`/battery_state`, sensor_msgs/BatteryState)에 실제 전압·전류를 내보내고
이 노드는 실행하지 않습니다. 아래를 쓰는 쪽은 전부 그 토픽만 보므로
바꿔 끼우면 그대로 동작합니다.

  - docking_server 의 SimpleChargingDock: `use_battery_status: true` 로
    두면 전류가 charging_threshold 를 넘는지로 "충전 중"을 판정합니다.
  - auto_dock.py: 잔량이 임계값 밑으로 내려가면 도크로 복귀시킵니다.

충전 여부 판정
--------------
실기는 접점에 전기가 흐르는지로 압니다. 시뮬레이터에는 전기가 없으므로
"동판 앞에 제대로 서 있는가"를 기하로 대신 판정합니다.

**SLAM 자세를 쓰면 안 됩니다.** 접촉은 물리적 사실인데 SLAM 자세에는
21 mm(90% 46 mm) 오차가 있습니다. SLAM 자세 + 여유 0.15 m 로 두었더니
아직 100 mm 앞인데 "충전 시작"이 되었고, `docking_server` 의 접근 루프는
isCharging 으로도 빠져나오므로 접근이 거기서 끝났습니다(세로 오차 119 mm).
그래서 Gazebo 지상 진실(`/ground_truth/odom`)을 씁니다.

허용치는 **핀 배열**이 동판 위에 얹히는 범위입니다. 브래킷(30 mm 각)이
아닙니다 — 커넥터가 6핀 2x3 / 피치 2.54 mm 라 닿아야 하는 면적은
6.1 x 3.5 mm 뿐이고, 브래킷은 동판 밖으로 걸쳐도 무방합니다. 브래킷
기준으로 재면 닿아 있는데 "충전 안 됨"이 됩니다.
  세로 ±48 mm / 가로 ±34 mm  = (동판 100x75 - 핀배열 3.5x6.1) / 2
  각도 ±6.3도 — 가이드 벽이 없어 물리적으로 막아 주는 것이 없습니다.
                축을 따로 보는 근사라, 각도가 크면 가로 여유를 잡아먹습니다.

전류 모델은 Orin + 모터 2개 + 센서 기준 대략치입니다. 정확한 값이 아니라
"움직이면 더 닳는다"는 성질이 목적입니다.
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
