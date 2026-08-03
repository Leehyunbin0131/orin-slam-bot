#!/usr/bin/env python3
"""Registers the dock pose when the robot boots while charging.

    /battery_state --> [ DockRegister ] --> dock database file
                                        --> staged_dock parameters

Moving the station only requires pushing the robot in once and rebooting.

Stores the *docked* pose, not the staging pose -- staged_dock derives staging
from approach_distance, so pre-offsetting it doubles the distance.

Reverse docking stores yaw - 180 deg: dock_yaw is the approach heading, while
the docked robot faces away from the dock.
"""

import math
import os
import threading
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DockRegister(Node):

    def __init__(self):
        super().__init__('dock_register')
        p = self.declare_parameter

        p('dock_id', 'home_dock')
        p('dock_type', 'orinbot_dock')
        p('map_frame', 'map')
        p('base_frame', 'base_footprint')
        # Must match staged_dock.reverse_dock.
        p('reverse_dock', True)
        # Nav2 dock_database format, so docking_mode:=smooth can reuse it.
        p('database_path', os.path.expanduser('~/.ros/orinbot_docks.yaml'))

        p('auto_register', True)          # off = ~/register service only
        # Contact is physical, so judge it by charge current, not by pose.
        p('charge_threshold', 0.5)        # [A]
        # Never judge on the first sample -- contact inputs are not up yet at
        # boot and the battery reads as discharging for a moment.
        p('charge_wait', 20.0)            # [s]
        p('settle_tolerance', 0.01)       # [m]
        # Yaw settles separately: it becomes the approach heading.
        p('settle_tolerance_yaw', 0.0087)  # [rad] 0.5 deg
        p('settle_samples', 10)
        p('settle_timeout', 120.0)        # [s]
        p('target_nodes', ['/staged_dock'])

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.dock_id = g('dock_id')
        self.dock_type = g('dock_type')
        self.map_frame = g('map_frame')
        self.base = g('base_frame')
        self.reverse = g('reverse_dock')
        self.db_path = os.path.expanduser(g('database_path'))
        self.auto = g('auto_register')
        self.charge_thr = g('charge_threshold')
        self.charge_wait = g('charge_wait')
        self.settle_tol = g('settle_tolerance')
        self.settle_tol_yaw = g('settle_tolerance_yaw')
        self.settle_n = int(g('settle_samples'))
        self.settle_timeout = g('settle_timeout')
        self.targets = [t for t in g('target_nodes') if t]

        # Registration blocks for seconds waiting on TF. Without a reentrant
        # group and a multi-threaded executor that wait starves the TF and
        # service callbacks it depends on.
        self.cb = ReentrantCallbackGroup()
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.current = None
        self.first_battery = None
        self.create_subscription(BatteryState, 'battery_state',
                                 self._battery, 10, callback_group=self.cb)
        self.create_service(Trigger, '~/register', self._srv_register,
                            callback_group=self.cb)

        # Reentrant group: timers overlap. Claim under the lock before blocking.
        self.lock = threading.Lock()
        self.busy = False
        self.done = False
        if self.auto:
            self.timer = self.create_timer(2.0, self._try_auto,
                                           callback_group=self.cb)

        self.get_logger().info(
            '도크 등록 대기 — 자동 %s / DB %s / 후진 %s'
            % ('켬' if self.auto else '끔', self.db_path, self.reverse))

    # ------------------------------------------------------------------
    def _battery(self, msg):
        if self.first_battery is None:
            self.first_battery = time.time()
        self.current = msg.current

    def _srv_register(self, _req, res):
        if not self._claim():
            res.success, res.message = False, '등록이 이미 진행 중입니다'
            return res
        try:
            ok, msg = self.register(force=True)
        finally:
            self._release()
        res.success, res.message = ok, msg
        return res

    def _claim(self):
        with self.lock:
            if self.busy:
                return False
            self.busy = True
            return True

    def _release(self):
        with self.lock:
            self.busy = False

    def _try_auto(self):
        if self.done or self.current is None:
            return
        if self.current < self.charge_thr:
            if time.time() - self.first_battery < self.charge_wait:
                return
            self.get_logger().info(
                '충전 중이 아니므로 자동 등록하지 않습니다 (%.0f초 동안 전류가 '
                '%+.2f A 에 머물렀습니다). 수동 등록은 ~/register 서비스입니다.'
                % (self.charge_wait, self.current))
            self.done = True
            self.timer.cancel()
            return
        if not self._claim():
            return
        try:
            ok, msg = self.register(force=False)
        finally:
            self._release()
        if ok:
            self.done = True
            self.timer.cancel()
        else:
            self.get_logger().warning('등록 보류 — %s' % msg)

    # ------------------------------------------------------------------
    def _settled_pose(self):
        """Return map->base once SLAM has settled."""
        hist = []
        t0 = time.time()
        while time.time() - t0 < self.settle_timeout:
            try:
                t = self.buf.lookup_transform(
                    self.map_frame, self.base, rclpy.time.Time()).transform
            except Exception:                              # noqa: BLE001
                time.sleep(0.3)
                continue
            hist.append((t.translation.x, t.translation.y, yaw_of(t.rotation)))
            if len(hist) > self.settle_n:
                hist.pop(0)
            if len(hist) == self.settle_n:
                xs = [h[0] for h in hist]
                ys = [h[1] for h in hist]
                # Relative to the first sample: yaw can wrap at +-pi.
                ds = [wrap(h[2] - hist[0][2]) for h in hist]
                if (max(xs) - min(xs) < self.settle_tol
                        and max(ys) - min(ys) < self.settle_tol
                        and max(ds) - min(ds) < self.settle_tol_yaw):
                    return hist[-1]
            time.sleep(0.3)
        return None

    def register(self, force):
        if self.current is None and not force:
            return False, '/battery_state 를 아직 못 받았습니다'
        if not force and (self.current or 0.0) < self.charge_thr:
            return False, '충전 중이 아닙니다 (전류 %+.2f A)' % (self.current or 0.0)

        pose = self._settled_pose()
        if pose is None:
            return False, '%s->%s 자세가 안정되지 않았습니다' % (self.map_frame, self.base)
        x, y, yaw_docked = pose

        # Reverse docking: the docked pose is opposite the approach heading.
        dock_yaw = wrap(yaw_docked - math.pi) if self.reverse else yaw_docked

        try:
            self._write_db(x, y, dock_yaw)
        except Exception as exc:                           # noqa: BLE001
            return False, 'DB 기록 실패: %s' % exc

        self.get_logger().info(
            '도크 등록 — 위치 (%.3f, %.3f) / 도킹 시 자세 %+.1f도 '
            '-> 접근 방향 %+.1f도%s'
            % (x, y, math.degrees(yaw_docked), math.degrees(dock_yaw),
               ' (후진이라 180도 반대)' if self.reverse else ''))
        self.get_logger().info('DB 기록 완료: %s' % self.db_path)
        self._push_params(x, y, dock_yaw)
        return True, '(%.3f, %.3f, %.4f)' % (x, y, dock_yaw)

    def _write_db(self, x, y, yaw):
        """Write in Nav2 dock_database format (docks/type/frame/pose)."""
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        tmp = self.db_path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write('# dock_register.py 가 자동 생성했습니다. 직접 고치지 마세요.\n')
            f.write('# pose 는 [x, y, yaw] 이고 yaw 는 **접근 방향**입니다\n')
            f.write('# (후진 도킹이면 도킹 완료 자세는 여기서 180도 돌아간 상태).\n')
            f.write('docks:\n')
            f.write('  %s:\n' % self.dock_id)
            f.write('    type: %s\n' % self.dock_type)
            f.write('    frame: %s\n' % self.map_frame)
            f.write('    pose: [%.4f, %.4f, %.4f]\n' % (x, y, yaw))
        os.replace(tmp, self.db_path)      # atomic: never leaves a partial file

    def _push_params(self, x, y, yaw):
        """Push the pose to running nodes without restarting them."""
        for node in self.targets:
            cli = self.create_client(SetParameters, '%s/set_parameters' % node,
                                     callback_group=self.cb)
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning('%s 없음 — 파라미터 반영 건너뜀' % node)
                continue
            req = SetParameters.Request()
            for name, val in (('dock_x', x), ('dock_y', y), ('dock_yaw', yaw)):
                req.parameters.append(Parameter(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=val)))
            fut = cli.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 5.0:
                time.sleep(0.05)
            res = fut.result()
            ok = res is not None and all(r.successful for r in res.results)
            self.get_logger().info(
                '%s 파라미터 반영 %s' % (node, '완료' if ok else '실패'))
            self.destroy_client(cli)


def main():
    rclpy.init()
    n = DockRegister()
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
