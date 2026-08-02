#!/usr/bin/env python3
"""Staged control based docking and undocking server.

    /dock_robot   (nav2_msgs/DockRobot)
    /undock_robot (nav2_msgs/UndockRobot)

Complies with opennav_docking interface standard while maintaining stationary vision measurement precision.
Executes lateral crab maneuvers and final straight-line entry.
"""

import math
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  (PoseStamped transform registration)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty

# DockRobot.Feedback status values (matching opennav_docking spec)
FB_NAV, FB_PERCEIVE, FB_CONTROL, FB_RETRY = 1, 2, 3, 5


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class StagedDock(Node):

    def __init__(self):
        super().__init__('staged_dock')
        p = self.declare_parameter
        self.cb = ReentrantCallbackGroup()

        # Target dock pose (must match home_dock.pose in docking.yaml)
        p('dock_id', 'home_dock')
        p('dock_x', 1.0)
        p('dock_y', -3.60)
        p('dock_yaw', -1.5708)
        # 마커면에서 최종 도킹 자세까지 [m]
        p('dock_distance', 0.224)
        # 후진 도킹: 정렬은 도크를 마주 본 채로 하고, 회전점에서 180도 돌아
        # 뒤로 들어갑니다. 충전 내내 카메라가 벽이 아니라 방을 보게 하려는
        # 것입니다 — 벽 0.3 m 앞에서는 시각 오도메트리가 깨집니다.
        p('reverse_dock', True)
        # 180도 회전 지점까지의 거리 [m]. 제자리 회전에 외접반경 0.283 m 가
        # 필요하므로 이보다 가까이서 돌면 패널을 칩니다.
        p('rotate_distance', 0.30)
        # Distance from marker surface to approach staging pose [m]
        p('approach_distance', 0.65)

        # Alignment criteria
        # Budgeted contact lateral error [m] (half of physical copper pad tolerance +-34mm)
        p('contact_lateral_budget', 0.015)
        # Yaw threshold limit [rad] for error anomaly monitoring
        p('yaw_tolerance', 0.0175)        # 1.0 deg
        p('max_align_iters', 6)
        # 마커가 안 보일 때 한 번에 물러나는 거리 [m]
        p('search_backoff', 0.15)
        # Minimum standoff distance from dock station
        p('min_standoff', 0.62)

        # Maneuver parameters
        p('crab_angle', 0.5236)           # 30 deg
        p('v_rotate', 0.35)               # [rad/s]
        p('v_forward', 0.08)              # [m/s]
        p('settle_time', 0.7)             # Settling time after stopping [s]
        p('measure_samples', 12)
        p('measure_timeout', 5.0)
        p('undock_distance', 0.5)         # Backing out distance [m]

        # 도킹 구간에 멈출 것들 (std_srvs/Empty). 실기에서 카메라 스트림을
        # 끊는 서비스가 생기면 여기에 이름만 추가하면 됩니다.
        p('slam_pause_services', ['/rtabmap/pause', '/vodom_tf_relay/pause'])
        p('slam_resume_services', ['/rtabmap/resume', '/vodom_tf_relay/resume'])

        p('base_frame', 'base_footprint')
        p('odom_frame', 'odom')
        p('map_frame', 'map')
        # Marker pose to dock pose rotation pitch [rad]
        p('marker_pitch', 1.5708)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.dock_id = g('dock_id')
        self.dock_pose = (g('dock_x'), g('dock_y'), g('dock_yaw'))
        self.D = g('dock_distance')
        self.reverse = g('reverse_dock')
        self.R = g('rotate_distance')
        # 정렬 뒤 직진해서 멈출 지점 — 후진 도킹이면 회전점입니다.
        self.stop_at = self.R if self.reverse else self.D
        self.A = g('approach_distance')
        self.yaw_tol = g('yaw_tolerance')
        self.budget = g('contact_lateral_budget')
        self.max_iters = int(g('max_align_iters'))
        self.search_back = g('search_backoff')
        self.min_standoff = g('min_standoff')
        self.crab = g('crab_angle')
        self.w_rot = g('v_rotate')
        self.v_fwd = g('v_forward')
        self.settle = g('settle_time')
        self.n_samples = int(g('measure_samples'))
        self.meas_timeout = g('measure_timeout')
        self.undock_d = g('undock_distance')
        self.base = g('base_frame')
        self.odom = g('odom_frame')
        self.map_frame = g('map_frame')
        self.mpitch = g('marker_pitch')
        self.slam_pause = [x for x in g('slam_pause_services') if x]
        self.slam_resume = [x for x in g('slam_resume_services') if x]

        from tf2_ros import Buffer, TransformListener
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

        self.marker = None
        self.create_subscription(PoseStamped, 'detected_dock_pose',
                                 self._marker, 10, callback_group=self.cb)
        self.cmd = self.create_publisher(TwistStamped, 'cmd_vel_dock', 10)

        self.nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                   callback_group=self.cb)

        self.dock_as = ActionServer(
            self, DockRobot, 'dock_robot',
            execute_callback=self._do_dock,
            goal_callback=lambda _gh: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cb)

        self.undock_as = ActionServer(
            self, UndockRobot, 'undock_robot',
            execute_callback=self._do_undock,
            goal_callback=lambda _gh: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cb)

        self.get_logger().info(
            'Staged dock server started: staging %.2f m, dock %.3f m, '
            'lateral budget %.0f mm (yaw limit %.1f deg)'
            % (self.A, self.D, self.budget * 1000, math.degrees(self.yaw_tol)))

    # ---------------- Inputs ----------------
    def _marker(self, msg):
        self.marker = msg

    def _drive(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.cmd.publish(m)

    def _stop(self):
        self._drive(0.0, 0.0)

    # ---------------- Measurement ----------------
    def measure(self):
        """Measure dock pose in stationary state -> (forward dist, lateral error, yaw error)."""
        self.marker = None
        t0 = time.time()
        samples = []
        while len(samples) < self.n_samples and time.time() - t0 < self.meas_timeout:
            time.sleep(0.05)
            if self.marker is None:
                continue
            b = _stamp0(self.marker)
            try:
                b = self.buf.transform(b, self.base, timeout=rclpy.duration.Duration(seconds=0.1))
            except Exception:
                continue
            psi = wrap(yaw_of(_rot_pitch(b.pose.orientation, self.mpitch)))
            mx, my = b.pose.position.x, b.pose.position.y
            lat = mx * math.sin(psi) - my * math.cos(psi)
            samples.append((mx, my, psi, lat))
        if len(samples) < 3:
            return None
        n = len(samples)
        mx = sum(s[0] for s in samples) / n
        my = sum(s[1] for s in samples) / n
        psi = sum(s[2] for s in samples) / n
        lat = sum(s[3] for s in samples) / n
        fwd = mx * math.cos(psi) + sum(s[1] for s in samples) / n * math.sin(psi)
        return fwd, lat, psi, n

    def predict(self, fwd, lat, psi):
        """Predict expected contact lateral error [m] when proceeding straight."""
        return lat - max(0.0, fwd - self.stop_at) * math.sin(psi)

    # ---------------- Maneuver ----------------
    def rotate(self, delta):
        """In-place rotation using odometry feedback."""
        if abs(delta) < 1e-4:
            return True
        x0, y0, a0 = self._odom_pose()
        target = wrap(a0 + delta)
        t0 = time.time()
        while time.time() - t0 < 15.0:
            _, _, a = self._odom_pose()
            err = wrap(target - a)
            if abs(err) < 0.004:
                break
            w = max(0.05, min(self.w_rot, abs(err) * 1.5)) * (1 if err > 0 else -1)
            self._drive(0.0, w)
            time.sleep(0.02)
        self._stop()
        time.sleep(self.settle)
        return True

    def forward(self, dist):
        """Straight movement using odometry feedback."""
        if abs(dist) < 1e-4:
            return True
        x0, y0, _ = self._odom_pose()
        t0 = time.time()
        sign = 1 if dist > 0 else -1
        target = abs(dist)
        while time.time() - t0 < 30.0:
            x, y, _ = self._odom_pose()
            traveled = math.hypot(x - x0, y - y0)
            err = target - traveled
            if err <= 0.002:
                break
            v = max(0.02, min(self.v_fwd, err * 1.0)) * sign
            self._drive(v, 0.0)
            time.sleep(0.02)
        self._stop()
        time.sleep(self.settle)
        return True

    def crab_move(self, shift):
        """Perform crab maneuver to shift laterally by shift amount."""
        s = abs(shift) / math.sin(self.crab)
        turn = -self.crab if shift > 0 else self.crab
        self.rotate(turn)
        self.forward(-s)
        self.rotate(-turn)

    # ---------------- Actions ----------------
    def _do_dock(self, handle):
        r = DockRobot.Result()
        fb = DockRobot.Feedback()

        def emit(state):
            fb.state = state
            handle.publish_feedback(fb)

        goal = handle.request
        self.get_logger().info('Starting staged dock (dock_id=%s)' % goal.dock_id)

        # Stage 1: Move to staging pose
        if goal.navigate_to_staging_pose:
            emit(FB_NAV)
            if not self._goto_entry():
                r.success, r.error_code = False, DockRobot.Result.FAILED_TO_STAGE
                r.error_msg = 'Failed to reach staging pose'
                handle.abort()
                return r

        # Nav2 의 각도를 믿으면 안 됩니다. 언도킹 직후에는 로봇이 이미 진입점
        # 0.1 m 안에 있어 위치 조건만으로 목표가 즉시 성공 처리되고, 도크를
        # 등진 채로 정렬이 시작돼 마커를 한 장도 못 봅니다. 여기서 지도
        # 기준 방위를 직접 확인하고 오도메트리로 돌려 세웁니다.
        self._face_dock()
        self._restore_standoff()

        # 여기서부터 지도 갱신과 시각 오도메트리를 얼립니다.
        # 카메라가 벽을 0.3~0.9 m 앞에서 보며 제자리 회전과 크랩을 반복하는
        # 구간이라 시각 오도메트리가 깨지고, 그 오차가 포즈 그래프에 그대로
        # 들어갑니다 (실측: 도킹 한 사이클에 자세 오차 2 mm -> 935 mm).
        # 도킹은 odom 기준이고 이동량도 1 m 미만이라 휠+IMU 로 충분합니다.
        self._freeze_slam(True)

        # Stages 2-4: Measure -> Align yaw -> Crab correction -> Remeasure
        emit(FB_PERCEIVE)
        aligned = None
        prev_pred = None

        for i in range(self.max_iters):
            if handle.is_cancel_requested:
                handle.canceled()
                return r
            m = self.measure()
            if m is None:
                # 지도 좌표로 "충분히 멀다"고 판단하면 안 됩니다 — SLAM 오차가
                # 수십 cm 면 실제로는 검출 절벽(0.55 m) 안쪽에 서 있게 됩니다
                # (실측: 지도 0.70 m 인데 실제 0.53 m). 도크를 마주 본 상태이므로
                # 뒤로 물러나면 마커가 화각에 다시 들어옵니다.
                self.get_logger().warning(
                    '마커 미검출 (%d회) — %.2f m 물러나 다시 봅니다'
                    % (i + 1, self.search_back))
                emit(FB_RETRY)
                self.forward(-self.search_back)
                continue
            fwd, lat, psi, n = m
            pred = self.predict(fwd, lat, psi)
            self.get_logger().info(
                '  [%d] fwd %.3f m / lat %+.1f mm / yaw %+.2f deg '
                '-> predicted contact %+.1f mm (samples %d)'
                % (i + 1, fwd, lat * 1000, math.degrees(psi),
                   pred * 1000, n))
            if abs(pred) <= self.budget and abs(psi) <= self.yaw_tol:
                aligned = m
                break
            if abs(psi) > self.yaw_tol:
                self.rotate(psi)
                continue
            if prev_pred is not None and abs(pred) > abs(prev_pred) + 0.002:
                self.get_logger().error(
                    'Crab correction increased error (%.1f -> %.1f mm). Aborting alignment.'
                    % (prev_pred * 1000, pred * 1000))
                break
            prev_pred = pred
            self.crab_move(-pred)

        if aligned is None:
            r.success, r.error_code = False, DockRobot.Result.FAILED_TO_DETECT_DOCK
            r.error_msg = 'Dock detection failed during alignment'
            self._freeze_slam(False)     # 실패해도 반드시 되돌립니다
            handle.abort()
            return r

        # Stage 5: Straight entry
        emit(FB_CONTROL)
        fwd, lat, psi, _ = aligned

        if fwd < self.min_standoff:
            back = self.min_standoff - fwd
            self.get_logger().info(
                'Distance to dock %.3f m < min standoff, backing out %.3f m'
                % (fwd, back))
            self.forward(-back)
            m = self.measure()
            if m is not None:
                fwd, lat, psi, _ = m
                pred = self.predict(fwd, lat, psi)
                if abs(pred) > self.budget:
                    self.get_logger().info(
                        'Re-correcting lateral drift after backout (%+.1f mm)'
                        % (pred * 1000))
                    self.crab_move(-pred)
                    m = self.measure()
                    if m is not None:
                        fwd, lat, psi, _ = m

        run = max(0.0, fwd - self.stop_at)
        self.get_logger().info(
            'Alignment complete -- standoff %.3f m / lat %+.1f mm / yaw %+.2f deg '
            '-> predicted contact %+.1f mm (budget %.0f mm). '
            'Executing final %.3f m straight run'
            % (fwd, lat * 1000, math.degrees(psi),
               self.predict(fwd, lat, psi) * 1000, self.budget * 1000, run))
        if run > 0.001:
            self.forward(run)

        if self.reverse:
            # 회전점에 섰습니다. 여기서 180도 돌고 짧게 후진해 들어갑니다.
            # 회전은 마커가 아니라 오도메트리로 닫습니다 — 돌고 나면 마커가
            # 뒤에 있어 볼 수 없습니다. 짧은 회전의 휠 오도메트리는 0.23도
            # 분해능이고, 남은 후진이 0.08 m 라 흘러감은 mm 수준입니다.
            back = max(0.0, self.R - self.D)
            self.get_logger().info(
                '회전점 도달 — 180도 회전 후 %.3f m 후진합니다' % back)
            self.rotate(math.pi)
            if back > 0.001:
                self.forward(-back)

        r.success, r.error_code = True, 0
        handle.succeed()
        self.get_logger().info('Docking complete')
        return r

    def _do_undock(self, handle):
        r = UndockRobot.Result()
        # 도킹 때 얼렸더라도 auto_dock 의 절전 해제가 먼저 풀어 놓았을 수 있어
        # 여기서 다시 겁니다. 후진 구간도 카메라가 벽을 보고 있습니다.
        self._freeze_slam(True)
        # 후진 도킹이면 로봇이 이미 벽을 등지고 있으므로 전진이 이탈입니다.
        out = self.undock_d if self.reverse else -self.undock_d
        self.get_logger().info(
            '언도킹 — %s %.2f m' % ('전진' if out > 0 else '후진', abs(out)))
        self.forward(out)
        self._stop()
        self._freeze_slam(False)
        r.success, r.error_code = True, 0
        handle.succeed()
        self.get_logger().info('Undocking complete')
        return r

    def _restore_standoff(self):
        """도크에 너무 붙어 있으면 물러섭니다.

        마커 3장이 다 보이는 한계가 0.55 m 부근이라, 앞선 시도가 실패해
        로봇이 도크 앞에 박힌 채 남으면 이후 모든 시도가 검출조차 못 합니다.
        """
        try:
            t = self.buf.lookup_transform(
                self.map_frame, self.base, rclpy.time.Time()).transform
        except Exception:                                  # noqa: BLE001
            return
        dx, dy, dyaw = self.dock_pose
        d = math.hypot(t.translation.x - dx, t.translation.y - dy) + self.D
        if d >= self.A:
            return
        back = self.A - d + 0.05
        self.get_logger().info(
            '도크에서 %.2f m 뿐이라 %.2f m 물러섭니다 (정렬에 %.2f m 필요)'
            % (d, back, self.A))
        self.forward(-back)

    def _face_dock(self):
        """도크를 마주 보도록 제자리에서 돌립니다 (지도 기준 방위 사용)."""
        want = self.dock_pose[2]
        for _ in range(3):
            try:
                t = self.buf.lookup_transform(
                    self.map_frame, self.base, rclpy.time.Time()).transform
            except Exception:                              # noqa: BLE001
                time.sleep(0.3)
                continue
            err = wrap(want - yaw_of(t.rotation))
            if abs(err) < math.radians(5.0):
                return True
            self.get_logger().info(
                '도크 방향으로 %+.1f도 회전' % math.degrees(err))
            self.rotate(err)
            return True
        self.get_logger().warning('map->base 조회 실패 — 방위 보정을 건너뜁니다')
        return False

    # ------------------------------------------------------------ 지도 동결
    def _freeze_slam(self, on):
        """도킹 구간 동안 포즈 그래프와 시각 오도메트리 보정을 멈춥니다."""
        for name in (self.slam_pause if on else self.slam_resume):
            cli = self.create_client(Empty, name)
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning('%s 없음 — 건너뜁니다' % name)
                continue
            fut = cli.call_async(Empty.Request())
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 3.0:
                time.sleep(0.02)
        self.get_logger().info('SLAM %s' % ('동결' if on else '재개'))

    def _goto_entry(self):
        """Navigate to approach staging pose."""
        dx, dy, dyaw = self.dock_pose
        back = self.A - self.D
        ex = dx - back * math.cos(dyaw)
        ey = dy - back * math.sin(dyaw)

        g = NavigateToPose.Goal()
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.header.frame_id = self.map_frame
        g.pose.pose.position.x = ex
        g.pose.pose.position.y = ey
        g.pose.pose.position.z = 0.0
        g.pose.pose.orientation.z = math.sin(dyaw / 2.0)
        g.pose.pose.orientation.w = math.cos(dyaw / 2.0)

        if not self.nav_ac.wait_for_server(timeout_sec=5.0):
            return False
        fut = self.nav_ac.send_goal_async(g)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not fut.done() or not fut.result().accepted:
            return False
        gh = fut.result()
        rfut = gh.get_result_async()
        t0 = time.time()
        while not rfut.done() and time.time() - t0 < 120.0:
            time.sleep(0.1)
        if not rfut.done():
            return False
        res = rfut.result()
        return res is not None and res.status == GoalStatus.STATUS_SUCCEEDED

    def _odom_pose(self):
        try:
            tf = self.buf.lookup_transform(
                self.odom, self.base, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            p = tf.transform.translation
            r = tf.transform.rotation
            return p.x, p.y, yaw_of(r)
        except Exception:
            return 0.0, 0.0, 0.0


def _stamp0(msg):
    out = PoseStamped()
    out.header.frame_id = msg.header.frame_id
    out.pose = msg.pose
    return out


def _rot_pitch(q, pitch):
    """마커 자세에 y축 회전을 곱합니다: q (x) q_pitch.

    마커 광학 규약(z 가 마커 밖)에서 도크 규약(x 가 도크 정면)으로 옮기는
    변환이라, 축을 틀리면 yaw 에 90도가 통째로 섞여 들어갑니다.
    """
    ey, ew = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    return Quaternion(
        x=q.x * ew - q.z * ey,
        y=q.y * ew + q.w * ey,
        z=q.z * ew + q.x * ey,
        w=q.w * ew - q.y * ey)


def main():
    rclpy.init()
    n = StagedDock()
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
