#!/usr/bin/env python3
"""단계별 정지-측정-보정(Staged) 제어 기반 충전 도킹 및 언도킹 서버.

    /dock_robot   (nav2_msgs/DockRobot)
    /undock_robot (nav2_msgs/UndockRobot)

opennav_docking 인터페이스 표준을 준수하며 정지 상태 비전 측정 정확도를 유지한 상태에서 횡방향 보정(Crab maneuver) 및 최종 직진 진입을 수행합니다.
"""

import math
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  (PoseStamped 변환 등록)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# DockRobot.Feedback 의 상태값 (opennav_docking 과 같은 의미로 씁니다)
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

        # 도크 위치 (docking.yaml 의 home_dock.pose 와 같아야 합니다)
        p('dock_id', 'home_dock')
        p('dock_x', 1.0)
        p('dock_y', -3.60)
        p('dock_yaw', -1.5708)
        # 마커면에서 도킹 완료 자세까지 [m]
        p('dock_distance', 0.294)
        # 마커면에서 진입점까지 [m]. 여기서 정지해 정렬합니다.
        p('approach_distance', 0.65)

        # 정렬 판정 (위 "무엇을 정렬 완료로 볼 것인가" 참고)
        # 접촉 시점의 예상 횡오차 예산 [m]. 동판 허용치 ±34 mm 의 절반.
        p('contact_lateral_budget', 0.015)
        # 각도 상한 [rad]. 정밀도가 아니라 측정 이상 감시용입니다.
        p('yaw_tolerance', 0.0175)        # 1.0도
        p('max_align_iters', 4)
        # 정렬이 끝난 뒤 스테이션과 최소한 이만큼 떨어져 있어야 합니다.
        # 후진 크랩이라 보통 저절로 만족하지만, 진입 자체가 너무 가까웠던
        # 경우를 위한 보호입니다. 마커 3장이 모두 보이는 한계가 0.55 m
        # 부근이라 그보다 여유 있게 잡습니다.
        p('min_standoff', 0.62)

        # 기동 파라미터
        p('crab_angle', 0.5236)           # 30도
        p('v_rotate', 0.35)               # [rad/s]
        p('v_forward', 0.08)              # [m/s]
        p('settle_time', 0.7)             # 정지 후 안정화 대기 [s]
        p('measure_samples', 12)
        p('measure_timeout', 5.0)
        p('undock_distance', 0.5)         # 후진 이탈 거리 [m]

        p('base_frame', 'base_footprint')
        p('odom_frame', 'odom')
        p('map_frame', 'map')
        # 마커 자세 -> 도크 자세 회전 (dock_calib.py 로 구한 값)
        p('marker_pitch', 1.5708)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.dock_id = g('dock_id')
        self.dock_pose = (g('dock_x'), g('dock_y'), g('dock_yaw'))
        self.D = g('dock_distance')
        self.A = g('approach_distance')
        self.yaw_tol = g('yaw_tolerance')
        self.budget = g('contact_lateral_budget')
        self.max_iters = int(g('max_align_iters'))
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

        from tf2_ros import Buffer, TransformListener
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

        self.marker = None            # 최근 /detected_dock_pose
        self.create_subscription(PoseStamped, 'detected_dock_pose',
                                 self._marker, 10, callback_group=self.cb)
        self.cmd = self.create_publisher(TwistStamped, 'cmd_vel_dock', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                callback_group=self.cb)

        self.dock_srv = ActionServer(
            self, DockRobot, 'dock_robot', self._do_dock,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self.cb)
        self.undock_srv = ActionServer(
            self, UndockRobot, 'undock_robot', self._do_undock,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self.cb)

        self.get_logger().info(
            '단계 도킹 서버 시작: 진입 %.2f m, 도킹 %.3f m, '
            '접촉 횡오차 예산 %.0f mm (각도 상한 %.1f도)'
            % (self.A, self.D, self.budget * 1000, math.degrees(self.yaw_tol)))

    # ------------------------------------------------------------ 입력
    def _marker(self, msg):
        self.marker = msg

    def _stop(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base
        self.cmd.publish(m)

    def _drive(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.cmd.publish(m)

    def _odom_pose(self):
        t = self.buf.lookup_transform(self.odom, self.base,
                                      rclpy.time.Time()).transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    # ------------------------------------------------------------ 측정
    def measure(self):
        """정지 상태에서 도크를 재서 (전방거리, 횡오차, 각도오차)를 낸다.

        전방거리 : 로봇 중심에서 마커면까지 [m]
        횡오차   : 도크 중심선에서 로봇이 벗어난 양 [m] (+ 가 왼쪽)
        각도오차 : 로봇 heading 기준 도크 축 방향 [rad].
                   + 면 도크 축이 로봇 왼쪽이므로 **로봇이 오른쪽을 보고
                   있다**는 뜻이고, rotate(+psi) 로 폅니다.
        """
        self.marker = None
        t0 = time.time()
        samples = []
        while time.time() - t0 < self.meas_timeout and len(samples) < self.n_samples:
            m = self.marker
            self.marker = None
            if m is None:
                time.sleep(0.02)
                continue
            try:
                b = self.buf.transform(_stamp0(m), self.base)
            except Exception:                              # noqa: BLE001
                continue
            # 마커 좌표계 -> 도크 좌표계 (pitch 회전 후 yaw 만 취함)
            psi = wrap(yaw_of(_rot_pitch(b.pose.orientation, self.mpitch)))
            mx, my = b.pose.position.x, b.pose.position.y
            # 도크 중심선(마커를 지나 도크 축 방향)에서 로봇까지의 수직거리
            lat = mx * math.sin(psi) - my * math.cos(psi)
            samples.append((mx, my, psi, lat))
        if len(samples) < 3:
            return None
        n = len(samples)
        mx = sum(s[0] for s in samples) / n
        psi = math.atan2(sum(math.sin(s[2]) for s in samples) / n,
                         sum(math.cos(s[2]) for s in samples) / n)
        lat = sum(s[3] for s in samples) / n
        # 전방거리는 도크 축 방향 성분으로 잡습니다 (약간 틀어져 있어도 정확)
        fwd = mx * math.cos(psi) + sum(s[1] for s in samples) / n * math.sin(psi)
        return fwd, lat, psi, n

    def predict(self, fwd, lat, psi):
        """지금 이대로 직진했을 때 접촉 시점의 횡오차 [m] (+ 가 왼쪽).

        로봇은 도크 축 대비 -psi 로 틀어져 있으므로, 남은 거리
        (fwd - D) 를 직진하는 동안 -(fwd-D)*sin(psi) 만큼 흘러갑니다.
        정렬 판정도 크랩 보정량도 이 값 하나로 합니다.
        """
        return lat - max(0.0, fwd - self.D) * math.sin(psi)

    # ------------------------------------------------------------ 기동
    def rotate(self, delta):
        """제자리 회전. 오도메트리로 닫습니다."""
        if abs(delta) < 1e-4:
            return True
        x0, y0, a0 = self._odom_pose()
        target = wrap(a0 + delta)
        t0 = time.time()
        while time.time() - t0 < 20.0:
            _, _, a = self._odom_pose()
            err = wrap(target - a)
            if abs(err) < 0.004:                # 0.23도
                break
            # 남은 각도에 비례해 줄이되 최소 속도는 남깁니다
            w = max(0.05, min(self.w_rot, abs(err) * 1.5)) * (1 if err > 0 else -1)
            self._drive(0.0, w)
            time.sleep(0.02)
        self._stop()
        time.sleep(self.settle)
        return True

    def forward(self, dist):
        """직진. 오도메트리로 거리를 닫습니다 (마커는 보지 않습니다)."""
        if abs(dist) < 1e-4:
            return True
        x0, y0, _ = self._odom_pose()
        t0 = time.time()
        sign = 1.0 if dist > 0 else -1.0
        while time.time() - t0 < 60.0:
            x, y, _ = self._odom_pose()
            gone = math.hypot(x - x0, y - y0)
            left = abs(dist) - gone
            if left <= 0.002:
                break
            v = max(0.02, min(self.v_fwd, left * 0.8)) * sign
            self._drive(v, 0.0)
            time.sleep(0.02)
        self._stop()
        time.sleep(self.settle)
        return True

    def crab_move(self, shift):
        """로봇을 옆으로 shift 만큼 옮긴다 (회전 -> 직진 -> 반대 회전).

        차동구동은 옆으로 못 가므로 이 3동작이 필요합니다.
        **shift 는 "로봇 기준 왼쪽으로 얼마"** 입니다 (+ 가 왼쪽).

        부호 주의: measure() 가 내는 `lat` 은 "도크 중심선에서 로봇이
        벗어난 양"이고 왼쪽 법선 기준이라, lat < 0 이면 로봇이 중심선
        오른쪽에 있다는 뜻이므로 **왼쪽으로** 가야 합니다.
        즉 shift = -lat 입니다. 이 부호를 뒤집으면 보정이 오차를 두 배로
        키웁니다 (실측: 76 mm -> 154 mm, 그 뒤 바깥 마커가 화각을 벗어나
        검출까지 잃었습니다).
        """
        s = abs(shift) / math.sin(self.crab)
        # **후진으로 게걸음합니다.** 전진으로 하면 도크 방향으로
        # s*cos(30도) = 횡오차의 1.73배 만큼 다가갑니다 (실측: 횡 87 mm
        # 보정에 154 mm 접근). 정렬이 끝났을 때 스테이션과의 여유가
        # 오히려 줄어드는 셈이라, 보정할수록 위험해집니다.
        # 후진이면 같은 양만큼 **멀어지므로** 여유가 계속 확보됩니다.
        #
        # 후진할 때는 회전 부호가 뒤집힙니다. 왼쪽으로 옮기려면
        # 오른쪽으로 틀고 뒤로 가야 합니다.
        turn = -self.crab if shift > 0 else self.crab
        self.rotate(turn)
        self.forward(-s)
        self.rotate(-turn)

    # ------------------------------------------------------------ 액션
    def _do_dock(self, handle):
        r = DockRobot.Result()
        fb = DockRobot.Feedback()

        def emit(state):
            fb.state = state
            handle.publish_feedback(fb)

        goal = handle.request
        self.get_logger().info('단계 도킹 시작 (dock_id=%s)' % goal.dock_id)

        # --- 1단계: 진입점으로 이동 ---
        if goal.navigate_to_staging_pose:
            emit(FB_NAV)
            if not self._goto_entry():
                r.success, r.error_code = False, DockRobot.Result.FAILED_TO_STAGE
                r.error_msg = '진입점 이동 실패'
                handle.abort()
                return r

        # --- 2~4단계: 정지 측정 -> 각도 정렬 -> 횡 보정 -> 재측정 ---
        emit(FB_PERCEIVE)
        aligned = None
        prev_pred = None
        for i in range(self.max_iters):
            if handle.is_cancel_requested:
                self._stop()
                handle.canceled()
                r.success = False
                return r
            m = self.measure()
            if m is None:
                self.get_logger().warning('도크를 보지 못했습니다 (%d회차)' % (i + 1))
                emit(FB_RETRY)
                continue
            fwd, lat, psi, n = m
            pred = self.predict(fwd, lat, psi)
            self.get_logger().info(
                '  [%d] 전방 %.3f m / 횡 %+.1f mm / 각도 %+.2f도 '
                '-> 접촉 예상 %+.1f mm (표본 %d)'
                % (i + 1, fwd, lat * 1000, math.degrees(psi),
                   pred * 1000, n))
            # 판정은 접촉 시점의 예상 횡오차 하나로 합니다.
            # 각도 상한은 정밀도용이 아니라 측정 이상 감시용입니다.
            if abs(pred) <= self.budget and abs(psi) <= self.yaw_tol:
                aligned = m
                break
            # 각도가 상한을 넘으면 그것부터 폅니다. 크게 틀어진 채로
            # 크랩을 하면 크랩 방향 자체가 틀어집니다.
            if abs(psi) > self.yaw_tol:
                self.rotate(psi)
                continue
            # 보정이 오차를 키우면 즉시 멈춥니다. 부호가 틀리면 조용히
            # 발산하면서 마커까지 놓치기 때문입니다.
            if prev_pred is not None and abs(pred) > abs(prev_pred) + 0.002:
                self.get_logger().error(
                    '횡 보정이 오차를 키웠습니다 (%.1f -> %.1f mm). 중단합니다'
                    % (prev_pred * 1000, pred * 1000))
                break
            prev_pred = pred
            self.crab_move(-pred)
        if aligned is None:
            aligned = self.measure()
        if aligned is None:
            self._stop()
            r.success, r.error_code = False, DockRobot.Result.FAILED_TO_DETECT_DOCK
            r.error_msg = '정렬 후 도크를 보지 못했습니다'
            handle.abort()
            return r

        # --- 5단계: 직진 ---
        emit(FB_CONTROL)
        fwd, lat, psi, _ = aligned

        # 여유 확보. 직진 후진은 횡·각도를 흐트러뜨리지 않습니다.
        # 다만 직진 거리가 늘어나므로 남겨 둔 각도가 만드는 흘러감이
        # 그만큼 커집니다 — 예산을 넘으면 크랩 한 번으로 되돌립니다.
        if fwd < self.min_standoff:
            back = self.min_standoff - fwd
            self.get_logger().info(
                '스테이션과 %.3f m 밖에 안 떨어져 있어 %.3f m 물러납니다'
                % (fwd, back))
            self.forward(-back)
            m = self.measure()
            if m is not None:
                fwd, lat, psi, _ = m
                pred = self.predict(fwd, lat, psi)
                if abs(pred) > self.budget:
                    self.get_logger().info(
                        '물러나며 접촉 예상이 %+.1f mm 로 벌어져 다시 잡습니다'
                        % (pred * 1000))
                    self.crab_move(-pred)
                    m = self.measure()
                    if m is not None:
                        fwd, lat, psi, _ = m

        run = fwd - self.D
        self.get_logger().info(
            '정렬 완료 — 여유 %.3f m / 횡 %+.1f mm / 각도 %+.2f도 '
            '-> 접촉 예상 %+.1f mm (예산 %.0f mm). '
            '%.3f m 직진합니다 (이후 조향 없음)'
            % (fwd, lat * 1000, math.degrees(psi),
               self.predict(fwd, lat, psi) * 1000, self.budget * 1000, run))
        if run > 0.001:
            self.forward(run)
        self._stop()

        r.success, r.error_code, r.num_retries = True, 0, 0
        handle.succeed()
        self.get_logger().info('도킹 완료')
        return r

    def _do_undock(self, handle):
        r = UndockRobot.Result()
        self.get_logger().info('언도킹 — %.2f m 후진' % self.undock_d)
        self.forward(-self.undock_d)
        self._stop()
        r.success, r.error_code = True, 0
        handle.succeed()
        return r

    def _goto_entry(self):
        """도크 중심선 위, 마커에서 approach_distance 떨어진 지점으로."""
        dx, dy, dyaw = self.dock_pose
        # 도크 자세는 "도킹 완료 시 로봇 자세" 이므로, 거기서 뒤로
        # (approach - dock) 만큼 물러난 곳이 진입점입니다.
        back = self.A - self.D
        ex = dx - back * math.cos(dyaw)
        ey = dy - back * math.sin(dyaw)

        g = NavigateToPose.Goal()
        g.pose.header.frame_id = self.map_frame
        g.pose.pose.position.x, g.pose.pose.position.y = ex, ey
        g.pose.pose.orientation.z = math.sin(dyaw / 2.0)
        g.pose.pose.orientation.w = math.cos(dyaw / 2.0)
        if not self.nav.wait_for_server(timeout_sec=30.0):
            return False
        fut = self.nav.send_goal_async(g)
        gh = _wait(fut, 30.0)
        if gh is None or not gh.accepted:
            return False
        res = _wait(gh.get_result_async(), 180.0)
        return res is not None and res.status == GoalStatus.STATUS_SUCCEEDED


def _wait(fut, timeout):
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.02)
    return fut.result() if fut.done() else None


def _stamp0(msg):
    """가장 최근 TF 로 변환하도록 시각을 0 으로 둔 사본.

    영상 타임스탬프로 조회하면 odom TF 보다 앞서 도착해 "extrapolation
    into the future" 로 실패합니다. 정지 상태라 시각 차이는 무의미합니다.
    """
    out = PoseStamped()
    out.header.frame_id = msg.header.frame_id
    out.pose = msg.pose
    return out


def _rot_pitch(q, pitch):
    """q 에 pitch 회전을 오른쪽에서 곱한다 (마커 -> 도크 좌표계)."""
    ey, ew = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    from geometry_msgs.msg import Quaternion
    return Quaternion(
        x=q.w * 0.0 + q.x * ew + q.y * 0.0 - q.z * ey,
        y=q.w * ey - q.x * 0.0 + q.y * ew + q.z * 0.0,
        z=q.w * 0.0 + q.x * ey - q.y * 0.0 + q.z * ew,
        w=q.w * ew - q.x * 0.0 - q.y * ey - q.z * 0.0)


def main():
    rclpy.init()
    n = StagedDock()
    # **스레드 수를 반드시 지정해야 합니다.** 인자 없이 만들면 rclpy 가
    # CPU 코어 수만큼(이 PC 는 28개) 스레드를 만들고, 그 스레드들이
    # 대기셋을 두고 경합하며 놀고 있어도 CPU 를 태웁니다.
    # 실측: 지정 없이 두었을 때 idle 상태에서 89 %p.
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
