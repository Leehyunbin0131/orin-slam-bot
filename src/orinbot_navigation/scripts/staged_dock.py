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
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

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
        # Distance from marker surface to final docked pose [m]
        p('dock_distance', 0.294)
        # Distance from marker surface to approach staging pose [m]
        p('approach_distance', 0.65)

        # Alignment criteria
        # Budgeted contact lateral error [m] (half of physical copper pad tolerance +-34mm)
        p('contact_lateral_budget', 0.015)
        # Yaw threshold limit [rad] for error anomaly monitoring
        p('yaw_tolerance', 0.0175)        # 1.0 deg
        p('max_align_iters', 4)
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

        p('base_frame', 'base_footprint')
        p('odom_frame', 'odom')
        p('map_frame', 'map')
        # Marker pose to dock pose rotation pitch [rad]
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
        return lat - max(0.0, fwd - self.D) * math.sin(psi)

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
                self.get_logger().warning('Dock marker not detected (attempt %d)' % (i + 1))
                emit(FB_RETRY)
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

        run = max(0.0, fwd - self.D)
        self.get_logger().info(
            'Alignment complete -- standoff %.3f m / lat %+.1f mm / yaw %+.2f deg '
            '-> predicted contact %+.1f mm (budget %.0f mm). '
            'Executing final %.3f m straight run'
            % (fwd, lat * 1000, math.degrees(psi),
               self.predict(fwd, lat, psi) * 1000, self.budget * 1000, run))
        if run > 0.001:
            self.forward(run)

        r.success, r.error_code = True, 0
        handle.succeed()
        self.get_logger().info('Docking complete')
        return r

    def _do_undock(self, handle):
        r = UndockRobot.Result()
        self.get_logger().info('Undocking -- moving backward %.2f m' % self.undock_d)
        self.forward(-self.undock_d)
        self._stop()
        r.success, r.error_code = True, 0
        handle.succeed()
        self.get_logger().info('Undocking complete')
        return r

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
    ey, ew = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    from geometry_msgs.msg import Quaternion
    return Quaternion(
        x=q.x * ew + q.w * ey - q.z * 0.0,
        y=q.y * ew + q.z * ey + q.x * 0.0,
        z=q.z * ew - q.y * ey + q.w * 0.0,
        w=q.w * ew - q.x * ey - q.y * 0.0)


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
