#!/usr/bin/env python3
"""Staged docking/undocking server.

    /dock_robot   (nav2_msgs/DockRobot)
    /undock_robot (nav2_msgs/UndockRobot)

Same action interface as opennav_docking, but instead of a smooth curve it
repeats stop -> measure -> correct -> stop -> re-measure, carrying standstill
perception accuracy into the final pose. Lateral error is removed by a crab
manoeuvre since a differential drive cannot strafe.

Rationale: docs/ros2-lessons.md chapter 6.
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
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

        # Dock pose. dock_register overwrites these at boot; these are the
        # fallback. All distances below are measured from the marker face.
        p('dock_id', 'home_dock')
        p('dock_x', 1.0)
        p('dock_y', -3.64)
        p('dock_yaw', -1.5708)
        p('dock_distance', 0.254)         # marker face -> docked pose [m]
        # Reverse docking keeps the camera facing the room while charging;
        # visual odometry breaks down 0.3 m from a wall.
        p('reverse_dock', True)
        # 180 deg turn point [m]. Limited by the casters, not the chassis --
        # see docking.yaml for the derivation.
        p('rotate_distance', 0.60)
        p('rotate_tolerance', 0.02)

        # --- Close the reverse leg with the rear lidar ---
        # After the turn the markers are behind the robot, so the reverse
        # would be open loop. The 360 deg lidar scans above the dock panel and
        # sees the wall beyond it.
        p('use_rear_lidar', True)
        # Rear range at the docked pose [m]. Moves together with
        # dock_distance; if they disagree the closed loop and the open-loop
        # fallback stop at different points.
        p('rear_target', 0.46)
        # Only beams within this angle of straight back [rad]. Each is
        # projected onto the reverse axis, so a wider window adds no bias.
        p('rear_window', 0.0873)          # +-5 deg
        p('rear_tolerance', 0.005)
        p('rear_min_beams', 5)
        p('rear_timeout', 30.0)
        p('approach_distance', 0.65)      # marker face -> alignment point [m]
        # Marker face -> Nav2 staging goal [m]. Kept apart from the alignment
        # point: costmap inflation (0.40 m) covers the area in front of the
        # wall, so aiming Nav2 at 0.65 m stops the robot right against the
        # dock. Far enough out to clear inflation, close enough that marker
        # accuracy still holds.
        p('staging_distance', 1.05)
        # Tolerance for re-checking staging arrival by pose. The alignment
        # loop re-measures with the markers, so "markers in view" is enough.
        p('entry_position_tolerance', 0.25)   # [m]
        p('entry_yaw_tolerance', 0.35)        # [rad] ~20 deg

        # Alignment is judged by one quantity -- predicted lateral error at
        # contact -- not per axis. See docking.yaml.
        p('contact_lateral_budget', 0.006)
        # Yaw ceiling [rad]: a sanity check on the measurement, not precision.
        p('yaw_tolerance', 0.0175)        # 1.0 deg
        p('max_align_iters', 6)
        p('search_backoff', 0.15)         # back off per marker-search step [m]
        p('min_standoff', 0.62)           # minimum clearance to align [m]

        # Motion
        p('crab_angle', 0.5236)           # 30 deg
        p('v_rotate', 0.35)               # [rad/s]
        p('v_forward', 0.08)              # [m/s]
        # Creep speed and stop deadband together set the floor on
        # longitudinal error.
        p('v_creep', 0.01)                # [m/s]
        p('forward_tolerance', 0.0005)    # [m]
        p('settle_time', 0.7)             # wait after stopping [s]
        p('measure_samples', 12)
        p('measure_timeout', 5.0)
        p('undock_distance', 0.5)         # [m]

        # Paused for the docking leg (std_srvs/Empty). Add real-hardware
        # camera stream services here when they exist.
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
        self.rot_tol = g('rotate_tolerance')
        self.use_rear = g('use_rear_lidar')
        self.rear_target = g('rear_target')
        self.rear_window = g('rear_window')
        self.rear_tol = g('rear_tolerance')
        self.rear_min_beams = int(g('rear_min_beams'))
        self.rear_timeout = g('rear_timeout')
        # Where the straight run stops: the turn point when reversing.
        self.stop_at = self.R if self.reverse else self.D
        self.A = g('approach_distance')
        self.S = g('staging_distance')
        self.entry_pos_tol = g('entry_position_tolerance')
        self.entry_yaw_tol = g('entry_yaw_tolerance')
        self.yaw_tol = g('yaw_tolerance')
        self.budget = g('contact_lateral_budget')
        self.max_iters = int(g('max_align_iters'))
        self.search_back = g('search_backoff')
        self.min_standoff = g('min_standoff')
        self.crab = g('crab_angle')
        self.w_rot = g('v_rotate')
        self.v_fwd = g('v_forward')
        self.v_creep = g('v_creep')
        self.fwd_tol = g('forward_tolerance')
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
        # Sensor QoS (BEST_EFFORT) is compatible with a RELIABLE publisher.
        self.scan = None
        self.create_subscription(LaserScan, 'scan', self._scan,
                                 qos_profile_sensor_data, callback_group=self.cb)
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
            '단계 도킹 서버 시작 — 복귀 목표 %.2f m / 정렬 %.2f m / '
            '회전 %.2f m / 도킹 %.3f m (마커면 기준), 접촉 예산 %.0f mm '
            '(각도 상한 %.1f도)'
            % (self.S, self.A, self.R, self.D, self.budget * 1000,
               math.degrees(self.yaw_tol)))

    # ---------------- Inputs ----------------
    def _marker(self, msg):
        self.marker = msg

    def _scan(self, msg):
        self.scan = msg

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
            if err <= self.fwd_tol:
                break
            # Creep speed sets the stop resolution: one 0.02 s tick overshoots
            # by v_creep * 0.02. Tune it together with forward_tolerance.
            v = max(self.v_creep, min(self.v_fwd, err * 1.0)) * sign
            self._drive(v, 0.0)
            time.sleep(0.02)
        self._stop()
        time.sleep(self.settle)
        return True

    # --------------------------------------------------------- Rear lidar
    def rear_distance(self):
        """Perpendicular range to the wall straight behind [m], or None.

        Each beam is projected onto the reverse axis (r*cos) so oblique hits
        do not read long. The median ignores a few outliers.
        """
        s = self.scan
        if s is None:
            return None
        vals = []
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            off = abs(wrap(s.angle_min + i * s.angle_increment - math.pi))
            if off <= self.rear_window:
                vals.append(r * math.cos(off))
        if len(vals) < self.rear_min_beams:
            return None
        vals.sort()
        return vals[len(vals) // 2]

    def backward_to_rear(self, target, expect):
        """Reverse until the rear lidar reads the target range.

        `expect` is the odometry estimate of the distance, used only as a
        guard: travelling past 1.5x it aborts, so a bad lidar reading cannot
        push the robot into the dock.
        """
        d0 = self.rear_distance()
        if d0 is None:
            return False
        limit = max(0.05, expect * 1.5)
        x0, y0, _ = self._odom_pose()
        t0 = time.time()
        while time.time() - t0 < self.rear_timeout:
            d = self.rear_distance()
            if d is None:
                self._stop()
                self.get_logger().warning('후진 중 뒤쪽 빔을 잃었습니다')
                return False
            err = d - target
            if err <= self.rear_tol:
                break
            x, y, _ = self._odom_pose()
            if math.hypot(x - x0, y - y0) > limit:
                self._stop()
                self.get_logger().error(
                    '후진량이 예상(%.3f m)의 1.5배를 넘었습니다 — 중단'
                    % expect)
                return False
            self._drive(-max(self.v_creep, min(self.v_fwd, err)), 0.0)
            time.sleep(0.02)
        self._stop()
        time.sleep(self.settle)
        d1 = self.rear_distance()
        self.get_logger().info(
            '후진 완료 — 뒤쪽 거리 %.3f -> %.3f m (목표 %.3f, 이동 %.3f m)'
            % (d0, d1 if d1 is not None else float('nan'), target,
               math.hypot(*[a - b for a, b in
                            zip(self._odom_pose()[:2], (x0, y0))])))
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
        # Re-read: dock_register can update these at runtime.
        self.dock_pose = (self.get_parameter('dock_x').value,
                          self.get_parameter('dock_y').value,
                          self.get_parameter('dock_yaw').value)
        self.get_logger().info(
            'Starting staged dock (dock_id=%s, dock (%.3f, %.3f, %+.4f))'
            % (goal.dock_id, *self.dock_pose))

        # Stage 1: Move to staging pose
        if goal.navigate_to_staging_pose:
            emit(FB_NAV)
            if not self._goto_entry():
                r.success, r.error_code = False, DockRobot.Result.FAILED_TO_STAGE
                r.error_msg = 'Failed to reach staging pose'
                handle.abort()
                return r

        # Do not trust Nav2's final heading: right after undocking the robot
        # is already within 0.1 m of the goal, so it succeeds on position
        # alone and alignment would start facing away from the dock.
        self._face_dock()
        self._restore_standoff()

        # Freeze mapping and visual odometry corrections from here. The
        # camera stares at a wall 0.3-0.9 m away through repeated spins and
        # crabs, which corrupts the pose graph. Docking works in odom and
        # moves under 1 m, so wheels + IMU are enough.
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
                # SLAM error can be tens of cm, so map coordinates cannot
                # tell us we are outside the detection cliff. Backing off
                # brings the markers into view either way.
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
            self._freeze_slam(False)     # always restore, even on failure
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
            # Close the turn point on the markers: rotate_distance sits above
            # the detection cliff, so they are still visible here. The casters
            # cannot climb the contact plate, so a drifting turn point snags.
            here = None
            for _ in range(3):
                m = self.measure()
                if m is None:
                    break
                here = m[0]
                adj = here - self.R
                if abs(adj) <= self.rot_tol:
                    break
                self.get_logger().info(
                    '회전점 보정 %+.0f mm (측정 %.3f m, 목표 %.3f ±%.0f mm)'
                    % (adj * 1000, here, self.R, self.rot_tol * 1000))
                self.forward(adj)
            if here is None:
                here = self.R
                self.get_logger().warning(
                    '회전점에서 마커 미검출 — 오도메트리 값 %.3f m 로 진행합니다'
                    % here)
            # Reverse distance comes from the range just measured, not a
            # constant -- after the turn the markers are behind and cannot
            # correct anything, so residual error must be absorbed here.
            back = max(0.0, here - self.D)
            self.get_logger().info(
                '회전점 %.3f m — 180도 회전 후 %.3f m 후진합니다' % (here, back))
            self.rotate(math.pi)
            # Only after the turn do the rear beams face the dock. Closing on
            # lidar instead of odometry also absorbs drift from the turn.
            done = False
            if self.use_rear and back > 0.001:
                done = self.backward_to_rear(self.rear_target, back)
                if not done:
                    self.get_logger().warning(
                        '뒤쪽 라이다를 쓸 수 없어 오도메트리로 후진합니다')
            if not done and back > 0.001:
                self.forward(-back)

        r.success, r.error_code = True, 0
        handle.succeed()
        self.get_logger().info('Docking complete')
        return r

    def _do_undock(self, handle):
        r = UndockRobot.Result()
        # Re-freeze: auto_dock's power-save release may have resumed these.
        # The camera still faces the wall through the undock leg.
        self._freeze_slam(True)
        # When reversed the robot already faces away, so forward is out.
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
        """Move to the alignment standoff (approach_distance).

        Usually pulls forward, since staging sits further back to clear
        costmap inflation. Backs off instead when a failed attempt left the
        robot inside the detection cliff.

        Measured in map coordinates, so it carries SLAM error: stop `safety`
        short of the dock and let the marker-based alignment loop finish.
        """
        safety = 0.05
        try:
            t = self.buf.lookup_transform(
                self.map_frame, self.base, rclpy.time.Time()).transform
        except Exception:                                  # noqa: BLE001
            return
        dx, dy, dyaw = self.dock_pose
        d = math.hypot(t.translation.x - dx, t.translation.y - dy) + self.D
        err = d - self.A
        if abs(err) <= safety:
            return
        if err > 0:
            fwd = err - safety
            self.get_logger().info(
                '도크에서 %.2f m 라 %.2f m 다가갑니다 (정렬에 %.2f m 필요)'
                % (d, fwd, self.A))
            self.forward(fwd)
            return
        back = -err + safety
        self.get_logger().info(
            '도크에서 %.2f m 뿐이라 %.2f m 물러섭니다 (정렬에 %.2f m 필요)'
            % (d, back, self.A))
        self.forward(-back)

    def _face_dock(self):
        """Turn in place to face the dock, using the map heading."""
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

    # --------------------------------------------------------- Map freeze
    def _freeze_slam(self, on):
        """Stop pose-graph updates and visual odometry corrections."""
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
        """Drive to the Nav2 staging goal, which sits behind the alignment point."""
        dx, dy, dyaw = self.dock_pose
        back = self.S - self.D
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

        self.get_logger().info(
            '진입점 주행 목표 발신 (%.3f, %.3f, %+.1f도)'
            % (ex, ey, math.degrees(dyaw)))
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
        if res is not None and res.status == GoalStatus.STATUS_SUCCEEDED:
            return True

        # A preempted goal reports ABORTED on this handle even though the
        # robot reached staging on the goal that replaced it. Arrival is a
        # physical fact, so check it by pose.
        return self._at_entry(ex, ey, dyaw)

    def _at_entry(self, ex, ey, dyaw):
        """Check by map pose whether the robot is actually at the staging point."""
        try:
            tf = self.buf.lookup_transform(
                self.map_frame, self.base, rclpy.time.Time()).transform
        except Exception:                                  # noqa: BLE001
            return False
        d = math.hypot(tf.translation.x - ex, tf.translation.y - ey)
        dyawe = abs(wrap(yaw_of(tf.rotation) - dyaw))
        ok = d <= self.entry_pos_tol and dyawe <= self.entry_yaw_tol
        self.get_logger().info(
            '진입점 주행이 실패로 끝났지만 실제 위치는 %.3f m / %+.1f도 차이 — %s'
            % (d, math.degrees(dyawe), '진행합니다' if ok else '중단합니다'))
        return ok

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
    """Multiply the marker pose by a y-axis rotation: q (x) q_pitch.

    Converts the marker optical convention (z out of the marker) to the dock
    convention (x out of the dock face); a wrong axis folds 90 deg into yaw.
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
