#!/usr/bin/env python3
"""프론티어 탐사 — 지도 없이 시작해 스스로 미탐색 구역을 돌며 지도를 만듭니다.

    ros2 launch orinbot_navigation explore.launch.py
    # 또는 전체 스택과 함께
    ros2 launch orinbot_navigation navigation.launch.py explore:=true

원리
----
점유격자에서 셀은 셋 중 하나입니다: 빈 곳(0) / 막힌 곳(100) / 모르는 곳(-1).
**프론티어**는 "빈 곳인데 바로 옆이 모르는 곳"인 셀입니다. 즉 로봇이 갈 수
있으면서 그 너머는 아직 안 본 경계선입니다. 여기로 가면 반드시 새로운 것을
보게 되므로, 프론티어가 하나도 안 남을 때까지 반복하면 공간이 다 채워집니다.

    1. /map 에서 프론티어 셀을 찾는다
    2. 붙어 있는 것끼리 묶어 덩어리(cluster)로 만들고, 너무 작은 건 버린다
    3. `거리 - gain*크기` 가 가장 작은 덩어리를 고른다
       (가까울수록 좋고, 경계가 길수록 한 번에 많이 밝혀지므로 좋다)
    4. 그 지점으로 NavigateToPose 목표를 보낸다
    5. 도착/실패하면 1번으로. 프론티어가 없으면 종료.

외부 패키지(explore_lite 등)를 쓰지 않은 이유
---------------------------------------------
Jazzy apt 저장소에 탐사 패키지가 없고(직접 확인), explore_lite 계열은 자기
costmap_2d 인스턴스를 하나 더 굴립니다. Orin Nano 예산에서 코스트맵 하나는
무시할 수 없는 비용인데, 정작 필요한 건 이미 발행 중인 /map 뿐입니다.

이 로봇에 맞춘 부분
-------------------
- **목표 방향을 미탐색 쪽으로 돌립니다.** D435i 화각이 87도뿐이라 목표에
  도착했을 때 엉뚱한 데를 보고 있으면 그 자리에서 새로 밝혀지는 게 없습니다.
  yaw 를 "로봇 -> 프론티어" 방향으로 주어 도착과 동시에 미탐색 구역을 봅니다.
- **좁은 통로를 막지 않도록 여유값을 작게 잡습니다** (`clearance` 0.22 m,
  내접반경 0.20 + 2 cm). 여기서 하는 건 "목표점 고르기"일 뿐이고 실제 충돌
  판정은 Nav2 가 footprint 로 합니다. 크게 잡으면 폭 0.60 m 통로의 프론티어가
  통째로 걸러져 그 너머를 영영 탐사하지 않습니다.
- **RTAB-Map 의 `Grid/RangeMax` 가 5 m** 라 넓은 방 한가운데서도 반경 5 m 링에
  프론티어가 생깁니다. 정상입니다. 로봇이 전진하면 링도 같이 밀려납니다.
- **실패한 목표는 블랙리스트에 넣습니다.** RTAB-Map 의 격자는 루프 클로저로
  소급 수정되기 때문에, 한때 프론티어였던 곳이 사실은 벽 뒤일 수 있습니다.
  Nav2 가 포기한 지점을 계속 다시 고르면 무한 루프가 됩니다.
"""

import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from scipy import ndimage
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        p = self.declare_parameter
        # 탐색 주기 [s]. /map 이 갱신되는 속도보다 빨라야 의미가 없습니다.
        p('period', 2.0)
        # 덩어리로 인정할 최소 프론티어 셀 수. 격자 0.05 m 기준 8 셀 = 0.4 m.
        # 이보다 작은 건 대개 격자 잡음이라 쫓아가면 시간만 버립니다.
        p('min_frontier_cells', 8)
        # 여유값 두 개는 "들어갈 수 있는가"가 아니라 **"들어가서 돌아 나올 수
        # 있는가"** 를 기준으로 잡습니다.
        #
        # 로봇 0.40 m 정사각 -> 내접반경 0.20, **외접반경 0.283**.
        # 제자리 회전에는 외접반경이 필요합니다. 거리변환 값은 폭 W 통로의
        # 중앙에서 W/2 이므로, 임계값이 곧 최소 통로 폭의 절반입니다.
        #
        #   통로 폭   중앙 여유    0.18(옛값)   0.33(현재)
        #   0.55 m     0.275        통과         기각
        #   0.60 m     0.300        통과         기각
        #   0.70 m     0.350        통과         통과
        #
        # 외접반경 0.283 을 그대로 쓰면 0.60 m(0.300)가 통과해 버립니다.
        # 0.70 m 를 살리고 0.60 m 를 거르는 값이 0.33 입니다.
        #
        # 왜 이렇게까지 하는가: 옛값(0.18)으로 돌린 실측에서 탐사기가 로봇이
        # 돌아 나올 수 없는 협소 구역을 목표로 골랐고, 로봇이 벽에 갈리며
        # 회전하다 바퀴가 미끄러져 **지도 전체가 몇 도 기울었습니다**
        # (완주 4526초, 끼임 19회, 후진 38회 중 30회 실패).
        # 주행만 실패하는 게 아니라 지도까지 잃습니다.
        #
        # 대가: 0.60 m 이하로만 닿는 구역은 탐사하지 않습니다. 그런 곳까지
        # 지도에 넣어야 한다면 내려야 하지만, 그때는 위 실측을 각오할 것.
        p('min_clearance', 0.33)
        # 덩어리 안에서 목표 셀을 고를 때 선호하는 여유 [m]. 기각 기준이
        # 아니라 선호 기준입니다 (이만큼 되는 셀이 없으면 가장 넓은 셀로 갑니다).
        p('clearance', 0.35)
        # 점수 = 거리 - gain * 경계길이[m]. 크면 "멀어도 큰 미탐색"을 선호합니다.
        p('gain', 1.5)
        # 실패한 목표 주변 이 반경 [m] 안은 다시 고르지 않습니다.
        p('blacklist_radius', 0.6)
        # 블랙리스트 유효 시간 [s]. 0 이면 영구.
        # RTAB-Map 격자는 루프 클로저로 소급 수정되므로, 한 번 못 갔다고
        # 영원히 포기하면 나중에 열린 길을 놓칩니다.
        p('blacklist_ttl', 120.0)
        # 전역 코스트맵을 한 번이라도 받은 뒤 이 시간 [s] 안의 실패는
        # 블랙리스트에 넣지 않습니다 (아래 준비 판정과 함께 씁니다).
        p('startup_grace', 15.0)
        # 같은 지점에서 이 횟수만큼 실패하면 영구 포기합니다.
        # 장애물 뒤 그늘처럼 원리적으로 도달 불가능한 미탐색이 남는데,
        # TTL 만 있으면 그 한 곳 때문에 탐사가 영원히 안 끝납니다.
        p('max_retries', 2)
        # 목표를 보낸 뒤 최소 이만큼 [s] 은 바꾸지 않습니다. 매 틱 목표를
        # 갈아치우면 Nav2 가 계속 preemption 만 처리하다 한 발도 못 뗍니다.
        p('min_goal_dwell', 5.0)
        # 한 목표에 이만큼 [s] 매달리면 포기하고 블랙리스트에 넣습니다.
        p('goal_timeout', 90.0)
        # 이 시간 [s] 동안 min_progress 미만으로 움직이면 끼인 것으로 봅니다.
        p('stuck_time', 30.0)
        p('min_progress', 0.15)
        # Nav2 가 성공을 반환했는데 로봇이 목표에서 이보다 멀면 "가짜 성공"
        # 으로 봅니다. NavfnPlanner 는 목표가 도달 불가(장애물 팽창 영역 안)
        # 이면 tolerance(0.5 m) 안의 가장 가까운 지점으로 경로를 잘라 주는데,
        # 그 지점이 이미 로봇 위치이면 컨트롤러가 즉시 "Reached the goal!" 을
        # 냅니다. 이걸 도착으로 믿으면 같은 목표를 무한히 반복합니다
        # (실측: 같은 지점에 253회 "도착").
        p('arrive_tolerance', 0.5)
        # 현재 목표 주변 이 반경 [m] 에 프론티어가 남아 있으면 목표를 유지합니다.
        # 없어졌다 = 이미 그쪽을 다 봤다 = 다른 데로 갈 때입니다.
        p('goal_stale_radius', 0.7)
        # 프론티어가 연속 이만큼 없으면 탐사 완료로 봅니다. RTAB-Map 격자는
        # 한 틱 비었다가 다시 나타나기도 해서 한 번으로 끝내면 안 됩니다.
        p('done_ticks', 3)
        # 탐사가 끝나면 출발점으로 돌아갈지
        p('return_home', True)
        # 전체 제한 시간 [s]. 0 이면 무제한.
        p('explore_timeout', 0.0)
        p('publish_markers', True)
        # 점유격자 임계값 (0~100). RTAB-Map 은 0/100 만 쓰지만 일반화해 둡니다.
        p('free_threshold', 25)
        p('occupied_threshold', 65)
        p('robot_frame', 'base_footprint')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.period = g('period')
        self.min_cells = int(g('min_frontier_cells'))
        self.clearance = g('clearance')
        self.min_clearance = g('min_clearance')
        self.gain = g('gain')
        self.blacklist_radius = g('blacklist_radius')
        self.blacklist_ttl = g('blacklist_ttl')
        self.max_retries = int(g('max_retries'))
        self.startup_grace = g('startup_grace')
        self.min_goal_dwell = g('min_goal_dwell')
        self.goal_timeout = g('goal_timeout')
        self.stuck_time = g('stuck_time')
        self.min_progress = g('min_progress')
        self.arrive_tolerance = g('arrive_tolerance')
        self.stale_radius = g('goal_stale_radius')
        self.done_ticks = int(g('done_ticks'))
        self.return_home = g('return_home')
        self.explore_timeout = g('explore_timeout')
        self.publish_markers = g('publish_markers')
        self.free_thr = g('free_threshold')
        self.occ_thr = g('occupied_threshold')
        self.robot_frame = g('robot_frame')

        self.map_msg = None
        # [[(x, y), 만료시각_s 또는 None(영구), 실패횟수]]
        self.blacklist = []
        self.goal_handle = None
        self.goal_xy = None
        self.goal_time = None
        # 목표 일련번호. 새 목표를 보내면 이전 목표는 Nav2 가 preemption 으로
        # 끝내면서 ABORTED 를 돌려줍니다. 그건 "실패"가 아니라 "내가 밀어낸 것"
        # 이므로, 지금 번호와 다른 결과는 전부 무시해야 합니다.
        # (이 구분이 없으면 멀쩡한 프론티어가 전부 블랙리스트로 갑니다)
        self.goal_seq = 0
        self.all_blacklisted_warned = False
        self.empty_ticks = 0
        self.finished = False
        self.home = None
        self.last_pose = None
        self.last_progress_t = None
        self.visited = 0             # 성공한 목표 수
        self.t0 = None

        # /map 은 RELIABLE + TRANSIENT_LOCAL 이고, RTAB-Map 은 지도가 바뀔 때만
        # 발행합니다. 기본 QoS(VOLATILE)로 구독하면 로봇이 멈춰 있는 동안
        # 붙었을 때 마지막 지도를 못 받아 영영 기다립니다 (실측으로 확인).
        # TRANSIENT_LOCAL 로 맞춰야 붙는 즉시 최신 지도를 받습니다.
        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)

        # Nav2 가 "정말 목표를 받을 수 있는가"의 판정 기준
        # ------------------------------------------------------------------
        # navigate_to_pose 액션 서버의 존재만으로는 부족합니다. 그건
        # bt_navigator 가 활성이라는 뜻일 뿐이고, planner_server 가 아직
        # 활성화 중이거나 전역 코스트맵이 /map 크기로 자리를 잡기 전이면
        # 모든 목표가 즉시 ABORT 됩니다. 그 실패로 지점을 판단하면 프론티어가
        # 통째로 블랙리스트에 들어가 탐사가 시작도 못 합니다 (실측 2회).
        #
        # 전역 코스트맵이 발행됐다는 것은 planner_server 가 활성이고 코스트맵이
        # 준비됐다는 직접 증거이므로, 이것을 준비 신호로 씁니다.
        # (nav2_params.yaml 의 always_send_full_costmap: True 라 주기적으로 옵니다)
        self.costmap_seen = None
        self.create_subscription(
            OccupancyGrid, 'global_costmap/costmap', self._on_costmap, latched)
        # 외부에서 탐사를 잠시 멈추는 스위치 (auto_dock.py 가 씁니다).
        # 충전 복귀는 DockRobot 이 내부적으로 NavigateToPose 를 거는데,
        # 여기서 2초마다 새 목표를 계속 보내면 그 이동을 밀어내 버려
        # 로봇이 도크로 가지 못합니다.
        # 기본값은 활성입니다 — 이 토픽을 아무도 발행하지 않는 구성에서
        # 탐사가 멈춰 있으면 안 되기 때문입니다.
        self.paused = False
        self.create_subscription(Bool, 'exploration_enabled', self._on_enable, 10)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.markers = self.create_publisher(MarkerArray, 'frontiers', 1)

        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            '프론티어 탐사 시작 대기 — /map 과 navigate_to_pose 를 기다립니다')

    # ------------------------------------------------------------------ 입력
    def _on_map(self, msg):
        self.map_msg = msg

    def _on_enable(self, msg):
        want_pause = not msg.data
        if want_pause == self.paused:
            return
        self.paused = want_pause
        if self.paused:
            # 진행 중이던 목표를 반드시 취소합니다. 그냥 두면 Nav2 가
            # 계속 그쪽으로 몰고 가면서 도킹용 이동과 싸웁니다.
            self._cancel()
            self.get_logger().info('탐사 일시 정지 (외부 요청)')
        else:
            self.get_logger().info('탐사 재개')

    def _on_costmap(self, msg):
        if self.costmap_seen is None:
            self.costmap_seen = self.get_clock().now()
            self.get_logger().info('전역 코스트맵 확인 — Nav2 준비됨')

    def _robot_xy(self):
        try:
            t = self.tf_buf.lookup_transform(
                self.map_msg.header.frame_id, self.robot_frame, Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y

    # -------------------------------------------------------------- 프론티어
    def _frontiers(self):
        """(중심 xy, 셀 수, 프론티어 셀 좌표배열) 목록을 돌려줍니다."""
        m = self.map_msg
        info = m.info
        res = info.resolution
        # OccupancyGrid.data 는 row-major, 행이 y 입니다.
        grid = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width)

        free = (grid >= 0) & (grid <= self.free_thr)
        unknown = grid < 0
        occupied = grid >= self.occ_thr

        # 4-이웃 중 하나라도 미탐색이면 프론티어
        nbr = np.zeros_like(unknown)
        nbr[1:, :] |= unknown[:-1, :]
        nbr[:-1, :] |= unknown[1:, :]
        nbr[:, 1:] |= unknown[:, :-1]
        nbr[:, :-1] |= unknown[:, 1:]
        frontier = free & nbr
        if not frontier.any():
            return []

        # 여유값으로 프론티어를 "걸러내면" 안 됩니다 (실측으로 확인)
        # ------------------------------------------------------------------
        # 예전에는 장애물을 clearance 만큼 팽창시켜 프론티어에서 빼고, 남은
        # 것으로 덩어리 크기를 쟀습니다. 그랬더니 폭 0.60 m 통로 입구에서
        # 살아남는 띠가 2셀뿐이라 최소 8셀 기준에 미달해 통로 4개가 통째로
        # 사라졌고, 탐사가 80초 만에 "다 봤다"며 끝났습니다.
        # 크기 판정은 원본 프론티어로 하고, 여유값은 "덩어리 안 어디를
        # 목표로 찍을까"에만 씁니다.
        dist = ndimage.distance_transform_edt(~occupied) * res

        # 8-연결로 덩어리 묶기
        lbl, n = ndimage.label(frontier, structure=np.ones((3, 3), bool))
        out = []
        for i in range(1, n + 1):
            ys, xs = np.nonzero(lbl == i)
            if len(xs) < self.min_cells:
                continue
            d = dist[ys, xs]
            if d.max() < self.min_clearance:
                continue          # 로봇이 물리적으로 못 들어가는 틈
            # 여유가 충분한 셀 중 덩어리 중심에 가장 가까운 것을 고릅니다.
            # 충분한 게 없으면 그나마 가장 넓은 셀로 갑니다.
            cx, cy = xs.mean(), ys.mean()
            ok = np.nonzero(d >= min(self.clearance, d.max()))[0]
            k = ok[np.argmin((xs[ok] - cx) ** 2 + (ys[ok] - cy) ** 2)]
            wx = info.origin.position.x + (xs[k] + 0.5) * res
            wy = info.origin.position.y + (ys[k] + 0.5) * res
            wxs = info.origin.position.x + (xs + 0.5) * res
            wys = info.origin.position.y + (ys + 0.5) * res
            out.append(((wx, wy), len(xs), np.stack([wxs, wys], axis=1)))
        return out

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _blacklisted(self, xy):
        """(막혀 있는가, 영구인가)"""
        t = self._now()
        self.blacklist = [e for e in self.blacklist if e[1] is None or e[1] > t]
        hits = [e for e in self.blacklist if math.dist(xy, e[0]) < self.blacklist_radius]
        return bool(hits), any(e[1] is None for e in hits)

    # ------------------------------------------------------------------ 목표
    def _send_goal(self, xy, from_xy):
        yaw = math.atan2(xy[1] - from_xy[1], xy[0] - from_xy[0])
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = self.map_msg.header.frame_id
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(xy[0])
        g.pose.pose.position.y = float(xy[1])
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_xy = xy
        self.goal_time = self.get_clock().now()
        self.last_progress_t = self.goal_time
        self.last_pose = from_xy
        self.goal_handle = None
        fut = self.ac.send_goal_async(g)
        fut.add_done_callback(lambda f: self._on_accepted(seq, f))
        self.get_logger().info(
            '목표 #%d -> (%.2f, %.2f)  거리 %.2f m'
            % (seq, xy[0], xy[1], math.dist(xy, from_xy)))

    def _on_accepted(self, seq, fut):
        if seq != self.goal_seq:
            return                      # 이미 다음 목표로 넘어갔음
        gh = fut.result()
        if gh is None or not gh.accepted:
            # 거절 = "이 지점이 나쁘다"가 아니라 "서버가 아직 못 받는다"
            # 입니다 (bt_navigator 가 activate 되기 전 등). 블랙리스트에
            # 넣으면 안 되고, 그냥 다음 틱에 다시 시도하면 됩니다.
            self.get_logger().info('Nav2 가 아직 목표를 받지 않습니다 — 재시도')
            self.goal_xy = None
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self._on_result(seq, f))

    def _on_result(self, seq, fut):
        if seq != self.goal_seq:
            # 내가 새 목표로 밀어낸(preempt) 예전 목표의 결과입니다.
            # Nav2 는 이때 ABORTED 를 돌려주는데 실패가 아니므로 무시합니다.
            return
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            me = self._robot_xy()
            gap = math.dist(me, self.goal_xy) if me and self.goal_xy else 0.0
            if gap > self.arrive_tolerance:
                self.get_logger().warn(
                    '성공했다는데 목표에서 %.2f m 떨어져 있음 — 도달 불가로 처리' % gap)
                self._abandon()
                return
            self.visited += 1
            self.get_logger().info('도착 (%d 번째)' % self.visited)
            self.goal_handle = None
            self.goal_xy = None
        elif status == GoalStatus.STATUS_CANCELED:
            self.goal_handle = None
            self.goal_xy = None
        elif status == GoalStatus.STATUS_UNKNOWN:
            # 서버가 사라졌거나 결과를 제대로 못 받은 경우입니다.
            # 지점의 문제가 아니므로 블랙리스트에 넣지 않습니다.
            self.get_logger().info('결과 상태 불명 — 재시도')
            self._cancel()
        else:
            self.get_logger().warn('목표 실패(status=%d)' % status)
            self._abandon()

    def _in_grace(self):
        if self.costmap_seen is None:
            return True
        dt = (self.get_clock().now() - self.costmap_seen).nanoseconds * 1e-9
        return dt < self.startup_grace

    def _abandon(self):
        """현재 목표를 블랙리스트에 넣고 중단합니다. (취소까지 함께 합니다)"""
        if self._in_grace():
            # Nav2 가 아직 자리를 잡는 중입니다. 이 실패로 지점을 판단하면 안 됩니다.
            self.get_logger().info('기동 유예 중 실패 — 블랙리스트에 넣지 않고 재시도')
            self._cancel()
            return
        if self.goal_xy:
            xy = self.goal_xy
            hit = next((e for e in self.blacklist
                        if math.dist(xy, e[0]) < self.blacklist_radius), None)
            fails = (hit[2] if hit else 0) + 1
            if hit:
                self.blacklist.remove(hit)
            if fails >= self.max_retries or self.blacklist_ttl <= 0:
                self.blacklist.append([xy, None, fails])
                self.get_logger().warn(
                    '(%.2f, %.2f) %d 회 실패 — 도달 불가로 보고 영구 제외' % (xy[0], xy[1], fails))
            else:
                self.blacklist.append([xy, self._now() + self.blacklist_ttl, fails])
        self._cancel()

    def _cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_xy = None

    # ------------------------------------------------------------------ 본체
    def _tick(self):
        if self.finished or self.paused or self.map_msg is None:
            return
        if not self.ac.server_is_ready() or self.costmap_seen is None:
            return          # Nav2 가 아직 목표를 처리할 수 없는 상태
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('탐사 개시')
        if self.explore_timeout > 0 and \
                (now - self.t0).nanoseconds * 1e-9 > self.explore_timeout:
            self._finish('제한 시간 도달')
            return

        me = self._robot_xy()
        if me is None:
            return
        if self.home is None:
            self.home = me

        clusters = self._frontiers()
        if self.publish_markers:
            self._draw(clusters)

        # 프론티어가 정말 없어야 탐사 완료입니다. 후보가 전부 블랙리스트인
        # 것은 "다 봤다"가 아니라 "지금은 못 간다"이므로, TTL 이 풀릴 때까지
        # 기다립니다. 여기서 끝내 버리면 한 번의 실패가 탐사를 통째로 끝냅니다.
        if not clusters:
            self.empty_ticks += 1
            if self.empty_ticks >= self.done_ticks:
                self._cancel()
                self._finish('남은 프론티어 없음')
            return
        self.empty_ticks = 0

        cands, temporary = [], 0
        for c, n, _ in clusters:
            blocked, permanent = self._blacklisted(c)
            if not blocked:
                cands.append((c, n))
            elif not permanent:
                temporary += 1
        if not cands:
            if temporary == 0:
                # 남은 게 전부 "영구 제외" — 장애물 뒤 그늘처럼 갈 수 없는
                # 미탐색만 남았다는 뜻이라 더 기다려도 달라지지 않습니다.
                self._cancel()
                self._finish('도달 가능한 미탐색 없음 (그늘 %d 곳 제외)' % len(clusters))
                return
            if not self.all_blacklisted_warned:
                self.get_logger().warn(
                    '남은 프론티어 %d 곳이 전부 블랙리스트입니다 — 해제를 기다립니다'
                    % len(clusters))
                self.all_blacklisted_warned = True
            return
        self.all_blacklisted_warned = False

        res = self.map_msg.info.resolution
        best, _ = min(cands, key=lambda cn: math.dist(me, cn[0]) - self.gain * cn[1] * res)

        # 진행 중인 목표가 있으면 바꿀 이유가 있을 때만 바꿉니다.
        if self.goal_xy is not None:
            elapsed = (now - self.goal_time).nanoseconds * 1e-9
            # 보낸 직후에는 무조건 유지합니다. 안 그러면 Nav2 가 preemption 만
            # 반복 처리하다 한 발도 못 뗍니다 (실제로 그렇게 동작했습니다).
            if elapsed < self.min_goal_dwell:
                return
            if elapsed > self.goal_timeout:
                self.get_logger().warn('목표 시간 초과 (%.0f s) — 포기' % elapsed)
                self._abandon()
                return
            moved = math.dist(me, self.last_pose) if self.last_pose else 0.0
            if moved > self.min_progress:
                self.last_pose, self.last_progress_t = me, now
            elif (now - self.last_progress_t).nanoseconds * 1e-9 > self.stuck_time:
                self.get_logger().warn('%.0f 초간 제자리 — 포기' % self.stuck_time)
                self._abandon()
                return
            # 목표 근처에 프론티어가 남아 있으면 계속 갑니다.
            alive = any(
                np.any(np.hypot(pts[:, 0] - self.goal_xy[0],
                                pts[:, 1] - self.goal_xy[1]) < self.stale_radius)
                for _, _, pts in clusters)
            if alive:
                return
            # 명시적 cancel 을 보내지 않습니다. 새 목표를 보내면 Nav2 가
            # 알아서 preemption 으로 갈아탑니다. cancel 을 따로 보내면 그것이
            # 새 목표보다 늦게 도착해 갓 받은 목표를 취소해 버릴 수 있습니다.
            self.get_logger().info('목표 지점이 이미 밝혀짐 — 다음으로')

        self._send_goal(best, me)

    def _finish(self, why):
        self.finished = True
        dt = (self.get_clock().now() - self.t0).nanoseconds * 1e-9 if self.t0 else 0.0
        self.get_logger().info(
            '=== 탐사 종료: %s ===  방문 %d 곳, 포기 %d 곳, %.0f 초'
            % (why, self.visited, len(self.blacklist), dt))
        if self.return_home and self.home is not None:
            self.get_logger().info('출발점 (%.2f, %.2f) 으로 복귀' % self.home)
            self._send_goal(self.home, self._robot_xy() or self.home)
        self.get_logger().info(
            '지도는 RTAB-Map 데이터베이스(~/.ros/orinbot_rtabmap.db)에 이미 저장돼 있습니다. '
            "이미지 파일로 뽑으려면: ros2 run nav2_map_server map_saver_cli -f ~/my_map")

    # ---------------------------------------------------------------- 시각화
    def _draw(self, clusters):
        arr = MarkerArray()
        d = Marker()
        d.action = Marker.DELETEALL
        arr.markers.append(d)
        for i, (c, n, pts) in enumerate(clusters):
            m = Marker()
            m.header.frame_id = self.map_msg.header.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'frontier', i, Marker.POINTS, Marker.ADD
            m.scale.x = m.scale.y = 0.05
            hot = self.goal_xy is not None and math.dist(c, self.goal_xy) < 0.3
            m.color = ColorRGBA(r=1.0, g=0.2 if hot else 0.8, b=0.0, a=1.0)
            m.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in pts]
            arr.markers.append(m)
        self.markers.publish(arr)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
