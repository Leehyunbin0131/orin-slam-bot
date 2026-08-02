#!/usr/bin/env python3
"""통로별로 전역 경로가 실제로 나오는지 확인한다.

월드의 통로 뱅크는 x=3.2 벽에 뚫린 문 4개다 (generate_room.py:227-230).
각 통로의 y 중심에서 벽 양쪽 (2.4, y) -> (4.2, y) 로 경로를 요청하고,
경로가 통로를 그대로 지나가는지(길이 ~1.8 m) 아니면 크게 우회/실패하는지
본다.

NavfnPlanner 는 tolerance 0.5 m 안으로 목표를 당겨서 "성공"을 낼 수 있으므로
성공 여부만 보면 안 되고 **끝점이 실제 목표에 닿았는지**까지 봐야 한다.
"""

import math
import sys

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# (이름, 통로 중심 y, 설계 폭)
CORRIDORS = [
    ('c090', 2.6, 0.90),
    ('c070', 0.6, 0.70),
    ('c055', -1.6, 0.55),
    ('c060', -3.0, 0.60),
]
X0, X1 = 2.4, 4.2                      # 벽(x=3.2) 양쪽


def pose(x, y):
    from geometry_msgs.msg import PoseStamped
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.w = 1.0
    return p


def main():
    rclpy.init()
    n = Node('corridor_plan')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    if not ac.wait_for_server(timeout_sec=30):
        print('compute_path_to_pose 서버 없음')
        return 1

    print('%-6s %5s  %-9s %8s %9s  %s'
          % ('통로', '폭', '결과', '경로길이', '끝점오차', '판정'))
    for name, y, w in CORRIDORS:
        g = ComputePathToPose.Goal()
        g.start = pose(X0, y)
        g.goal = pose(X1, y)
        g.use_start = True
        fut = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=20)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('%-6s %5.2f  %-9s' % (name, w, '거절됨'))
            continue
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=30)
        res = rf.result()
        if res is None:
            print('%-6s %5.2f  %-9s' % (name, w, '시간초과'))
            continue
        r = res.result
        pts = [(p.pose.position.x, p.pose.position.y) for p in r.path.poses]
        if not pts:
            print('%-6s %5.2f  %-9s %8s %9s  %s'
                  % (name, w, '경로없음', '-', '-', '차단됨'))
            continue
        length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        gap = math.dist(pts[-1], (X1, y))
        # 직선 거리 1.8 m. 통로를 지나면 1.8~2.2 m, 우회하면 훨씬 길다.
        if gap > 0.3:
            verdict = '목표 미도달(잘림)'
        elif length < 2.5:
            verdict = '통로 통과'
        else:
            verdict = '우회 (%.1f 배)' % (length / (X1 - X0))
        print('%-6s %5.2f  %-9s %8.2f %9.2f  %s'
              % (name, w, '성공', length, gap, verdict))

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
