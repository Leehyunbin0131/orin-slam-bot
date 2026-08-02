#!/usr/bin/env python3
"""maze.sdf 미로 월드 SDF 모델 생성 스크립트.

    python3 generate_maze.py [폭_m] > maze.sdf
"""

import sys

import numpy as np

TEX = 'model://room_materials/materials/textures'

SEED = 20260803
N = 7                  # 셀 격자 N x N
# 통로 폭 [m]. 첫 인자로 덮어쓸 수 있습니다:
#     python3 generate_maze.py 0.90 > maze90.sdf
# 시드가 같으면 미로 형상(어느 벽이 뚫렸는지)은 그대로이고 간격만 바뀌므로,
# **폭만 바꾼 대조 실험**이 됩니다.
CORRIDOR = float(sys.argv[1]) if len(sys.argv) > 1 else 0.75
WALL_T = 0.10
WALL_H = 1.2
PITCH = CORRIDOR + WALL_T
BRAID = 0.30           # 막다른 곳 중 이 비율만큼 터서 고리를 만듭니다
NARROW = (0.55, 0.60, 0.70)   # 주머니 입구 폭 (음성 시험용)

rng = np.random.default_rng(SEED)
_tex = 0


def w(s=''):
    sys.stdout.write(s + '\n')


def next_tex():
    global _tex
    t = _tex
    _tex += 1
    return '%s/maze_%03d.png' % (TEX, t)


def cx(i):
    return (i - (N - 1) / 2.0) * PITCH


def slab(name, x, y, sx, sy, texture):
    """바닥에 세운 벽 조각 하나 (시각 + 충돌)."""
    w('    <model name="%s">' % name)
    w('      <static>true</static>')
    w('      <pose>%.4f %.4f %.4f 0 0 0</pose>' % (x, y, WALL_H / 2.0))
    w('      <link name="link">')
    w('        <collision name="c">')
    w('          <geometry><box><size>%.4f %.4f %.4f</size></box></geometry>'
      % (sx, sy, WALL_H))
    w('        </collision>')
    w('        <visual name="v">')
    w('          <geometry><box><size>%.4f %.4f %.4f</size></box></geometry>'
      % (sx, sy, WALL_H))
    w('          <material>')
    w('            <diffuse>1 1 1 1</diffuse>')
    w('            <pbr><metal>')
    w('              <albedo_map>%s</albedo_map>' % texture)
    w('              <roughness>0.9</roughness>')
    w('            </metal></pbr>')
    w('          </material>')
    w('        </visual>')
    w('      </link>')
    w('    </model>')


# ---------------------------------------------------------------- 미로 생성
# vw[i][j] : 셀 (i-1,j) 와 (i,j) 사이의 세로벽. i = 0..N (양끝은 외벽)
# hw[i][j] : 셀 (i,j-1) 과 (i,j) 사이의 가로벽. j = 0..N
vw = [[True] * N for _ in range(N + 1)]
hw = [[True] * (N + 1) for _ in range(N)]

visited = [[False] * N for _ in range(N)]
stack = [(N // 2, N // 2)]
visited[N // 2][N // 2] = True
while stack:
    i, j = stack[-1]
    nbrs = []
    if i > 0 and not visited[i - 1][j]:
        nbrs.append((i - 1, j, 'v', i))
    if i < N - 1 and not visited[i + 1][j]:
        nbrs.append((i + 1, j, 'v', i + 1))
    if j > 0 and not visited[i][j - 1]:
        nbrs.append((i, j - 1, 'h', j))
    if j < N - 1 and not visited[i][j + 1]:
        nbrs.append((i, j + 1, 'h', j + 1))
    if not nbrs:
        stack.pop()
        continue
    ni, nj, kind, idx = nbrs[rng.integers(len(nbrs))]
    if kind == 'v':
        vw[idx][j] = False
    else:
        hw[i][idx] = False
    visited[ni][nj] = True
    stack.append((ni, nj))


def openings(i, j):
    o = []
    if not vw[i][j]:
        o.append(('v', i, j))
    if not vw[i + 1][j]:
        o.append(('v', i + 1, j))
    if not hw[i][j]:
        o.append(('h', i, j))
    if not hw[i][j + 1]:
        o.append(('h', i, j + 1))
    return o


# 땋기 — 막다른 곳 일부를 터서 고리를 만듭니다 (루프 클로저용).
dead = [(i, j) for i in range(N) for j in range(N) if len(openings(i, j)) == 1]
for i, j in dead:
    if rng.random() > BRAID:
        continue
    cand = []
    if i > 0 and vw[i][j]:
        cand.append(('v', i, j))
    if i < N - 1 and vw[i + 1][j]:
        cand.append(('v', i + 1, j))
    if j > 0 and hw[i][j]:
        cand.append(('h', i, j))
    if j < N - 1 and hw[i][j + 1]:
        cand.append(('h', i, j + 1))
    if not cand:
        continue
    kind, a, b = cand[rng.integers(len(cand))]
    if kind == 'v':
        vw[a][b] = False
    else:
        hw[a][b] = False

# 남은 막다른 곳 중 3개를 골라 입구를 좁힙니다 (음성 시험).
dead = [(i, j) for i in range(N) for j in range(N) if len(openings(i, j)) == 1]
narrow = {}
for k, gap in enumerate(NARROW):
    if k >= len(dead):
        break
    i, j = dead[k]
    narrow[openings(i, j)[0]] = (gap, (i, j))

# ---------------------------------------------------------------- SDF 출력
w('<?xml version="1.0" ?>')
w('<!-- generate_maze.py 로 생성됨. 직접 수정하지 말고 생성기를 고치세요. -->')
w('<sdf version="1.9">')
w('  <world name="maze">')
w("""
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <!-- 카메라/뎁스 렌더링에 필요 -->
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>

    <gravity>0 0 -9.8</gravity>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.82 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.7 0.7 0.7 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.35 0.35 0.4 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.5 -0.4 -0.8</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
""")

# 바닥 타일 — 윗면이 정확히 z=0 이라야 ground_plane 과 z-파이팅이 안 납니다.
span = N * PITCH / 2.0 + WALL_T
w('    <model name="floor">')
w('      <static>true</static>')
w('      <pose>0 0 -0.005 0 0 0</pose>')
w('      <link name="link"><visual name="v">')
w('        <geometry><box><size>%.3f %.3f 0.01</size></box></geometry>'
  % (2 * span, 2 * span))
w('        <material><diffuse>1 1 1 1</diffuse><pbr><metal>')
w('          <albedo_map>%s/floor.png</albedo_map><roughness>0.95</roughness>' % TEX)
w('        </metal></pbr></material>')
w('      </visual></link>')
w('    </model>')

n_full = n_narrow = 0

# 세로벽: x = cx(i) - PITCH/2, 셀 j 구간을 덮습니다.
for i in range(N + 1):
    for j in range(N):
        x = cx(i) - PITCH / 2.0
        y = cx(j)
        if vw[i][j]:
            slab('vw_%d_%d' % (i, j), x, y, WALL_T, PITCH, next_tex())
            n_full += 1
        elif ('v', i, j) in narrow:
            gap = narrow[('v', i, j)][0]
            stub = (PITCH - gap) / 2.0
            for s, sy in ((-1, stub), (1, stub)):
                slab('vwn_%d_%d_%s' % (i, j, 'ab'[s > 0]),
                     x, y + s * (PITCH - stub) / 2.0, WALL_T, sy, next_tex())
            n_narrow += 1

# 가로벽: y = cx(j) - PITCH/2
for i in range(N):
    for j in range(N + 1):
        x = cx(i)
        y = cx(j) - PITCH / 2.0
        if hw[i][j]:
            slab('hw_%d_%d' % (i, j), x, y, PITCH, WALL_T, next_tex())
            n_full += 1
        elif ('h', i, j) in narrow:
            gap = narrow[('h', i, j)][0]
            stub = (PITCH - gap) / 2.0
            for s in (-1, 1):
                slab('hwn_%d_%d_%s' % (i, j, 'ab'[s > 0]),
                     x + s * (PITCH - stub) / 2.0, y, stub, WALL_T, next_tex())
            n_narrow += 1

w('  </world>')
w('</sdf>')

# ---------------------------------------------------------------- 요약
e = sys.stderr.write
e('미로 %dx%d 셀, 통로 %.2f m, 벽 %.2f m -> 전체 %.2f x %.2f m\n'
  % (N, N, CORRIDOR, WALL_T, 2 * span, 2 * span))
e('벽 조각 %d 개 (온전 %d + 좁힌 입구 %d 곳의 스텁)\n'
  % (_tex, n_full, n_narrow))
e('필요 텍스처: maze_000.png ~ maze_%03d.png  (%d 장)\n' % (_tex - 1, _tex))
e('  -> models/room_materials/generate_textures.py 의 N_MAZE_SEGMENTS 를 %d 이상으로\n' % _tex)
for (kind, a, b), (gap, cell) in narrow.items():
    e('좁힌 주머니 입구: %s 벽 (%d,%d), 폭 %.2f m, 셀 %s -> 중심 (%.2f, %.2f)\n'
      % (kind, a, b, gap, cell, cx(cell[0]), cx(cell[1])))
e('로봇 시작 권장: (%.2f, %.2f) = 중앙 셀\n' % (cx(N // 2), cx(N // 2)))
