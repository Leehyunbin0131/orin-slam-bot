#!/usr/bin/env python3
"""Maze world dominated by narrow corridors.

    python3 generate_maze.py > maze.sdf        (summary goes to stderr)

room.sdf is mostly open, so one exploration run barely touches tight spaces.
This world exercises them every run. It does not replace room.sdf, which
remains the regression baseline.

Design notes:
  - 0.75 m corridors: the measured minimum is 0.70 m (turning in place), and
    one extra cell keeps grid discretisation from breaking paths.
  - Only three dead-end mouths are narrowed to 0.55/0.60/0.70 as negative
    tests; narrowing the main route would make every run measure how stuck
    the robot got that day.
  - Braided, not perfect: some dead ends are opened into loops, without which
    RTAB-Map never sees a place from two directions and loop closure fails.
  - Unique texture per wall piece (maze_XXX.png); reuse makes distinct places
    look identical and jumps the pose by metres. Generate them with
    models/room_materials/generate_textures.py maze.
  - No dock: run this world with dock:=false.

The seed is fixed, so the maze is reproducible. Changing it can change the
texture count -- check the stderr summary.
"""

import sys

import numpy as np

TEX = 'model://room_materials/materials/textures'

SEED = 20260803
N = 7                  # N x N cell grid
# Corridor width [m], overridable by the first argument:
#     python3 generate_maze.py 0.90 > maze90.sdf
# With the same seed the maze shape is unchanged and only the spacing
# differs, giving a controlled width-only comparison.
CORRIDOR = float(sys.argv[1]) if len(sys.argv) > 1 else 0.75
WALL_T = 0.10
WALL_H = 1.2
PITCH = CORRIDOR + WALL_T
BRAID = 0.30           # fraction of dead ends opened into loops
NARROW = (0.55, 0.60, 0.70)   # pocket mouth widths (negative tests)

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
    """One wall piece standing on the floor (visual + collision)."""
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


# ------------------------------------------------------- Maze generation
# vw[i][j]: vertical wall between cells (i-1,j) and (i,j); i = 0..N
# hw[i][j]: horizontal wall between cells (i,j-1) and (i,j); j = 0..N
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


# Braid: open some dead ends into loops so loop closure has a chance.
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

# Narrow three remaining dead-end mouths as negative tests.
dead = [(i, j) for i in range(N) for j in range(N) if len(openings(i, j)) == 1]
narrow = {}
for k, gap in enumerate(NARROW):
    if k >= len(dead):
        break
    i, j = dead[k]
    narrow[openings(i, j)[0]] = (gap, (i, j))

# ------------------------------------------------------------ SDF output
w('<?xml version="1.0" ?>')
w('<!-- Generated by generate_maze.py. Edit the generator, not this file. -->')
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
    <!-- Required for camera/depth rendering -->
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

# Floor tiles: the top must be exactly z=0 or it z-fights with ground_plane.
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

# Vertical walls at x = cx(i) - PITCH/2, spanning cell j.
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

# Horizontal walls at y = cx(j) - PITCH/2
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

# --------------------------------------------------------------- Summary
e = sys.stderr.write
e('maze %dx%d cells, corridor %.2f m, wall %.2f m -> %.2f x %.2f m\n'
  % (N, N, CORRIDOR, WALL_T, 2 * span, 2 * span))
e('%d wall pieces (%d full + stubs at %d narrowed mouths)\n'
  % (_tex, n_full, n_narrow))
e('textures needed: maze_000.png ~ maze_%03d.png  (%d)\n' % (_tex - 1, _tex))
e('  -> set N_MAZE_SEGMENTS >= %d in generate_textures.py\n' % _tex)
for (kind, a, b), (gap, cell) in narrow.items():
    e('narrowed mouth: %s wall (%d,%d), width %.2f m, cell %s -> (%.2f, %.2f)\n'
      % (kind, a, b, gap, cell, cx(cell[0]), cx(cell[1])))
e('suggested spawn: (%.2f, %.2f) = centre cell\n' % (cx(N // 2), cx(N // 2)))
