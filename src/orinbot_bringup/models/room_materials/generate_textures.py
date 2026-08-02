#!/usr/bin/env python3
"""시각 오도메트리용 텍스처 생성.

단색 벽에서는 ORB/GFTT 같은 특징점 검출기가 코너를 거의 못 찾아
visual odometry 가 즉시 실패합니다. 여기서 만드는 텍스처는
"고주파 + 비반복" 패턴이라 특징점이 고르게 분포하고, 패턴이 반복되지
않으므로 서로 다른 위치를 같은 곳으로 오인하는 문제도 없습니다.

    python3 generate_textures.py

materials/textures/ 아래에 PNG 를 씁니다. 시드를 고정해 두어 실행할
때마다 같은 결과가 나옵니다.
"""

import math
import os

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'materials', 'textures')

rng = np.random.default_rng(20260801)

# ---- 도킹 스테이션 표적 (ArUco) ----
# 여기 두 값은 orinbot_navigation/config/docking.yaml 의
# dock_marker_detector 파라미터(dictionary / marker_id)와 반드시 같아야
# 합니다. 다르면 마커가 보여도 검출되지 않습니다.
#
# 사전을 4x4(=cv2.aruco.DICT_4X4_50, 0번)로 고른 이유:
# 카메라가 424x240 이라 멀리서는 마커가 몇십 픽셀밖에 안 됩니다.
# 4x4 는 테두리 포함 6칸이라 24 px 정도면 읽히지만, 기본값인
# 6x6(10번)은 8칸이라 32 px 이상 필요해 검출 거리가 눈에 띄게 짧아집니다.
# 방 하나에 도크 하나뿐이라 50개 사전으로 충분합니다.
ARUCO_DICT = 0
# 마커 3개를 하나의 보드로 씁니다. 한 장만 쓰면 정면에 가까운 평면
# 마커의 자세 모호성 때문에 각도가 1.3 m 에서 4.77도까지 튑니다(실측).
# 세 장의 코너 12개를 함께 풀면(estimatePoseBoard) 그 흔들림이 크게
# 줄고, 특히 좌우로 벌려 놓은 배치가 yaw 를 강하게 구속합니다.
# 순서는 도크 기준 왼쪽(-x) / 가운데 / 오른쪽(+x) 입니다.
ARUCO_IDS = (1, 0, 2)


def save(img, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    img.save(path)
    print('wrote', path)


def speckle(base_rgb, size=512, amount=38, blobs=900):
    """바탕색 + 랜덤 얼룩. 어느 방향으로 잘라도 특징점이 남습니다."""
    arr = np.full((size, size, 3), base_rgb, dtype=np.int16)
    arr += rng.integers(-amount, amount, (size, size, 3), dtype=np.int16)
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))

    d = ImageDraw.Draw(img)
    for _ in range(blobs):
        x, y = rng.integers(0, size, 2)
        r = int(rng.integers(2, 9))
        shade = int(rng.integers(-60, 60))
        c = tuple(int(np.clip(base_rgb[i] + shade, 0, 255)) for i in range(3))
        d.ellipse([x - r, y - r, x + r, y + r], fill=c)
    return img


def brick_wall(size=512):
    """벽돌: 강한 직선 코너를 제공합니다."""
    img = speckle((176, 122, 98), size, amount=22, blobs=500)
    d = ImageDraw.Draw(img)
    rows, bh = 12, size // 12
    for r in range(rows):
        y = r * bh
        d.line([(0, y), (size, y)], fill=(232, 228, 220), width=4)
        offset = (bh * 2) if r % 2 else 0
        x = offset
        while x < size + bh * 4:
            d.line([(x, y), (x, y + bh)], fill=(232, 228, 220), width=4)
            x += bh * 4
    return img


def tiles(size=512):
    """바닥 타일. 색을 칸마다 조금씩 흔들어 반복성을 깹니다."""
    img = Image.new('RGB', (size, size), (150, 150, 155))
    d = ImageDraw.Draw(img)
    n, cell = 8, size // 8
    for i in range(n):
        for j in range(n):
            base = 150 + int(rng.integers(-26, 26))
            d.rectangle([j * cell, i * cell, (j + 1) * cell - 1, (i + 1) * cell - 1],
                        fill=(base, base, base + 6))
    arr = np.asarray(img).astype(np.int16)
    arr += rng.integers(-14, 14, arr.shape, dtype=np.int16)
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    for i in range(n + 1):
        d.line([(0, i * cell), (size, i * cell)], fill=(96, 96, 100), width=3)
        d.line([(i * cell, 0), (i * cell, size)], fill=(96, 96, 100), width=3)
    return img


def aruco_marker(marker_id=0, dictionary=ARUCO_DICT):
    """도킹 스테이션 표적판.

    cv2 로 직접 그립니다. 비트 패턴을 손으로 옮겨 적으면 사전(dictionary)
    정의와 한 비트라도 어긋났을 때 "검출이 그냥 안 되는" 상태가 되는데,
    원인을 찾기가 매우 어렵습니다. 검출기(image_proc/track_marker_node)와
    같은 라이브러리로 생성해 두면 둘이 어긋날 수 없습니다.

    치수 (도크 모델·검출기 파라미터와 함께 맞춰야 하는 값):
        판 전체 0.14 m     <- generate_room.py 의 DOCK_PLATE
        검은 사각형 0.10 m <- dock_marker_board.py 의 marker_size
    504 px 판에서 마커가 360 px 이므로 사방 여백이 72 px(=0.02 m)입니다.
    이 흰 여백(quiet zone)이 없으면 마커 경계가 배경에 묻혀 검출이
    불안정해집니다.
    """
    import cv2  # 생성 시에만 필요합니다 (ROS 스택에 이미 들어 있음)

    size, inner = 504, 360
    d = cv2.aruco.Dictionary_get(dictionary)
    bits = cv2.aruco.drawMarker(d, marker_id, inner)

    img = Image.new('RGB', (size, size), (255, 255, 255))
    pad = (size - inner) // 2
    img.paste(Image.fromarray(bits).convert('RGB'), (pad, pad))
    return img


def poster(size=512, seed_shapes=26):
    """벽에 붙일 포스터. 각 장이 서로 다른 모양이라 위치 식별에 유리합니다."""
    bg = tuple(int(x) for x in rng.integers(215, 245, 3))
    img = Image.new('RGB', (size, size), bg)
    d = ImageDraw.Draw(img)
    for _ in range(seed_shapes):
        x0, y0 = rng.integers(0, size - 60, 2)
        w, h = rng.integers(40, 190, 2)
        c = tuple(int(x) for x in rng.integers(0, 210, 3))
        if rng.random() < 0.45:
            d.rectangle([x0, y0, x0 + w, y0 + h], outline=c, width=int(rng.integers(3, 10)))
        elif rng.random() < 0.7:
            d.ellipse([x0, y0, x0 + w, y0 + h], outline=c, width=int(rng.integers(3, 10)))
        else:
            d.line([x0, y0, x0 + w, y0 + h], fill=c, width=int(rng.integers(3, 12)))
    return img


# 벽 세그먼트 개수 (generate_room.py 와 맞춰야 합니다)
# 외벽 18 + 칸막이 2 + 문 1 + 통로벽 4 = 25 개가 필요합니다.
# 여유를 두고 28 개를 만듭니다.
# 하나라도 재사용하면 perceptual aliasing 이 생겨
# 잘못된 루프 클로저로 지도가 튑니다 (실측: map->odom 보정 0.35m).
N_WALL_SEGMENTS = 28
N_POSTERS = 8


def mark(img, idx):
    """벽마다 서로 다른 큰 도형을 얹어 장소를 구별 가능하게 만듭니다.

    모든 벽에 같은 벽돌 텍스처만 쓰면 서로 다른 위치가 똑같이 보여
    (perceptual aliasing) RTAB-Map 이 엉뚱한 루프 클로저를 맺습니다.
    실제로 그래프가 망가져 자세가 3m 이상 튀는 것을 확인했습니다.
    실제 실내에도 표지판·가구·배관 같은 고유한 표식이 있으므로,
    이렇게 두는 편이 오히려 현실적입니다.
    """
    d = ImageDraw.Draw(img)
    size = img.size[0]
    hue = (idx * 47) % 360
    # HSV -> RGB 를 간단히 계산
    c = int(127 + 110 * math.cos(math.radians(hue)))
    m = int(127 + 110 * math.cos(math.radians(hue + 120)))
    y = int(127 + 110 * math.cos(math.radians(hue + 240)))
    col = (c, m, y)

    cx, cy = size // 2, size // 2
    r = size // 3
    kind = idx % 5
    if kind == 0:
        d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=col, width=18)
    elif kind == 1:
        d.rectangle([cx - r, cy - r, cx + r, cy + r], outline=col, width=18)
    elif kind == 2:
        d.polygon([(cx, cy - r), (cx + r, cy + r), (cx - r, cy + r)],
                  outline=col, width=18)
    elif kind == 3:
        d.line([cx - r, cy - r, cx + r, cy + r], fill=col, width=22)
        d.line([cx - r, cy + r, cx + r, cy - r], fill=col, width=22)
    else:
        for k in range(3):
            rr = r - k * 34
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=col, width=14)

    # 세그먼트 번호를 굵은 막대 패턴으로 (숫자 폰트 의존성을 피하려고)
    for b in range(6):
        if idx & (1 << b):
            d.rectangle([30, 30 + b * 26, 30 + 60, 30 + b * 26 + 16], fill=col)
    return img


def main():
    for i in range(N_WALL_SEGMENTS):
        save(mark(brick_wall(), i), 'wall_%02d.png' % i)
    save(tiles(), 'floor.png')
    for i in range(1, N_POSTERS + 1):
        save(poster(), 'poster%d.png' % i)
    save(speckle((120, 140, 170), blobs=1200), 'crate.png')
    for _i in ARUCO_IDS:
        save(aruco_marker(_i), 'dock_marker_%d.png' % _i)
    # 도크 몸통. 방 안의 다른 무엇과도 안 닮은 색이라야 RTAB-Map 이
    # 도크 앞을 다른 장소로 착각하지 않습니다.
    save(speckle((60, 70, 90), blobs=800), 'dock_body.png')


if __name__ == '__main__':
    main()
