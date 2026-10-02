"""벨로다인 BEV 입력 만들기: 점군 → 원본 BEV → 정규화 HSV. 노드와 학습 도구(tools/velodyne_train)가 함께 쓰는 정본.

원본 BEV (bev_from_points): 600x600 uint8 BGR, ±15 m, 0.05 m/픽셀, 센서 = 이미지 중심.
  B = 점 개수(30개에서 포화), G = 반사강도 평균, R = 최고 높이(z -2.5~2.0 m → 0~255).
  학습 데이터의 정본 이미지가 이 형식이다. 정규화 HSV는 되돌릴 수 없으므로 원본을 따로 남긴다.
정규화 HSV (hsv_v1): 원본 BEV → uint8 BGR. 색상 = 지면 기준 높이, 채도 = 반사강도, 명도 = 주변 점유율.
  지면 위 0.1~2.0 m 띠 밖은 회색, 점 없는 픽셀은 검정이다. 식은 velodyne_dataset/README.md "표현 변환" 절.
"""
import numpy as np
import cv2

VOXEL = 0.05
X_RANGE, Y_RANGE, Z_RANGE = (-15.0, 15.0), (-15.0, 15.0), (-2.5, 2.0)
MAX_PTS_PER_VOXEL = 30


def bev_from_points(pts, voxel=VOXEL, x_rng=X_RANGE, y_rng=Y_RANGE, z_rng=Z_RANGE, max_pts=MAX_PTS_PER_VOXEL):
    """pts (n, 4) x, y, z, intensity → 원본 BEV."""
    h_px, w_px = int((x_rng[1] - x_rng[0]) / voxel), int((y_rng[1] - y_rng[0]) / voxel)
    if not pts.size:
        return np.zeros((h_px, w_px, 3), np.uint8)
    x, y, z, inten = pts.T
    m = (x_rng[0] <= x) & (x < x_rng[1]) & (y_rng[0] <= y) & (y < y_rng[1]) & (z_rng[0] <= z) & (z < z_rng[1])
    x, y, z, inten = x[m], y[m], z[m], inten[m]
    ix = np.clip(((y_rng[1] - y) / voxel).astype(np.int32), 0, w_px - 1)
    iy = np.clip(((x_rng[1] - x) / voxel).astype(np.int32), 0, h_px - 1)
    hmap = np.full((h_px, w_px), z_rng[0], np.float32); imap = np.zeros_like(hmap); dmap = np.zeros_like(hmap, np.int32)
    np.maximum.at(hmap, (iy, ix), z)
    np.add.at(imap, (iy, ix), inten)
    np.add.at(dmap, (iy, ix), 1)
    h = ((np.clip(hmap, *z_rng) - z_rng[0]) / (z_rng[1] - z_rng[0]) * 255).astype(np.uint8)
    i = np.zeros_like(imap, np.uint8); i[dmap > 0] = np.clip(imap[dmap > 0] / dmap[dmap > 0], 0, 255).astype(np.uint8)
    d = (np.clip(dmap, 0, max_pts) / max_pts * 255).astype(np.uint8)
    return cv2.merge([d, i, h])


# ── 정규화 HSV ──────────────────────────────────────────────────────────────
N = 600                          # BEV 한 변 픽셀
ZL = 255 / (Z_RANGE[1] - Z_RANGE[0])   # 높이 채널 1 m 당 단위
_r = np.hypot(*np.mgrid[-N // 2:N // 2, -N // 2:N // 2]) * VOXEL
RING = (_r > 3) & (_r < 8)       # 전체 지면 추정에 쓰는 3~8 m 고리
TILE = 60                        # 칸별 지면 칸 크기 (3 m)
_tid = (np.arange(N)[:, None] // TILE) * (N // TILE) + (np.arange(N)[None, :] // TILE)

# 색상 누적 변환: 학습 ERP 픽셀 지면 위 높이의 0·10·50·90·100% 지점 (2026-10-03 측정)
Z_PCT = [0.10, 0.23, 0.48, 1.43, 2.00]
Z_CDF = [0.0, 0.10, 0.50, 0.90, 1.0]


def occupancy(bev):
    return (bev[..., 0] | bev[..., 1] | bev[..., 2]) > 0


def ground_map(bev, occ=None):
    """칸별 지면 높이(높이 채널 단위, 600x600 float).
    3 m 칸마다 '칸 최저 높이 + 0.3 m 이내' 점 높이의 최빈값. 점 30개 미만 칸은 3~8 m 고리 최빈값.
    칸 값에 3x3 가우시안 후 선형으로 늘린다. 비탈·차체 기울기·수풀(중앙값이 끌려 올라감)에 강하다."""
    occ = occupancy(bev) if occ is None else occ
    h = bev[..., 2].astype(np.int64)
    v = h[occ & RING]
    if v.size == 0: v = h[occ]
    g0 = float(np.bincount(v).argmax()) if v.size else 0.0
    n = N // TILE
    hist = np.bincount(_tid[occ] * 256 + h[occ], minlength=n * n * 256).reshape(n * n, 256)
    cnt = hist.sum(1); mn = (hist > 0).argmax(1)
    hist[np.arange(256)[None, :] > (mn + 0.3 * ZL)[:, None]] = 0
    G = np.where(cnt >= 30, hist.argmax(1), g0).astype(np.float32).reshape(n, n)
    return cv2.resize(cv2.GaussianBlur(G, (3, 3), 0), (N, N), interpolation=cv2.INTER_LINEAR)


def hsv_v1(bev):
    """원본 BEV → 정규화 HSV v1 (uint8 BGR 600x600x3)."""
    i8, h8 = bev[..., 1], bev[..., 2]
    occ = occupancy(bev)
    zag = (h8.astype(np.float32) - ground_map(bev, occ)) / ZL
    H = (1 - np.interp(zag, Z_PCT, Z_CDF)) * 120                       # OpenCV 색상 0~180 = 0~360°, 0 m 파랑(240°) → 2 m 빨강(0°)
    Sat = 0.4 + 0.6 * np.clip(np.log1p(i8.astype(np.float32)) / np.log1p(30), 0, 1)
    V = 0.6 + 0.4 * np.clip(cv2.blur(occ.astype(np.float32), (5, 5)) / 0.6, 0, 1)
    gnd, tall = zag < 0.1, zag > 2.0
    Sat[gnd | tall] = 0
    V[gnd] = 0.35; V[tall] = 0.8
    out = cv2.cvtColor(np.dstack([H, Sat * 255, V * 255]).astype(np.uint8), cv2.COLOR_HSV2BGR)
    out[~occ] = 0
    return out


REPRESENTATIONS = {"hsv_v1": hsv_v1, "raw": lambda bev: bev}
