"""Livox Horizon 지면 제거 (Patchwork++ 개념을 Horizon 시야에 맞춤). livox_camera_fusion 의 horizon_ground.cpp 와 같은 알고리즘.

시야(±41°) 안 극좌표 칸마다 최저점 씨앗 + PCA 평면, 직립도·높이·평탄도 검사, 실패 칸은 이웃 칸으로 대체.
전역 사전 평면은 1차 통과 칸들의 씨앗으로 다시 맞추고(2단), 평면은 인라이어 하위 25%에 맞춘다.
"""
import numpy as np

RE = [2.9, 4.5, 6.5, 9.0, 12.0, 15.6]      # 링 경계 [m], 2.9 m = 지면이 보이기 시작하는 거리
NS = [4, 6, 8, 8, 8]                        # 링별 부채꼴 수
FOV = np.radians(41.0)
ZREF = -0.72                                # 평지 기준 지면 높이 (센서 기준)
TH_SEED, TH_DIST, FLAT, TH = 0.125, 0.125, 0.003, 0.2
UPRIGHT = np.cos(np.radians(8))
NMIN = 15


def _pca(Q):
    c = Q.mean(0)
    w, v = np.linalg.eigh(np.cov((Q - c).T))
    n = v[:, 0]
    n = n if n[2] > 0 else -n
    return n, -(n @ c), w[0]


def _pass(p, prior, elev, collect_seeds):
    pa, pb, pc = prior
    r = np.hypot(p[:, 0], p[:, 1])
    th = np.arctan2(p[:, 1], p[:, 0])
    ri = np.clip(np.searchsorted(RE, r, side="right") - 1, -1, len(RE) - 2)   # -1: 첫 링 안쪽 (첫 링 평면 연장)
    si = [np.clip(((th + FOV) / (2 * FOV) * NS[k]).astype(int), 0, NS[k] - 1) for k in range(len(NS))]
    PL, own, seeds = {}, set(), []
    for k in range(len(NS)):
        for s in range(NS[k]):
            Z = p[(ri == k) & (si[k] == s)]
            if len(Z) >= NMIN:
                Z = Z[np.abs(Z[:, 2] - (pa * Z[:, 0] + pb * Z[:, 1] + pc)) < 0.3]
            if len(Z) < NMIN:
                continue
            zs = np.sort(Z[:, 2])
            lpr = zs[:max(3, min(20, len(zs) // 5))].mean()
            S = Z[Z[:, 2] < lpr + TH_SEED]
            for _ in range(3):
                if len(S) < NMIN:
                    break
                n, d, lam = _pca(S)
                S = Z[np.abs(Z @ n + d) < TH_DIST]
            if len(S) < NMIN:
                continue
            n, d, lam = _pca(S)
            d = d - np.percentile(S @ n + d, 25)          # 씨앗 띠 때문에 떠 있는 평면을 내린다
            a, b, c = -n[0] / n[2], -n[1] / n[2], -d / n[2]
            rc = (RE[k] + RE[k + 1]) / 2
            tc = -FOV + (s + 0.5) * 2 * FOV / NS[k]
            xc, yc = rc * np.cos(tc), rc * np.sin(tc)
            ed = abs((a * xc + b * yc + c) - (pa * xc + pb * yc + pc))
            if n[2] >= UPRIGHT and ed <= elev and lam <= FLAT:
                PL[(k, s)] = (a, b, c)
                own.add((k, s))
                if collect_seeds:
                    seeds.append(S)
    # 실패 칸: 같은 링 이웃 → 앞뒤 링의 겹치는 칸 평균, 없으면 사전 평면 (앞에서 채운 대체값도 이웃으로 쓴다)
    for k in range(len(NS)):
        for s in range(NS[k]):
            if (k, s) in own:
                continue
            cand = [PL[(k, t)] for t in (s - 1, s + 1) if (k, t) in PL]
            for kk in (k - 1, k + 1):
                if 0 <= kk < len(NS):
                    t = int((s + 0.5) * NS[kk] / NS[k])
                    if (kk, t) in PL:
                        cand.append(PL[(kk, t)])
            PL[(k, s)] = tuple(np.mean(cand, 0)) if cand else prior
    zg = np.empty(len(p))
    for k in range(-1, len(NS)):
        kk = max(k, 0)
        for s in range(NS[kk]):
            m = (ri == k) & (si[kk] == s)
            if m.any():
                a, b, c = PL[(kk, s)]
                zg[m] = a * p[m, 0] + b * p[m, 1] + c
    m = r >= RE[-1]
    zg[m] = pa * p[m, 0] + pb * p[m, 1] + pc
    return p[:, 2] - zg > TH, (np.vstack(seeds) if seeds else np.zeros((0, 3)))


def non_ground(p):
    """p: (N,3) 센서 좌표. 반환: 비지면 mask, 전역 지면 평면 (a, b, c) — z = a x + b y + c"""
    _, seeds = _pass(p, (0.0, 0.0, ZREF), 0.3, True)
    prior = (0.0, 0.0, ZREF)
    if len(seeds) >= 50:
        A = np.c_[seeds[:, 0], seeds[:, 1], np.ones(len(seeds))]
        a, b, c = np.linalg.lstsq(A, seeds[:, 2], rcond=None)[0]
        if abs(a) <= 0.1 and abs(b) <= 0.1 and abs(c - ZREF) <= 0.25:
            prior = (a, b, c)
    mask, _ = _pass(p, prior, 0.15, False)
    return mask, prior
