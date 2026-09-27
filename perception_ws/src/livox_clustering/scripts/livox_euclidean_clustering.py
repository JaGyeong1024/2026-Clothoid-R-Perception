#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Livox LiDAR: 지면 제거(horizon_ground) + XY 클러스터링 + Kalman Tracking
ROS Noetic 기준
"""

import rospy, time, numpy as np
from sensor_msgs.msg import PointCloud2, PointCloud
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import std_msgs.msg

from livox_clustering import horizon_ground

# ---- 기본 파라미터 (yaml / rosparam 으로 덮어씀) ----
ROI_X_MIN, ROI_X_MAX = 0, 8
ROI_Y_MIN, ROI_Y_MAX = -3, 3
ROI_Z_MAX        = 1.5
VOXEL_SIZE       = 0.1

DROR_MIN_NEIGHBORS = 3
DROR_MIN_RADIUS    = 0.1
DROR_RADIUS_SCALE  = 0.1
DROR_MAX_RADIUS    = 0.2

# 지면 평면은 발행 ROI 보다 넓은 영역(지면이 보이는 2.9 m ~ 15.6 m)에서 맞춘다
GROUND_X_MAX, GROUND_Y_ABS, GROUND_Z_ABS = 15.0, 7.0, 2.0

# 물체: 지면 제거 후 점을 XY 고정 간격으로 묶는다. 콘 벽(콘 간격 ~0.5 m)도 콘 단위로 갈라진다.
CLUSTER_EPS        = 0.15
CLUSTER_MIN_POINTS = 3
OBJ_MIN_HEIGHT     = 0.3    # 지면 평면 위 최고점 높이
OBJ_MAX_HEIGHT     = 2.0
OBJ_MAX_FOOTPRINT  = 2.5    # 수평 최대 변

TRACKER_MAX_MISS = 5
MATCH_DIST       = 1.5
TRACKER_MIN_HITS = 3   # 이 프레임 수 이상 연속 관측된 트랙만 발행

def load_algorithm_params():
    g = globals()
    for name in ("ROI_X_MIN", "ROI_X_MAX", "ROI_Y_MIN", "ROI_Y_MAX", "ROI_Z_MAX", "VOXEL_SIZE",
                 "DROR_MIN_RADIUS", "DROR_RADIUS_SCALE", "DROR_MAX_RADIUS",
                 "CLUSTER_EPS", "OBJ_MIN_HEIGHT", "OBJ_MAX_HEIGHT", "OBJ_MAX_FOOTPRINT", "MATCH_DIST"):
        g[name] = float(rospy.get_param("~" + name.lower(), g[name]))
    for name in ("DROR_MIN_NEIGHBORS", "CLUSTER_MIN_POINTS", "TRACKER_MAX_MISS", "TRACKER_MIN_HITS"):
        g[name] = int(rospy.get_param("~" + name.lower(), g[name]))

# ---------- 보조 클래스 ----------
class KalmanFilter:
    def __init__(self, dt=0.1, q=5.0, r=0.1):
        self.A = np.array([[1,0,dt,0], [0,1,0,dt], [0,0,1,0], [0,0,0,1]])
        self.H = np.array([[1,0,0,0],  [0,1,0,0]])
        self.Q = q*np.eye(4)
        self.R = r*np.eye(2)
        self.P = np.eye(4)
        self.x = np.zeros((4,1))
    def predict(self):
        self.x = self.A@self.x
        self.P = self.A@self.P@self.A.T + self.Q
        return self.x[:2].flatten()
    def update(self, z):
        z = np.reshape(z,(2,1))
        y = z - self.H@self.x
        S = self.H@self.P@self.H.T + self.R
        K = self.P@self.H.T@np.linalg.inv(S)
        self.x += K@y
        self.P = (np.eye(4)-K@self.H)@self.P

class Tracker:
    def __init__(self, c, tid, dt=0.1):
        self.id = tid; self.kf = KalmanFilter(dt); self.kf.update(c)
        self.miss = 0; self.last = c; self.hits = 1
    def predict(self):
        self.last = self.kf.predict(); return self.last
    def update(self, c): self.kf.update(c); self.last=c; self.miss=0; self.hits+=1
    def no_update(self): self.kf.predict(); self.miss+=1

# ---------- 유틸리티 ----------
def _pointcloud2_rows(msg):
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, msg.point_step), dtype=np.uint8)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.row_step == msg.point_step * msg.width:
        return raw.reshape(n, msg.point_step)

    rows = []
    for row in range(msg.height):
        start = row * msg.row_step
        end = start + msg.point_step * msg.width
        rows.append(raw[start:end].reshape(msg.width, msg.point_step))
    return np.concatenate(rows, axis=0) if rows else np.empty((0, msg.point_step), dtype=np.uint8)

def parse_xyz_points(msg):
    fields = {f.name: f for f in msg.fields}
    required = ("x", "y", "z")
    if any(name not in fields or fields[name].datatype != PointField.FLOAT32 for name in required):
        return np.array([[p[0], p[1], p[2]] for p in
                         pc2.read_points(msg, field_names=required, skip_nans=True)])

    raw = _pointcloud2_rows(msg)
    dtype = ">f4" if msg.is_bigendian else "<f4"
    xyz = np.empty((raw.shape[0], 3), dtype=np.float32)
    for i, name in enumerate(required):
        off = fields[name].offset
        xyz[:, i] = raw[:, off:off + 4].copy().view(dtype).reshape(-1)

    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz.astype(np.float64, copy=False)

def voxel_downsample(pts, vs):
    if len(pts)==0: return pts
    idx = np.unique(np.floor(pts/vs), axis=0, return_index=True)[1]
    return pts[idx]

def dror_mask(pts):
    """거리 비례 반경 안 이웃이 DROR_MIN_NEIGHBORS 미만인 외톨이 점 제거"""
    if len(pts)==0: return np.zeros(0, bool)
    rng = np.linalg.norm(pts[:,:2],axis=1)
    radii = np.clip(DROR_MIN_RADIUS + DROR_RADIUS_SCALE*rng,
                    DROR_MIN_RADIUS, DROR_MAX_RADIUS)
    tree = cKDTree(pts)
    counts = tree.query_ball_point(pts, radii, return_length=True)
    return counts - 1 >= DROR_MIN_NEIGHBORS

def cluster_objects(pts, ground):
    """XY 고정 간격 연결 성분 → 크기·높이 조건을 통과한 클러스터의 XY 중심"""
    if len(pts) < CLUSTER_MIN_POINTS: return np.zeros((0, 2))
    pairs = cKDTree(pts[:,:2]).query_pairs(CLUSTER_EPS, output_type="ndarray")
    g = coo_matrix((np.ones(len(pairs)), (pairs[:,0], pairs[:,1])), shape=(len(pts), len(pts)))
    _, lbl = connected_components(g, directed=False)
    a, b, c = ground
    out = []
    for l in np.unique(lbl):
        C = pts[lbl == l]
        if len(C) < CLUSTER_MIN_POINTS: continue
        foot = max(np.ptp(C[:,0]), np.ptp(C[:,1]))
        h = (C[:,2] - (a*C[:,0] + b*C[:,1] + c)).max()
        if foot <= OBJ_MAX_FOOTPRINT and OBJ_MIN_HEIGHT <= h <= OBJ_MAX_HEIGHT:
            out.append(C[:,:2].mean(0))
    return np.asarray(out, float).reshape(-1, 2)


class LivoxEuclideanClustering:
    def __init__(self):
        rospy.init_node("livox_euclidean_clustering")

        self.input_topic = rospy.get_param("~input_topic", "/livox/lidar")
        self.centroid_topic = rospy.get_param("~centroid_topic", "/perception/livox/centroids")
        self.preprocessed_topic = rospy.get_param("~preprocessed_topic", "/perception/livox/preprocessed")
        self.frame_id = rospy.get_param("~frame_id", "livox_frame")
        load_algorithm_params()

        self.trackers = {}
        self.tid_seq = 0

        self.pre_pub = rospy.Publisher(self.preprocessed_topic, PointCloud2, queue_size=1)
        self.cent_pub = rospy.Publisher(self.centroid_topic, PointCloud, queue_size=1)
        rospy.Subscriber(self.input_topic, PointCloud2, self.pc_callback, queue_size=1)

        rospy.loginfo(f"[livox_euclidean_clustering] subscribe={self.input_topic} "
                      f"-> centroid={self.centroid_topic}, preprocessed={self.preprocessed_topic}")

    def publish_preprocessed(self, pts):
        hdr = std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        self.pre_pub.publish(pc2.create_cloud_xyz32(hdr, pts))

    def publish_centroids(self, c_list):
        pc_msg = PointCloud()
        pc_msg.header.stamp = rospy.Time.now()
        pc_msg.header.frame_id = self.frame_id
        pc_msg.points = [Point32(x, y, 0.0) for x, y in c_list]
        self.cent_pub.publish(pc_msg)

    def _parse_points(self, msg):
        return parse_xyz_points(msg)

    def _preprocess(self, pts):
        """지면 제거 → 발행 ROI → voxel → DROR. 반환: 물체 후보 점, 지면 평면"""
        g = pts[(pts[:, 0] >= 0) & (pts[:, 0] <= GROUND_X_MAX) &
                (np.abs(pts[:, 1]) <= GROUND_Y_ABS) & (np.abs(pts[:, 2]) <= GROUND_Z_ABS)]
        if len(g) == 0:
            return g, (0.0, 0.0, horizon_ground.ZREF)
        ng, ground = horizon_ground.non_ground(g)
        q = g[ng]
        q = q[(ROI_X_MIN <= q[:, 0]) & (q[:, 0] <= ROI_X_MAX) &
              (ROI_Y_MIN <= q[:, 1]) & (q[:, 1] <= ROI_Y_MAX) & (q[:, 2] <= ROI_Z_MAX)]
        q = voxel_downsample(q, VOXEL_SIZE)
        return q[dror_mask(q)], ground

    def _cluster_observations(self, pts, ground):
        return cluster_objects(pts, ground)

    def _track(self, observed):
        preds = np.array([t.predict() for t in self.trackers.values()]) if self.trackers else np.zeros((0, 2))
        keys = list(self.trackers.keys())
        assigned_trk, assigned_obs = set(), set()
        if len(observed) and len(preds):
            dist = np.linalg.norm(observed[:, None, :] - preds[None, :, :], axis=2)
            while True:
                i, j = np.unravel_index(np.argmin(dist), dist.shape)
                if dist[i, j] > MATCH_DIST:
                    break
                self.trackers[keys[j]].update(observed[i])
                assigned_trk.add(keys[j]); assigned_obs.add(i)
                dist[i, :] = dist[:, j] = np.inf
                if np.isinf(dist).all():
                    break
        for k, t in list(self.trackers.items()):
            if k not in assigned_trk:
                t.no_update()
                if t.miss > TRACKER_MAX_MISS:
                    del self.trackers[k]
        for i, c in enumerate(observed):
            if i not in assigned_obs:
                self.trackers[self.tid_seq] = Tracker(c, self.tid_seq)
                self.tid_seq += 1

    def pc_callback(self, msg):
        start = time.time()

        pts = self._parse_points(msg)
        if pts.size == 0:
            self.publish_preprocessed([])
            self.publish_centroids([])
            return

        pts, ground = self._preprocess(pts)
        self.publish_preprocessed(pts)

        observed = self._cluster_observations(pts, ground)
        self._track(observed)
        self.publish_centroids([t.last for t in self.trackers.values()
                                if t.hits >= TRACKER_MIN_HITS and t.miss == 0])
        rospy.logdebug(f"callback {(time.time()-start):.3f}s")

# ---------- 노드 초기화 ----------
if __name__ == "__main__":
    LivoxEuclideanClustering()
    rospy.spin()
