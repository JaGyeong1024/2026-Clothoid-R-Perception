#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Livox LiDAR Pre-processing + Distance-weighted Euclidean Clustering + Kalman Tracking
ROS Noetic 기준
"""

import rospy, time, numpy as np
from sensor_msgs.msg import PointCloud2, PointCloud
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32
from scipy.spatial import cKDTree
from sklearn.linear_model import RANSACRegressor
import std_msgs.msg

# -------------------- 하드코딩 파라미터 --------------------
PITCH_DEG        = 0.1
ROI_X_MIN, ROI_X_MAX = 0, 12
ROI_Y_MIN, ROI_Y_MAX = -4, 4
ROI_Z_MIN, ROI_Z_MAX = -2, 1.5
VOXEL_SIZE       = 0.1

DROR_MIN_NEIGHBORS = 3
DROR_MIN_RADIUS    = 0.1
DROR_RADIUS_SCALE  = 0.1
DROR_MAX_RADIUS    = 0.2

GRID_CELL_SIZE        = 0.2
GRID_MAX_HEIGHT_DIFF  = 0.2
GRID_MIN_POINTS       = 10

GROUND_THRESH = 0.3

EUCLIDEAN_MIN_CLUSTER_SIZE = 5
CLUSTER_MERGE_GAP          = 0.5
MAX_LENGTH, MAX_WIDTH, MAX_HEIGHT = 2.5, 2.5, 1.0
MIN_LENGTH, MIN_WIDTH, MIN_HEIGHT = 0.5, 0.5, 0.3
EUCLIDEAN_BASE_DIST, EUCLIDEAN_DIST_SCALE = 0.05, 0.05

TRACKER_MAX_MISS = 5
MATCH_DIST       = 1.5
# ---------------------------------------------------------

def load_algorithm_params():
    global PITCH_DEG, ROI_X_MIN, ROI_X_MAX, ROI_Y_MIN, ROI_Y_MAX, ROI_Z_MIN, ROI_Z_MAX
    global VOXEL_SIZE, DROR_MIN_NEIGHBORS, DROR_MIN_RADIUS, DROR_RADIUS_SCALE, DROR_MAX_RADIUS
    global GRID_CELL_SIZE, GRID_MAX_HEIGHT_DIFF, GRID_MIN_POINTS, GROUND_THRESH
    global EUCLIDEAN_MIN_CLUSTER_SIZE, CLUSTER_MERGE_GAP
    global MAX_LENGTH, MAX_WIDTH, MAX_HEIGHT, MIN_LENGTH, MIN_WIDTH, MIN_HEIGHT
    global EUCLIDEAN_BASE_DIST, EUCLIDEAN_DIST_SCALE, TRACKER_MAX_MISS, MATCH_DIST

    PITCH_DEG = rospy.get_param("~pitch_deg", PITCH_DEG)
    ROI_X_MIN = rospy.get_param("~roi_x_min", ROI_X_MIN)
    ROI_X_MAX = rospy.get_param("~roi_x_max", ROI_X_MAX)
    ROI_Y_MIN = rospy.get_param("~roi_y_min", ROI_Y_MIN)
    ROI_Y_MAX = rospy.get_param("~roi_y_max", ROI_Y_MAX)
    ROI_Z_MIN = rospy.get_param("~roi_z_min", ROI_Z_MIN)
    ROI_Z_MAX = rospy.get_param("~roi_z_max", ROI_Z_MAX)
    VOXEL_SIZE = rospy.get_param("~voxel_size", VOXEL_SIZE)

    DROR_MIN_NEIGHBORS = int(rospy.get_param("~dror_min_neighbors", DROR_MIN_NEIGHBORS))
    DROR_MIN_RADIUS = rospy.get_param("~dror_min_radius", DROR_MIN_RADIUS)
    DROR_RADIUS_SCALE = rospy.get_param("~dror_radius_scale", DROR_RADIUS_SCALE)
    DROR_MAX_RADIUS = rospy.get_param("~dror_max_radius", DROR_MAX_RADIUS)

    GRID_CELL_SIZE = rospy.get_param("~grid_cell_size", GRID_CELL_SIZE)
    GRID_MAX_HEIGHT_DIFF = rospy.get_param("~grid_max_height_diff", GRID_MAX_HEIGHT_DIFF)
    GRID_MIN_POINTS = int(rospy.get_param("~grid_min_points", GRID_MIN_POINTS))
    GROUND_THRESH = rospy.get_param("~ground_thresh", GROUND_THRESH)

    EUCLIDEAN_MIN_CLUSTER_SIZE = int(rospy.get_param("~euclidean_min_cluster_size", EUCLIDEAN_MIN_CLUSTER_SIZE))
    CLUSTER_MERGE_GAP = rospy.get_param("~cluster_merge_gap", CLUSTER_MERGE_GAP)
    MAX_LENGTH = rospy.get_param("~max_length", MAX_LENGTH)
    MAX_WIDTH = rospy.get_param("~max_width", MAX_WIDTH)
    MAX_HEIGHT = rospy.get_param("~max_height", MAX_HEIGHT)
    MIN_LENGTH = rospy.get_param("~min_length", MIN_LENGTH)
    MIN_WIDTH = rospy.get_param("~min_width", MIN_WIDTH)
    MIN_HEIGHT = rospy.get_param("~min_height", MIN_HEIGHT)
    EUCLIDEAN_BASE_DIST = rospy.get_param("~euclidean_base_dist", EUCLIDEAN_BASE_DIST)
    EUCLIDEAN_DIST_SCALE = rospy.get_param("~euclidean_dist_scale", EUCLIDEAN_DIST_SCALE)

    TRACKER_MAX_MISS = int(rospy.get_param("~tracker_max_miss", TRACKER_MAX_MISS))
    MATCH_DIST = rospy.get_param("~match_dist", MATCH_DIST)

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
        self.miss = 0; self.last = c
    def predict(self):
        self.last = self.kf.predict(); return self.last
    def update(self, c): self.kf.update(c); self.last=c; self.miss=0
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

def rot_pitch(points, deg):
    th = np.deg2rad(deg)
    R = np.array([[ np.cos(th),0, np.sin(th)],
                  [ 0,          1, 0        ],
                  [-np.sin(th),0, np.cos(th)]])
    return points @ R.T

def voxel_downsample(pts, vs):
    if len(pts)==0: return pts
    idx = np.unique(np.floor(pts/vs), axis=0, return_index=True)[1]
    return pts[idx]

def dror_filter(pts):
    if len(pts)==0: return pts
    rng = np.linalg.norm(pts[:,:2],axis=1)
    radii = np.clip(DROR_MIN_RADIUS + DROR_RADIUS_SCALE*rng,
                    DROR_MIN_RADIUS, DROR_MAX_RADIUS)
    tree = cKDTree(pts)
    counts = tree.query_ball_point(pts, radii, return_length=True)
    return pts[counts - 1 >= DROR_MIN_NEIGHBORS]

def grid_ground_remove(pts):
    if len(pts)==0: return pts
    ix = np.floor((pts[:,0]-ROI_X_MIN)/GRID_CELL_SIZE).astype(np.int64)
    iy = np.floor((pts[:,1]-ROI_Y_MIN)/GRID_CELL_SIZE).astype(np.int64)
    key = ix*100000 + iy
    _, inv = np.unique(key, return_inverse=True)
    cnt = np.bincount(inv)
    min_z = np.full(cnt.size, np.inf)
    np.minimum.at(min_z, inv, pts[:,2])
    ground = ((cnt[inv] >= GRID_MIN_POINTS) &
              (pts[:,2] - min_z[inv] < GRID_MAX_HEIGHT_DIFF))
    return pts[~ground]

def ransac_ground_remove(pts):
    if len(pts)==0: return pts
    X, y = pts[:,:2], pts[:,2]
    try:
        ransac = RANSACRegressor(residual_threshold=GROUND_THRESH).fit(X,y)
        res = np.abs(y - ransac.predict(X))
        return pts[res>GROUND_THRESH]
    except Exception:
        return pts

def dist_euclid_labels(pts):
    if len(pts)==0: return np.array([],int)
    tree = cKDTree(pts[:,:2])
    n = len(pts)
    visited = np.zeros(n,bool); lbl = -np.ones(n,int); cid=0
    for i in range(n):
        if visited[i]: continue
        d0 = EUCLIDEAN_BASE_DIST + EUCLIDEAN_DIST_SCALE*abs(pts[i,0])
        Q = tree.query_ball_point(pts[i,:2],d0)
        if len(Q) < EUCLIDEAN_MIN_CLUSTER_SIZE:
            visited[i]=True; continue
        stack = list(Q); lbl[stack]=cid; visited[stack]=True
        while stack:
            cur=stack.pop()
            d = EUCLIDEAN_BASE_DIST+EUCLIDEAN_DIST_SCALE*abs(pts[cur,0])
            for nb in tree.query_ball_point(pts[cur,:2],d):
                if not visited[nb]:
                    visited[nb]=True
                    if len(tree.query_ball_point(pts[nb,:2],d))>=EUCLIDEAN_MIN_CLUSTER_SIZE:
                        stack.append(nb)
                    lbl[nb]=cid
        cid+=1
    return lbl

def merge_clusters(pts, lbl):
    uniq=set(lbl); uniq.discard(-1)
    if not uniq: return lbl
    cent={l:np.mean(pts[lbl==l],axis=0) for l in uniq}
    rep={}; merged=set()
    for l1 in uniq:
        if l1 in merged: continue
        rep[l1]=l1
        for l2 in uniq:
            if l1==l2 or l2 in merged: continue
            if np.linalg.norm(cent[l1][:2]-cent[l2][:2])<CLUSTER_MERGE_GAP:
                rep[l2]=l1; merged.add(l2)
    return np.array([rep.get(x,-1) if x!=-1 else -1 for x in lbl])

def bbox_ok(pts):
    if len(pts)==0: return False
    xl,yl,zl = np.ptp(pts[:,0]), np.ptp(pts[:,1]), np.ptp(pts[:,2])
    return (MIN_LENGTH<xl<MAX_LENGTH and
            MIN_WIDTH <yl<MAX_WIDTH  and
            MIN_HEIGHT<zl<MAX_HEIGHT)

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
        pts = rot_pitch(pts, PITCH_DEG)
        mask = ((ROI_X_MIN <= pts[:, 0]) & (pts[:, 0] <= ROI_X_MAX) &
                (ROI_Y_MIN <= pts[:, 1]) & (pts[:, 1] <= ROI_Y_MAX) &
                (ROI_Z_MIN <= pts[:, 2]) & (pts[:, 2] <= ROI_Z_MAX))
        pts = pts[mask]
        pts = dror_filter(voxel_downsample(pts, VOXEL_SIZE))
        return ransac_ground_remove(grid_ground_remove(pts))

    def _cluster_observations(self, pts):
        lbl = merge_clusters(pts, dist_euclid_labels(pts))
        observed = [np.mean(pts[lbl == cid][:, :2], axis=0)
                    for cid in set(lbl) if cid != -1 and bbox_ok(pts[lbl == cid])]
        return np.asarray(observed, dtype=float)

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

        pts = self._preprocess(pts)
        self.publish_preprocessed(pts)

        observed = self._cluster_observations(pts)
        self._track(observed)
        self.publish_centroids([t.last for t in self.trackers.values()])
        rospy.logdebug(f"callback {(time.time()-start):.3f}s")

# ---------- 노드 초기화 ----------
if __name__ == "__main__":
    LivoxEuclideanClustering()
    rospy.spin()
