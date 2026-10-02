#!/usr/bin/env python3
# Velodyne PointCloud -> BEV -> 정규화 HSV -> YOLO(NMS 추론) -> OC-SORT

import os
# numpy(OpenBLAS) 연산을 1스레드로. 기본 다중 스레드는 물체 검출 중 CPU 270%까지 치솟고 출력은 같다.
# numpy 를 가져오기 전에 설정해야 적용된다 (차량은 rosrun 이라 launch 의 env 로는 안 됨).
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import sys, rospy, numpy as np, cv2
from sensor_msgs.msg import PointCloud2, PointCloud, Image
from sensor_msgs.msg import PointField
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, _HERE)   # bev_input (점군 → BEV → 정규화 HSV, 학습 도구와 같은 정본)
# 레포 안 ultralytics(perception_ws/yolo26, YOLO26). 시스템 8.0.196 은 yolo26 모델을 못 읽는다
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "../../../yolo26")))
from ultralytics import YOLO
from bev_input import bev_from_points, REPRESENTATIONS

# OC_SORT 경로: env OC_SORT_PATH 또는 rosparam ~ocsort_path
_OCSORT_DEFAULT = os.environ.get("OC_SORT_PATH", "/opt/OC_SORT")

# ---- 기본 파라미터 (rosparam 으로 덮어씀) ----
VOXEL_SIZE          = 0.05
X_RANGE             = (-15.0, 15.0)
Y_RANGE             = (-15.0, 15.0)
Z_RANGE             = (-2.5,  2.0)
MAX_PTS_PER_VOXEL   = 30
DETECT_CONF         = 0.5
NMS_IOU             = 0.01   # BEV 에서는 물체가 겹칠 수 없다. 겹친 박스는 중복이다
REPRESENTATION      = "hsv_v1"
OCSORT_IOU_THRESH   = 0.25
OCSORT_MAX_AGE      = 10
OCSORT_MIN_HITS     = 3
OCSORT_DELTA_T      = 1
HEARTBEAT_HZ        = 10


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


def parse_xyzi_points(msg):
    fields = {f.name: f for f in msg.fields}
    xyz_names = ("x", "y", "z")
    has_xyz = all(name in fields and fields[name].datatype == PointField.FLOAT32
                  for name in xyz_names)
    has_intensity = ("intensity" in fields and
                     fields["intensity"].datatype == PointField.FLOAT32)
    if not has_xyz:
        use_i = any(f.name == "intensity" for f in msg.fields)
        read_fields = ("x", "y", "z", "intensity") if use_i else ("x", "y", "z")
        pts = np.asarray([p for p in pc2.read_points(msg, field_names=read_fields, skip_nans=True)],
                         dtype=np.float32)
        if not use_i and pts.size:
            pts = np.hstack([pts, np.zeros((pts.shape[0], 1), np.float32)])
        return pts

    raw = _pointcloud2_rows(msg)
    dtype = ">f4" if msg.is_bigendian else "<f4"
    pts = np.empty((raw.shape[0], 4), dtype=np.float32)
    for i, name in enumerate(xyz_names):
        off = fields[name].offset
        pts[:, i] = raw[:, off:off + 4].copy().view(dtype).reshape(-1)

    if has_intensity:
        off = fields["intensity"].offset
        pts[:, 3] = raw[:, off:off + 4].copy().view(dtype).reshape(-1)
    else:
        pts[:, 3] = 0.0

    pts = pts[np.isfinite(pts[:, :3]).all(axis=1)]
    return pts


class VelodyneBevDetection:
    def __init__(self):
        rospy.init_node("velodyne_bev_detection", anonymous=True)
        rospy.loginfo("[velodyne_bev_detection] start")

        # OC_SORT path
        ocsort_path = rospy.get_param("~ocsort_path", _OCSORT_DEFAULT)
        if ocsort_path not in sys.path:
            sys.path.insert(0, ocsort_path)
        from trackers.ocsort_tracker.ocsort import OCSort

        # Model
        model_path = rospy.get_param("~model_path", os.path.join(
            _HERE, "..", "model", "velodyne_v7.pt"))
        if not model_path:
            rospy.logerr("[velodyne_bev_detection] ~model_path not set")
            raise RuntimeError("~model_path is required")
        self.detector = YOLO(model_path)
        self.device   = rospy.get_param("~device", "cuda")

        # BEV params
        self.voxel   = rospy.get_param("~voxel_size", VOXEL_SIZE)
        self.x_rng   = tuple(rospy.get_param("~x_range", list(X_RANGE)))
        self.y_rng   = tuple(rospy.get_param("~y_range", list(Y_RANGE)))
        self.z_rng   = tuple(rospy.get_param("~z_range", list(Z_RANGE)))
        self.max_pts = rospy.get_param("~max_points_per_voxel", MAX_PTS_PER_VOXEL)
        self.conf    = rospy.get_param("~detect_conf", DETECT_CONF)
        self.nms_iou = rospy.get_param("~nms_iou", NMS_IOU)
        rep_name     = rospy.get_param("~representation", REPRESENTATION)   # 모델과 짝: v7 = hsv_v1, v6 = raw
        self.rep     = REPRESENTATIONS[rep_name]
        self.frame_id = rospy.get_param("~frame_id", "velodyne")

        self.img_h = int((self.x_rng[1] - self.x_rng[0]) / self.voxel)
        self.img_w = int((self.y_rng[1] - self.y_rng[0]) / self.voxel)

        # 첫 추론은 CUDA 초기화로 1~2초 걸린다. 그동안 들어온 스캔이 버려지지 않게 시작할 때 한 번 미리 돌린다
        self.detector.predict(np.zeros((self.img_h, self.img_w, 3), np.uint8), device=self.device, conf=self.conf,
                              iou=self.nms_iou, end2end=False, verbose=False)

        self.tracker = OCSort(det_thresh    = self.conf,
                              iou_threshold = OCSORT_IOU_THRESH,
                              max_age       = OCSORT_MAX_AGE,
                              min_hits      = OCSORT_MIN_HITS,
                              delta_t       = OCSORT_DELTA_T)

        # 트랙 발행 규칙
        #   streak:    OCSort 기본. 1프레임만 놓쳐도 hit_streak 이 0 이 되어 다시 min_hits 연속까지 발행 안 함
        #   confirmed: 한 번 min_hits 에 도달한 트랙은 confirmed_max_gap 프레임 이내로 놓쳤다가
        #              다시 매칭되면 바로 발행. 공백이 더 길면 확정을 풀고 다시 min_hits 연속을 요구
        self.publish_policy = rospy.get_param("~publish_policy", "confirmed")
        if self.publish_policy not in ("streak", "confirmed"):
            rospy.logwarn(f"[velodyne_bev_detection] unknown publish_policy={self.publish_policy}; using confirmed")
            self.publish_policy = "confirmed"
        self.confirmed_max_gap = int(rospy.get_param("~confirmed_max_gap", 3))
        self.confirmed_ids = set()
        self.last_match = {}   # 트랙 id -> 마지막으로 매칭된 tracker.frame_count

        self.bridge = CvBridge()

        # Topics
        input_topic    = rospy.get_param("~input_topic",    "/velodyne_points")
        centroid_topic = rospy.get_param("~centroid_topic", "/perception/velodyne/centroids")
        marker_topic   = rospy.get_param("~marker_topic",   "/perception/velodyne/markers")
        image_topic    = rospy.get_param("~image_topic",    "/perception/velodyne/bev_image")

        # buff_size: 점군 메시지(약 0.5 MB)가 기본 버퍼(64 KB)보다 커서, 없으면 처리가 밀릴 때 옛 메시지가 쌓여 지연이 누적된다
        rospy.Subscriber(input_topic, PointCloud2, self.lidar_cb, queue_size=1, buff_size=2**24)
        self.pcl_pub    = rospy.Publisher(centroid_topic, PointCloud, queue_size=10)
        self.marker_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=10)
        self.img_pub    = rospy.Publisher(image_topic, Image, queue_size=10)

        self.last_msg = PointCloud()
        self.last_msg.header.frame_id = self.frame_id
        # 입력이 stale_timeout 이상 끊기면 빈 메시지 발행 (유령 장애물 방지)
        self.stale_timeout = rospy.get_param("~stale_timeout", 0.5)
        self.last_input_time = None
        rospy.Timer(rospy.Duration(1.0 / HEARTBEAT_HZ), self.timer_cb)

        rospy.loginfo(f"[velodyne_bev_detection] subscribe={input_topic} "
                      f"-> centroids={centroid_topic}")
        rospy.loginfo(f"[velodyne_bev_detection] model={model_path} representation={rep_name} "
                      f"conf={self.conf} nms_iou={self.nms_iou}")
        rospy.loginfo(f"[velodyne_bev_detection] publish_policy={self.publish_policy} "
                      f"confirmed_max_gap={self.confirmed_max_gap}")

    def timer_cb(self, _):
        now = rospy.get_time()
        if self.last_input_time is None or now - self.last_input_time > self.stale_timeout:
            rospy.logwarn_throttle(2.0, "[velodyne_bev_detection] input stale (>%.1fs); publishing empty" % self.stale_timeout)
            empty = PointCloud()
            empty.header.frame_id = self.frame_id
            empty.header.stamp = rospy.Time.now()
            self.pcl_pub.publish(empty)
            self.last_msg = empty
            return
        self.pcl_pub.publish(self.last_msg)   # 직전 결과를 스캔 시각 그대로 다시 낸다(나이를 알 수 있게)

    def pc2_to_bev(self, pts):
        return bev_from_points(pts, self.voxel, self.x_rng, self.y_rng, self.z_rng, self.max_pts)

    def track_step(self, dets):
        """dets [M,5] -> 발행할 트랙 [K,5] (x1,y1,x2,y2,id)."""
        tracks = self.tracker.update(dets,
                                     (self.img_h, self.img_w),
                                     (self.img_h, self.img_w))
        if self.publish_policy == "streak":
            return tracks

        # OCSort.update 의 발행 루프와 같은 순서·박스 선택에 '확정 후 짧은 공백' 조건만 더한다
        f = self.tracker.frame_count
        out = []
        for trk in reversed(self.tracker.trackers):
            if trk.time_since_update >= 1:      # 이번 프레임에 매칭되지 않음
                continue
            gap = f - self.last_match.get(trk.id, f - 1) - 1
            self.last_match[trk.id] = f
            if gap > self.confirmed_max_gap:
                self.confirmed_ids.discard(trk.id)
            if trk.hit_streak >= OCSORT_MIN_HITS:
                self.confirmed_ids.add(trk.id)
            if (trk.hit_streak >= OCSORT_MIN_HITS or f <= OCSORT_MIN_HITS
                    or trk.id in self.confirmed_ids):
                if trk.last_observation.sum() < 0:
                    box = trk.get_state()[0]
                else:
                    box = trk.last_observation[:4]
                out.append(np.concatenate((box, [trk.id + 1])))
        alive = {trk.id for trk in self.tracker.trackers}
        self.confirmed_ids &= alive
        self.last_match = {k: v for k, v in self.last_match.items() if k in alive}
        return np.asarray(out, dtype=np.float64).reshape(-1, 5)

    def lidar_cb(self, msg: PointCloud2):
        self.last_input_time = rospy.get_time()
        pts = parse_xyzi_points(msg)

        bev = self.pc2_to_bev(pts)
        vis = bev.copy()

        # NMS 추론(end2end=False): yolo26 기본(NMS 없음)은 한 물체에 거의 같은 박스를 가끔 두 개 내서 트랙이 쪼개진다
        dets = np.asarray([[*b.xyxy[0].cpu().numpy(), float(b.conf[0])]
                           for b in self.detector.predict(self.rep(bev), device=self.device, conf=self.conf,
                                                          iou=self.nms_iou, end2end=False, verbose=False)[0].boxes],
                          dtype=np.float32)
        if dets.size == 0:
            dets = np.empty((0,5), np.float32)

        tracks = self.track_step(dets)

        pcl_msg = PointCloud()
        pcl_msg.header.frame_id = self.frame_id
        pcl_msg.header.stamp = msg.header.stamp   # 스캔 시각
        m_arr = MarkerArray()

        for x1,y1,x2,y2,tid in tracks:
            cx, cy = (x1+x2)/2, (y1+y2)/2
            lx = self.x_rng[1] - cy*self.voxel   # 픽셀 iy = (x_max - x)/voxel 의 역변환
            ly = self.y_rng[1] - cx*self.voxel
            pcl_msg.points.append(Point32(lx,ly,0.0))

            mk = Marker(); mk.header = pcl_msg.header
            mk.ns, mk.id = "track", int(tid); mk.type = Marker.TEXT_VIEW_FACING
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = lx, ly, 0.6
            mk.text = f"{int(tid)}"; mk.scale.z = 0.45
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1,1,0,1
            m_arr.markers.append(mk)

            cv2.rectangle(vis,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),2)
            cv2.putText(vis,f"ID{int(tid)}",(int(x1),int(y1)-7),
                        cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,0),1)

        self.pcl_pub.publish(pcl_msg)
        self.marker_pub.publish(m_arr)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(vis,"bgr8"))
        self.last_msg = pcl_msg


if __name__ == "__main__":
    VelodyneBevDetection()
    rospy.spin()
