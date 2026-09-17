#!/home/cnu/anaconda3/envs/yolo/bin/python
import os
import sys
import logging
import warnings

import cv2
import numpy as np
import rospy
from detect_msgs.msg import Objects, Yolo_Objects
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

# 커스텀 ultralytics: <이 워크스페이스>/yolo26 (절대경로 하드코딩 제거 — clothoid-r_jg 격리 지원)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "yolo26")))
from ultralytics import YOLO

logging.getLogger("ultralytics").setLevel(logging.ERROR)
warnings.filterwarnings("ignore", category=UserWarning)

# 검출 결과 영상 표시 여부 기본값 — rosparam ~show_image로 제어 (대회/headless 기본: False)
SHOW_DETECTION_IMAGE = False

# True 이면 검출 결과 로그 영역만 갱신해서 현재 상태만 깔끔하게 보여줍니다.
# 초기화 로그(MODEL LOADED, pt_weights 등)는 그대로 유지됩니다.
CLEAR_TERMINAL_ON_DETECTION = False

WINDOW_NAME = "YOLO BBox"
DEFAULT_SOURCE_TOPIC = "/camera/image_raw/compressed"
DEFAULT_PUBLISH_TOPIC = "/perception/camera/yolo"
DEFAULT_FRAME_ID = "camera_link"
PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
# 모델 구조는 yaml에서 만들고, checkpoint는 weight 로딩에만 사용합니다.
DEFAULT_YAML_CFG = os.path.join(PACKAGE_DIR, "models", "best.yaml")
DEFAULT_PT_WEIGHTS = os.path.join(PACKAGE_DIR, "models", "best.pt")
DEFAULT_POSTPROCESS_NMS_IOU = 0.5
DEFAULT_POSTPROCESS_CONTAINMENT_IOA = 0.8
DEFAULT_POSTPROCESS_AREA_WEIGHT = 0.1
DEFAULT_POSTPROCESS_MAX_DET = 0
DEFAULT_POSTPROCESS_DEBUG = False


# 클래스별 기본 설정입니다.
# publish 값을 True로 둔 클래스만 /perception/camera/yolo 토픽으로 publish 합니다.
# name: 로그와 디버그 화면에 표시할 클래스 이름입니다.
# publish: True이면 해당 클래스를 publish하고, False이면 검출되어도 버립니다.
# confidence: 클래스별 최소 신뢰도입니다. 이 값보다 낮은 bbox는 publish하지 않습니다.
DEFAULT_CLASS_CONFIG = {
    0: {
        "name": "ERP-42",
        "publish": True,
        "confidence": 0.5,
    },
    1: {
        "name": "drum",
        "publish": False,
        "confidence": 0.5,
    },
    2: {
        "name": "cone",
        "publish": False,
        "confidence": 0.5,
    },
}

def box_area(box):
    return max(0.0, box["x2"] - box["x1"]) * max(0.0, box["y2"] - box["y1"])


def intersection_area(a, b):
    x1 = max(a["x1"], b["x1"])
    y1 = max(a["y1"], b["y1"])
    x2 = min(a["x2"], b["x2"])
    y2 = min(a["y2"], b["y2"])
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def iou(a, b):
    inter = intersection_area(a, b)
    if inter <= 0.0:
        return 0.0
    union = box_area(a) + box_area(b) - inter
    return inter / union if union > 0.0 else 0.0


def smaller_ioa(a, b):
    smaller = min(box_area(a), box_area(b))
    if smaller <= 0.0:
        return 0.0
    return intersection_area(a, b) / smaller


def add_area_scores(detections, area_weight):
    if not detections:
        return []

    max_area = max(box_area(det) for det in detections)
    max_area = max(max_area, 1.0)
    scored = []
    for det in detections:
        item = det.copy()
        item["area"] = box_area(item)
        item["score"] = item["conf"] + area_weight * (item["area"] / max_area)
        scored.append(item)
    return scored


def area_aware_nms(detections, iou_threshold, area_weight):
    pending = sorted(
        add_area_scores(detections, area_weight),
        key=lambda det: (det["score"], det["area"], det["conf"]),
        reverse=True,
    )
    kept = []

    while pending:
        current = pending.pop(0)
        kept.append(current)
        pending = [
            det for det in pending
            if iou(current, det) <= iou_threshold
        ]

    return kept


def containment_suppression(detections, ioa_threshold):
    detections = sorted(
        detections,
        key=lambda det: (det["area"], det["score"], det["conf"]),
        reverse=True,
    )
    suppressed = [False] * len(detections)

    for i, large in enumerate(detections):
        if suppressed[i]:
            continue
        for j, small in enumerate(detections):
            if i == j or suppressed[j]:
                continue
            if large["area"] < small["area"]:
                continue
            if smaller_ioa(large, small) >= ioa_threshold:
                suppressed[j] = True

    return [
        det for det, is_suppressed in zip(detections, suppressed)
        if not is_suppressed
    ]


def postprocess_detections(
    detections,
    iou_threshold=0.5,
    containment_ioa_threshold=0.8,
    area_weight=0.1,
    max_det=0,
):
    by_class = {}
    for det in detections:
        by_class.setdefault(det["cls_id"], []).append(det)

    kept = []
    for cls_id in sorted(by_class):
        class_dets = by_class[cls_id]
        class_kept = area_aware_nms(class_dets, iou_threshold, area_weight)
        class_kept = containment_suppression(class_kept, containment_ioa_threshold)
        kept.extend(class_kept)

    kept = sorted(
        kept,
        key=lambda det: (
            det.get("score", det["conf"]),
            det.get("area", box_area(det)),
            det["conf"],
        ),
        reverse=True,
    )

    if max_det and max_det > 0:
        kept = kept[:max_det]

    for det in kept:
        det.pop("area", None)
        det.pop("score", None)
    return kept



class YoloDetectNode:
    def __init__(self):
        rospy.init_node("yolo_detect_node")

        self.win_name = WINDOW_NAME
        self.class_config = {
            class_id: config.copy()
            for class_id, config in DEFAULT_CLASS_CONFIG.items()
        }
        self.previous_status_line_count = 0
        global SHOW_DETECTION_IMAGE
        SHOW_DETECTION_IMAGE = bool(rospy.get_param("~show_image", SHOW_DETECTION_IMAGE))
        # 추론 해상도. 0 = 원본 해상도(기본, 학습 해상도와 일치). 모델을 저해상도로 재학습하면
        # 이 파라미터로 맞출 것 (실측: 960 추론 시 15.7→7.0ms, 단 학습 해상도와 불일치 상태 검증 필요)
        self.imgsz = int(rospy.get_param("~imgsz", 0))
        source_topic = rospy.get_param("~source", DEFAULT_SOURCE_TOPIC)
        publish_topic = rospy.get_param("~output_topic", DEFAULT_PUBLISH_TOPIC)
        yaml_cfg = rospy.get_param("~yaml_cfg", DEFAULT_YAML_CFG)
        pt_weights = rospy.get_param("~pt_weights", DEFAULT_PT_WEIGHTS)
        self.frame_id = rospy.get_param("~frame_id", DEFAULT_FRAME_ID)
        self.postprocess_nms_iou = rospy.get_param(
            "~postprocess_nms_iou",
            DEFAULT_POSTPROCESS_NMS_IOU,
        )
        self.postprocess_containment_ioa = rospy.get_param(
            "~postprocess_containment_ioa",
            DEFAULT_POSTPROCESS_CONTAINMENT_IOA,
        )
        self.postprocess_area_weight = rospy.get_param(
            "~postprocess_area_weight",
            DEFAULT_POSTPROCESS_AREA_WEIGHT,
        )
        self.postprocess_max_det = rospy.get_param(
            "~postprocess_max_det",
            rospy.get_param("~max_det", DEFAULT_POSTPROCESS_MAX_DET),
        )
        self.postprocess_debug = rospy.get_param(
            "~postprocess_debug",
            DEFAULT_POSTPROCESS_DEBUG,
        )
        self.erp42_confidence = rospy.get_param(
            "~erp42_confidence",
            self.class_config[0]["confidence"],
        )
        self.drum_confidence = rospy.get_param(
            "~drum_confidence",
            self.class_config[1]["confidence"],
        )
        self.cone_confidence = rospy.get_param(
            "~cone_confidence",
            self.class_config[2]["confidence"],
        )
        self.class_config[0]["confidence"] = self.erp42_confidence
        self.class_config[1]["confidence"] = self.drum_confidence
        self.class_config[2]["confidence"] = self.cone_confidence
        self.publish_classes = {
            class_id
            for class_id, config in self.class_config.items()
            if config["publish"]
        }
        published_confidences = [
            config["confidence"]
            for config in self.class_config.values()
            if config["publish"]
        ]
        self.conf_thres = min(published_confidences) if published_confidences else 1.0

        self.pub = rospy.Publisher(publish_topic, Yolo_Objects, queue_size=1)

        self.model = YOLO(yaml_cfg, task="detect").load(pt_weights)
        rospy.loginfo(f"[yolo_detect_node] YOLOv12 MODEL LOADED")
        rospy.loginfo(f"[yolo_detect_node] yaml_cfg: {yaml_cfg}")
        rospy.loginfo(f"[yolo_detect_node] pt_weights: {pt_weights}")
        rospy.loginfo(f"[yolo_detect_node] frame_id: {self.frame_id}")
        rospy.loginfo(
            f"[yolo_detect_node] inference confidence: {self.conf_thres}"
        )
        rospy.loginfo(
            f"[yolo_detect_node] erp42_confidence: {self.erp42_confidence}"
        )
        rospy.loginfo(
            f"[yolo_detect_node] drum_confidence: {self.drum_confidence}"
        )
        rospy.loginfo(
            f"[yolo_detect_node] cone_confidence: {self.cone_confidence}"
        )
        rospy.loginfo(
            f"[yolo_detect_node] publish_classes: "
            f"{sorted(self.publish_classes) if self.publish_classes else []}"
        )
        rospy.loginfo(
            f"[yolo_detect_node] postprocess: "
            f"nms_iou={self.postprocess_nms_iou}, "
            f"containment_ioa={self.postprocess_containment_ioa}, "
            f"area_weight={self.postprocess_area_weight}, "
            f"max_det={self.postprocess_max_det}, "
            f"debug={self.postprocess_debug}"
        )

        rospy.Subscriber(source_topic,
                         CompressedImage,
                         self.callback,
                         queue_size=1,
                         buff_size=2**24)
        rospy.loginfo(f"[yolo_detect_node] Subscribed to {source_topic}")
        rospy.loginfo(f"[yolo_detect_node] Publishing to {publish_topic}")

    def _print_detection_status(self, status_lines):
        if not status_lines:
            return

        if CLEAR_TERMINAL_ON_DETECTION and sys.stdout.isatty():
            if self.previous_status_line_count > 0:
                sys.stdout.write(f"\033[{self.previous_status_line_count}F")
                sys.stdout.write("\033[J")

            for line in status_lines:
                sys.stdout.write(f"{line}\n")
            sys.stdout.flush()
            self.previous_status_line_count = len(status_lines)
            return

        for line in status_lines:
            print(line)

    def callback(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        h0, w0 = frame.shape[:2]

        imgsz = self.imgsz if self.imgsz > 0 else (h0, w0)
        results = self.model(frame, imgsz=imgsz, conf=self.conf_thres)[0]

        frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
        out = Yolo_Objects()
        out.header = Header(stamp=msg.header.stamp, frame_id=frame_id)

        idx_counter = 0
        total_boxes = len(results.boxes)
        status_lines = []
        publish_candidates = []

        for box in results.boxes:
            cls_id = int(box.cls.cpu().item())
            conf = float(box.conf.cpu().item())
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())
            class_config = self.class_config.get(cls_id)

            if class_config is None or not class_config["publish"]:
                continue

            if conf < class_config["confidence"]:
                continue

            publish_candidates.append({
                "cls_id": cls_id,
                "conf": conf,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
            })

        filtered_count = len(publish_candidates)
        publish_candidates = postprocess_detections(
            publish_candidates,
            iou_threshold=self.postprocess_nms_iou,
            containment_ioa_threshold=self.postprocess_containment_ioa,
            area_weight=self.postprocess_area_weight,
            max_det=self.postprocess_max_det,
        )

        if self.postprocess_debug:
            rospy.loginfo(
                "[yolo_detect_node] postprocess boxes: "
                f"raw={total_boxes}, class_conf={filtered_count}, "
                f"final={len(publish_candidates)}"
            )

        if not publish_candidates:
            status_lines.append("[yolo_detect_node] NO DETECT")

        for candidate in publish_candidates:
            cls_id = candidate["cls_id"]
            conf = candidate["conf"]
            x1 = candidate["x1"]
            y1 = candidate["y1"]
            x2 = candidate["x2"]
            y2 = candidate["y2"]
            class_name = self.class_config.get(cls_id, {}).get("name", f"unknown({cls_id})")

            obj = Objects()
            obj.id = idx_counter
            obj.Class = cls_id
            obj.x1, obj.y1, obj.x2, obj.y2 = x1, y1, x2, y2
            out.yolo_objects.append(obj)
            idx_counter += 1
            status_lines.append(
                f"[yolo_detect_node] DETECT class={class_name} conf={conf:.2f}"
            )

            if SHOW_DETECTION_IMAGE:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        self._print_detection_status(status_lines)

        self.pub.publish(out)

        if SHOW_DETECTION_IMAGE:
            cv2.imshow(self.win_name, frame)
            cv2.waitKey(1)

    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down YOLOv12 viewer.")
        finally:
            if SHOW_DETECTION_IMAGE:
                cv2.destroyAllWindows()

if __name__ == "__main__":
    node = YoloDetectNode()
    node.spin()
