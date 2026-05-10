#!/usr/bin/env python3
import os
import logging
import warnings
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from detect_msgs.msg import Yolo_Objects, Objects

# yolov8_prune 경로는 환경 변수로 (default: $(rospack find yolov8)/../yolov8_prune).
# launch에서 YOLOV8_PRUNE_PATH=... 또는 ~ultralytics_path param으로 override 가능.
import sys
_PRUNE_DEFAULT = os.environ.get("YOLOV8_PRUNE_PATH", "")
if _PRUNE_DEFAULT and _PRUNE_DEFAULT not in sys.path:
    sys.path.insert(0, _PRUNE_DEFAULT)

logging.getLogger("ultralytics").setLevel(logging.ERROR)
warnings.filterwarnings("ignore", category=UserWarning)


class CameraYoloDetection:
    def __init__(self):
        rospy.init_node("camera_yolo_detection")

        # 추가 ultralytics_path가 param으로 들어오면 sys.path에 추가
        extra_path = rospy.get_param("~ultralytics_path", "")
        if extra_path and extra_path not in sys.path:
            sys.path.insert(0, extra_path)
        from ultralytics import YOLO

        self.class_names = {
            0: "ERP-42",
            1: "drum",
            2: "robbercone",
        }

        # Topics / params
        source_topic    = rospy.get_param("~source",     "/camera/image_raw/compressed")
        output_topic    = rospy.get_param("~output_topic", "/perception/camera/yolo_objects")
        yaml_cfg        = rospy.get_param("~yaml_cfg",   "")
        pt_weights      = rospy.get_param("~pt_weights", "")
        self.conf_thres = rospy.get_param("~confidence", 0.5)
        self.frame_id   = rospy.get_param("~frame_id",   "camera_link")

        if not yaml_cfg or not pt_weights:
            rospy.logerr("[camera_yolo_detection] ~yaml_cfg and ~pt_weights are required")
            raise RuntimeError("yaml_cfg / pt_weights required")

        self.pub = rospy.Publisher(output_topic, Yolo_Objects, queue_size=1)
        self.model = YOLO(yaml_cfg, task='detect').load(pt_weights)
        rospy.loginfo(f"[camera_yolo_detection] model loaded: {yaml_cfg}, {pt_weights}")

        rospy.Subscriber(source_topic,
                         CompressedImage,
                         self.callback,
                         queue_size=1,
                         buff_size=2**24)
        rospy.loginfo(f"[camera_yolo_detection] subscribe={source_topic} -> {output_topic}")

    def callback(self, msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        h0, w0 = frame.shape[:2]

        results = self.model(frame, imgsz=(h0, w0), conf=self.conf_thres)[0]

        frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
        out = Yolo_Objects()
        out.header = Header(stamp=msg.header.stamp, frame_id=frame_id)

        idx_counter = 0
        for box in results.boxes:
            cls_id = int(box.cls.cpu().item())
            if cls_id != 0:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())

            obj = Objects()
            obj.id = idx_counter
            obj.Class = cls_id
            obj.x1, obj.y1, obj.x2, obj.y2 = x1, y1, x2, y2
            out.yolo_objects.append(obj)
            idx_counter += 1

        self.pub.publish(out)

    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("[camera_yolo_detection] shutting down")


if __name__ == "__main__":
    node = CameraYoloDetection()
    node.spin()
