#!/home/cnu/anaconda3/envs/yolo/bin/python
import os
import sys

import cv2
import numpy as np
import rospy
from detect_msgs.msg import Objects, Yolo_Objects
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

sys.path.insert(0, "/home/cnu/clothoid-r/perception_ws/yolov12")
from ultralytics import YOLO

# 사용자가 자주 바꿀 수 있는 설정값 모음
# True 이면 검출 결과 영상을 화면에 띄우고, False 이면 화면 출력 없이 ROS 토픽만 publish 합니다.
SHOW_DETECTION_IMAGE = True

# True 이면 검출 결과 로그 영역만 갱신해서 현재 상태만 깔끔하게 보여줍니다.
# 초기화 로그(MODEL LOADED, yaml_cfg, pt_weights 등)는 그대로 유지됩니다.
CLEAR_TERMINAL_ON_DETECTION = True

WINDOW_NAME = "YOLOv12 BBox"
DEFAULT_SOURCE_TOPIC = "/camera/image_raw/compressed"
DEFAULT_PUBLISH_TOPIC = "/perception/camera/yolo"
DEFAULT_CONFIDENCE = 0.5
DEFAULT_FRAME_ID = "camera_link"
PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_YAML_CFG = os.path.join(PACKAGE_DIR, "models", "0516_prune.yaml")
DEFAULT_PT_WEIGHTS = os.path.join(PACKAGE_DIR, "models", "0516_prune.pt")
# publish할 클래스 목록을 여기에서 직접 설정합니다.
# 0: ERP-42
# 1: drum
# 2: cone
# 예시: [0, 1, 2] 모두 publish, [0] ERP-42만 publish, [0, 2] ERP-42와 cone만 publish
DEFAULT_PUBLISH_CLASSES = [0, 1, 2]
CLASS_NAMES = {
    0: "ERP-42",
    1: "drum",
    2: "cone",
}

class YoloDetectNode:
    def __init__(self):
        rospy.init_node("yolo_detect_node")

        self.win_name = WINDOW_NAME
        self.class_names = CLASS_NAMES
        self.previous_status_line_count = 0
        source_topic = rospy.get_param("~source", DEFAULT_SOURCE_TOPIC)
        publish_topic = rospy.get_param("~output_topic", DEFAULT_PUBLISH_TOPIC)
        yaml_cfg = rospy.get_param("~yaml_cfg", DEFAULT_YAML_CFG)
        pt_weights = rospy.get_param("~pt_weights", DEFAULT_PT_WEIGHTS)
        self.conf_thres = rospy.get_param("~confidence", DEFAULT_CONFIDENCE)
        self.frame_id = rospy.get_param("~frame_id", DEFAULT_FRAME_ID)
        self.publish_classes = set(int(class_id) for class_id in DEFAULT_PUBLISH_CLASSES)

        self.pub = rospy.Publisher(publish_topic, Yolo_Objects, queue_size=1)

        self.model = YOLO(yaml_cfg, task='detect').load(pt_weights)
        rospy.loginfo(f"[yolo_detect_node] YOLOv12 MODEL LOADED")
        rospy.loginfo(f"[yolo_detect_node] yaml_cfg: {yaml_cfg}")
        rospy.loginfo(f"[yolo_detect_node] pt_weights: {pt_weights}")
        rospy.loginfo(f"[yolo_detect_node] frame_id: {self.frame_id}")
        rospy.loginfo(
            f"[yolo_detect_node] publish_classes: "
            f"{sorted(self.publish_classes) if self.publish_classes else []}"
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

        results = self.model(frame, imgsz=(h0, w0), conf=self.conf_thres)[0]

        frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
        out = Yolo_Objects()
        out.header = Header(stamp=msg.header.stamp, frame_id=frame_id)

        idx_counter = 0
        total_boxes = len(results.boxes)
        status_lines = []
        if total_boxes == 0:
            status_lines.append("[yolo_detect_node] NO DETECT")

        for raw_idx, box in enumerate(results.boxes):
            cls_id = int(box.cls.cpu().item())
            conf = float(box.conf.cpu().item())
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())
            class_name = self.class_names.get(cls_id, f"unknown({cls_id})")

            if cls_id not in self.publish_classes:
                continue

            obj = Objects()
            obj.id = idx_counter
            obj.Class = cls_id
            obj.x1, obj.y1, obj.x2, obj.y2 = x1, y1, x2, y2
            out.yolo_objects.append(obj)
            idx_counter += 1
            status_lines.append(
                f"[yolo_detect_node] DETECT class={class_name} conf={conf:.3f}"
            )

            if SHOW_DETECTION_IMAGE:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, class_name, (x1, y1 - 10),
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
