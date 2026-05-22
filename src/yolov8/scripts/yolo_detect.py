#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import logging
import warnings
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from detect_msgs.msg import Yolo_Objects, Objects
import sys
sys.path.insert(0, "/home/cnu/clothoid-r/perception_ws/yolov8_prune")
from ultralytics import YOLO

# ───── 로그 · 경고 억제 ─────
logging.getLogger("ultralytics").setLevel(logging.ERROR)
warnings.filterwarnings("ignore", category=UserWarning)

class YoloDetectNode:
    def __init__(self):
        rospy.init_node("yolo_detect_node")

        # (1) 화면창 이름
        self.win_name = "YOLOv8 BBox"

        # (2) 클래스 이름 매핑
        self.class_names = {
            0: "ERP-42",
            1: "drum",
            2: "robbercone"
        }

        # (3) 파라미터
        source_topic   = rospy.get_param("~source",     "/camera/image_raw/compressed")
        yaml_cfg       = rospy.get_param("~yaml_cfg",   "./best.yaml")
        pt_weights     = rospy.get_param("~pt_weights", "./best.pt")
        self.conf_thres = rospy.get_param("~confidence", 0.5)

        # (4) 퍼블리셔
        self.pub = rospy.Publisher("yolov8_pub", Yolo_Objects, queue_size=1)

        # (5) 모델 로드
        self.model = YOLO(yaml_cfg, task='detect').load(pt_weights)
        rospy.loginfo(f"[yolo_detect_node] Model loaded: {yaml_cfg}, {pt_weights}")

        # (6) 서브스크라이버
        rospy.Subscriber(source_topic,
                         CompressedImage,
                         self.callback,
                         queue_size=1,
                         buff_size=2**24)
        rospy.loginfo(f"[yolo_detect_node] Subscribed to {source_topic}")

    def callback(self, msg: CompressedImage):
        # 1) jpg → BGR
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        h0, w0 = frame.shape[:2]

        # 2) 추론
        results = self.model(frame, imgsz=(h0, w0), conf=self.conf_thres)[0]

        # 3) Yolo_Objects 메시지 준비
        frame_id = msg.header.frame_id if msg.header.frame_id else "camera_link"
        out = Yolo_Objects()
        out.header = Header(stamp=msg.header.stamp, frame_id=frame_id)

        # 4) 결과 loop (클래스 0만)
        idx_counter = 0
        for box in results.boxes:
            cls_id = int(box.cls.cpu().item())
            if cls_id != 0:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())

            # 메시지 채우기
            obj = Objects()
            obj.id = idx_counter
            obj.Class = cls_id
            obj.x1, obj.y1, obj.x2, obj.y2 = x1, y1, x2, y2
            out.yolo_objects.append(obj)
            idx_counter += 1

            # BBox 시각화
            class_name = self.class_names.get(cls_id, f"unknown({cls_id})")
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, class_name, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 5) Publish
        self.pub.publish(out)

        # 6) 실시간 창 출력
        cv2.imshow(self.win_name, frame)
        cv2.waitKey(1)

    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down YOLOv8 viewer.")
        finally:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    node = YoloDetectNode()
    node.spin()
