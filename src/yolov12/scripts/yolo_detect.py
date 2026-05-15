#!/home/cnu/anaconda3/envs/yolo/bin/python
# shebang은 타겟 PC(cnu)에 미리 세팅된 conda env(yolo)의 python을 직접 호출한다.
# 다른 환경에서 실행하려면 이 경로의 conda env(또는 동일한 site-packages를 가진 env)가
# 존재해야 한다. launch에서 별도 conda activate 처리는 하지 않는다.
import logging
import os
import sys
import warnings
from pathlib import Path

import cv2
import numpy as np
import rospy
from detect_msgs.msg import Objects, Yolo_Objects
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

logging.getLogger("ultralytics").setLevel(logging.ERROR)
warnings.filterwarnings("ignore", category=UserWarning)

CLASS_NAMES = {
    0: "ERP-42",
    1: "drum",
    2: "cone",
}


class YoloDetectNode:
    def __init__(self):
        rospy.init_node("yolo_detect_node")

        # ultralytics(yolov12 fork) import 경로. launch에서 ~ultralytics_path 또는
        # YOLOV12_PATH 환경변수로 외부 yolov12 소스 디렉토리를 지정할 수 있다.
        extra_path = rospy.get_param("~ultralytics_path", os.environ.get("YOLOV12_PATH", ""))
        if extra_path and extra_path not in sys.path:
            sys.path.insert(0, extra_path)
        from ultralytics import YOLO

        package_dir = Path(__file__).resolve().parents[1]
        default_yaml = package_dir / "models" / "prune_0510.yaml"
        default_pt = package_dir / "models" / "prune_0510.pt"

        source_topic = rospy.get_param("~source", "/camera/image_raw/compressed")
        output_topic = rospy.get_param("~output_topic", "/perception/camera/yolo")
        yaml_cfg = rospy.get_param("~yaml_cfg", str(default_yaml))
        pt_weights = rospy.get_param("~pt_weights", str(default_pt))
        self.conf_thres = rospy.get_param("~confidence", 0.4)
        self.frame_id = rospy.get_param("~frame_id", "camera_link")
        # publish할 클래스 ID 리스트. 빈 리스트면 전부 publish.
        # 0: ERP-42, 1: drum, 2: cone
        publish_classes = rospy.get_param("~publish_classes", [0, 1, 2])
        self.publish_classes = set(int(c) for c in publish_classes)
        self.show_detection_image = rospy.get_param("~show_detection_image", False)
        self.win_name = rospy.get_param("~window_name", "YOLOv12 BBox")

        self.pub = rospy.Publisher(output_topic, Yolo_Objects, queue_size=1)
        self.model = YOLO(yaml_cfg, task="detect").load(pt_weights)
        rospy.loginfo(f"[yolo_detect_node] YOLOv12 model loaded: {yaml_cfg}, {pt_weights}")
        rospy.loginfo(
            f"[yolo_detect_node] publish_classes={sorted(self.publish_classes) if self.publish_classes else 'ALL'}"
        )

        rospy.Subscriber(
            source_topic,
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.loginfo(f"[yolo_detect_node] subscribe={source_topic} -> publish={output_topic}")

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
            if self.publish_classes and cls_id not in self.publish_classes:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())

            obj = Objects()
            obj.id = idx_counter
            obj.Class = cls_id
            obj.x1, obj.y1, obj.x2, obj.y2 = x1, y1, x2, y2
            out.yolo_objects.append(obj)
            idx_counter += 1

            if self.show_detection_image:
                class_name = CLASS_NAMES.get(cls_id, f"unknown({cls_id})")
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame, class_name, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                )

        self.pub.publish(out)

        if self.show_detection_image:
            cv2.imshow(self.win_name, frame)
            cv2.waitKey(1)

    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("[yolo_detect_node] shutting down")
        finally:
            if self.show_detection_image:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    node = YoloDetectNode()
    node.spin()
