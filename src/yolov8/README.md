# yolov8

카메라 RGB 이미지에서 YOLOv8로 객체를 검출하는 노드 (Python).

## 토픽

### 입력 (구독)

| 토픽 | 메시지 |
|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` |

### 출력 (발행)

| 토픽 | 메시지 | 용도 |
|---|---|---|
| `/perception/camera/yolo_objects` | `detect_msgs/Yolo_Objects` | livox_camera_fusion의 입력 + 디버깅 |

## 파라미터 (`~private`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `source` | `/camera/image_raw/compressed` | 입력 카메라 토픽 |
| `output_topic` | `/perception/camera/yolo_objects` | 출력 토픽 |
| `yaml_cfg` | (필수) | YOLO 모델 yaml |
| `pt_weights` | (필수) | YOLO 가중치 .pt |
| `confidence` | `0.4` | confidence threshold |
| `frame_id` | `camera_link` | 출력 frame_id |
| `ultralytics_path` | `""` | 추가 ultralytics 경로 (env `YOLOV8_PRUNE_PATH`로도 설정 가능) |

## 클래스 매핑

| ID | 이름 |
|---|---|
| 0 | ERP-42 |
| 1 | drum |
| 2 | robbercone |

> 현재 코드는 클래스 0(ERP-42)만 발행. 다른 클래스도 발행이 필요하면 `callback()`의 `if cls_id != 0: continue` 제거.

## Conda 환경 의존성

이 노드는 `ultralytics` 라이브러리를 사용하므로 conda env(`clothoid`) 안에서 실행되어야 합니다. `launch/yolov8.launch`의 `launch-prefix`가 conda 활성화를 자동 처리합니다.

## 알고리즘 요약

```
CompressedImage → cv2 decode
  → ultralytics YOLO inference
  → 클래스 0 bbox만 추출
  → detect_msgs/Yolo_Objects 발행
```
