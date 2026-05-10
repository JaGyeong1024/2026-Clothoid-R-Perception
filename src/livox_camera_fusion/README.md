# livox_camera_fusion

Livox LiDAR + 카메라 YOLO 결과를 시간 동기화하여 객체의 중점 좌표를 발행하는 **센서 퓨전 노드** (C++).

## 토픽

### 입력 (구독)

| 토픽 | 메시지 | 비고 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Livox 포인트클라우드 |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 카메라 RGB |
| `/perception/camera/yolo_objects` | `detect_msgs/Yolo_Objects` | yolov8 노드의 bbox 결과 |

세 토픽은 `message_filters::ApproximateTime`(slop 50ms)으로 동기화.

### 출력 (발행)

| 토픽 | 메시지 | 용도 |
|---|---|---|
| `/perception/fusion/centroids` | `sensor_msgs/PointCloud` | **외부 인터페이스** — planning이 구독 |
| `/perception/fusion/filtered_cloud` | `sensor_msgs/PointCloud2` | 디버깅 (RViz 시각화용) |

## 파라미터 (`~private`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `lidar_topic` | `/livox/lidar` | 입력 LiDAR 토픽 |
| `camera_topic` | `/camera/image_raw/compressed` | 입력 카메라 토픽 |
| `yolo_topic` | `/perception/camera/yolo_objects` | yolov8 결과 토픽 |
| `centroid_topic` | `/perception/fusion/centroids` | 출력 centroid 토픽 |
| `filtered_cloud_topic` | `/perception/fusion/filtered_cloud` | 출력 필터링 cloud 토픽 |
| `frame_name` | `livox_frame` | 출력 PointCloud frame_id |
| `camera_matrix/data` | `config/projection.yaml` | 카메라 intrinsic 3x3 행렬 |
| `extrinsic_matrix/data` | `config/projection.yaml` | LiDAR→카메라 extrinsic 3x4 행렬 |

## 알고리즘 요약

```
LiDAR + Camera + YOLO 동기화
  → projection_matrix로 LiDAR → 카메라 픽셀 좌표 투영
  → 각 YOLO bbox 안의 LiDAR 포인트 추출
  → bbox 중심 기준 ROI 반경 컷
  → RANSAC 지면 제거
  → Euclidean clustering → 가장 큰 클러스터의 (x, y) 중심
  → 칼만 추적 (전역 trackers)
  → /perception/fusion/centroids 발행
```
