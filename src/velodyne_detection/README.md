# velodyne_detection

Velodyne LiDAR 포인트클라우드를 BEV 이미지로 변환 후 YOLO + OC-SORT로 객체를 검출/추적하는 노드 (Python).

## 토픽

### 입력 (구독)

| 토픽 | 메시지 |
|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` |

### 출력 (발행)

| 토픽 | 메시지 | 용도 |
|---|---|---|
| `/perception/velodyne/centroids` | `sensor_msgs/PointCloud` | **외부 인터페이스** — planning이 구독. 10Hz heartbeat 보장 |
| `/perception/velodyne/markers` | `visualization_msgs/MarkerArray` | 디버깅 (트랙 ID 텍스트) |
| `/perception/velodyne/bev_image` | `sensor_msgs/Image` | 디버깅 (BEV + bbox 시각화) |

## 파라미터 (`~private`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `input_topic` | `/velodyne_points` | 입력 LiDAR 토픽 |
| `centroid_topic` | `/perception/velodyne/centroids` | 출력 centroid 토픽 |
| `marker_topic` | `/perception/velodyne/markers` | 출력 marker 토픽 |
| `image_topic` | `/perception/velodyne/bev_image` | 출력 BEV 이미지 토픽 |
| `model_path` | (필수) | YOLO 가중치 경로 (launch에서 `$(find velodyne_detection)/model/<ver>.pt`) |
| `ocsort_path` | `/opt/OC_SORT` | OC-SORT clone 경로 (또는 `OC_SORT_PATH` env) |
| `device` | `cuda` | YOLO inference device |
| `detect_conf` | `0.4` | YOLO confidence threshold |
| `voxel_size` | `0.05` | BEV voxel 해상도 (m) |
| `x_range` / `y_range` / `z_range` | `[-15, 15]` / `[-15, 15]` / `[-2.5, 2]` | BEV 영역 |
| `max_points_per_voxel` | `30` | density 채널 정규화 max |
| `frame_id` | `velodyne` | 출력 frame_id |

## 알고리즘 요약

```
PointCloud2 read (intensity 있으면 사용)
  → BEV 이미지 변환 (height / intensity / density 3채널)
  → YOLO inference (ultralytics)
  → OC-SORT 추적 (id 부여)
  → BEV 픽셀 좌표 → LiDAR 좌표 역변환
  → /perception/velodyne/centroids 발행
  → MarkerArray + BEV 이미지 발행
```

## 리팩토링 todo (Phase 4)

- [ ] `pc2_to_bev` 메서드를 `BevConverter` 클래스로 분리
- [ ] OC-SORT 의존성을 vendor 또는 git submodule로 패키지에 포함 (현재 `/opt/OC_SORT` 외부 경로)
- [ ] BEV 시각화(`vis` 그리기) 코드를 별도 메서드로 분리
- [ ] CUDA fallback (CPU 환경 자동 감지)

## 모델 가중치

`model/` 디렉토리에 `.pt` 파일 보관:

| 파일 | 용도 |
|---|---|
| `velodyne_v4.pt` | 현재 launch 기본값 |
| `velodyne_v5.pt`, `velodyne_v6.pt` | 이전 버전 fallback |

launch에서 `model_version` arg로 선택:
```bash
roslaunch velodyne_detection velodyne_detection.launch model_version:=velodyne_v6
```
