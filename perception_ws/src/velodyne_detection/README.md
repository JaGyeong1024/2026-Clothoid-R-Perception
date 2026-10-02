# velodyne_detection

Velodyne LiDAR 포인트클라우드를 BEV 이미지 → 정규화 HSV로 바꾼 뒤 YOLO26 + OC-SORT로 ERP를 검출·추적하는 노드 (Python).

- 실행 파이썬은 시스템 python3(3.8)이다. ultralytics는 레포 안 `perception_ws/yolo26`(8.4.78)을 스크립트 기준 상대 경로로 불러온다. 시스템 ultralytics 8.0.196은 yolo26 모델을 못 읽는다.
- BEV·정규화 HSV 변환은 `scripts/bev_input.py`가 정본이다. 학습 도구(`tools/velodyne_train`, 로컬 전용)도 이 파일을 쓴다.

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
| `detect_conf` | `0.5` | YOLO 신뢰도 문턱 (스크립트·launch 같음) |
| `nms_iou` | `0.01` | NMS IoU. BEV에서는 물체가 겹칠 수 없어서 조금이라도 겹친 박스는 중복으로 지운다 |
| `representation` | `hsv_v1` | 모델 입력 표현. 모델과 짝이다: v7 = `hsv_v1`, v6 = `raw` |
| `voxel_size` | `0.05` | BEV voxel 해상도 (m) |
| `x_range` / `y_range` / `z_range` | `[-15, 15]` / `[-15, 15]` / `[-2.5, 2]` | BEV 영역 |
| `max_points_per_voxel` | `30` | density 채널 정규화 max |
| `frame_id` | `velodyne` | 출력 frame_id |
| `publish_policy` | `confirmed` | 트랙 발행 규칙. `confirmed` = 아래 표의 확정 후 재개, `streak` = OC-SORT 기본(이전 동작) |
| `confirmed_max_gap` | `3` | `confirmed`에서 확정 트랙이 바로 다시 발행되는 최대 공백(프레임) |

### 트랙 기준과 범위

| 항목 | `streak` (이전) | `confirmed` (기본) |
|---|---|---|
| 새 트랙 확정 | `detect_conf` 이상 검출이 3번 연속 매칭 | 같음 |
| 매칭 기준 | BEV 픽셀 박스 IoU ≥ 0.25 (OC-SORT, 거리·크기 검사 없음) | 같음 |
| 미검출 트랙 유지 | 최대 10프레임, 11번째 연속 미검출에 삭제 (`max_age`) | 같음 |
| 미검출 프레임 발행 | 안 함 | 안 함 |
| 다시 검출될 때 | 3번 연속 매칭될 때까지 발행 안 함 (1프레임 놓치면 최소 3프레임 빠짐) | 공백이 `confirmed_max_gap` 이하면 바로 발행, 더 길면 3번 연속을 다시 요구 |

## 알고리즘 요약

```
PointCloud2 read (intensity 있으면 사용)
  → BEV 이미지 변환 (height / intensity / density 3채널, bev_input.bev_from_points)
  → 정규화 HSV (지면 기준 높이 / 반사강도 / 점유율, bev_input.hsv_v1)
  → YOLO inference (레포 ultralytics, NMS 추론 end2end=False, IoU 0.01)
  → OC-SORT 추적 (id 부여)
  → BEV 픽셀 좌표 → LiDAR 좌표 역변환 (x = 15 − cy·0.05, y = 15 − cx·0.05)
  → /perception/velodyne/centroids 발행 (header.stamp = 스캔 시각. 10 Hz 유지 발행은 직전 결과를 같은 스캔 시각으로 다시 낸다)
  → MarkerArray + BEV 이미지 발행
```

## 모델 가중치

`model/` 디렉토리에 `.pt` 파일 보관:

| 파일 | 용도 |
|---|---|
| `velodyne_v7.pt` | 기본(launch `model_version`, 스크립트 기본값). yolo26n, 정규화 HSV 입력, 2026-10-03 |
| `velodyne_v6.pt` | 이전 모델(원본 BEV 입력). 되돌릴 때: `model_version:=velodyne_v6 representation:=raw` |

이전 버전(`velodyne_v4.pt`, `velodyne_v5.pt`)은 참조되지 않아 제거했다. 필요하면 git 히스토리에서 복구한다.

launch에서 `model_version` arg로 선택:
```bash
roslaunch velodyne_detection velodyne_detection.launch model_version:=velodyne_v6
```
