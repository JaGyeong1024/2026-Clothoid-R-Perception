# livox_clustering

Livox LiDAR 단독 유클리디안 클러스터링 + 칼만 추적 노드 (Python).

## 토픽

### 입력 (구독)

| 토픽 | 메시지 |
|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` |

### 출력 (발행)

| 토픽 | 메시지 | 용도 |
|---|---|---|
| `/perception/livox/centroids` | `sensor_msgs/PointCloud` | **외부 인터페이스** — planning이 구독 |
| `/perception/livox/preprocessed` | `sensor_msgs/PointCloud2` | 디버깅 (전처리 결과 시각화) |

## 파라미터 (`~private`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `input_topic` | `/livox/lidar` | 입력 LiDAR 토픽 |
| `centroid_topic` | `/perception/livox/centroids` | 출력 centroid 토픽 |
| `preprocessed_topic` | `/perception/livox/preprocessed` | 출력 전처리 PointCloud2 |
| `frame_id` | `livox_frame` | 출력 frame_id |
| `config` | `config/livox_clustering.yaml` | 알고리즘 파라미터 yaml |

> 알고리즘 파라미터(ROI, voxel size, DROR, RANSAC, 클러스터링 임계, 트래커)는 `config/livox_clustering.yaml`에서 기본값을 로드합니다. 현재 값은 기존 코드 상수와 동일합니다.

## 알고리즘 요약

```
PointCloud2 read
  → pitch 보정
  → ROI 박스 컷
  → voxel downsample
  → DROR (Dynamic Radius Outlier Removal)
  → grid 기반 ground 제거
  → RANSAC 잔류 지면 제거
  → /perception/livox/preprocessed 발행
  → 거리 가중치 Euclidean clustering
  → 클러스터 병합 (CLUSTER_MERGE_GAP)
  → bbox 크기 필터
  → 칼만 트래커 매칭/생성/소거
  → /perception/livox/centroids 발행
```
