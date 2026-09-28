# livox_clustering

Livox LiDAR 단독 물체 검출 노드 (Python): 지면 제거 + XY 클러스터링 + 칼만 추적. 카메라 없이 모르는 장애물을 잡는 보조 계통.

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

> 알고리즘 파라미터(ROI, voxel size, DROR, 클러스터링·물체 조건, 트래커)는 `config/livox_clustering.yaml`에서 로드합니다.

## 알고리즘 요약

```
PointCloud2 read
  → 지면 제거 (`horizon_ground` 패키지, 퓨전 노드와 같은 구현: x ≤ 15 m, |y| ≤ 7 m 에서 칸별 지면 평면 추정)
  → 발행 ROI 컷 (x 0~8 m, |y| ≤ 3 m)
  → voxel downsample
  → DROR (Dynamic Radius Outlier Removal)
  → /perception/livox/preprocessed 발행
  → XY 고정 간격(0.15 m) 연결 성분 클러스터링
  → 물체 조건: 지면 위 높이 0.3~2.0 m, 수평 최대 변 ≤ 2.5 m
  → 칼만 트래커 매칭/생성/소거
  → /perception/livox/centroids 발행 (3프레임 이상 연속 관측된 트랙)
```
