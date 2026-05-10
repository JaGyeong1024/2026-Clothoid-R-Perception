# Perception Pipeline Optimization Notes

대상은 `src/livox_clustering/scripts/livox_euclidean_clustering.py` 중심입니다. 파라미터 튜닝이나 검출 결과가 달라질 수 있는 알고리즘 변경은 실차/rosbag 검증 전까지 보류합니다.

## Applied

### PointCloud2 fast parsing

- `pc2.read_points()` 기반 Python tuple/list 변환을 기본 경로에서 제거.
- `msg.data`를 NumPy buffer로 읽고 `x/y/z` 오프셋에서 `float32` 값을 추출.
- `PointField.FLOAT32`가 아니거나 필드가 예상과 다르면 기존 `read_points()` 경로로 fallback.
- 기존 `skip_nans=True` 의미를 유지하도록 finite 필터 적용.
- Velodyne BEV 노드도 동일하게 `x/y/z/intensity` 빠른 파싱을 적용. intensity가 없으면 기존처럼 0 채널을 추가.

### DROR vectorization

- `cKDTree.query_ball_point()`를 점별 loop 대신 배열 입력 + `return_length=True`로 호출.
- 기존 조건 `neighbor_count - 1 >= DROR_MIN_NEIGHBORS` 유지.

### Grid ground removal vectorization

- Python dict 기반 cell min-z/count 계산을 `np.unique`, `np.bincount`, `np.minimum.at`으로 대체.
- ground 판정식은 기존 조건 그대로 유지.

## Deferred Until Rosbag/Track Validation

아래 항목은 성능 이득 가능성이 있지만 결과 분포, 클러스터 경계, ID 매칭, latency jitter가 달라질 수 있어 현재는 보류합니다.

- `ransac_ground_remove`: sklearn RANSAC 직접 구현으로 교체
- `dist_euclid_labels`: graph/connected-components 기반 클러스터링으로 교체
- `merge_clusters`: KDTree/union-find 기반 최적화
- tracker association: greedy matching에서 Hungarian matching으로 변경
- `msg.header.stamp` 기반 publish timestamp 전환

## Next Measurement

실험장 rosbag 또는 실차 로그 확보 후 다음 값을 비교합니다.

- callback mean / p95 / p99 / std
- 입력 포인트 수와 전처리 후 포인트 수
- frame drop 여부
- centroid 개수/위치 diff
- tracker ID continuity

보류 항목은 위 지표에서 병목이 확인될 때만 단계적으로 적용합니다.
