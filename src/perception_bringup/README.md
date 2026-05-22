# perception_bringup

Perception 스택의 통합 launch와 RViz 설정을 모은 메타 패키지.

## 통합 실행

```bash
roslaunch perception_bringup perception.launch
```

이 launch는 다음 3개 노드만 띄움. YOLO 노드(yolov12, yolov8)는 환경 격리 문제로 통합에서 빼고 `rosrun`으로 별도 실행한다.

| 노드 | 패키지 | env 처리 |
|---|---|---|
| `livox_camera_fusion` | livox_camera_fusion | 시스템 (C++ 노드) |
| `livox_euclidean_clustering` | livox_clustering | 격리 (launch-prefix로 `conda_env` 활성화) |
| `velodyne_bev_detection` | velodyne_detection | 격리 (launch-prefix로 `conda_env` 활성화) |

YOLO 노드는 별도 터미널에서:

```bash
rosrun yolov12 yolo_detect.py   # fusion이 구독하는 카메라 YOLO
rosrun yolov8  yolo_detect.py   # velodyne_detection이 쓰는 YOLO
```

## Launch arguments

| 이름 | 기본값 | 설명 |
|---|---|---|
| `conda_base` | `$(env HOME)/anaconda3` | conda 설치 경로 (livox_clustering / velodyne_detection 용) |
| `conda_env` | `clothoid` | livox_clustering, velodyne_detection 용 env |
| `camera_yolo_topic` | `/perception/camera/yolo` | 카메라 YOLO 출력 + fusion 입력 |
| `livox_centroid_topic` | `/perception/livox/centroids` | Livox clustering 외부 출력 |
| `fusion_centroid_topic` | `/perception/fusion/centroids` | Livox-camera fusion 외부 출력 |
| `velodyne_centroid_topic` | `/perception/velodyne/centroids` | Velodyne BEV 외부 출력 |
| `fusion_projection_config` | `$(find livox_camera_fusion)/config/projection.yaml` | fusion projection 파라미터 |
| `livox_clustering_config` | `$(find livox_clustering)/config/livox_clustering.yaml` | Livox clustering 알고리즘 파라미터 |

머신마다 다르면 override:

```bash
roslaunch perception_bringup perception.launch \
  conda_base:=/opt/miniconda3 conda_env:=my_env
```

## RViz 설정

`rviz/perception.rviz` — 디버깅 토픽(전처리 클라우드, BEV 이미지, marker 등)을 표시하는 사전 구성 RViz 레이아웃.

```bash
rviz -d $(rospack find perception_bringup)/rviz/perception.rviz
```

> 현재 RViz config는 옛 토픽명(`/jagyeong`, `/minjae` 등) 기반일 수 있습니다. 새 토픽명에 맞게 업데이트 필요.
