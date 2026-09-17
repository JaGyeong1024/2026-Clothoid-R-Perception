# perception_bringup

Perception 스택의 통합 launch와 RViz 설정을 모은 메타 패키지.

## 통합 실행

```bash
roslaunch perception_bringup perception.launch
```

다음 2개 노드를 띄움:

| 노드 | 패키지 | 인터프리터 |
|---|---|---|
| `livox_camera_fusion` | livox_camera_fusion | 시스템 (C++ 노드) |
| `livox_euclidean_clustering` | livox_clustering | 시스템 python3 |

`velodyne_detection`, `yolov12`는 환경 격리 / 차량 운영 방식에 맞춰 이 launch에서 분리되어 있음. 별도 터미널에서:

```bash
rosrun velodyne_detection velodyne_bev_detection.py   # 시스템 python3
rosrun yolov12 yolo_detect.py                         # conda yolo env (shebang)
```

## Launch arguments

| 이름 | 기본값 | 설명 |
|---|---|---|
| `livox_lidar_topic` | `/livox/lidar` | Livox 입력 토픽 |
| `camera_image_topic` | `/camera/image_raw/compressed` | 카메라 입력 토픽 |
| `camera_yolo_topic` | `/perception/camera/yolo` | YOLO 결과 (fusion 입력) |
| `livox_centroid_topic` | `/perception/livox/centroids` | Livox clustering 출력 |
| `fusion_centroid_topic` | `/perception/fusion/centroids` | Livox-camera fusion 출력 |
| `fusion_projection_config` | `$(find livox_camera_fusion)/config/projection.yaml` | fusion projection 파라미터 |
| `livox_clustering_config` | `$(find livox_clustering)/config/livox_clustering.yaml` | Livox clustering 알고리즘 파라미터 |

## RViz 설정

`rviz/perception.rviz` — 디버깅 토픽(전처리 클라우드, BEV 이미지, marker 등)을 표시하는 사전 구성 RViz 레이아웃.

```bash
rviz -d $(rospack find perception_bringup)/rviz/perception.rviz
```
