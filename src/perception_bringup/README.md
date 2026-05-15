# perception_bringup

Perception 스택의 통합 launch와 RViz 설정을 모은 메타 패키지.

## 통합 실행

```bash
roslaunch perception_bringup perception.launch
```

`LD_LIBRARY_PATH`는 yolov12.launch의 `<env>` 태그가 yolov12 노드 프로세스에만 set하므로 별도 export 불필요. 부모 셸/다른 노드 무영향.

다음 4개 노드를 한꺼번에 띄움:

| 노드 | 패키지 | env 처리 |
|---|---|---|
| `yolo_detect_node` | yolov12 | launch-prefix(`python_interp`)로 conda env python 강제 + `<env>` 태그로 LD_LIBRARY_PATH 노드 한정 set |
| `livox_camera_fusion` | livox_camera_fusion | 시스템 (C++ 노드) |
| `livox_euclidean_clustering` | livox_clustering | 격리 (launch-prefix로 `conda_env` 활성화) |
| `velodyne_bev_detection` | velodyne_detection | 격리 (launch-prefix로 `conda_env` 활성화) |

## Launch arguments

| 이름 | 기본값 | 설명 |
|---|---|---|
| `conda_base` | `$(env HOME)/anaconda3` | conda 설치 경로 (livox_clustering / velodyne_detection 용) |
| `conda_env` | `clothoid` | livox_clustering, velodyne_detection 용 env |
| `python_interp` | `/home/cnu/anaconda3/envs/yolo/bin/python` | yolov12 노드의 인터프리터(launch-prefix로 강제). catkin wrapper의 시스템 python shebang을 우회 |
| `conda_env_lib` | `/home/cnu/anaconda3/envs/yolo/lib` | yolov12 노드 LD_LIBRARY_PATH에 prepend되는 lib 경로 |
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
