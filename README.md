# 2026 Clothoid-R Perception

Clothoid-R 자율주행 시스템의 Perception ROS workspace.

카메라와 LiDAR 기반 객체 검출, 클러스터링, 추적, 센서 퓨전 패키지 구성.

## Team

<table>
<tr>
<td align="center"><a href="https://github.com/JaGyeong1024"><img src="https://avatars.githubusercontent.com/u/92356313?s=400&u=9df94c6f0e773e86773cb4fcc379f1204a7dcff7&v=4" width="100px;" alt=""/><br /><sub><b>구자경</b></sub></a><br />Perception Architecture</td>
</tr>
</table>

## Pipeline

| Pipeline | Input | Output | Package |
|---|---|---|---|
| Livox clustering | `/livox/lidar` | `/perception/livox/centroids` | `livox_clustering` |
| Livox-camera fusion | `/livox/lidar`, `/camera/image_raw/compressed`, `/perception/camera/yolo_objects` | `/perception/fusion/centroids` | `livox_camera_fusion` |
| Velodyne BEV detection | `/velodyne_points` | `/perception/velodyne/centroids` | `velodyne_detection` |

## Packages

| Package | Role |
|---|---|
| `perception_bringup` | 통합 launch |
| `detect_msgs` | 공통 perception message |
| `yolov8` | Camera YOLO detection |
| `livox_clustering` | Livox point cloud clustering and tracking |
| `livox_camera_fusion` | Livox-camera YOLO fusion |
| `velodyne_detection` | Velodyne BEV YOLO detection and OC-SORT tracking |

## Output Topics

| Topic | Type |
|---|---|
| `/perception/livox/centroids` | `sensor_msgs/PointCloud` |
| `/perception/fusion/centroids` | `sensor_msgs/PointCloud` |
| `/perception/velodyne/centroids` | `sensor_msgs/PointCloud` |

## Requirements

- Ubuntu 20.04
- ROS Noetic
- Docker
- `clothoid-noetic:latest` Docker image
- NVIDIA GPU/CUDA, optional

## Docker Setup

Docker image 확인:

```bash
docker images | grep clothoid-noetic
```

Repository clone:

```bash
git clone https://github.com/JaGyeong1024/2026-Clothoid-R-Perception.git
cd 2026-Clothoid-R-Perception
```

Perception image build:

```bash
docker build \
  -f docker/Dockerfile.perception \
  -t clothoid-perception:latest \
  .
```

Container 실행:

```bash
docker run --rm -it \
  --net=host \
  --ipc=host \
  --privileged \
  --gpus all \
  -v $(pwd):/ws \
  -w /ws \
  clothoid-perception:latest \
  bash
```

GPU 옵션 미사용:

```bash
docker run --rm -it \
  --net=host \
  --ipc=host \
  --privileged \
  -v $(pwd):/ws \
  -w /ws \
  clothoid-perception:latest \
  bash
```

Workspace build:

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

통합 실행:

```bash
roslaunch perception_bringup perception.launch \
  conda_base:=/opt/miniforge3 \
  conda_env:=clothoid
```

Velodyne BEV CPU 실행:

```bash
roslaunch perception_bringup perception.launch \
  conda_base:=/opt/miniforge3 \
  conda_env:=clothoid \
  velodyne_device:=cpu
```

## Local Setup

Ubuntu 20.04 + ROS Noetic 환경용.

Conda environment 생성:

```bash
conda env create -f environment.yml
conda activate clothoid
```

Custom YOLO package 설치:

```bash
cd yolov8_prune
pip install -e .
cd ..
```

OC-SORT 설치:

```bash
sudo git clone https://github.com/noahcao/OC_SORT /opt/OC_SORT
```

Workspace build:

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

통합 실행:

```bash
roslaunch perception_bringup perception.launch
```

## Launch Arguments

| Argument | Default |
|---|---|
| `conda_base` | `$(env HOME)/anaconda3` |
| `conda_env` | `clothoid` |
| `livox_lidar_topic` | `/livox/lidar` |
| `velodyne_points_topic` | `/velodyne_points` |
| `camera_image_topic` | `/camera/image_raw/compressed` |
| `camera_yolo_topic` | `/perception/camera/yolo_objects` |
| `livox_centroid_topic` | `/perception/livox/centroids` |
| `fusion_centroid_topic` | `/perception/fusion/centroids` |
| `velodyne_centroid_topic` | `/perception/velodyne/centroids` |
| `velodyne_device` | `cuda` |
| `ocsort_path` | `/opt/OC_SORT` |

Sensor topic override 예시:

```bash
roslaunch perception_bringup perception.launch \
  conda_base:=/opt/miniforge3 \
  conda_env:=clothoid \
  livox_lidar_topic:=/livox/lidar \
  velodyne_points_topic:=/velodyne_points \
  camera_image_topic:=/camera/image_raw/compressed
```

## Verification

Node 확인:

```bash
rosnode list
```

Expected nodes:

```text
/camera_yolo_detection
/livox_camera_fusion
/livox_euclidean_clustering
/velodyne_bev_detection
```

Topic 확인:

```bash
rostopic list | grep perception
```

Output publisher 확인:

```bash
rostopic info /perception/livox/centroids
rostopic info /perception/fusion/centroids
rostopic info /perception/velodyne/centroids
```
