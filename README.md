# 2026 Clothoid-R Perception

Clothoid-R 자율주행 시스템의 Perception ROS workspace.

카메라와 LiDAR 기반 객체 검출, 클러스터링, 추적, 센서 퓨전 패키지 구성.

## Team

<table>
<tr>
<td align="center"><a href="https://github.com/lovelyoverflow"><img src="https://avatars.githubusercontent.com/u/14028864?v=4" width="100px;" alt=""/><br /><sub><b>구자경</b></sub></a><br />SLAM, Navigation</td>
<td align="center"><a href="https://github.com/gyeongseoMin"><img src="https://avatars.githubusercontent.com/u/67200721?v=4" width="100px;" alt=""/><br /><sub><b>서준혁</b></sub></a><br />LiDAR, Obstacle Avoidance</td>
<td align="center"><a href="https://github.com/SeoooooNyeong"><img src="https://avatars.githubusercontent.com/u/113419106?v=4" width="100px;" alt=""/><br /><sub><b>김민재</b></sub></a><br />LiDAR, Obstacle Avoidance</td>
<td align="center"><a href="https://github.com/JOONHOGITHUB"><img src="https://avatars.githubusercontent.com/u/105336903?v=4" width="100px;" alt=""/><br /><sub><b>지연수</b></sub></a><br />Camera, Lane Detection</td>
<td align="center"><a href="https://github.com/leeharam2004"><img src="https://avatars.githubusercontent.com/u/44737337?v=4" width="100px;" alt=""/><br /><sub><b>유인석</b></sub></a><br />Camera, Lane Detection</td>
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
- Git
- NVIDIA GPU/CUDA, optional

## Setup

ROS environment:

```bash
source /opt/ros/noetic/setup.bash
```

System packages:

```bash
sudo apt update
sudo apt install -y \
  wget \
  git \
  build-essential \
  cmake \
  python3-pip \
  python3-rosdep \
  ros-noetic-cv-bridge \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-message-filters \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-visualization-msgs
```

rosdep:

```bash
sudo rosdep init
rosdep update
```

Miniconda:

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh
bash /tmp/miniconda.sh -b -p $HOME/anaconda3
source $HOME/anaconda3/etc/profile.d/conda.sh
conda init bash
```

Repository clone:

```bash
git clone https://github.com/JaGyeong1024/2026-Clothoid-R-Perception.git
cd 2026-Clothoid-R-Perception
```

ROS dependency install:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Conda environment:

```bash
source $HOME/anaconda3/etc/profile.d/conda.sh
conda env create -f environment.yml
conda activate clothoid
```

Custom YOLO package:

```bash
cd yolov8_prune
pip install -e .
cd ..
```

OC-SORT:

```bash
sudo git clone https://github.com/noahcao/OC_SORT /opt/OC_SORT
```

Workspace build:

```bash
source /opt/ros/noetic/setup.bash
source $HOME/anaconda3/etc/profile.d/conda.sh
conda activate clothoid
catkin_make
source devel/setup.bash
```

## Integrated Launch

Default launch:

```bash
roslaunch perception_bringup perception.launch
```

Custom conda path:

```bash
roslaunch perception_bringup perception.launch \
  conda_base:=/opt/miniconda3 \
  conda_env:=clothoid
```

Sensor topic override:

```bash
roslaunch perception_bringup perception.launch \
  livox_lidar_topic:=/livox/lidar \
  velodyne_points_topic:=/velodyne_points \
  camera_image_topic:=/camera/image_raw/compressed
```

Velodyne BEV CPU mode:

```bash
roslaunch perception_bringup perception.launch \
  velodyne_device:=cpu
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

## Verification

Node check:

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

Topic check:

```bash
rostopic list | grep perception
```

Output publisher check:

```bash
rostopic info /perception/livox/centroids
rostopic info /perception/fusion/centroids
rostopic info /perception/velodyne/centroids
```
