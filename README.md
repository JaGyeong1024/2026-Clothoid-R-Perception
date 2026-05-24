# 2026 Clothoid-R Perception

Clothoid-R 자율주행 시스템의 Perception ROS workspace.

카메라와 LiDAR 기반 객체 검출, 클러스터링, 추적, 센서 퓨전 패키지 구성.

## Team

<table>
<tr>
<td align="center" width="180">
  <a href="https://github.com/JaGyeong1024"><img src="https://avatars.githubusercontent.com/u/92356313?s=400&u=9df94c6f0e773e86773cb4fcc379f1204a7dcff7&v=4" width="100px;" alt=""/><br /><sub><b>구자경</b></sub></a><br />Perception Architecture
</td>
<td align="center" width="180">
  <a href="https://github.com/Minjea31"><img src="https://avatars.githubusercontent.com/u/80508437?v=4" width="100px;" alt=""/><br /><sub><b>김민재</b></sub></a><br />Computer Vision, <br />DL Pruning
</td>
<td align="center" width="180">
  <a href="https://github.com/namgyu021210"><img src="https://avatars.githubusercontent.com/u/203391491?v=4" width="100px;" alt=""/><br /><sub><b>이남규</b></sub></a><br />Attacker
</td>
</tr>
</table>

## Pipeline

| Pipeline | Input | Output | Package |
|---|---|---|---|
| Livox clustering | `/livox/lidar` | `/perception/livox/centroids` | `livox_clustering` |
| Livox-camera fusion | `/livox/lidar`, `/camera/image_raw/compressed`, `/perception/camera/yolo` | `/perception/fusion/centroids` | `livox_camera_fusion` |
| Velodyne BEV detection | `/velodyne_points` | `/perception/velodyne/centroids` | `velodyne_detection` |

## Packages

| Package | Role |
|---|---|
| `perception_bringup` | 통합 launch |
| `detect_msgs` | 공통 perception message |
| `yolov12` | Camera YOLO detection |
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

System python3 perception deps (livox_clustering, velodyne_detection 노드용):

```bash
sudo python3 -m pip install --no-cache-dir \
  --extra-index-url https://download.pytorch.org/whl/cu121 \
  'typing-extensions==4.13.2' 'numpy>=1.24,<1.25' 'pillow>=10.0,<11.0' \
  'scikit-learn>=1.3,<1.4' \
  torch==2.4.1+cu121 torchvision==0.19.1+cu121 \
  ultralytics==8.4.51 ultralytics-thop==2.0.19 opencv-python==4.13.0.92 \
  filterpy==1.4.5 lap==0.5.12
```

(Dockerfile은 위 핀과 동일 — 컨테이너 기반으로 돌리면 이 단계 불필요)

YOLO env (담당자 셋업 기준 — 타겟 PC `cnu`):

`yolov12` 노드는 shebang(`#!/home/cnu/anaconda3/envs/yolo/bin/python`)으로 conda env를 직접 호출합니다. 이 env에 다음이 깔려 있어야 함:
- yolov12 지원 ultralytics fork (담당자 PC `/home/cnu/clothoid-r/perception_ws/yolov12`)
- `torch`, `numpy`, `opencv-python`, `rospkg`, `thop` — `yolo-requirements.txt` 참조

OC-SORT:

```bash
sudo git clone https://github.com/noahcao/OC_SORT /opt/OC_SORT
```

Workspace build:

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Run

세 명령어를 따로 띄움 (각각 터미널 분리):

```bash
roslaunch perception_bringup perception.launch         # fusion + livox_clustering
rosrun velodyne_detection velodyne_bev_detection.py    # 시스템 python3
rosrun yolov12 yolo_detect.py                          # conda yolo env via shebang
```

Sensor topic override:

```bash
roslaunch perception_bringup perception.launch \
  livox_lidar_topic:=/livox/lidar \
  camera_image_topic:=/camera/image_raw/compressed
```

## Launch Arguments

| Argument | Default |
|---|---|
| `livox_lidar_topic` | `/livox/lidar` |
| `camera_image_topic` | `/camera/image_raw/compressed` |
| `camera_yolo_topic` | `/perception/camera/yolo` |
| `livox_centroid_topic` | `/perception/livox/centroids` |
| `fusion_centroid_topic` | `/perception/fusion/centroids` |

## Verification

Node check:

```bash
rosnode list
```

Expected nodes:

```text
/yolo_detect_node
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
