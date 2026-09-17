# 2026 Clothoid-R Perception

Clothoid-R 자율주행 시스템의 Perception ROS workspace.  
카메라와 LiDAR 기반 객체 검출, 클러스터링, 추적, 센서 퓨전 패키지로 구성됩니다.

## Team

<div align="center">
<table align="center">
<tr>
<td align="center" width="180">
  <a href="https://github.com/JaGyeong1024"><img src="https://avatars.githubusercontent.com/u/92356313?s=400&u=9df94c6f0e773e86773cb4fcc379f1204a7dcff7&v=4" width="100px;" alt=""/><br /><sub><b>구자경</b></sub></a><br />Perception Architecture
</td>
<td align="center" width="180">
  <a href="https://github.com/Minjea31"><img src="https://avatars.githubusercontent.com/u/80508437?v=4" width="100px;" alt=""/><br /><sub><b>김민재</b></sub></a><br />Computer Vision, <br />Model Optimization
</td>
<td align="center" width="180">
  <a href="https://github.com/esem5377"><img src="https://avatars.githubusercontent.com/u/268164639?v=4" width="100px;" alt=""/><br /><sub><b>김은수</b></sub></a><br />Computer Vision, <br />Model Optimization
</td>
<td align="center" width="180">
  <a href="https://github.com/ysjee0229"><img src="https://avatars.githubusercontent.com/u/187412639?v=4" width="100px;" alt=""/><br /><sub><b>지연수</b></sub></a><br />Computer Vision, <br />Edge Deployment
</td>
</tr>
</table>
</div>

## Pipeline

| Pipeline | Input | Output | Package |
|---|---|---|---|
| Camera publish | 웹캠 (V4L2/GStreamer) | `/camera/image_raw`, `/camera/image_raw/compressed` | `camera_start` (system_ws) |
| Camera YOLO | `/camera/image_raw/compressed` | `/perception/camera/yolo` | `yolo26` |
| Livox clustering | `/livox/lidar` | `/perception/livox/centroids` | `livox_clustering` |
| Livox-camera fusion | `/livox/lidar`, `/camera/image_raw/compressed`, `/perception/camera/yolo` | `/perception/fusion/centroids` | `livox_camera_fusion` |
| Velodyne BEV detection | `/velodyne_points` | `/perception/velodyne/centroids` | `velodyne_detection` |

## Layout

| Path | Contents |
|---|---|
| `perception_ws/` | 인지 패키지 (아래 Packages) + `yolo26/` 커스텀 ultralytics |
| `system_ws/` | 센서 드라이버: `camera_start`(웹캠), `livox_ros_driver`, `velodyne` |
| `docs/CNU_SERVER.md` | 차량 PC(cnu) 하드웨어·소프트웨어 환경 |
| `2026_pipeline.drawio/png` | 파이프라인 다이어그램 |

## Packages

| Package | Role |
|---|---|
| `perception_bringup` | 통합 launch |
| `detect_msgs` | 공통 perception message |
| `yolo26` | Camera YOLO detection (구 `yolov12` 대체, 2026-08-16) |
| `livox_clustering` | Livox point cloud clustering and tracking |
| `livox_camera_fusion` | Livox-camera YOLO fusion |
| `velodyne_detection` | Velodyne BEV YOLO detection and OC-SORT tracking |

`system_ws/src`:

| Package | Role |
|---|---|
| `camera_start` | 웹캠 캡처 + undistort 퍼블리시 (빈 프레임 연속 시 자동 재오픈) |
| `livox_ros_driver` | Livox LiDAR 드라이버 |
| `velodyne` | Velodyne VLP-16 드라이버 (`velodyne_pointcloud`) |

## Output Topics

| Topic | Type |
|---|---|
| `/perception/camera/yolo` | `detect_msgs/Yolo_Objects` |
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
rosdep install --from-paths perception_ws/src system_ws/src --ignore-src -r -y
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

(로컬 개발용 Dockerfile은 레포에 포함하지 않음 — `.gitignore` 참조)

YOLO env (담당자 셋업 기준 — 타겟 PC `cnu`):

`yolo26` 노드는 shebang(`#!/home/cnu/anaconda3/envs/yolo/bin/python`)으로 conda env를 직접 호출합니다. 이 env에 다음이 깔려 있어야 함:
- 커스텀 ultralytics (`perception_ws/yolo26`, `yolo_detect.py`가 상대경로로 참조)
- `torch`, `numpy`, `opencv-python`, `rospkg`, `thop` — `yolo-requirements.txt` 참조

OC-SORT:

```bash
sudo git clone https://github.com/noahcao/OC_SORT /opt/OC_SORT
```

Workspace build (두 워크스페이스 각각):

```bash
source /opt/ros/noetic/setup.bash
cd system_ws && catkin_make && source devel/setup.bash && cd ..
cd perception_ws && catkin_make && source devel/setup.bash && cd ..
```

## Run

센서/카메라 (system_ws) 먼저, 인지 노드는 각각 터미널 분리:

```bash
roslaunch livox_ros_driver livox_lidar.launch          # system_ws
roslaunch velodyne_pointcloud VLP16_points.launch      # system_ws
roslaunch camera_start start.launch                    # system_ws (웹캠 + undistort)
roslaunch perception_bringup perception.launch         # fusion + livox_clustering
rosrun velodyne_detection velodyne_bev_detection.py    # 시스템 python3
rosrun yolo26 yolo_detect.py                           # conda yolo env via shebang
```

자주 쓰는 노드 파라미터 (rosrun은 `_이름:=값`, launch는 arg):

```bash
rosrun yolo26 yolo_detect.py _show_image:=true          # 검출 영상 창 표시 (기본 False, headless)
rosrun yolo26 yolo_detect.py _imgsz:=960                # 추론 해상도 (기본 0 = 원본 해상도)
rosrun velodyne_detection velodyne_bev_detection.py _stale_timeout:=0.5   # 입력 두절 시 빈 발행까지 초
roslaunch velodyne_detection velodyne_detection.launch model_version:=velodyne_v6
```

주의: `velodyne_detection.launch`의 `model_version` 기본값은 `velodyne_v4`, `rosrun` 직접 실행 시 스크립트 기본값은 `velodyne_v6`입니다.
`livox_clustering`의 트랙 확정 프레임 수(`tracker_min_hits`, 기본 3)와 크기 게이트는 `config/livox_clustering.yaml`에서 조정합니다.
launch로 띄운 인지 노드(`camera_start`, fusion, clustering, velodyne)는 `respawn="true"`라 죽으면 2초 후 자동 재시작됩니다.

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
| `fusion_projection_config` | `$(find livox_camera_fusion)/config/projection.yaml` |
| `livox_clustering_config` | `$(find livox_clustering)/config/livox_clustering.yaml` |

개별 launch (`livox_camera_fusion.launch`, `livox_clustering.launch`, `velodyne_detection.launch`)의 인자는 각 파일 상단 `<arg>` 참조.

## Verification

Node check:

```bash
rosnode list
```

Expected nodes:

```text
/camera_start_node
/yolo_detect_node
/livox_camera_fusion
/livox_euclidean_clustering
/velodyne_bev_detection_<id>     # anonymous=True 라 접미사가 붙음
```

Topic check:

```bash
rostopic list | grep perception
```

Output publisher check:

```bash
rostopic hz /camera/image_raw/compressed
rostopic info /perception/camera/yolo
rostopic info /perception/livox/centroids
rostopic info /perception/fusion/centroids
rostopic info /perception/velodyne/centroids
```
