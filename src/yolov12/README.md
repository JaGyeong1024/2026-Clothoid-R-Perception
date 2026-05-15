# yolov12

카메라 압축 이미지에서 YOLOv12로 객체를 검출하는 노드 (Python).

## 토픽

### 입력 (구독)

| 토픽 | 메시지 |
|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` |

### 출력 (발행)

| 토픽 | 메시지 | 용도 |
|---|---|---|
| `/perception/camera/yolo` | `detect_msgs/Yolo_Objects` | livox_camera_fusion의 입력 + 디버깅 |

## 파라미터 (`~private`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `source` | `/camera/image_raw/compressed` | 입력 카메라 토픽 |
| `output_topic` | `/perception/camera/yolo` | 출력 토픽 |
| `yaml_cfg` | `$(find yolov12)/models/prune_0510.yaml` | YOLO 모델 yaml |
| `pt_weights` | `$(find yolov12)/models/prune_0510.pt` | YOLO 가중치 .pt |
| `confidence` | `0.4` | confidence threshold |
| `frame_id` | `camera_link` | 출력 frame_id |
| `publish_classes` | `[0, 1, 2]` | 발행 클래스 ID 리스트. 빈 리스트면 전부 발행 |
| `ultralytics_path` | `""` | yolov12 fork 소스 경로(필요 시). `YOLOV12_PATH` 환경변수로도 지정 가능 |
| `show_detection_image` | `false` | BBox 그려서 OpenCV 창에 표시 |

## 클래스 매핑

| ID | 이름 |
|---|---|
| 0 | ERP-42 |
| 1 | drum |
| 2 | cone |

`publish_classes` 파라미터로 일부 클래스만 선별 발행 가능.

## 실행 환경 (담당자 셋업 기준)

`yolo_detect.py`의 shebang은 담당자 의도 표현용으로 유지합니다.

```python
#!/home/cnu/anaconda3/envs/yolo/bin/python
```

다만 catkin이 `devel/lib/yolov12/yolo_detect.py`에 wrapper(`#!/usr/bin/python3` 사용)를 생성해 실행 시 shebang을 우회하므로, **실제 인터프리터는 launch-prefix로 강제**합니다. wrapper는 그 인터프리터에서 `exec()`로 원본 코드를 실행하므로 결과적으로 cnu conda env python이 yolov12 코드를 돌립니다.

`yolov12.launch`가 처리하는 것:
- `launch-prefix="$(arg python_interp)"` — 노드 인터프리터를 conda env python으로 강제
- `<env name="LD_LIBRARY_PATH" value="..."/>` — 이 노드 프로세스에만 LD_LIBRARY_PATH set
- 부모 셸/다른 노드/이미지 전역 환경 무영향

타겟 env(`/home/cnu/anaconda3/envs/yolo`)에는 다음이 미리 설치되어 있어야 합니다:

- yolov12 지원 ultralytics fork (담당자 PC `/home/cnu/clothoid-r/perception_ws/yolov12`)
- `torch`, `numpy`, `opencv-python`, `rospkg` 등

### 실행 절차

타겟 PC(cnu)에서는 launch 한 번이면 끝. 별도 export 없음.

```bash
roslaunch yolov12 yolov12.launch
```

다른 환경(다른 PC, 도커 등)이면 인터프리터/lib 경로 override:

```bash
roslaunch yolov12 yolov12.launch \
  python_interp:=$HOME/anaconda3/envs/yolo/bin/python \
  conda_env_lib:=$HOME/anaconda3/envs/yolo/lib
```

> `<env>` 태그는 노드 프로세스만 영향 주므로 안전합니다. 다만 `~/.bashrc` 등에 `LD_LIBRARY_PATH`를 전역 등록하는 짓은 **절대 하지 말 것** — 시스템 .so 로딩 순서가 깨져 다른 ROS 패키지가 망가집니다.

### 다른 환경에서 실행하려면

타겟 PC(cnu)가 아닌 환경(다른 워크스테이션/도커)에서 동작시키려면:

1. 동일 경로(`/home/cnu/anaconda3/envs/yolo`)에 conda env 구성 — 또는 shebang을 해당 환경의 python 경로로 교체
2. 그 env에 yolov12 fork(ultralytics) install
3. `LD_LIBRARY_PATH`를 해당 env의 `lib` 경로로 export 후 roslaunch

shebang이 절대경로라 환경이 다르면 노드 기동에서 바로 실패합니다.

## 알고리즘 요약

```
CompressedImage → cv2 decode
  → ultralytics YOLO(v12) inference
  → publish_classes 필터링
  → detect_msgs/Yolo_Objects 발행
```
