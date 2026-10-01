#!/home/cnu/anaconda3/envs/yoloTRT/bin/python
"""
TensorRT 엔진 여러 개(기본: src/yolo26/models 의 모든 .engine)를 같은 영상에 돌려
검출 결과와 추론 속도를 한 창에서 나란히 비교하는 스크립트.

    - 매 프레임을 모든 모델에 넣고(같은 프레임, 같은 전처리) 패널을 격자로 표시
      (3개 이하는 한 줄, 4개부터는 두 줄 — 4개면 위 2개 / 아래 2개. --cols 로 변경)
    - 패널 상단에 모델별 추론 시간(ms) / FPS / 검출 수 / 평균 confidence 표시
    - 모델 간 검출 차이를 박스 색으로 표시 (아래 "박스 색" 참고)
    - 패널 아래에 모델별 평균 confidence 를 한 그래프로 표시 (최근 프레임 구간)
    - --save 로 화면 그대로 mp4 저장 + 전체 구간 confidence 그래프 PNG 저장
    - 종료 시 모델별 통계(시간, FPS, 검출 수, 평균 confidence, 차이 박스 수) 요약 출력

입력·추론 조건은 src/yolo26/scripts/yolo_detect.py 에 맞춘다:
    - 같은 파이썬 환경(쉐뱅), 같은 토픽(/camera/image_raw/compressed, CompressedImage)
    - 같은 전처리(Ultralytics LetterBox 640x640), 같은 기본 confidence
    - yolo_detect.py 의 히스테리시스(keep_confidence)·area-aware NMS·containment 후처리는
      모델과 무관한 공통 로직이라 넣지 않았다 → 패널에는 conf 이상인 모델 출력 그대로 표시

직접 실행하는 용도 (쉐뱅이 yoloTRT 환경을 가리키므로 conda activate 불필요). 예:
    yolo26/compare_models.py                                        # yolo_detect.py 와 같은 토픽 구독
    yolo26/compare_models.py --source /camera/image_raw/compressed
    yolo26/compare_models.py --source ~/Videos/test.mp4             # 영상 파일
    yolo26/compare_models.py --source 0                             # 카메라
    yolo26/compare_models.py --source test.mp4 --save               # compare_<날짜시간>.mp4 로 저장
    yolo26/compare_models.py --source test.mp4 --save out/run1.mp4 --max-speed
    yolo26/compare_models.py --source test.mp4 --loop --conf 0.3
    yolo26/compare_models.py --source test.mp4 --cols 4             # 4개를 한 줄로
    yolo26/compare_models.py --source test.mp4 \
        --models src/yolo26/models/small_fp32.engine src/yolo26/models/small_fp16.engine   # 고른 것만

조작:
    q / ESC : 종료        space : 일시정지 (아무 키나 누르면 재개)

박스 색 (같은 클래스이고 IoU 가 MATCH_IOU 이상이면 같은 물체로 본다):
    초록 실선          : 모든 모델이 검출
    자홍 실선          : 이 모델은 검출했지만 일부 모델만 검출 (다른 모델이 놓침)
    빨강 점선          : 이 모델은 놓침 (다른 모델이 검출한 위치)
    패널 상단의 +n / -n 칩이 그 프레임의 자홍 / 빨강 박스 수.
    영상 위에는 박스만 그린다 (클래스 이름·confidence 글자 없음. confidence 는 상단 띠와 그래프로).

측정 방식:
    모델들을 스레드로 동시에 돌리면 GPU 를 나눠 써서 서로의 시간을 늘린다. 그래서
    한 프레임을 모델마다 차례로(매 프레임 순서를 돌려가며) 실행하고 엔진 실행 시간만 잰다.
    → 화면의 ms / FPS 는 "그 모델 하나만 돌렸을 때"의 순수 추론 속도.
    전처리(letterbox)·후처리는 모든 모델이 같으므로 시간 비교에서 제외.
    평균 confidence 는 conf 이상으로 남은 검출들의 평균 (화면·그래프는 최근 SMOOTH 프레임 이동 평균).

사전 준비:
    - torch(CUDA) + tensorrt + opencv(GUI)  : yoloTRT conda 환경 (쉐뱅)
    - 엔진은 build_trt_engine.py 로 만든 것 (end2end 출력 [1, 300, 6], 메타데이터 없음)
    - 토픽 입력일 때만: roscore + ROS 환경 (source /opt/ros/noetic/setup.bash)
"""
from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

try:
    import tensorrt as trt
    import torch
except Exception as exc:  # pragma: no cover
    sys.exit(
        "[에러] torch / tensorrt import 실패.\n"
        "       yoloTRT 환경의 파이썬으로 실행하세요 (./compare_models.py 또는 conda activate yoloTRT).\n"
        f"       (원인: {exc})"
    )


def _bgr(hex_color: str) -> tuple[int, int, int]:
    return tuple(int(hex_color[i:i + 2], 16) for i in (5, 3, 1))


MODELS_DIR = Path(__file__).resolve().parent.parent / "src" / "yolo26" / "models"
# 패널에 표시할 이름 (파일 이름 → 표시 이름). 여기 없는 엔진은 파일 이름 그대로 표시.
# 순서가 곧 그래프 선 색 순서라, 같은 엔진은 어떤 조합으로 돌려도 항상 같은 색이다.
LABELS = {"nano_fp32": "YOLO26n FP32", "nano_fp16": "YOLO26n FP16",
          "small_fp32": "YOLO26s FP32", "small_fp16": "YOLO26s FP16"}
SCREEN_W = 1920     # 패널 크기 자동 결정 기준 (전체 가로가 이 값을 넘지 않게)
SCREEN_H = 1000     # 창을 처음 띄울 때 넘지 않을 높이 (1080 에서 창 테두리·상단 바를 뺀 값)

# 아래 두 값은 yolo_detect.py 와 같게 둔다 (DEFAULT_SOURCE_TOPIC / DEFAULT_CLASS_CONFIG).
DEFAULT_SOURCE_TOPIC = "/camera/image_raw/compressed"
DEFAULT_CONF = 0.5

MATCH_IOU = 0.5     # 모델 간 같은 물체로 볼 최소 IoU (같은 클래스끼리만)
BOX_AGREE = (60, 220, 60)       # BGR — 모든 모델이 검출
BOX_PARTIAL = (255, 0, 255)     # 일부 모델만 검출 (이 모델은 검출)
BOX_MISS = (40, 40, 255)        # 이 모델만 놓침 (점선)
BOX_THICKNESS = 1               # 박스 선 굵기 (px) — 촘촘한 물체가 뭉개지지 않게 얇게

# 그래프 / 정보 띠 색. 선 색은 어두운 바탕용 범주 팔레트 (순서 고정, 돌려 쓰지 않음).
SERIES_COLORS = [_bgr(c) for c in ("#3987e5", "#d95926", "#199e70", "#c98500",
                                   "#d55181", "#008300", "#9085e9", "#e66767")]
# 선 무늬 (켜짐, 꺼짐 px). 패널 순서대로 하나씩 — 값이 거의 같아 선이 겹쳐도 두 색이 번갈아 보인다.
DASHES = [None, (14, 10), (4, 8), (26, 8)]
KEY_W = 40          # 범례 / 정보 띠의 선 표본 길이 (px)
SURFACE = _bgr("#1a1a19")
GRID = _bgr("#2c2c2a")
INK = (255, 255, 255)
INK_2 = _bgr("#c3c2b7")
MUTED = _bgr("#898781")

WINDOW = "YOLO26 TensorRT compare"
HEADER_H = 84       # 패널 상단 정보 띠 높이 (px)
FOOTER_H = 210      # 패널 아래 범례 + confidence 그래프 높이 (px)
LEGEND_W = 430      # 박스 색 범례 가로 (px)
GRAPH_FRAMES = 300  # 화면 그래프에 보이는 최근 프레임 수
GRAPH_EVERY = 3     # 화면 그래프를 다시 그리는 간격 (프레임)
WARMUP = 30         # 시작 전 모델별 더미 추론 횟수
SMOOTH = 30         # 화면 표시용 ms / confidence 이동 평균 프레임 수
LIVE_SAVE_FPS = 30.0  # 토픽·카메라 입력을 저장할 때의 mp4 FPS (실제 시간에 맞춰 기록)


# --------------------------------------------------------------------------- #
# 인자
# --------------------------------------------------------------------------- #
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="TensorRT 엔진 여러 개를 같은 영상으로 나란히 비교",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--source", default=DEFAULT_SOURCE_TOPIC,
                   help="ROS CompressedImage 토픽, 영상 파일 경로, 또는 카메라 번호")
    p.add_argument("--models", nargs="+", default=sorted(str(p) for p in MODELS_DIR.glob("*.engine")),
                   help="비교할 .engine 경로들 (패널 순서대로). 기본: models 폴더의 모든 .engine")
    p.add_argument("--conf", type=float, default=DEFAULT_CONF, help="confidence threshold")
    p.add_argument("--cols", type=int, default=0,
                   help="한 줄에 놓을 패널 수. 0 이면 자동 (3개 이하는 한 줄, 4개부터는 두 줄)")
    p.add_argument("--panel-width", type=int, default=0,
                   help=f"패널 하나의 가로 크기 (px). 0 이면 자동 (최대 640, 전체 {SCREEN_W} 이내)")
    p.add_argument("--save", nargs="?", const="", default=None, metavar="MP4",
                   help="화면을 mp4 로 저장 (+ 같은 이름의 _conf.png 그래프). "
                        "경로를 생략하면 현재 폴더에 compare_<날짜시간>.mp4")
    p.add_argument("--loop", action="store_true", help="영상 파일이 끝나면 처음부터 반복")
    p.add_argument("--max-speed", action="store_true",
                   help="영상 파일을 FPS 에 맞춰 기다리지 않고 최대 속도로 재생")
    return p.parse_args()


# --------------------------------------------------------------------------- #
# 입력  (read() → (계속 여부, 프레임).  아직 새 프레임이 없으면 프레임은 None)
# --------------------------------------------------------------------------- #
class VideoSource:
    """영상 파일 / 카메라 (cv2.VideoCapture)."""

    def __init__(self, source: str, loop: bool, max_speed: bool) -> None:
        self.realtime = source.isdigit()            # 카메라
        self.cap = cv2.VideoCapture(int(source) if self.realtime else str(Path(source).expanduser()))
        if not self.cap.isOpened():
            sys.exit(f"[에러] 영상을 열 수 없습니다: {source}")
        self.loop = loop
        self.n_read = 0
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.info = f"{source}  ({self.fps:.1f} FPS)"
        # 파일은 원래 속도로 재생 (--max-speed 면 기다리지 않음). 카메라는 read() 가 알아서 기다린다.
        self.period = 1.0 / self.fps if self.fps > 0 and not self.realtime and not max_speed else 0.0

    def read(self) -> tuple[bool, np.ndarray | None]:
        ok, frame = self.cap.read()
        if not ok and self.loop and self.n_read:    # 한 프레임도 못 읽은 영상이면 반복하지 않음
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.n_read = 0
            ok, frame = self.cap.read()
        self.n_read += ok
        return ok, frame

    def release(self) -> None:
        self.cap.release()


class TopicSource:
    """ROS CompressedImage 토픽 (yolo_detect.py 와 같은 입력). 항상 가장 최근 이미지만 쓴다."""

    period = 0.0
    realtime = True
    fps = 0.0

    def __init__(self, topic: str) -> None:
        try:
            import rosgraph
            import rospy
            from sensor_msgs.msg import CompressedImage
        except Exception as exc:
            sys.exit(
                "[에러] rospy import 실패 — ROS 환경을 source 한 셸에서 실행하세요.\n"
                "       (source /opt/ros/noetic/setup.bash)\n"
                f"       (원인: {exc})"
            )
        if not rosgraph.is_master_online():
            sys.exit("[에러] ROS master 에 연결할 수 없습니다 — roscore 가 떠 있는지 확인하세요.\n"
                     "       영상 파일로 볼 때는 --source <파일 경로>")
        self.rospy = rospy
        self.msg = None
        self.arrived = threading.Event()
        self.info = f"{topic}  (ROS 토픽 — 이미지가 들어오면 표시)"
        rospy.init_node("yolo_compare_node", anonymous=True)
        rospy.Subscriber(topic, CompressedImage, self._callback, queue_size=1, buff_size=2**24)

    def _callback(self, msg) -> None:
        self.msg = msg
        self.arrived.set()

    def read(self) -> tuple[bool, np.ndarray | None]:
        if self.rospy.is_shutdown():
            return False, None
        if not self.arrived.wait(0.05):             # 창이 멈추지 않도록 짧게만 기다린다
            return True, None
        self.arrived.clear()
        return True, cv2.imdecode(np.frombuffer(self.msg.data, np.uint8), cv2.IMREAD_COLOR)

    def release(self) -> None:
        self.rospy.signal_shutdown("compare_models 종료")


def open_source(args: argparse.Namespace) -> VideoSource | TopicSource:
    """'/a/b' 꼴이고 그런 파일이 없으면 ROS 토픽, 아니면 영상 파일 / 카메라."""
    if re.fullmatch(r"/[\w/]+", args.source) and not Path(args.source).exists():
        return TopicSource(args.source)
    return VideoSource(args.source, args.loop, args.max_speed)


# --------------------------------------------------------------------------- #
# TensorRT 엔진
# --------------------------------------------------------------------------- #
class TRTModel:
    """TensorRT 엔진 하나. 입출력 버퍼는 torch CUDA 텐서로 잡고 주소만 엔진에 넘긴다."""

    _TORCH_DTYPE = {trt.DataType.FLOAT: torch.float32, trt.DataType.HALF: torch.float16}

    def __init__(self, path: Path, runtime: trt.Runtime, color: tuple[int, int, int],
                 dash: tuple[int, int] | None) -> None:
        self.label = LABELS.get(path.stem, path.stem)
        self.color = color                          # 그래프 선 / 정보 띠의 모델 색
        self.dash = dash                            # 그래프 선 무늬 (None 이면 실선)
        if not path.is_file():
            sys.exit(f"[에러] 엔진 파일이 없습니다: {path}")
        print(f"  · {self.label:14} {path.name}  ({path.stat().st_size / 1e6:.1f} MB)")

        self.engine = runtime.deserialize_cuda_engine(path.read_bytes())
        if self.engine is None:
            sys.exit(f"[에러] 엔진 역직렬화 실패: {path}\n"
                     "       엔진을 빌드한 TensorRT 버전과 현재 환경이 같은지 확인하세요.")
        if self.engine.num_io_tensors != 2:
            sys.exit(f"[에러] 입력 1개 / 출력 1개 엔진만 지원합니다: {path}")
        self.context = self.engine.create_execution_context()

        buffers = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            if -1 in shape:
                sys.exit(f"[에러] 동적 shape 엔진은 지원하지 않습니다: {path}  ({name} {shape})")
            dtype = self._TORCH_DTYPE[self.engine.get_tensor_dtype(name)]
            buffers.append(torch.zeros(shape, dtype=dtype, device="cuda"))
        self.inp, self.out = buffers
        self.bindings = [b.data_ptr() for b in buffers]
        self.size = tuple(self.inp.shape[2:])       # (H, W)

        self.times: list[float] = []                # 프레임별 엔진 실행 시간 (ms)
        self.recent: deque[float] = deque(maxlen=SMOOTH)
        self.n_det = 0
        self.conf_sum = 0.0                         # 전체 검출의 confidence 합
        self.conf_recent: deque[tuple[float, int]] = deque(maxlen=SMOOTH)   # 프레임별 (합, 개수)
        self.conf_series: list[float] = []          # 프레임별 이동 평균 confidence (검출 없으면 nan)
        self.n_partial = 0                          # 일부 모델만 검출한 박스 수 (누적)
        self.n_miss = 0                             # 이 모델만 놓친 박스 수 (누적)

    def infer(self, blob: torch.Tensor) -> tuple[np.ndarray, float]:
        """blob (1,3,H,W) CUDA 텐서 → ((300,6) [x1,y1,x2,y2,conf,cls], 엔진 실행 시간 ms)."""
        self.inp.copy_(blob)
        torch.cuda.synchronize()                    # 입력 복사가 끝난 뒤부터 잰다
        t0 = time.perf_counter()
        ok = self.context.execute_v2(self.bindings)  # 동기 실행 (끝날 때까지 블록)
        ms = (time.perf_counter() - t0) * 1e3
        if not ok:
            sys.exit(f"[에러] 엔진 실행 실패: {self.label}")
        return self.out[0].float().cpu().numpy(), ms

    def warmup(self) -> None:
        blob = torch.zeros(self.inp.shape, device="cuda")
        for _ in range(WARMUP):
            self.infer(blob)

    def record(self, ms: float, det: np.ndarray) -> None:
        """한 프레임의 시간 / 검출 결과를 통계에 쌓는다."""
        self.times.append(ms)
        self.recent.append(ms)
        self.n_det += len(det)
        conf = float(det[:, 4].sum())
        self.conf_sum += conf
        self.conf_recent.append((conf, len(det)))
        n = sum(c for _, c in self.conf_recent)
        self.conf_series.append(sum(s for s, _ in self.conf_recent) / n if n else float("nan"))


# --------------------------------------------------------------------------- #
# 전처리 / 후처리  (Ultralytics LetterBox · scale_boxes 와 같은 계산)
# --------------------------------------------------------------------------- #
def preprocess(frame: np.ndarray, size: tuple[int, int]) -> tuple[torch.Tensor, float, int, int]:
    """BGR 프레임 → letterbox(114 패딩, 가운데) → RGB, 0~1, (1,3,H,W) CUDA 텐서."""
    h, w = frame.shape[:2]
    H, W = size
    r = min(H / h, W / w)
    nw, nh = round(w * r), round(h * r)
    if (nw, nh) != (w, h):
        frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, left = round((H - nh) / 2 - 0.1), round((W - nw) / 2 - 0.1)
    img = cv2.copyMakeBorder(frame, top, H - nh - top, left, W - nw - left,
                             cv2.BORDER_CONSTANT, value=(114, 114, 114))
    chw = np.ascontiguousarray(img[..., ::-1].transpose(2, 0, 1))
    blob = torch.from_numpy(chw).cuda().float().div_(255)[None]
    return blob, r, left, top


def postprocess(out: np.ndarray, conf: float, r: float, left: int, top: int,
                frame_shape: tuple[int, ...]) -> np.ndarray:
    """end2end 출력(NMS 불필요)을 conf 로 거르고 원본 프레임 좌표로 되돌린다."""
    det = out[out[:, 4] >= conf]
    h, w = frame_shape[:2]
    det[:, [0, 2]] = ((det[:, [0, 2]] - left) / r).clip(0, w)
    det[:, [1, 3]] = ((det[:, [1, 3]] - top) / r).clip(0, h)
    return det


# --------------------------------------------------------------------------- #
# 모델 간 검출 비교
# --------------------------------------------------------------------------- #
def box_iou(box: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    """box (4,) 와 boxes (K,4) 의 IoU (K,)."""
    w = (np.minimum(box[2], boxes[:, 2]) - np.maximum(box[0], boxes[:, 0])).clip(0)
    h = (np.minimum(box[3], boxes[:, 3]) - np.maximum(box[1], boxes[:, 1])).clip(0)
    inter = w * h
    union = ((box[2] - box[0]) * (box[3] - box[1])
             + (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1]) - inter)
    return inter / np.maximum(union, 1e-9)


def match_across_models(dets: list[np.ndarray]) -> tuple[list[np.ndarray], list[list[tuple]]]:
    """모델들의 검출을 같은 물체(같은 클래스, IoU >= MATCH_IOU)끼리 묶는다.

    반환:
        votes[i][j] : 모델 i 의 j 번째 박스와 같은 물체를 검출한 모델 수 (자기 포함)
        missed[i]   : 모델 i 만 놓친 물체들 [(x1, y1, x2, y2, cls, 검출한 모델 수), ...]
    """
    # conf 높은 박스부터, 그 모델의 박스가 아직 없는 같은 클래스 묶음 중 IoU 가 가장 큰 곳에 넣는다.
    order = sorted(((float(d[4]), i, j) for i, det in enumerate(dets) for j, d in enumerate(det)),
                   reverse=True)
    reps = np.empty((0, 5))                         # 묶음의 대표 박스 [x1, y1, x2, y2, cls]
    members: list[dict[int, int]] = []              # 묶음마다 {모델 번호: 박스 번호}
    for _, i, j in order:
        box = dets[i][j]
        best = -1
        if members:
            iou = box_iou(box[:4], reps[:, :4])
            iou[reps[:, 4] != box[5]] = 0
            iou[[i in mem for mem in members]] = 0
            if iou.max() >= MATCH_IOU:
                best = int(iou.argmax())
        if best < 0:
            reps = np.vstack([reps, box[[0, 1, 2, 3, 5]]])
            members.append({i: j})
        else:
            members[best][i] = j

    votes = [np.zeros(len(det), int) for det in dets]
    missed: list[list[tuple]] = [[] for _ in dets]
    for rep, mem in zip(reps, members):
        for i in range(len(dets)):
            if i in mem:
                votes[i][mem[i]] = len(mem)
            else:
                missed[i].append((*rep, len(mem)))
    return votes, missed


# --------------------------------------------------------------------------- #
# 화면
# --------------------------------------------------------------------------- #
def put_text(img: np.ndarray, text: str, org: tuple[int, int], scale: float,
             color: tuple[int, int, int], thickness: int = 1) -> int:
    """글자를 쓰고 끝나는 x 좌표를 돌려준다."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)
    return org[0] + cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0][0]


def put_tag(img: np.ndarray, text: str, x: int, y: int, fill: tuple[int, int, int],
            scale: float = 0.45) -> int:
    """색 채운 바탕 위에 글자 (글자색은 바탕 밝기에 따라 검정/흰색). (x, y) 는 왼쪽 아래."""
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    cv2.rectangle(img, (x, y - th - 6), (x + tw + 4, y), fill, -1)
    ink = (0, 0, 0) if 0.114 * fill[0] + 0.587 * fill[1] + 0.299 * fill[2] > 140 else INK
    put_text(img, text, (x + 2, y - 4), scale, ink)
    return x + tw + 4


def styled_polyline(img: np.ndarray, x: np.ndarray, y: np.ndarray, color: tuple[int, int, int],
                    dash: tuple[int, int] | None, phase: float = 0.0, thickness: int = 2) -> None:
    """x 가 증가하는 꺾은선. dash=(켜짐, 꺼짐) px 이면 그 무늬의 점선 (phase 로 무늬 위치를 정한다)."""
    xi = np.arange(np.ceil(x[0]), x[-1] + 1, 2)     # 2px 간격으로 다시 찍는다
    if len(xi) < 2:
        return
    pts = np.stack([xi, np.interp(xi, x, y)], axis=1).round().astype(np.int32)
    if dash is None:
        cv2.polylines(img, [pts], False, color, thickness, cv2.LINE_AA)
        return
    on = (xi + phase) % sum(dash) < dash[0]
    for seg in np.split(np.arange(len(xi)), np.flatnonzero(np.diff(on.astype(np.int8))) + 1):
        if on[seg[0]]:
            cv2.polylines(img, [pts[seg]], False, color, thickness, cv2.LINE_AA)


def series_key(img: np.ndarray, x: int, y: int, model: TRTModel) -> None:
    """모델의 선 표본 (색 + 무늬)."""
    styled_polyline(img, np.array([x, x + KEY_W], float), np.array([y, y], float),
                    model.color, model.dash, phase=-x, thickness=3)


def dashed_rect(img: np.ndarray, p1: tuple[int, int], p2: tuple[int, int],
                color: tuple[int, int, int], dash: int = 4) -> None:
    (x1, y1), (x2, y2) = p1, p2
    for x in range(x1, x2, dash * 2):
        xe = min(x + dash, x2)
        cv2.line(img, (x, y1), (xe, y1), color, BOX_THICKNESS)
        cv2.line(img, (x, y2), (xe, y2), color, BOX_THICKNESS)
    for y in range(y1, y2, dash * 2):
        ye = min(y + dash, y2)
        cv2.line(img, (x1, y), (x1, ye), color, BOX_THICKNESS)
        cv2.line(img, (x2, y), (x2, ye), color, BOX_THICKNESS)


def draw_panel(view: np.ndarray, det: np.ndarray, votes: np.ndarray, missed: list[tuple],
               scale: float, model: TRTModel, n_models: int) -> np.ndarray:
    """축소된 프레임(view) 복사본에 박스를 그리고 위에 정보 띠를 붙인 패널을 만든다."""
    img = view.copy()

    def corners(x1, y1, x2, y2):
        return (int(x1 * scale), int(y1 * scale)), (int(x2 * scale), int(y2 * scale))

    # 영상 위에는 박스만 그린다 (글자 없음).
    # 모두가 검출한 박스를 먼저, 차이 나는 박스를 나중에 그려 차이가 가려지지 않게 한다.
    for j in np.argsort(-votes, kind="stable"):
        p1, p2 = corners(*det[j][:4])
        cv2.rectangle(img, p1, p2, BOX_AGREE if votes[j] == n_models else BOX_PARTIAL, BOX_THICKNESS)
    for miss in missed:
        p1, p2 = corners(*miss[:4])
        dashed_rect(img, p1, p2, BOX_MISS)

    n_partial = int((votes < n_models).sum())
    ms = sum(model.recent) / len(model.recent)
    conf = model.conf_series[-1]
    header = np.full((HEADER_H, img.shape[1], 3), SURFACE, np.uint8)
    series_key(header, 10, 19, model)               # 그래프 선과 같은 색 / 무늬
    put_text(header, model.label, (18 + KEY_W, 27), 0.7, INK, 2)
    put_text(header, f"{ms:.2f} ms   {1e3 / ms:.0f} FPS", (10, 51), 0.55, INK_2)
    x = put_text(header, f"det {len(det)}   conf " + (f"{conf:.3f}" if conf == conf else "-"),
                 (10, 74), 0.5, INK_2)
    for count, sign, color in ((n_partial, "+", BOX_PARTIAL), (len(missed), "-", BOX_MISS)):
        if count:
            x = put_tag(header, f"{sign}{count}", x + 14, 78, color, 0.5)
        else:
            x = put_text(header, f"{sign}0", (x + 16, 74), 0.5, MUTED)
    panel = np.vstack([header, img])
    panel[:, -1] = GRID                             # 패널 사이 구분선
    return panel


def draw_legend(h: int) -> np.ndarray:
    """박스 색 범례."""
    img = np.full((h, LEGEND_W, 3), SURFACE, np.uint8)
    put_text(img, f"Boxes (same object = same class, IoU >= {MATCH_IOU})", (12, 20), 0.5, INK)
    rows = [(52, BOX_AGREE, "detected by every model", None),
            (88, BOX_PARTIAL, "detected here, but not by every model", "+n"),
            (146, BOX_MISS, "not detected here, another model did", "-n")]
    for y, color, text, chip in rows:
        if color == BOX_MISS:
            dashed_rect(img, (14, y - 12), (44, y + 6), color)
        else:
            cv2.rectangle(img, (14, y - 12), (44, y + 6), color, BOX_THICKNESS)
        put_text(img, text, (56, y + 2), 0.45, INK_2)
        if chip:
            x = put_tag(img, chip, 56, y + 26, color, 0.4)
            put_text(img, "in the panel header = how many in this frame", (x + 8, y + 22), 0.4, MUTED)
    return img


def nice_step(span: float, max_ticks: int) -> float:
    """눈금 간격: span 을 max_ticks 개 이하로 나누는 1·2·5 × 10^n."""
    raw = span / max_ticks
    mag = 10.0 ** np.floor(np.log10(raw))
    return next(m * mag for m in (1, 2, 5, 10) if m * mag >= raw)


def draw_graph(w: int, h: int, models: list[TRTModel], start: int, stop: int) -> np.ndarray:
    """모든 모델의 평균 confidence(이동 평균)를 한 그래프에 그린다 — 프레임 [start, stop) 구간."""
    img = np.full((h, w, 3), SURFACE, np.uint8)
    stride = max(1, (stop - start) // 2000)         # 긴 구간은 점을 솎아 그린다
    xs = np.arange(start, stop, stride)
    ys = [np.asarray(m.conf_series[start:stop:stride], float) for m in models]

    put_text(img, f"Mean confidence of detections ({SMOOTH}-frame moving average)", (12, 20), 0.5, INK)
    x, key_y = 12, 40                               # 범례: 선 표본 + 이름 + 현재 값 (글자는 잉크색)
    for m, y in zip(models, ys):
        now = f"{y[-1]:.3f}" if len(y) and np.isfinite(y[-1]) else "-"
        text = f"{m.label}  {now}"
        item_w = KEY_W + 8 + cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0][0]
        if x > 12 and x + item_w > w - 10:          # 한 줄에 다 안 들어가면 다음 줄로
            x, key_y = 12, key_y + 20
        series_key(img, x, key_y, m)
        x = put_text(img, text, (x + KEY_W + 8, key_y + 5), 0.45, INK_2) + 26

    left, right, top, bottom = 58, w - 22, key_y + 22, h - 26
    finite = np.concatenate(ys)[np.isfinite(np.concatenate(ys))]
    lo, hi = (finite.min(), finite.max()) if finite.size else (0.0, 1.0)
    ystep = nice_step(max(hi - lo, 0.01), 4)
    lo, hi = np.floor(lo / ystep) * ystep, np.ceil(hi / ystep) * ystep
    if hi - lo < ystep / 2:
        hi = lo + ystep

    def px(frame):
        return left + (frame - start) / max(stop - 1 - start, 1) * (right - left)

    def py(value):
        return bottom - (value - lo) / (hi - lo) * (bottom - top)

    digits = 3 if ystep < 0.01 else 2
    for v in np.arange(lo, hi + ystep / 2, ystep):
        y = int(round(py(v)))
        cv2.line(img, (left, y), (right, y), GRID, 1)
        text = f"{v:.{digits}f}"
        tw = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0]
        put_text(img, text, (left - 8 - tw, y + 4), 0.4, MUTED)
    xstep = max(1, int(nice_step(max(stop - start, 1), 8)))
    for f in range(-(-start // xstep) * xstep, stop, xstep):
        text = str(f)
        tw = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0]
        put_text(img, text, (int(px(f)) - tw // 2, bottom + 17), 0.4, MUTED)
    put_text(img, "frame", (12, bottom + 17), 0.4, MUTED)

    # 무늬를 프레임 번호에 묶어 둔다 (그래프가 옆으로 흘러도 무늬가 선을 따라 움직이게)
    phase = start * (right - left) / max(stop - 1 - start, 1) - left
    for m, y in sorted(zip(models, ys), key=lambda t: t[0].dash is not None):   # 실선 먼저, 점선을 위에
        ok = np.isfinite(y)
        # 검출이 없던 구간(nan)에서는 선을 끊는다
        for run in np.split(np.arange(len(y)), np.flatnonzero(np.diff(ok.astype(np.int8))) + 1):
            if len(run) and ok[run[0]]:
                styled_polyline(img, px(xs[run]), py(y[run]), m.color, m.dash, phase)
        if len(y) and ok[-1]:                       # 끝점: 바탕색 테두리를 둘러 겹쳐도 보이게
            end = (int(round(px(xs[-1]))), int(round(py(y[-1]))))
            cv2.circle(img, end, 6, SURFACE, -1, cv2.LINE_AA)
            cv2.circle(img, end, 4, m.color, -1, cv2.LINE_AA)
    return img


# --------------------------------------------------------------------------- #
# 저장
# --------------------------------------------------------------------------- #
class Recorder:
    """화면(canvas)을 mp4 로 저장한다.

    영상 파일 입력은 프레임마다 한 장씩 원본 FPS 로, 토픽·카메라 입력은 들어오는 간격이
    일정하지 않으므로 실제 흐른 시간에 맞춰 LIVE_SAVE_FPS 로 기록한다 (재생 속도 = 실제 속도).
    """

    def __init__(self, path: Path, src: VideoSource | TopicSource) -> None:
        self.path = path
        self.realtime = src.realtime
        self.fps = LIVE_SAVE_FPS if src.realtime or src.fps <= 0 else src.fps
        self.writer = None
        self.last = None
        self.due = 0.0

    def write(self, canvas: np.ndarray) -> None:
        if self.writer is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"),
                                          self.fps, (canvas.shape[1], canvas.shape[0]))
            if not self.writer.isOpened():
                sys.exit(f"[에러] 저장 파일을 열 수 없습니다: {self.path}")
        n = 1
        now = time.perf_counter()
        if self.realtime and self.last is not None:
            self.due += min(now - self.last, 1.0) * self.fps    # 일시정지 같은 긴 공백은 1초로 자른다
            n = int(self.due)
            self.due -= n
        self.last = now
        for _ in range(n):
            self.writer.write(canvas)

    def release(self) -> None:
        if self.writer is not None:
            self.writer.release()


def print_summary(models: list[TRTModel], n_frames: int) -> None:
    if not n_frames:
        print("\n[결과] 처리한 프레임이 없습니다.")
        return
    print(f"\n[결과] {n_frames} 프레임  —  엔진 실행 시간(ms), FPS = 1000 / mean")
    print("       conf = 검출 전체의 평균 confidence,  +some = 일부 모델만 검출한 박스 수,"
          "  -miss = 이 모델만 놓친 박스 수")
    print(f"  {'model':16} {'mean':>7} {'p50':>7} {'p99':>7} {'max':>7} {'FPS':>8} "
          f"{'det/frame':>10} {'conf':>7} {'+some':>7} {'-miss':>7}")
    for m in models:
        t = np.asarray(m.times)
        conf = f"{m.conf_sum / m.n_det:7.3f}" if m.n_det else f"{'-':>7}"
        print(f"  {m.label:16} {t.mean():7.2f} {np.percentile(t, 50):7.2f} "
              f"{np.percentile(t, 99):7.2f} {t.max():7.2f} {1e3 / t.mean():8.1f} "
              f"{m.n_det / n_frames:10.2f} {conf} {m.n_partial:7d} {m.n_miss:7d}")


# --------------------------------------------------------------------------- #
def main() -> None:
    args = parse_args()
    if not torch.cuda.is_available():
        sys.exit("[에러] CUDA 디바이스가 보이지 않습니다 (torch.cuda.is_available() == False).")
    if not args.models:
        sys.exit(f"[에러] 비교할 엔진이 없습니다: {MODELS_DIR}/*.engine")
    if len(args.models) > len(SERIES_COLORS):
        sys.exit(f"[에러] 한 번에 비교할 수 있는 엔진은 {len(SERIES_COLORS)}개까지입니다.")
    n = len(args.models)
    cols = min(n, args.cols) if args.cols > 0 else (n if n <= 3 else -(-n // 2))
    panel_width = args.panel_width or min(640, SCREEN_W // cols)

    src = open_source(args)
    recorder = None
    if args.save is not None:
        recorder = Recorder(Path(args.save or time.strftime("compare_%Y%m%d_%H%M%S.mp4")).expanduser(), src)

    logger = trt.Logger(trt.Logger.WARNING)
    trt.init_libnvinfer_plugins(logger, "")
    runtime = trt.Runtime(logger)
    print(f"[로드] TensorRT {trt.__version__}  /  {torch.cuda.get_device_name(0)}")
    # 선 색: LABELS 에 있는 엔진은 항상 자기 자리 색, 나머지는 남는 색을 차례로
    paths = [Path(p).expanduser() for p in args.models]
    known = list(LABELS)
    stems = [p.stem for p in paths]
    spare = (list(range(len(known), len(SERIES_COLORS)))
             + [k for k, stem in enumerate(known) if stem not in stems])
    models = [TRTModel(p, runtime, SERIES_COLORS[known.index(p.stem) if p.stem in known else spare.pop(0)],
                       DASHES[k % len(DASHES)])
              for k, p in enumerate(paths)]
    for m in models:
        m.warmup()

    print(f"[시작] {src.info}   q/ESC 종료, space 일시정지")
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    legend = draw_legend(FOOTER_H)
    n_frames = 0
    try:
        while True:
            t_frame = time.perf_counter()
            ok, frame = src.read()
            if not ok:
                break

            if frame is not None:
                # 모든 모델에 같은 프레임. 실행 순서는 매 프레임 돌려 순서에 따른 유불리를 없앤다.
                pre: dict[tuple[int, int], tuple] = {}
                dets: list[np.ndarray | None] = [None] * len(models)
                for k in range(len(models)):
                    i = (k + n_frames) % len(models)
                    m = models[i]
                    if m.size not in pre:
                        pre[m.size] = preprocess(frame, m.size)
                    blob, r, left, top = pre[m.size]
                    out, ms = m.infer(blob)
                    dets[i] = postprocess(out, args.conf, r, left, top, frame.shape)
                    m.record(ms, dets[i])
                n_frames += 1

                votes, missed = match_across_models(dets)
                for m, v, miss in zip(models, votes, missed):
                    m.n_partial += int((v < len(models)).sum())
                    m.n_miss += len(miss)

                scale = panel_width / frame.shape[1]
                view = cv2.resize(frame, (panel_width, int(frame.shape[0] * scale) // 2 * 2),
                                  interpolation=cv2.INTER_LINEAR)
                panels = [draw_panel(view, d, v, miss, scale, m, len(models))
                          for d, v, miss, m in zip(dets, votes, missed, models)]
                panels += [np.full_like(panels[0], SURFACE)] * (-len(panels) % cols)   # 마지막 줄 빈칸
                panels = np.vstack([np.hstack(panels[k:k + cols]) for k in range(0, len(panels), cols)])
                if (n_frames - 1) % GRAPH_EVERY == 0:
                    graph = draw_graph(panels.shape[1] - LEGEND_W, FOOTER_H, models,
                                       max(0, n_frames - GRAPH_FRAMES), n_frames)
                canvas = np.vstack([panels, np.hstack([legend, graph])])
                cv2.imshow(WINDOW, canvas)
                if n_frames == 1:                   # 화면보다 크면 비율을 유지한 채 줄여서 띄운다
                    fit = min(1.0, SCREEN_W / canvas.shape[1], SCREEN_H / canvas.shape[0])
                    cv2.resizeWindow(WINDOW, int(canvas.shape[1] * fit), int(canvas.shape[0] * fit))
                if recorder:
                    recorder.write(canvas)

            wait_ms = max(1, int((src.period - (time.perf_counter() - t_frame)) * 1e3))
            key = cv2.waitKey(wait_ms) & 0xFF
            if key == ord(" "):
                key = cv2.waitKey(0) & 0xFF             # 일시정지: 아무 키나 누르면 재개
            if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:                       # Ctrl+C 로 끝내도 요약 출력 / mp4 마무리는 한다
        pass

    src.release()
    cv2.destroyAllWindows()
    print_summary(models, n_frames)
    if recorder and n_frames:
        recorder.release()
        graph_path = recorder.path.with_name(recorder.path.stem + "_conf.png")
        cv2.imwrite(str(graph_path), draw_graph(1600, 520, models, 0, n_frames))
        print(f"\n[저장] 영상   : {recorder.path}  ({recorder.fps:.1f} FPS)")
        print(f"       그래프 : {graph_path}")


if __name__ == "__main__":
    main()
