#!/usr/bin/env python
"""
model/best.pt  ->  ONNX  ->  TensorRT 엔진(.engine)  2단계 변환 스크립트.

    [1] PyTorch(.pt) -> ONNX        : Ultralytics 의 model.export(format="onnx")
    [2] ONNX        -> TensorRT     : TensorRT Python API 로 엔진 직접 빌드

직접 실행하는 용도. 예:
    conda activate yolo
    python TensorRT/build_trt_engine.py --fp16
    python TensorRT/build_trt_engine.py \
        --weights model/best.pt --imgsz 640 --fp16 --workspace 4 \
        --onnx model/best.onnx --engine model/best.engine
    python TensorRT/build_trt_engine.py --skip-engine          # [1]만 (ONNX까지)
    python TensorRT/build_trt_engine.py --skip-onnx --fp16     # 기존 ONNX 재사용, [2]만

사전 준비:
    - torch 2.x + ultralytics 포크        : `cd yolo26 && pip install -r requirements.txt`
    - ONNX export 확인용(선택)            : onnx, onnxruntime  (이미 yolo env 에 있음)
    - TensorRT (엔진 빌드에 필수)         : `pip install tensorrt`  (CUDA 버전에 맞는 휠)
                                           또는 NVIDIA tar/deb 설치 후 PYTHONPATH 설정
    - NVIDIA GPU + 드라이버

산출물 (경로 미지정 시 <weights> 이름 기준, 정밀도 플래그가 붙으면 접미사 추가):
    - best.engine / best_fp16.engine / best_int8.engine / best_fp16_int8.engine
                                   : 직렬화된 TensorRT 엔진 (--fp16 / --int8 조합에 따라)
                                     --engine 로 직접 지정하면 그 경로를 그대로 쓴다
    - best.onnx  (또는 --half 시 best_fp16.onnx) : 변환된 ONNX (Netron 으로 그래프 확인)
                                     --fp16 만으론 ONNX 는 FP32 그대로 → 접미사 없음
    - <engine>.layers.json         : TensorRT 가 fusion/정밀도 적용한 뒤의 레이어 정보
                                    ("TensorRT 적용 후 그래프" 그릴 때 쓸 재료)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


# --------------------------------------------------------------------------- #
# repo 루트 탐색 (어느 폴더에서 실행해도 동작)
# --------------------------------------------------------------------------- #
def find_repo_root() -> Path:
    here = Path(__file__).resolve()
    cwd = Path.cwd().resolve()
    for base in (here.parent, *here.parents, cwd, *cwd.parents):
        if (base / "yolo26" / "ultralytics" / "__init__.py").exists():
            return base
    return here.parent.parent


REPO_ROOT = find_repo_root()


# --------------------------------------------------------------------------- #
# 인자
# --------------------------------------------------------------------------- #
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="best.pt -> ONNX -> TensorRT engine (2단계 변환)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # 경로는 repo 루트 기준 상대경로. main() 에서 os.chdir(REPO_ROOT) 한다.
    p.add_argument("--weights", default="model/best.pt",
                   help="입력 가중치 (.pt) — repo 루트 기준")
    p.add_argument("--onnx", default=None,
                   help="ONNX 출력 경로 (기본: weights 와 같은 이름의 .onnx)")
    p.add_argument("--engine", default=None,
                   help="engine 출력 경로 (기본: weights 와 같은 이름의 .engine)")
    p.add_argument("--imgsz", type=int, default=640, help="입력 해상도 (정사각)")
    p.add_argument("--opset", type=int, default=None, help="ONNX opset (기본: ultralytics 기본값)")
    p.add_argument("--batch", type=int, default=1, help="정적 batch 크기")
    p.add_argument("--dynamic", action="store_true", help="동적 batch ONNX/엔진 생성")
    p.add_argument("--max-batch", type=int, default=8, help="--dynamic 일 때 최대 batch")
    p.add_argument("--simplify", action="store_true", help="onnxslim 으로 ONNX 단순화")
    p.add_argument("--half", action="store_true", help="ONNX 자체를 FP16 로 export (CUDA 필요)")

    p.add_argument("--fp16", action="store_true",
                   help="TensorRT FP16 모드로 빌드 (경로 미지정 시 best_fp16.engine)")
    p.add_argument("--int8", action="store_true",
                   help="TensorRT INT8 플래그 — 캘리브레이터 없음, 정확도 주의 (best_int8.engine)")
    p.add_argument("--workspace", type=float, default=4.0, help="TensorRT workspace (GiB)")
    p.add_argument("--verbose", action="store_true", help="TensorRT 로그를 VERBOSE 로")

    p.add_argument("--skip-onnx", action="store_true", help="[1] 건너뛰고 기존 ONNX 재사용")
    p.add_argument("--skip-engine", action="store_true", help="[2] 건너뛰고 ONNX 까지만")
    p.add_argument("--no-layer-info", dest="layer_info", action="store_false",
                   help="엔진 레이어 정보 JSON 덤프를 하지 않음")
    return p.parse_args()


def _default_path(weights: str, given: str | None, suffix: str, tag: str = "") -> Path:
    if given:
        return Path(given)
    p = Path(weights)
    stem = f"{p.stem}_{tag}" if tag else p.stem
    return p.with_name(stem + suffix)


def _precision_tag(args: argparse.Namespace) -> str:
    """정밀도 플래그 → 파일명 접미사 (fp16 / int8 / fp16_int8). 기본 FP32 는 빈 문자열."""
    parts = []
    if args.fp16:
        parts.append("fp16")
    if args.int8:
        parts.append("int8")
    return "_".join(parts)


# --------------------------------------------------------------------------- #
# [1] PyTorch(.pt) -> ONNX   (Ultralytics export)
# --------------------------------------------------------------------------- #
def export_onnx(args: argparse.Namespace) -> Path:
    # ONNX 파일명 접미사는 --half 일 때만 `_fp16` (그 경우 ONNX 가중치 자체가 FP16).
    # --fp16 은 엔진 정밀도라 ONNX 내용은 안 바뀌므로 접미사 없음.
    # args.half 는 아래 CUDA 체크에서 꺼질 수 있어 그때 경로를 다시 잡는다.
    out = _default_path(args.weights, args.onnx, ".onnx", "fp16" if args.half else "")

    if args.skip_onnx:
        if not out.exists():
            sys.exit(f"[에러] --skip-onnx 인데 ONNX 가 없습니다: {out}")
        print(f"[1/2] ONNX 재사용: {out}")
        return out

    # repo 안 커스텀 ultralytics 포크 우선 (main() 에서 이미 os.chdir(REPO_ROOT))
    sys.path.insert(0, "yolo26")
    try:
        import torch
        from ultralytics import YOLO
    except Exception as exc:  # pragma: no cover
        sys.exit(
            "[에러] torch / ultralytics import 실패.\n"
            "       `conda activate yolo` 후 실행하세요.\n"
            f"       (원인: {exc})"
        )

    # device="cpu" 로 export 하면 Ultralytics 가 CUDA_VISIBLE_DEVICES="" 로 GPU 를 숨겨
    # 같은 프로세스의 [2] TensorRT 빌드가 GPU 를 못 찾는다 → GPU 가 있으면 GPU 로 export
    device = "0" if torch.cuda.is_available() else "cpu"
    if args.half and device == "cpu":
        print("      [경고] --half 는 CUDA 필요 → FP32 ONNX 로 진행")
        args.half = False
        out = _default_path(args.weights, args.onnx, ".onnx", "")

    print(f"[1/2] PyTorch → ONNX   {Path(args.weights).name} → {out.name}")
    print(f"      imgsz={args.imgsz}  batch={args.batch}  dynamic={args.dynamic}  "
          f"half={args.half}  simplify={args.simplify}  device={device}")

    kwargs = dict(
        format="onnx",
        imgsz=args.imgsz,
        dynamic=args.dynamic,
        simplify=args.simplify,
        half=args.half,
        batch=args.batch,
        device=device,
    )
    if args.opset is not None:
        kwargs["opset"] = args.opset

    exported = Path(YOLO(args.weights).export(**kwargs))
    if exported.resolve() != out.resolve():
        out.parent.mkdir(parents=True, exist_ok=True)
        exported.replace(out)

    print(f"      완료: {out}  ({out.stat().st_size / 1e6:.1f} MB)")
    _check_onnx(out)
    return out


def _check_onnx(onnx_path: Path) -> None:
    """onnx / onnxruntime 가 있으면 간단히 유효성 + IO 를 확인 (선택)."""
    try:
        import onnx
        onnx.checker.check_model(str(onnx_path))
        print("      onnx.checker: OK")
    except Exception as exc:
        print(f"      [경고] onnx.checker 스킵/실패: {exc}")
    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
        for t in sess.get_inputs():
            print(f"      ONNX IN  {t.name:20} {t.shape}  {t.type}")
        for t in sess.get_outputs():
            print(f"      ONNX OUT {t.name:20} {t.shape}  {t.type}")
    except Exception as exc:
        print(f"      [경고] onnxruntime 확인 스킵: {exc}")


# --------------------------------------------------------------------------- #
# [2] ONNX -> TensorRT engine   (TensorRT Python API)
# --------------------------------------------------------------------------- #
def build_engine(onnx_path: Path, args: argparse.Namespace) -> Path | None:
    out = _default_path(args.weights, args.engine, ".engine", _precision_tag(args))

    if args.skip_engine:
        print("[2/2] --skip-engine : 엔진 빌드 건너뜀")
        return None

    try:
        import tensorrt as trt
    except Exception as exc:
        sys.exit(
            "[에러] `import tensorrt` 실패 — TensorRT 가 설치되지 않았습니다.\n"
            "       설치 예) pip install tensorrt      (CUDA 12 → tensorrt-cu12)\n"
            "       또는 NVIDIA 공식 tar/deb 설치 후 PYTHONPATH 설정.\n"
            f"       (원인: {exc})"
        )

    # --- CUDA 디바이스 가시성 확인 -------------------------------------------
    # TensorRT 는 보이는 device 가 0개면 Python 예외가 아니라 C++ std::terminate
    # (Aborted / core dumped) 로 죽는다. 그 전에 원인을 명확히 안내한다.
    cvd = os.environ.get("CUDA_VISIBLE_DEVICES")
    ldp = os.environ.get("LD_LIBRARY_PATH", "")
    try:
        import torch
        n_dev = torch.cuda.device_count()
    except Exception:
        n_dev = -1
    if n_dev == 0:
        stale = [p for p in ldp.split(":") if "cuda" in p.lower()]
        sys.exit(
            "[에러] CUDA 디바이스가 보이지 않습니다 (torch.cuda.device_count() == 0).\n"
            f"       CUDA_VISIBLE_DEVICES = {cvd!r}\n"
            f"       LD_LIBRARY_PATH 의 cuda 경로 = {stale or '(없음)'}\n\n"
            "       해결:\n"
            "        1) CUDA_VISIBLE_DEVICES 가 '' 또는 '-1' 이면 →  unset CUDA_VISIBLE_DEVICES\n"
            "        2) 구버전 시스템 CUDA(/usr/local/cuda-10.x 등)가 LD_LIBRARY_PATH 에 있으면\n"
            "           그 셸에서 아래처럼 깨끗한 환경으로 실행:\n"
            "             env -u LD_LIBRARY_PATH python TensorRT/build_trt_engine.py --skip-onnx" +
            ("" if args.fp16 is False else " --fp16") + "\n"
            "        3) ROS(setup.bash) 를 source 한 셸이면 새 터미널에서 conda 환경만 활성화 후 재실행\n"
        )
    # ---------------------------------------------------------------------

    trt_major = int(trt.__version__.split(".")[0])
    log_level = trt.Logger.VERBOSE if args.verbose else trt.Logger.INFO
    logger = trt.Logger(log_level)
    trt.init_libnvinfer_plugins(logger, "")          # EfficientNMS_TRT 등 표준 플러그인 등록

    print(f"[2/2] ONNX → TensorRT engine   (TensorRT {trt.__version__})")

    builder = trt.Builder(logger)

    # 네트워크: TRT<10 은 EXPLICIT_BATCH 플래그 필요, TRT>=10 은 기본 explicit
    flags = 0
    if trt_major < 10 and hasattr(trt, "NetworkDefinitionCreationFlag"):
        flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(flags)

    parser = trt.OnnxParser(network, logger)
    ok = (parser.parse_from_file(str(onnx_path))
          if hasattr(parser, "parse_from_file")
          else parser.parse(onnx_path.read_bytes()))
    if not ok:
        for i in range(parser.num_errors):
            print("      [ONNX parse]", parser.get_error(i))
        sys.exit("[에러] ONNX 파싱 실패")

    config = builder.create_builder_config()

    ws_bytes = int(args.workspace * (1 << 30))
    if hasattr(config, "set_memory_pool_limit"):
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, ws_bytes)
    else:                                            # TRT 8.3 이하
        config.max_workspace_size = ws_bytes

    if args.fp16:
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print("      FP16 enabled")
        else:
            print("      [경고] 빠른 FP16 미지원 플랫폼 → FP32 로 빌드")
    if args.int8:
        if builder.platform_has_fast_int8:
            config.set_flag(trt.BuilderFlag.INT8)
            print("      [주의] INT8 캘리브레이터 미제공 → 정확도 크게 저하될 수 있음")
        else:
            print("      [경고] INT8 미지원 → 무시")

    if args.layer_info and hasattr(trt, "ProfilingVerbosity"):
        config.profiling_verbosity = trt.ProfilingVerbosity.DETAILED  # 레이어 정보 상세화

    # 동적 shape → optimization profile
    inp = network.get_input(0)
    in_shape = tuple(inp.shape)
    if args.dynamic or -1 in in_shape or (in_shape and in_shape[0] == -1):
        c = in_shape[1] if in_shape[1] != -1 else 3
        h = in_shape[2] if in_shape[2] != -1 else args.imgsz
        w = in_shape[3] if in_shape[3] != -1 else args.imgsz
        opt_b = max(1, args.max_batch // 2)
        profile = builder.create_optimization_profile()
        profile.set_shape(inp.name, (1, c, h, w), (opt_b, c, h, w), (args.max_batch, c, h, w))
        config.add_optimization_profile(profile)
        print(f"      dynamic profile: min=1  opt={opt_b}  max={args.max_batch}  (C{c} H{h} W{w})")
    else:
        print(f"      static input: {in_shape}")

    print("      엔진 빌드 중... (수 초 ~ 수 분)")
    if hasattr(builder, "build_serialized_network"):
        serialized = builder.build_serialized_network(network, config)
        if serialized is None:
            sys.exit("[에러] 엔진 빌드 실패 (build_serialized_network → None)")
        data = bytes(serialized)
    else:                                            # 아주 옛 API
        engine = builder.build_engine(network, config)
        if engine is None:
            sys.exit("[에러] 엔진 빌드 실패 (build_engine → None)")
        data = engine.serialize()

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(data)
    print(f"      완료: {out}  ({len(data) / 1e6:.1f} MB)")

    _summarize_engine(trt, logger, data, out, args)
    return out


def _summarize_engine(trt, logger, data: bytes, out: Path, args: argparse.Namespace) -> None:
    """빌드된 엔진을 역직렬화해 IO 를 출력하고, 레이어 정보를 JSON 으로 덤프."""
    engine = trt.Runtime(logger).deserialize_cuda_engine(data)
    if engine is None:
        print("      [경고] 엔진 역직렬화 실패 → 요약 생략")
        return

    print("      --- 엔진 IO 텐서 ---")
    if hasattr(engine, "num_io_tensors"):           # TRT >= 8.5
        for i in range(engine.num_io_tensors):
            name = engine.get_tensor_name(i)
            is_in = engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            shape = tuple(engine.get_tensor_shape(name))
            dtype = engine.get_tensor_dtype(name)
            print(f"      {'IN ' if is_in else 'OUT'} {name:24} {shape}  {dtype}")
    else:                                           # 구버전 바인딩 API
        for i in range(engine.num_bindings):
            is_in = engine.binding_is_input(i)
            print(f"      {'IN ' if is_in else 'OUT'} {engine.get_binding_name(i):24} "
                  f"{tuple(engine.get_binding_shape(i))}")

    if not args.layer_info:
        return
    try:
        inspector = engine.create_engine_inspector()
        info = inspector.get_engine_information(trt.LayerInformationFormat.JSON)
        lp = Path(str(out) + ".layers.json")
        lp.write_text(info, encoding="utf-8")
        try:
            n = len(json.loads(info).get("Layers", []))
            print(f"      레이어 정보 {n}개 → {lp}")
        except Exception:
            print(f"      레이어 정보 → {lp}")
    except Exception as exc:
        print(f"      [경고] 레이어 정보 덤프 실패: {exc}")


# --------------------------------------------------------------------------- #
def main() -> None:
    os.chdir(REPO_ROOT)          # 이후 모든 상대경로는 repo 루트 기준
    args = parse_args()
    onnx_path = export_onnx(args)
    build_engine(onnx_path, args)
    print("\n[완료] 파이프라인 종료")
    print("  · ONNX 그래프  : Netron 으로 .onnx 열기 (https://netron.app)")
    print("  · TensorRT 그래프: .engine.layers.json (fusion/정밀도 반영된 레이어) 사용")


if __name__ == "__main__":
    main()
