# CNU 서버 환경

- **접속**: 접속 주소·계정은 팀 내부 문서 참조
- **조사일**: 2026-08-21

## 하드웨어

| 항목 | 사양 |
|------|------|
| 모델 | Neousys Nuvo-8108GC Series |
| CPU | Intel Xeon E-2278G @ 3.40GHz (8코어 / 16스레드, 최대 5.0GHz) |
| RAM | 62 GiB + Swap 2 GiB |
| GPU | NVIDIA GeForce RTX 3070 8GB |
| 저장장치 | NVMe 938GB (ext4) + 512MB EFI |

## 소프트웨어 스택

| 계층 | 버전 |
|------|------|
| OS | Ubuntu 20.04.6 LTS (focal) |
| 커널 | Linux 5.15.0-139-generic (x86-64) |
| NVIDIA 드라이버 | 535.183.01 |
| CUDA | 12.2 (드라이버 지원) / 설치본 `/usr/local/cuda-11.1` |
| Python | 3.8.10 (시스템), Anaconda3 (`~/anaconda3`) |
| ROS | Noetic (네이티브) |
| Docker | 28.1.1 |

### 주요 Python / ML 패키지

| 패키지 | 버전 |
|--------|------|
| PyTorch | 2.4.1 |
| torchvision | 0.19.1 |
| torchaudio | 2.4.1 |
| OpenCV | 4.8.0.76 |
| NumPy | 1.24.4 |
| scikit-learn | 1.3.2 |
| scikit-image | 0.16.2 |

### Docker 이미지 (컨테이너)

| 이미지 | 용도 |
|--------|------|
| `osrf/ros:humble-desktop-full` | ROS 2 Humble (realsense, Humble) |
| `osrf/ros:noetic-desktop-full` | ROS 1 Noetic (Clothoid-R) |
| `ctrack/sgr:noetic-sgr-v0.0.2` | ROS 1 SGR |
