#!/bin/bash
# Clothoid-R Perception 개발 컨테이너.
#
#   ./docker/clothoid.sh          컨테이너 진입 (없으면 만들고, 꺼져 있으면 켜서 붙는다)
#   ./docker/clothoid.sh <명령>   컨테이너 안에서 명령 하나 실행
#   ./docker/clothoid.sh build    dev 이미지를 로컬에서 빌드 (처음 한 번)
#   ./docker/clothoid.sh stop     컨테이너 정지
#   ./docker/clothoid.sh rm       컨테이너 삭제 (이미지는 남는다)
#
# 여러 노드를 각각 다른 터미널에서 띄우는 차량 운영 방식 그대로,
# 터미널마다 이 스크립트를 실행하면 같은 컨테이너에 붙는다.

CONTAINER_NAME=clothoid-r
IMAGE=clothoid-r-perception:dev
BASE_IMAGE=${BASE_IMAGE:-ghcr.io/jagyeong1024/clothoid-r-perception:base}
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_DIR=/home/cnu/clothoid-r-perception

case "$1" in
  build)
    echo "[1/2] base 이미지 받는 중: ${BASE_IMAGE}"
    docker pull "${BASE_IMAGE}" || exit 1
    echo "[2/2] 파이썬 의존성 설치해서 dev 이미지 만드는 중 (10~20분, 처음 한 번만)"
    docker build --build-arg BASE="${BASE_IMAGE}" \
      -f "${REPO_DIR}/docker/Dockerfile.dev" -t "${IMAGE}" "${REPO_DIR}/docker" || exit 1
    echo "[done] 이제 ./docker/clothoid.sh 로 들어가면 된다."
    exit 0
    ;;
  stop) docker stop "${CONTAINER_NAME}"; exit $? ;;
  rm)   docker rm -f "${CONTAINER_NAME}"; exit $? ;;
esac

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[err] ${IMAGE} 이미지가 없다. 먼저 './docker/clothoid.sh build' 를 실행할 것." >&2
    exit 1
fi

# X11 GUI (rviz, cv2.imshow)
xhost +local:docker >/dev/null 2>&1

# 대화형 터미널이 아니면(-t 없음) TTY 할당을 빼야 docker 가 거부하지 않는다.
TTY_FLAGS="-i"
[ -t 0 ] && TTY_FLAGS="-it"

# 컨테이너가 없으면 만든다. 메인 프로세스를 sleep 으로 두는 이유:
# 메인이 bash 면 셸을 나갈 때 컨테이너가 멈추고, 다시 start 해도 터미널이
# 붙어 있지 않아 bash 가 곧바로 죽는다(→ 이후 exec 가 실패). 메인을 계속
# 살려두고 진입은 항상 exec 로 하면 몇 번을 드나들어도 같은 컨테이너가 유지된다.
if [ ! "$(docker ps -aq -f name=^/${CONTAINER_NAME}$)" ]; then
    GPU_ARGS="--gpus all"
    if ! docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
        echo "[warn] nvidia container runtime 이 없다. GPU 없이 뜨며 YOLO 노드는 못 돈다." >&2
        echo "[warn] 설치: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html" >&2
        GPU_ARGS=""
    fi

    # X11 GUI (rviz, cv2.imshow)
    xhost +local:docker >/dev/null 2>&1

    docker run -d ${GPU_ARGS} --privileged \
      -e DISPLAY="$DISPLAY" \
      -e QT_X11_NO_MITSHM=1 \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /dev:/dev:rw \
      -v "${REPO_DIR}:${CONTAINER_DIR}" \
      --hostname "$(hostname)" \
      --network=host \
      --ipc=host \
      --workdir "${CONTAINER_DIR}" \
      --name "${CONTAINER_NAME}" \
      "${IMAGE}" \
      sleep infinity >/dev/null || exit 1
    echo "컨테이너를 새로 만들었습니다: ${CONTAINER_NAME}"
elif [ ! "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "컨테이너가 중지 상태입니다. 시작합니다."
    xhost +local:docker >/dev/null 2>&1
    docker start "${CONTAINER_NAME}" >/dev/null || exit 1
fi

exec docker exec ${TTY_FLAGS} "${CONTAINER_NAME}" bash ${1:+-lc "$*"}
