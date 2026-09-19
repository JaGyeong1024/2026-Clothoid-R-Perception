#!/usr/bin/env bash
# 인지 파이프라인 rosbag 녹화 + 자원/처리율 계측을 한 번에 시작하고,
# Ctrl+C 한 번으로 둘 다 정상 종료·저장한다.
#
#   ./tools/record_and_measure.sh              # 인지 토픽만 녹화
#   ./tools/record_and_measure.sh --all        # 전체 토픽(-a)
#   ./tools/record_and_measure.sh --no-bag     # 계측만
#   ./tools/record_and_measure.sh --out DIR --period 0.5
#
# 결과는 <레포>/runs/<날짜_시각>/ 에 남는다 (*.bag 은 .gitignore 로 추적 제외).
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

# 녹화할 인지 관련 토픽. 필요하면 여기만 고친다.
TOPICS=(
  /rosout
  /rosout_agg
  /camera/image_raw/compressed
  /livox/lidar
  /velodyne_points
  /perception/camera/yolo
  /perception/livox/centroids
  /perception/velodyne/centroids
  /perception/fusion/centroids
  /tf
  /tf_static
)

RECORD_ALL=0
DO_BAG=1
DO_MON=1
PERIOD=1
OUT=""

while [ $# -gt 0 ]; do
  case "$1" in
    --all)        RECORD_ALL=1 ;;
    --no-bag)     DO_BAG=0 ;;
    --no-monitor) DO_MON=0 ;;
    --out)        OUT="$2"; shift ;;
    --period)     PERIOD="$2"; shift ;;
    -h|--help)    sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "알 수 없는 옵션: $1" >&2; exit 2 ;;
  esac
  shift
done

if [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1091
  . /opt/ros/noetic/setup.bash
fi

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "roscore 에 연결할 수 없습니다. 먼저 스택을 올리세요." >&2
  exit 1
fi

[ -n "$OUT" ] || OUT="$REPO/runs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT" || exit 1

MON_PY="$HERE/cr_monitor.py"
if [ "$DO_MON" = 1 ] && [ ! -f "$MON_PY" ]; then
  echo "계측 스크립트를 찾을 수 없습니다: $MON_PY" >&2
  exit 1
fi

echo "출력 디렉토리: $OUT"

BAG_PID=""
MON_PID=""

if [ "$DO_BAG" = 1 ]; then
  if [ "$RECORD_ALL" = 1 ]; then
    echo "녹화: 전체 토픽 (-a)"
    setsid bash -c 'echo $$ > "$1/bag.pid"; cd "$1" && exec rosbag record -a -O record.bag' \
      _ "$OUT" > "$OUT/bag.log" 2>&1 &
  else
    # 지금 존재하지 않는 토픽은 빼고 넘긴다 (없는 토픽을 주면 경고만 쌓인다)
    LIVE=$(timeout 5 rostopic list 2>/dev/null)
    SEL=()
    for t in "${TOPICS[@]}"; do
      if printf '%s\n' "$LIVE" | grep -qx -- "$t"; then SEL+=("$t"); else echo "  (없음, 제외) $t"; fi
    done
    if [ ${#SEL[@]} -eq 0 ]; then
      echo "녹화할 토픽이 하나도 없습니다." >&2; exit 1
    fi
    echo "녹화 토픽 ${#SEL[@]}개: ${SEL[*]}"
    setsid bash -c 'D="$1"; shift; echo $$ > "$D/bag.pid"; cd "$D" && exec rosbag record -O record.bag "$@"' \
      _ "$OUT" "${SEL[@]}" > "$OUT/bag.log" 2>&1 &
  fi
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    [ -s "$OUT/bag.pid" ] && break; sleep 0.3
  done
  BAG_PID=$(cat "$OUT/bag.pid" 2>/dev/null)
fi

if [ "$DO_MON" = 1 ]; then
  # 0 = Ctrl+C 까지 계속
  setsid bash -c 'echo $$ > "$1/mon.pid"; exec python3 "$2" "$1/mon" 0 "$3"' \
    _ "$OUT" "$MON_PY" "$PERIOD" > "$OUT/monitor.log" 2>&1 &
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    [ -s "$OUT/mon.pid" ] && break; sleep 0.3
  done
  MON_PID=$(cat "$OUT/mon.pid" 2>/dev/null)
  echo "계측 시작 (주기 ${PERIOD}s)"
fi

echo
echo "기록 중...  종료하려면 Ctrl+C (녹화·계측 모두 저장하고 끝냅니다)"
START=$(date +%s)

SHUTTING_DOWN=0

finish() {
  SHUTTING_DOWN=1
  trap '' INT TERM
  echo
  echo "종료 중... (bag 마무리에 몇 초 걸릴 수 있습니다)"
  # rosbag 은 SIGINT 를 받아야 .bag.active 를 정상 .bag 으로 닫는다.
  # SIGINT 는 각 자식에게 정확히 한 번만. 두 번 보내면 요약을 쓰다 끊긴다.
  [ -n "$BAG_PID" ] && kill -INT "$BAG_PID" 2>/dev/null
  [ -n "$MON_PID" ] && kill -INT "$MON_PID" 2>/dev/null
  local waited=0
  while [ "$waited" -lt 120 ]; do
    local alive=0
    [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null && alive=1
    [ -n "$MON_PID" ] && kill -0 "$MON_PID" 2>/dev/null && alive=1
    [ "$alive" = 0 ] && break
    # 20초가 지나도 안 죽으면 TERM, 40초면 KILL 로 단계적 강제 종료
    if [ "$waited" = 40 ]; then
      echo "  응답이 없어 SIGTERM 을 보냅니다"
      [ -n "$BAG_PID" ] && kill -TERM "$BAG_PID" 2>/dev/null
      [ -n "$MON_PID" ] && kill -TERM "$MON_PID" 2>/dev/null
    fi
    if [ "$waited" = 80 ]; then
      echo "  강제 종료합니다 (bag 이 손상될 수 있습니다)"
      [ -n "$BAG_PID" ] && kill -9 "$BAG_PID" 2>/dev/null
      [ -n "$MON_PID" ] && kill -9 "$MON_PID" 2>/dev/null
    fi
    waited=$((waited + 1))
    sleep 0.5
  done
  rm -f "$OUT/bag.pid" "$OUT/mon.pid"

  # 혹시 .active 가 남으면 알려준다 (rosbag reindex 필요)
  if ls "$OUT"/*.bag.active >/dev/null 2>&1; then
    echo "경고: $(basename "$OUT"/*.bag.active) 가 정상 종료되지 않았습니다."
    echo "      'rosbag reindex' 로 복구하세요."
  fi

  local elapsed=$(( $(date +%s) - START ))
  echo
  echo "=== 기록 완료 (${elapsed}초) ==="
  if [ -f "$OUT/record.bag" ]; then
    local sz
    sz=$(stat -c %s "$OUT/record.bag")
    printf "bag : %s (%.1f MB, %.2f MB/s)\n" "$OUT/record.bag" \
      "$(echo "$sz" | awk '{print $1/1e6}')" \
      "$(echo "$sz $elapsed" | awk '{print ($2>0)? $1/1e6/$2 : 0}')"
  fi
  if [ -f "$OUT/mon/summary.txt" ]; then
    echo
    cat "$OUT/mon/summary.txt"
    echo "CSV : $OUT/mon/{proc,system,topics,irq}.csv"
  fi
  exit 0
}
trap finish INT TERM HUP

# 자식이 먼저 죽으면(예: 디스크 가득) 같이 정리한다
while true; do
  [ "$SHUTTING_DOWN" = 1 ] && break
  if [ -n "$BAG_PID" ] && ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo "rosbag 이 예기치 않게 종료되었습니다. bag.log 를 확인하세요."; finish
  fi
  if [ -n "$MON_PID" ] && ! kill -0 "$MON_PID" 2>/dev/null; then
    echo "계측이 예기치 않게 종료되었습니다. monitor.log 를 확인하세요."; finish
  fi
  sleep 1
done
