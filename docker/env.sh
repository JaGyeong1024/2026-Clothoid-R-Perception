# 컨테이너 셸 환경 — clothoid.sh 로 들어올 때마다 읽는다.
#
# 이미지에 굽지 않고 레포에 두는 이유: 여기를 고쳐도 dev 이미지를 다시 빌드할
# 필요가 없고, 팀원은 git pull 만으로 갱신된다. 레포는 컨테이너 안 고정 경로에
# 마운트되므로 이 파일은 항상 같은 자리에 있다.

source /opt/ros/noetic/setup.bash

# 빌드된 워크스페이스가 있으면 얹는다. 없으면 건너뛰므로 첫 빌드 전에도 안전하다.
# 순서(system_ws → perception_ws)는 README 의 빌드 순서와 같아야 한다 —
# perception_ws 의 devel 이 system_ws 를 체인으로 물고 있다.
_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for _ws in system_ws perception_ws; do
    _setup="${_repo}/${_ws}/devel/setup.bash"
    [ -f "${_setup}" ] && source "${_setup}"
done
unset _repo _ws _setup
