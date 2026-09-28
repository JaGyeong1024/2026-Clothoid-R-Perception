# horizon_ground

Livox Horizon 지면 제거. `livox_camera_fusion`(C++ 링크)과 `livox_clustering`(파이썬, ctypes)이 같은 구현을 쓴다.

## 알고리즘

```
시야 ±41° 안 극좌표 34칸 (거리 링 2.9/4.5/6.5/9.0/12.0/15.6 m × 부채꼴 4/6/8/8/8)
  → 칸마다 최저점 씨앗 + PCA 평면 (3회 반복), 직립도(≤ 8°)·사전 평면과 높이 차·평탄도 검사
  → 실패 칸은 같은 링 이웃 → 앞뒤 링 겹치는 칸 평균, 없으면 사전 평면
  → 전역 사전 평면은 1차 통과 칸들의 씨앗으로 다시 맞춘다 (2단)
  → 칸 평면 위 0.2 m 초과를 비지면으로 판정
```

## 사용

- C++: `#include <horizon_ground/horizon_ground.h>` → `horizon_ground::nonGround(pts, &ground)`. `package.xml`·`find_package(catkin COMPONENTS horizon_ground)`에 의존 추가.
- 파이썬: `from horizon_ground import non_ground` → `mask, (a, b, c) = non_ground(xyz)`. `libhorizon_ground.so`를 `LD_LIBRARY_PATH`(워크스페이스 `devel/setup.bash`)에서 찾는다.
