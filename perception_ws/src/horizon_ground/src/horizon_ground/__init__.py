"""Livox Horizon 지면 제거 — C++ 구현(libhorizon_ground.so)을 ctypes 로 부른다. 알고리즘은 src/horizon_ground.cpp 한 곳에만 있다."""
import ctypes

import numpy as np

try:
    _lib = ctypes.CDLL("libhorizon_ground.so")   # catkin 워크스페이스 devel/lib (setup.bash 가 LD_LIBRARY_PATH 에 넣는다)
except OSError as e:
    raise ImportError("libhorizon_ground.so 를 찾지 못했어요. perception_ws 를 catkin_make 하고 "
                      "devel/setup.bash 를 소싱했는지 확인하세요: %s" % e)
_lib.horizon_ground_non_ground.restype = None
_lib.horizon_ground_non_ground.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]


def non_ground(p):
    """p: (N,3) 센서 좌표. 반환: 비지면 mask, 전역 지면 평면 (a, b, c) — z = a x + b y + c"""
    p = np.ascontiguousarray(p, dtype=np.float64)
    mask = np.empty(len(p), dtype=np.uint8)
    plane = np.empty(3)
    _lib.horizon_ground_non_ground(p.ctypes.data, len(p), mask.ctypes.data, plane.ctypes.data)
    return mask.view(bool), tuple(plane)
