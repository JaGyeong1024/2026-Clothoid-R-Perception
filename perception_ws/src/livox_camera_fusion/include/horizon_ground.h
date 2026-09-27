// horizon_ground.h — Livox Horizon 지면 제거 (Patchwork++ 개념을 Horizon 시야에 맞춤)
#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace horizon_ground
{
/* 시야(±41°) 안 극좌표 칸마다 최저점 씨앗 + PCA 평면, 직립도·높이·평탄도 검사, 실패 칸은 이웃 칸으로 대체.
 * 전역 사전 평면은 1차 통과 칸들의 씨앗으로 다시 맞추고(2단), 평면은 인라이어 하위 25%에 맞춘다.
 * 반환: 점마다 비지면(1) / 지면(0). */
std::vector<char> nonGround(const std::vector<cv::Point3d> &pts);
}  // namespace horizon_ground
