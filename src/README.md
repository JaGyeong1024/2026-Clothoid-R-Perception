# 코드 설명

# 개발자

구자경, 김민재, 서준혁, 지연수, 허주영

---

## LiDAR와 카메라 융합 기반 객체 탐지 및 추적

해당 패키지는 yolov8에서 detect한 bbox와 Livox Lidar와 클러스터링을 통해 객체의 (x,y)를 publish 함.

### 파이프 라인

    ROS Topic 동기화(LiDAR+카메라+YOLO)
    ↓
    LiDAR 포인트를 카메라 좌표계로 투영
    ↓
    YOLO bbox와 매칭된 LiDAR 포인트 필터링
    ↓
    ROI 추출 및 RANSAC 기반 Ground 제거
    ↓
    Point Cloud 군집화 및 중심점(Centroid) 계산
    ↓
    Kalman Filter 기반 객체 추적(Tracking)
    ↓
    결과 시각화 및 PointCloud 발행


