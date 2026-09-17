// livox_camera_fusion.h
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <detect_msgs/Objects.h>
#include <detect_msgs/Yolo_Objects.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <memory>
#include <vector>
#include <map>

/* ===== Parameters ===== */
static constexpr double BBOX_SCALE_RATIO = 1.0;
static constexpr double CLUSTER_TOLERANCE = 0.4;
static constexpr int CLUSTER_MIN_SIZE = 3;
static constexpr int CLUSTER_MAX_SIZE = 100;
static constexpr double ROI_RADIUS_PX = 10.0;
static constexpr double MATCH_DIST = 0.7;  // 20km/h, 10Hz 기준 프레임당 ego 이동 ~0.56m + 여유
static constexpr int TRACKER_MAX_MISS = 10;
static constexpr int MIN_BBOX_EDGE_PX = 0;

/* ===== Livox 마운트 pitch (bag RANSAC 측정) ===== */
/* projection 은 raw 좌표(extrinsic 이 기울임 흡수), ROI·cluster bbox 는 pitch 보정한 leveled 좌표 기준 */
static constexpr double LIDAR_PITCH_DEG = 0.73;

/* ===== LiDAR 3D ROI (leveled 좌표) ===== */
static constexpr double LIDAR_ROI_X_MIN =  0.0;
static constexpr double LIDAR_ROI_X_MAX = 15.0;
static constexpr double LIDAR_ROI_Y_MIN = -7.0;
static constexpr double LIDAR_ROI_Y_MAX =  7.0;
static constexpr double LIDAR_ROI_Z_MIN = -2.0;
static constexpr double LIDAR_ROI_Z_MAX =  2.0;

/* ===== 지면 제거 (bbox 매칭 전 전체 클라우드에 grid + RANSAC) ===== */
/* per-bbox RANSAC 은 점이 적어 물체 하부를 지면으로 오인하므로 전체 클라우드 방식 사용 */
static constexpr bool   ENABLE_GROUND_REMOVAL = true;
static constexpr double GRID_CELL_SIZE        = 0.2;
static constexpr double GRID_MAX_HEIGHT_DIFF  = 0.2;
static constexpr int    GRID_MIN_POINTS       = 10;
static constexpr double GROUND_RANSAC_THRESH  = 0.2;  // livox_clustering(0.3)보다 보수적

/* ===== 3D bbox gate (cluster AABB: x=length, y=width, z=height) ===== */
/* MIN 은 노이즈 컷, MAX 는 사실상 미사용 */
static constexpr double CLUSTER_MIN_LENGTH = 0.1;
static constexpr double CLUSTER_MAX_LENGTH = 10.0;
static constexpr double CLUSTER_MIN_WIDTH  = 0.1;
static constexpr double CLUSTER_MAX_WIDTH  = 10.0;
static constexpr double CLUSTER_MIN_HEIGHT = 0.1;
static constexpr double CLUSTER_MAX_HEIGHT = 10.0;

/* ===== Kalman Tracker ===== */
struct KalmanTracker
{
    int id{-1};
    int miss_count{0};
    cv::KalmanFilter kf;
    cv::Point2f last_pos;

    KalmanTracker() = default;
    KalmanTracker(const cv::Point2f &pt, int tracker_id, float dt = 0.1f);
    cv::Point2f predict();
    void update(const cv::Point2f &pt);
    void miss();
};

/* ===== LivoxCameraFusion ===== */
class LivoxCameraFusion
{
private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,
        sensor_msgs::CompressedImage,
        detect_msgs::Yolo_Objects>;

    struct ImageBox
    {
        double x1{0};
        double y1{0};
        double x2{0};
        double y2{0};
        cv::Point2d center;
    };

    ros::NodeHandle nh;
    ros::Publisher centroid_pub;       // /perception/fusion/centroids
    ros::Publisher filtered_cloud_pub; // /perception/fusion/filtered_cloud

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>     sub_lidar;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::CompressedImage>> sub_camera;
    std::shared_ptr<message_filters::Subscriber<detect_msgs::Yolo_Objects>>    sub_yolo;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>>                 sync;

    std::string lidar_topic, camera_topic, yolo_topic;
    std::string centroid_topic, filtered_cloud_topic, frame_name;

    cv::Mat projection_matrix;
    cv::Mat camera_image;
    std::vector<cv::Point3d> lidar_points;
    std::vector<cv::Point2d> projected_list;
    std::vector<cv::Point2d> prev_centroids;

    void read_projection_matrix();
    void detectionCallback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                           const sensor_msgs::CompressedImage::ConstPtr &camera_msg,
                           const detect_msgs::Yolo_Objects::ConstPtr &yolo_msg);
    void convert_msg(const detect_msgs::Yolo_Objects::ConstPtr &yolo_msg,
                     const std_msgs::Header &header);
    bool build_scaled_bbox(const detect_msgs::Objects &obj, ImageBox &box) const;
    void collect_points_in_bbox(const ImageBox &box,
                                std::vector<cv::Point2d> &matched_px,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr local) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr extract_roi(
        const std::vector<cv::Point2d> &matched_px,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &local,
        const cv::Point2d &center) const;
    void remove_ground_full(std::vector<cv::Point3d> &pts,
                            std::vector<cv::Point2d> &proj);
    bool largest_cluster_centroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                  cv::Point2d &centroid,
                                  cv::Vec3d &extent) const;
    void draw_bbox_debug(const ImageBox &box);
    void publish_2D_pointcloud(const std::vector<cv::Point2d> &pts,
                               const std_msgs::Header &header);
    void track_and_visualize(const std::vector<cv::Point2d> &cents);
    void match_and_update_trackers(const std::vector<cv::Point2f> &cents,
                                   double match_dist = MATCH_DIST,
                                   int max_miss = TRACKER_MAX_MISS);
    std::vector<int> remove_ground_ransac(const std::vector<cv::Point3f> &pts,
                                          double threshold = GROUND_RANSAC_THRESH);

public:
    explicit LivoxCameraFusion(ros::NodeHandle *nh);
    ~LivoxCameraFusion();
};
