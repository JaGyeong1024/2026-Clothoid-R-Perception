// livox_camera_fusion.cpp
#include "livox_camera_fusion.h"

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <set>

/* ===== KalmanTracker Implementation ===== */
KalmanTracker::KalmanTracker(const cv::Point2f &pt, int tracker_id, float dt)
    : id(tracker_id)
{
    kf.init(4, 2, 0);
    kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, dt, 0,
                           0, 1, 0, dt,
                           0, 0, 1, 0,
                           0, 0, 0, 1);
    kf.measurementMatrix = cv::Mat::eye(2, 4, CV_32F);
    setIdentity(kf.processNoiseCov, cv::Scalar::all(5e-2));
    setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, cv::Scalar::all(1));
    kf.statePost = (cv::Mat_<float>(4, 1) << pt.x, pt.y, 0, 0);
    last_pos = pt;
}

cv::Point2f KalmanTracker::predict()
{
    cv::Mat pr = kf.predict();
    last_pos = {pr.at<float>(0), pr.at<float>(1)};
    return last_pos;
}

void KalmanTracker::update(const cv::Point2f &pt)
{
    cv::Mat m(2, 1, CV_32F);
    m.at<float>(0) = pt.x;
    m.at<float>(1) = pt.y;
    kf.correct(m);
    last_pos = pt;
    miss_count = 0;
}

void KalmanTracker::miss()
{
    predict();
    ++miss_count;
}

/* ===== global trackers ===== */
static std::map<int, KalmanTracker> trackers;
static int next_tracker_id = 0;

/* ===== LivoxCameraFusion ctor / dtor ===== */
LivoxCameraFusion::LivoxCameraFusion(ros::NodeHandle *nodeHandle)
    : nh(*nodeHandle)
{
    nh.param("lidar_topic",          lidar_topic,          std::string("/livox/lidar"));
    nh.param("camera_topic",         camera_topic,         std::string("/camera/image_raw/compressed"));
    nh.param("yolo_topic",           yolo_topic,           std::string("/perception/camera/yolo"));
    nh.param("centroid_topic",       centroid_topic,       std::string("/perception/fusion/centroids"));
    nh.param("filtered_cloud_topic", filtered_cloud_topic, std::string("/perception/fusion/filtered_cloud"));
    nh.param("frame_name",           frame_name,           std::string("livox_frame"));

    centroid_pub       = nh.advertise<sensor_msgs::PointCloud>(centroid_topic, 1);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic, 1);

    sub_lidar  = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, lidar_topic, 10);
    sub_camera = std::make_shared<message_filters::Subscriber<sensor_msgs::CompressedImage>>(nh, camera_topic, 10);
    sub_yolo   = std::make_shared<message_filters::Subscriber<detect_msgs::Yolo_Objects>>(nh, yolo_topic, 10);

    sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(20), *sub_lidar, *sub_camera, *sub_yolo);
    sync->setMaxIntervalDuration(ros::Duration(0.05));
    sync->registerCallback(
        boost::bind(&LivoxCameraFusion::detectionCallback, this, _1, _2, _3));

    read_projection_matrix();
    ROS_INFO("[livox_camera_fusion] start");
}

LivoxCameraFusion::~LivoxCameraFusion()
{
    ROS_INFO("[livox_camera_fusion] finish");
}

/* ===== detectionCallback ===== */
void LivoxCameraFusion::detectionCallback(
    const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
    const sensor_msgs::CompressedImage::ConstPtr &cam_msg,
    const detect_msgs::Yolo_Objects::ConstPtr &yolo_msg)
{
    camera_image = cv_bridge::toCvCopy(cam_msg, "bgr8")->image;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*lidar_msg, *pc);

    lidar_points.clear();
    for (const auto &p : pc->points)
    {
        lidar_points.emplace_back(p.x, p.y, p.z);
    }
    if (lidar_points.empty())
        return;

    cv::perspectiveTransform(lidar_points, projected_list, projection_matrix);
    convert_msg(yolo_msg, lidar_msg->header);
}

/* ===== convert_msg ===== */
void LivoxCameraFusion::convert_msg(
    const detect_msgs::Yolo_Objects::ConstPtr &yolo,
    const std_msgs::Header &header)
{
    std::vector<cv::Point2d> cur_centroids;
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    out_cloud->reserve(1024);

    for (const auto &Y : yolo->yolo_objects)
    {
        ImageBox box;
        if (!build_scaled_bbox(Y, box))
            continue;

        std::vector<cv::Point2d> matched_px;
        pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>);
        collect_points_in_bbox(box, matched_px, local);
        if (matched_px.size() < static_cast<size_t>(CLUSTER_MIN_SIZE))
            continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr roi = extract_roi(matched_px, local, box.center);
        if (roi->points.size() < static_cast<size_t>(CLUSTER_MIN_SIZE))
            continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr roi_ng = remove_ground_from_cloud(roi);
        if (roi_ng->empty())
            continue;

        cv::Point2d centroid;
        if (!largest_cluster_centroid(roi_ng, centroid))
            continue;
        cur_centroids.push_back(centroid);

        draw_bbox_debug(box);
        *out_cloud += *roi_ng;
    }

    prev_centroids = cur_centroids;
    track_and_visualize(prev_centroids);
    publish_2D_pointcloud(prev_centroids, header);

    if (!out_cloud->empty())
    {
        sensor_msgs::PointCloud2 msg_pc2;
        pcl::toROSMsg(*out_cloud, msg_pc2);
        msg_pc2.header = header;
        filtered_cloud_pub.publish(msg_pc2);
    }
}

bool LivoxCameraFusion::build_scaled_bbox(const detect_msgs::Objects &obj, ImageBox &box) const
{
    double x1 = obj.x1, y1 = obj.y1, x2 = obj.x2, y2 = obj.y2;
    const double cx = 0.5 * (x1 + x2);
    const double cy = 0.5 * (y1 + y2);
    const double hw = 0.5 * (x2 - x1) * BBOX_SCALE_RATIO;
    const double hh = 0.5 * (y2 - y1) * BBOX_SCALE_RATIO;

    box.x1 = std::max(0.0, cx - hw);
    box.y1 = std::max(0.0, cy - hh);
    box.x2 = std::min<double>(camera_image.cols - 1, cx + hw);
    box.y2 = std::min<double>(camera_image.rows - 1, cy + hh);
    box.center = cv::Point2d(cx, cy);

    return (box.x2 - box.x1) >= MIN_BBOX_EDGE_PX &&
           (box.y2 - box.y1) >= MIN_BBOX_EDGE_PX;
}

void LivoxCameraFusion::collect_points_in_bbox(
    const ImageBox &box,
    std::vector<cv::Point2d> &matched_px,
    pcl::PointCloud<pcl::PointXYZ>::Ptr local) const
{
    for (size_t i = 0; i < projected_list.size(); ++i)
    {
        double u = projected_list[i].x, v = projected_list[i].y;
        if (std::isnan(u) || std::isnan(v))
            continue;
        if (u >= box.x1 && u <= box.x2 && v >= box.y1 && v <= box.y2)
        {
            matched_px.emplace_back(u, v);
            local->points.emplace_back(lidar_points[i].x,
                                       lidar_points[i].y,
                                       lidar_points[i].z);
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LivoxCameraFusion::extract_roi(
    const std::vector<cv::Point2d> &matched_px,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &local,
    const cv::Point2d &center) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < matched_px.size(); ++i)
        if (cv::norm(matched_px[i] - center) <= ROI_RADIUS_PX)
            roi->points.push_back(local->points[i]);
    return roi;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LivoxCameraFusion::remove_ground_from_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<cv::Point3f> tmp;
    tmp.reserve(cloud->points.size());
    for (const auto &p : *cloud)
        tmp.emplace_back(p.x, p.y, p.z);

    const auto keep = remove_ground_ransac(tmp, GROUND_THRESH);
    if (keep.empty())
        return out;

    out->points.reserve(keep.size());
    for (int id : keep)
        out->points.push_back(cloud->points[id]);
    return out;
}

bool LivoxCameraFusion::largest_cluster_centroid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    cv::Point2d &centroid) const
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);
    ec.setMinClusterSize(CLUSTER_MIN_SIZE);
    ec.setMaxClusterSize(CLUSTER_MAX_SIZE);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters);
    if (clusters.empty())
        return false;

    const auto &largest = *std::max_element(
        clusters.begin(), clusters.end(),
        [](const pcl::PointIndices &a, const pcl::PointIndices &b)
        {
            return a.indices.size() < b.indices.size();
        });
    double sx = 0, sy = 0;
    for (int idx : largest.indices)
    {
        sx += cloud->points[idx].x;
        sy += cloud->points[idx].y;
    }
    centroid = cv::Point2d(sx / largest.indices.size(),
                           sy / largest.indices.size());
    return true;
}

void LivoxCameraFusion::draw_bbox_debug(const ImageBox &box)
{
    cv::rectangle(camera_image,
                  {int(box.x1), int(box.y1)},
                  {int(box.x2), int(box.y2)},
                  {0, 255, 0}, 2);
    cv::circle(camera_image, box.center, 4, {0, 0, 255}, 2);
}

/* ===== track matching ===== */
void LivoxCameraFusion::match_and_update_trackers(
    const std::vector<cv::Point2f> &cents,
    double match_dist, int max_miss)
{
    std::set<int> matched_t, matched_o;
    std::vector<cv::Point2f> preds;
    std::vector<int> keys;
    for (auto &kv : trackers)
    {
        preds.push_back(kv.second.predict());
        keys.push_back(kv.first);
    }
    if (!cents.empty() && !preds.empty())
    {
        cv::Mat D((int)cents.size(), (int)preds.size(), CV_32F);
        for (int i = 0; i < (int)cents.size(); ++i)
            for (int j = 0; j < (int)preds.size(); ++j)
                D.at<float>(i, j) = cv::norm(cents[i] - preds[j]);
        while (true)
        {
            double vmin;
            cv::Point loc;
            cv::minMaxLoc(D, &vmin, nullptr, &loc, nullptr);
            if (vmin > match_dist)
                break;
            int oi = loc.y, tj = loc.x;
            trackers[keys[tj]].update(cents[oi]);
            matched_t.insert(keys[tj]);
            matched_o.insert(oi);
            D.row(oi).setTo(1e9);
            D.col(tj).setTo(1e9);
        }
    }
    for (auto &kv : trackers)
        if (!matched_t.count(kv.first))
            kv.second.miss();
    std::vector<int> del;
    for (auto &kv : trackers)
        if (kv.second.miss_count > max_miss)
            del.push_back(kv.first);
    for (int k : del)
        trackers.erase(k);
    for (size_t i = 0; i < cents.size(); ++i)
        if (!matched_o.count(i))
            trackers[next_tracker_id] = KalmanTracker(cents[i], next_tracker_id++);
}

void LivoxCameraFusion::track_and_visualize(const std::vector<cv::Point2d> &cents)
{
    std::vector<cv::Point2f> c2f;
    for (const auto &c : cents)
        c2f.emplace_back((float)c.x, (float)c.y);

    match_and_update_trackers(c2f, MATCH_DIST, TRACKER_MAX_MISS);

    for (const auto &kv : trackers)
    {
        cv::circle(camera_image, kv.second.last_pos, 6, {255, 0, 255}, 2);
        cv::putText(camera_image, std::to_string(kv.first),
                    kv.second.last_pos + cv::Point2f(5, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 0}, 1);
    }
}

/* ===== RANSAC ground removal ===== */
std::vector<int> LivoxCameraFusion::remove_ground_ransac(
    const std::vector<cv::Point3f> &pts, double th)
{
    if (pts.size() < 10)
    {
        std::vector<int> id(pts.size());
        std::iota(id.begin(), id.end(), 0);
        return id;
    }
    int max_in = 0;
    double a = 0, b = 0, c = 0;
    std::default_random_engine gen;
    std::uniform_int_distribution<> d(0, pts.size() - 1);

    for (int iter = 0; iter < 30; ++iter)
    {
        int i1 = d(gen), i2 = d(gen), i3 = d(gen);
        double x1 = pts[i1].x, y1 = pts[i1].y, z1 = pts[i1].z,
               x2 = pts[i2].x, y2 = pts[i2].y, z2 = pts[i2].z,
               x3 = pts[i3].x, y3 = pts[i3].y, z3 = pts[i3].z;
        double den = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
        if (std::abs(den) < 1e-6)
            continue;
        double ta = (z1 * (y2 - y3) + z2 * (y3 - y1) + z3 * (y1 - y2)) / den;
        double tb = (x1 * (z2 - z3) + x2 * (z3 - z1) + x3 * (z1 - z2)) / den;
        double tc = z1 - ta * x1 - tb * y1;

        int in = 0;
        for (const auto &p : pts)
            if (std::abs(p.z - (ta * p.x + tb * p.y + tc)) < th)
                ++in;
        if (in > max_in)
        {
            max_in = in;
            a = ta;
            b = tb;
            c = tc;
        }
    }
    std::vector<int> idx;
    for (size_t i = 0; i < pts.size(); ++i)
        if (std::abs(pts[i].z - (a * pts[i].x + b * pts[i].y + c)) > th)
            idx.push_back((int)i);
    return idx;
}

/* ===== publish 2D PointCloud ===== */
void LivoxCameraFusion::publish_2D_pointcloud(
    const std::vector<cv::Point2d> &pts, const std_msgs::Header &hdr)
{
    sensor_msgs::PointCloud cloud;
    cloud.header = hdr;
    cloud.header.frame_id = frame_name;
    for (const auto &p : pts)
    {
        geometry_msgs::Point32 q;
        q.x = p.x;
        q.y = p.y;
        q.z = 0;
        cloud.points.push_back(q);
    }
    sensor_msgs::ChannelFloat32 ch;
    ch.name = "dummy";
    ch.values.resize(cloud.points.size(), 1.0f);
    cloud.channels.push_back(ch);
    centroid_pub.publish(cloud);
}

/* ===== read_projection_matrix ===== */
void LivoxCameraFusion::read_projection_matrix()
{
    std::vector<double> k_data;
    std::vector<double> t_data;

    cv::Mat K;
    cv::Mat T;
    if (nh.getParam("camera_matrix/data", k_data) &&
        nh.getParam("extrinsic_matrix/data", t_data) &&
        k_data.size() == 9 && t_data.size() == 12)
    {
        K = cv::Mat(3, 3, CV_64F, k_data.data()).clone();
        T = cv::Mat(3, 4, CV_64F, t_data.data()).clone();
    }
    else
    {
        ROS_WARN("[livox_camera_fusion] projection params missing or invalid; using built-in defaults");
        double fx = 1.9740e+03;
        double fy = 1.9718e+03;
        double cx = 0.9636e+03;
        double cy = 0.5849e+03;
        K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        T = (cv::Mat_<double>(3, 4) << 0.0026, -1.0000, -0.0036, 0.0070,
             0.0108, 0.0036, -0.9999, -0.0679,
             0.9999, 0.0026, 0.0108, 0.1865);
    }
    projection_matrix = K * T;
}
