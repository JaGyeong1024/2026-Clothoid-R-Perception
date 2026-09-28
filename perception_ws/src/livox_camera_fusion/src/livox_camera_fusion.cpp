// livox_camera_fusion.cpp
#include "livox_camera_fusion.h"
#include <horizon_ground/horizon_ground.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>

namespace
{
// JPEG SOF 마커에서 크기만 읽는다 (디코딩 없이). JPEG 가 아니거나 못 찾으면 false.
bool jpeg_size(const std::vector<uint8_t> &d, int &width, int &height)
{
    if (d.size() < 4 || d[0] != 0xFF || d[1] != 0xD8)
        return false;
    size_t i = 2;
    while (i + 9 < d.size())
    {
        if (d[i] != 0xFF)
            return false;
        const uint8_t m = d[i + 1];
        if (m == 0xFF) { ++i; continue; }                             // 채움 바이트
        if (m == 0x01 || (m >= 0xD0 && m <= 0xD7)) { i += 2; continue; }  // 길이 없는 마커
        if (m >= 0xC0 && m <= 0xCF && m != 0xC4 && m != 0xC8 && m != 0xCC)  // SOFn
        {
            height = (d[i + 5] << 8) | d[i + 6];
            width  = (d[i + 7] << 8) | d[i + 8];
            return width > 0 && height > 0;
        }
        i += 2 + ((d[i + 2] << 8) | d[i + 3]);
    }
    return false;
}

// 거리 tol 이내 점끼리 이은 연결 성분 — PCL EuclideanClusterExtraction 과 같은 결과.
// 한 변이 tol 인 격자에 점을 넣고 이웃 27칸의 점 쌍만 비교하며, 이미 같은 성분이면 거리 계산을 건너뛴다.
// 각 성분의 점 번호는 오름차순 (PCL 과 같은 합산 순서).
std::vector<std::vector<int>> euclidean_clusters(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                 double tol, int min_size)
{
    const auto &P = cloud.points;
    const int n = static_cast<int>(P.size());
    const float tol2 = static_cast<float>(tol * tol);   // PCL(FLANN)처럼 float 제곱 거리로 비교
    auto key = [](int x, int y, int z) {
        return (static_cast<int64_t>(x + (1 << 20)) << 42) |
               (static_cast<int64_t>(y + (1 << 20)) << 21) | static_cast<int64_t>(z + (1 << 20));
    };
    std::vector<std::array<int, 3>> cell(n);
    std::unordered_map<int64_t, std::vector<int>> grid;
    for (int i = 0; i < n; ++i)
    {
        cell[i] = {static_cast<int>(std::floor(P[i].x / tol)),
                   static_cast<int>(std::floor(P[i].y / tol)),
                   static_cast<int>(std::floor(P[i].z / tol))};
        grid[key(cell[i][0], cell[i][1], cell[i][2])].push_back(i);
    }

    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    auto root = [&parent](int i) {
        while (parent[i] != i)
            i = parent[i] = parent[parent[i]];
        return i;
    };
    for (const auto &kv : grid)
    {
        const std::vector<int> &A = kv.second;
        const auto &c = cell[A[0]];
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                {
                    const auto it = grid.find(key(c[0] + dx, c[1] + dy, c[2] + dz));
                    if (it == grid.end())
                        continue;
                    const std::vector<int> &B = it->second;
                    if (B[0] < A[0])
                        continue;   // 칸 쌍마다 한 번
                    const bool same = (&A == &B);
                    for (size_t a = 0; a < A.size(); ++a)
                        for (size_t b = same ? a + 1 : 0; b < B.size(); ++b)
                        {
                            const int ri = root(A[a]), rj = root(B[b]);
                            if (ri == rj)
                                continue;
                            const auto &p = P[A[a]], &q = P[B[b]];
                            const float ex = p.x - q.x, ey = p.y - q.y, ez = p.z - q.z;
                            if (ex * ex + ey * ey + ez * ez <= tol2)
                                parent[std::max(ri, rj)] = std::min(ri, rj);
                        }
                }
    }

    std::vector<int> id(n, -1);
    std::vector<std::vector<int>> clusters;
    for (int i = 0; i < n; ++i)
    {
        const int r = root(i);
        if (id[r] < 0)
        {
            id[r] = static_cast<int>(clusters.size());
            clusters.emplace_back();
        }
        clusters[id[r]].push_back(i);
    }
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                  [min_size](const std::vector<int> &c) { return static_cast<int>(c.size()) < min_size; }),
                   clusters.end());
    return clusters;
}
}  // namespace

/* ===== LivoxCameraFusion ctor / dtor ===== */
LivoxCameraFusion::LivoxCameraFusion(ros::NodeHandle *nodeHandle)
    : nh(*nodeHandle)
{
    nh.param("lidar_topic",          lidar_topic,          std::string("/livox/lidar"));
    nh.param("camera_topic",         camera_topic,         std::string("/camera/image_raw/compressed"));
    nh.param("yolo_topic",           yolo_topic,           std::string("/perception/camera/yolo"));
    nh.param("centroid_topic",       centroid_topic,       std::string("/perception/fusion/centroids"));
    nh.param("filtered_cloud_topic", filtered_cloud_topic, std::string("/perception/fusion/filtered_cloud"));
    nh.param("preprocessed_topic",   preprocessed_topic,   std::string("/perception/fusion/preprocessed_points"));
    nh.param("frame_name",           frame_name,           std::string("livox_frame"));

    centroid_pub       = nh.advertise<sensor_msgs::PointCloud>(centroid_topic, 1);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic, 1);
    preprocessed_pub   = nh.advertise<sensor_msgs::PointCloud2>(preprocessed_topic, 1);

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
    // 이미지는 bbox 를 화면 안으로 자르는 데 크기만 쓴다 → JPEG 헤더만 읽고, 아니면 디코딩
    if (!jpeg_size(cam_msg->data, image_width, image_height))
    {
        const cv::Mat img = cv::imdecode(cam_msg->data, cv::IMREAD_UNCHANGED);
        if (img.empty())
        {
            ROS_WARN_THROTTLE(5.0, "[livox_camera_fusion] camera image decode failed (format=%s)", cam_msg->format.c_str());
            return;
        }
        image_width  = img.cols;
        image_height = img.rows;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*lidar_msg, *pc);

    lidar_points.clear();
    lidar_points.reserve(pc->points.size());
    for (const auto &p : pc->points)
    {
        lidar_points.emplace_back(p.x, p.y, p.z);  // raw, projection 용
    }
    if (lidar_points.empty())
        return;

    // projection 은 raw 좌표 (extrinsic 이 기울임 흡수)
    cv::perspectiveTransform(lidar_points, projected_list, projection_matrix);

    // leveled 좌표로 회전 + ROI (projected_list 와 인덱스 동기 유지)
    const double th = LIDAR_PITCH_DEG * M_PI / 180.0;
    const double c = std::cos(th), s = std::sin(th);
    std::vector<cv::Point3d> kept_points;
    std::vector<cv::Point2d> kept_proj;
    kept_points.reserve(lidar_points.size());
    kept_proj.reserve(projected_list.size());
    for (size_t i = 0; i < lidar_points.size(); ++i)
    {
        const auto &q = lidar_points[i];
        const double x_lvl =  c * q.x + s * q.z;
        const double z_lvl = -s * q.x + c * q.z;
        if (x_lvl < LIDAR_ROI_X_MIN || x_lvl > LIDAR_ROI_X_MAX) continue;
        if (q.y   < LIDAR_ROI_Y_MIN || q.y   > LIDAR_ROI_Y_MAX) continue;
        if (z_lvl < LIDAR_ROI_Z_MIN || z_lvl > LIDAR_ROI_Z_MAX) continue;
        kept_points.emplace_back(x_lvl, q.y, z_lvl);
        kept_proj.push_back(projected_list[i]);
    }
    lidar_points  = std::move(kept_points);
    projected_list = std::move(kept_proj);
    if (lidar_points.empty())
        return;

    // 지면 제거 (horizon_ground)
    if (ENABLE_GROUND_REMOVAL)
        remove_ground_full(lidar_points, projected_list);

    publish_preprocessed(lidar_msg->header);
    if (lidar_points.empty())
        return;

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
        if (local->points.size() < static_cast<size_t>(CLUSTER_MIN_SIZE))
            continue;
        local->width  = local->points.size();
        local->height = 1;
        if (local->points.size() > static_cast<size_t>(BBOX_VOXEL_MIN_POINTS))
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr thin(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(local);
            vg.setLeafSize(BBOX_VOXEL_LEAF, BBOX_VOXEL_LEAF, BBOX_VOXEL_LEAF);
            vg.filter(*thin);
            local = thin;
        }

        cv::Point2d centroid;
        cv::Vec3d ext;
        if (!select_cluster_centroid(local, centroid, ext))
            continue;
        cur_centroids.push_back(centroid);

        *out_cloud += *local;
    }

    publish_2D_pointcloud(cur_centroids, header);

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
    box.x2 = std::min<double>(image_width - 1, cx + hw);
    box.y2 = std::min<double>(image_height - 1, cy + hh);

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

// 전체 클라우드 지면 제거 (horizon_ground). pts/proj 는 같은 mask 로 필터링.
void LivoxCameraFusion::remove_ground_full(std::vector<cv::Point3d> &pts,
                                           std::vector<cv::Point2d> &proj)
{
    const std::vector<char> keep = horizon_ground::nonGround(pts);
    std::vector<cv::Point3d> out_pts;
    std::vector<cv::Point2d> out_proj;
    out_pts.reserve(pts.size());
    out_proj.reserve(proj.size());
    for (size_t i = 0; i < pts.size(); ++i)
        if (keep[i])
        {
            out_pts.push_back(pts[i]);
            out_proj.push_back(proj[i]);
        }
    pts = std::move(out_pts);
    proj = std::move(out_proj);
}

// ROI + 지면 제거를 거친 점군 (클러스터링 입력) — rviz 확인용
void LivoxCameraFusion::publish_preprocessed(const std_msgs::Header &header)
{
    if (preprocessed_pub.getNumSubscribers() == 0)
        return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.reserve(lidar_points.size());
    for (const auto &p : lidar_points)
        cloud.points.emplace_back(p.x, p.y, p.z);
    cloud.width  = cloud.points.size();
    cloud.height = 1;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header = header;
    msg.header.frame_id = frame_name;
    preprocessed_pub.publish(msg);
}

// bbox 안 점을 클러스터링 → AABB 게이트 통과분 중 최근접(x 평균 최소) 채택.
// 점 수 최대를 고르면 콘 뒤 벽·수풀이 잡히고, 단순 최근접은 앞의 얇은 조각이 잡힌다.
bool LivoxCameraFusion::select_cluster_centroid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    cv::Point2d &centroid,
    cv::Vec3d &extent) const
{
    const std::vector<std::vector<int>> clusters = euclidean_clusters(*cloud, CLUSTER_TOLERANCE, CLUSTER_MIN_SIZE);

    bool found = false;
    double best_x = std::numeric_limits<double>::infinity();
    for (const auto &cl : clusters)
    {
        double sx = 0, sy = 0;
        double xmin =  std::numeric_limits<double>::infinity();
        double ymin =  std::numeric_limits<double>::infinity();
        double zmin =  std::numeric_limits<double>::infinity();
        double xmax = -std::numeric_limits<double>::infinity();
        double ymax = -std::numeric_limits<double>::infinity();
        double zmax = -std::numeric_limits<double>::infinity();
        for (int idx : cl)
        {
            const auto &p = cloud->points[idx];
            sx += p.x; sy += p.y;
            xmin = std::min(xmin, (double)p.x); xmax = std::max(xmax, (double)p.x);
            ymin = std::min(ymin, (double)p.y); ymax = std::max(ymax, (double)p.y);
            zmin = std::min(zmin, (double)p.z); zmax = std::max(zmax, (double)p.z);
        }
        // 3D AABB gate (length=x, width=y, height=z)
        const double length = xmax - xmin, width = ymax - ymin, height = zmax - zmin;
        if (length < CLUSTER_MIN_LENGTH || length > CLUSTER_MAX_LENGTH) continue;
        if (width  < CLUSTER_MIN_WIDTH  || width  > CLUSTER_MAX_WIDTH)  continue;
        if (height < CLUSTER_MIN_HEIGHT || height > CLUSTER_MAX_HEIGHT) continue;

        const double mx = sx / cl.size();
        if (mx >= best_x) continue;
        best_x   = mx;
        centroid = cv::Point2d(mx, sy / cl.size());
        extent   = cv::Vec3d(length, width, height);
        found    = true;
    }
    return found;
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
