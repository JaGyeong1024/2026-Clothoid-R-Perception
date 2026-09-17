#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

// 장치 열기: device_path가 주어지면 V4L2로 경로 열기(재연결 시 인덱스 밀림 대응),
// 아니면 기존 방식(GSTREAMER + 인덱스) 유지. 성공할 때까지 재시도.
static bool open_camera(cv::VideoCapture &cap, const std::string &device_path, int device_index)
{
    while (ros::ok())
    {
        if (!device_path.empty())
            cap.open(device_path, cv::CAP_V4L2);
        else
            cap.open(device_index, cv::CAP_GSTREAMER);

        if (cap.isOpened())
        {
            ROS_INFO("Camera opened (%s).",
                     device_path.empty() ? "index" : device_path.c_str());
            return true;
        }
        ROS_WARN_THROTTLE(5, "Failed to open webcam. Retrying...");
        ros::Duration(1.0).sleep();
    }
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "webcam_publisher");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    sensor_msgs::ImagePtr image_msg;
    sensor_msgs::CompressedImagePtr compressed_image_msg;
    cv::Mat frame;
    ros::Rate loop_rate(60); // Set the desired publishing rate (e.g., 30 Hz)
    double fx, fy, cx, cy, k1, k2, p1, p2, k3;
    cv::Mat camera_matrix, dist_coeffs;
    k3 = 0.0;

    nh.param<double>("fx", fx, 0);
    nh.param<double>("fy", fy, 0);
    nh.param<double>("cx", cx, 0);
    nh.param<double>("cy", cy, 0);
    nh.param<double>("k1", k1, 0);
    nh.param<double>("k2", k2, 0);
    nh.param<double>("p1", p1, 0);
    nh.param<double>("p2", p2, 0);

    std::string device_path;
    int device_index;
    // 연속 empty frame이 이 횟수를 넘으면 장치를 닫고 다시 연다
    int max_consecutive_failures;
    nh.param<std::string>("device_path", device_path, "");
    nh.param<int>("device_index", device_index, 0);
    nh.param<int>("max_consecutive_failures", max_consecutive_failures, 30);

    camera_matrix = (cv::Mat_<double>(3,3) <<
        fx, 0,  cx,
        0,  fy, cy,
        0,  0,  1);

    dist_coeffs = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);

    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 1);
    ros::Publisher compressed_image_pub = nh.advertise<sensor_msgs::CompressedImage>("/camera/image_raw/compressed", 1);
    //ros::Publisher camera_info_pub= nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);
    cv::VideoCapture cap;

    if (!open_camera(cap, device_path, device_index))
        return -1;

    // Intrinsic Calibration: 맵은 해상도가 정해지면 한 번만 계산
    cv::Mat undistorted_image, map1, map2;
    cv::Size map_size;
    int consecutive_failures = 0;

    while (nh.ok())
    {
        cap >> frame;

        if (frame.empty())
        {
            ++consecutive_failures;
            ROS_WARN_THROTTLE(1, "Failed to capture frame (%d consecutive).", consecutive_failures);
            if (consecutive_failures >= max_consecutive_failures)
            {
                ROS_ERROR("Too many empty frames. Reopening camera...");
                cap.release();
                if (!open_camera(cap, device_path, device_index))
                    return -1;
                consecutive_failures = 0;
            }
            loop_rate.sleep();
            continue;
        }
        consecutive_failures = 0;

        if (frame.size() != map_size)
        {
            cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), camera_matrix, frame.size(), CV_16SC2, map1, map2);
            map_size = frame.size();
        }
        cv::remap(frame, undistorted_image, map1, map2, cv::INTER_LINEAR);

        // Convert the OpenCV frame to an image message
        image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted_image).toImageMsg(); //undistorted_image
        image_msg->header.stamp = ros::Time::now();
        image_msg->header.frame_id = "";
        image_pub.publish(image_msg);

        // Convert the OpenCV frame to a compressed image message
        std::vector<uint8_t> img_data;
        cv::imencode(".jpg", undistorted_image, img_data); //undistorted_image

        compressed_image_msg = boost::make_shared<sensor_msgs::CompressedImage>();
        compressed_image_msg->header.stamp = ros::Time::now();
        compressed_image_msg->format = "jpeg";
        compressed_image_msg->data = img_data;
        compressed_image_pub.publish(compressed_image_msg);

        /*
        // Create a CameraInfo message
        sensor_msgs::CameraInfo camera_info_msg;
        ros::Time current_time = ros::Time::now();

        camera_info_msg.header.frame_id = "head_camera";
        camera_info_msg.header.stamp.sec = current_time.sec;
        camera_info_msg.header.stamp.nsec = current_time.nsec;
        camera_info_msg.height = 1080;
        camera_info_msg.width = 1920;
        camera_info_msg.distortion_model = "";
        camera_info_msg.D = {k1, k2, p1, p2, k3};
        camera_info_msg.K = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        camera_info_msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        camera_info_msg.P = {1735.9131074961197, 0.0, 982.8768921086725, 0.0, 0.0, 1833.27636590429, 616.4013788459191, 0.0, 0.0, 0.0, 1.0, 0.0};
        camera_info_msg.binning_x = 0;
        camera_info_msg.binning_y = 0;
        camera_info_msg.roi.x_offset = 0;
        camera_info_msg.roi.y_offset = 0;
        camera_info_msg.roi.height = 0;
        camera_info_msg.roi.width = 0;
        camera_info_msg.roi.do_rectify = false;

        camera_info_pub.publish(camera_info_msg);
        */

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
