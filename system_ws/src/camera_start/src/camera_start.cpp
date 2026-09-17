#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

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

    camera_matrix = (cv::Mat_<double>(3,3) << 
        fx, 0,  cx, 
        0,  fy, cy, 
        0,  0,  1);

    dist_coeffs = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);

    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 1);
    ros::Publisher compressed_image_pub = nh.advertise<sensor_msgs::CompressedImage>("/camera/image_raw/compressed", 1);
    //ros::Publisher camera_info_pub= nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);
    cv::VideoCapture cap(0, cv::CAP_GSTREAMER); // Open the default camera (usually webcam)

    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open webcam.");
        return -1;
    }
    
    while (nh.ok())
    {
        cap >> frame;

        if (frame.empty())
        {
            ROS_ERROR("Failed to capture frame.");
            break;
        }

        // Intrinsic Calibration
        cv::Mat undistorted_image, map1, map2;
        cv::Size image_size;

        image_size = frame.size();
        cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), camera_matrix, image_size, CV_16SC2, map1, map2);
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