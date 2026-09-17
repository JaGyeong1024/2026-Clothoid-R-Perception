#include "livox_camera_fusion.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_camera_fusion");
    ros::NodeHandle nh("~");
    LivoxCameraFusion node(&nh);
    ros::spin();
    return 0;
}
