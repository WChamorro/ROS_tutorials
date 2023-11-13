#include "ros/ros.h"
#include "camera_info_manager/camera_info_manager.h"
#include "sensor_msgs/CameraInfo.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_info_manager_node");
    ros::NodeHandle nh;

    // Specify the camera name and namespace
    std::string camera_name = "webcam";
    std::string namespace_str = "/webcam";

    // Create a CameraInfoManager
    camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name, namespace_str);

    // Set the camera information
    sensor_msgs::CameraInfo camera_info;
    camera_info.width = 640;  // Set the width of the image (adjust as needed)
    camera_info.height = 480;  // Set the height of the image (adjust as needed)

    // Load or set camera calibration data
    if (cam_info_manager.validateCameraInfo(camera_info)) {
        // Camera calibration data is valid
        cam_info_manager.setCameraInfo(camera_info);

        // Publish the camera_info message
        ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(namespace_str + "/camera_info", 1, true);
        camera_info_pub.publish(camera_info);

        ROS_INFO("Camera information set and published.");
    } else {
        ROS_ERROR("Invalid camera information. Please check camera calibration data.");
        return -1;
    }

    ros::spin();

    return 0;
}
