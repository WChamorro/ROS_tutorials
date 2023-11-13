#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Display the image
        cv::imshow("Webcam Subscriber", image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "webcam_subscriber");
    ros::NodeHandle nh;

    // Subscribe to the webcam image topic
    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);

    // OpenCV window setup
    cv::namedWindow("Webcam Subscriber");

    ros::spin();

    return 0;
}
