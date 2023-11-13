#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert ROS image message to OpenCV image
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        // Get image size
        cv::Size imageSize = image.size();
        
        // Print image size
        ROS_INFO("Image Size: %d x %d", imageSize.width, imageSize.height);


        // Convert the image to grayscale for corner detection
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // Detect corners using the Shi-Tomasi method
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(gray, corners, 100, 0.01, 10);

        // Draw corners on the original image
        for (const auto& corner : corners) {
            cv::circle(image, corner, 5, cv::Scalar(0, 255, 0), -1);
        }

        // Display the image with corners
        cv::imshow("Corners Detected", image);
        cv::waitKey(1);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "corner_detection_node");
    ros::NodeHandle nh;

    // Subscribe to the USB camera image topic
    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);

    // OpenCV window setup
    cv::namedWindow("Corners Detected");

    ros::spin();

    return 0;
}
