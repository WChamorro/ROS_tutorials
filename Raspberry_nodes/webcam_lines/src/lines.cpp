#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert ROS image message to OpenCV image
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Convert the image to grayscale for line detection
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // Apply GaussianBlur to reduce noise and improve line detection
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

        // Apply Canny edge detector
        cv::Mat edges;
        cv::Canny(gray, edges, 50, 150);

        // Apply Hough Line Transform to detect lines
        std::vector<cv::Vec2f> lines;
        cv::HoughLines(edges, lines, 1, CV_PI / 180, 100);

        // Draw lines on the original image
        for (const auto& line : lines) {
            float rho = line[0];
            float theta = line[1];
            double a = std::cos(theta), b = std::sin(theta);
            double x0 = a * rho, y0 = b * rho;
            cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
            cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
            cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }

        // Display the image with detected lines
        cv::imshow("Lines Detected", image);
        cv::waitKey(1);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_detection_node");
    ros::NodeHandle nh;

    // Subscribe to the USB camera image topic
    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);

    // OpenCV window setup
    cv::namedWindow("Lines Detected");

    ros::spin();

    return 0;
}
