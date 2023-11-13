#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "webcam_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/webcam/image_raw", 1);

    cv::VideoCapture cap(0);  // Adjust the camera index if needed

    if (!cap.isOpened()) {
        ROS_ERROR("Could not open webcam.");
        return -1;
    }

    ros::Rate rate(10);  // Set the publishing rate (Hz)

    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;

        if (!frame.empty()) {
            // Convert the OpenCV image to ROS format
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_pub.publish(msg);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
