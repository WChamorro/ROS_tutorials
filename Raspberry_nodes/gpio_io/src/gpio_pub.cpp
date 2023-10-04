#include <ros/ros.h>
#include <wiringPi.h>

#define LED_PIN 0 // change pin number here

int main (int argc, char **argv)
{
    ros::init(argc, argv, "gpio_ros");
    ros::NodeHandle nh;

    wiringPiSetupGpio();
    pinMode(LED_PIN, OUTPUT);
    ROS_INFO("GPIO has been set as OUTPUT.");

    while (ros::ok())
    {
        digitalWrite(LED_PIN, HIGH);
        ROS_INFO("Set GPIO HIGH");
        ros::Duration(1.0).sleep();
        digitalWrite(LED_PIN, LOW);
        ROS_INFO("Set GPIO LOW");
        ros::Duration(1.0).sleep();
    }
}

