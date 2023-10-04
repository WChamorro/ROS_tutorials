
#include <ros/ros.h>
#include <pigpio.h>

#define LED_PIN 0// change pin number here

int main (int argc, char **argv)
{
    ros::init(argc, argv, "gpio_ros");
    ros::NodeHandle nh;

    if (gpioInitialise() < 0)
{
 ROS_INFO("GPIO FAILED");
}
    ROS_INFO("GPIO has been set as OUTPUT.");
    gpioSetMode(2, PI_OUTPUT);;

    while (ros::ok())
    {
        gpioWrite(2, 1);
        ROS_INFO("Set GPIO HIGH");
        ros::Duration(1.0).sleep();


        gpioWrite(2, 0);
        ROS_INFO("Set GPIO LOW");
        ros::Duration(1.0).sleep();
ros::spinOnce();
    }
gpioTerminate();
ros::shutdown();
return 0;
}


