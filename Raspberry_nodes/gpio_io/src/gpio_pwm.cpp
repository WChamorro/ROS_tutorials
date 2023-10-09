#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <sstream>
int range = 255;
void pwm_callback(const std_msgs::Int32::ConstPtr & msg){
	int width = msg->data;
	if(width>range){
		width=255;
	}
	pwmWrite(3, width);
}

int main (int argc, char **argv)
{
	ros::init(argc, argv,"gpio_pwm_subscriber");
	ros::NodeHandle nh;
	ros::Subscriber pwm_subscriber = nh.subscribe("/pwm",10,pwm_callback);
	wiringPiSetupGpio();
	pinMode(3, OUTPUT);
	pwmSetRange(range);
	ros::spin();
	return 0;
}
