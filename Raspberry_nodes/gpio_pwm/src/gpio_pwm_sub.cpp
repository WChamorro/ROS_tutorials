#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>	
#include <std_msgs/Int32.h>
#include <iostream>
#include <sstream>
int range = 100;
int pin = 12;


void pwm_callback(const std_msgs::Int32::ConstPtr & msg){
	int width = msg->data;
	if(width>range){
		width=255;
	}
	softPwmWrite(pin,width);
	std::cout<<"pwm set in "<<width<<"\n";
}


int main (int argc, char **argv)
{
	ros::init(argc, argv,"gpio_pwm_subscriber");
	ros::NodeHandle nh;
	ros::Subscriber pwm_subscriber = nh.subscribe("/pwm",10,pwm_callback);

	wiringPiSetupGpio();
    	ROS_INFO("GPIO has been set as OUTPUT.");
   	 pinMode(pin,OUTPUT);
   	 softPwmCreate(pin,0,range);
	ros::spin();
	return 0;
}
