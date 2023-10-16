#include <ros/ros.h>
#include <wiringPi.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <sstream>


void pin_callback(const std_msgs::Int32::ConstPtr & msg){
	int pin = msg->data;
	wiringPiSetupGpio();
	pinMode(pin, OUTPUT);
	digitalWrite(pin, HIGH);
	std::cout<<"Set GPIO HIGH "<<"\n";
	delay(1);
	digitalWrite(pin, LOW);

}

int main (int argc, char **argv)
{
	ros::init(argc, argv,"gpio_write_subscriber");
	ros::NodeHandle nh;
	ros::Subscriber pin_subscriber = nh.subscribe("/pinNumber",10,pin_callback);
	ros::spin();
	return 0;
}
