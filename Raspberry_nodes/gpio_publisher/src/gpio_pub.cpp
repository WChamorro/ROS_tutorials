#include <ros/ros.h>
#include <wiringPi.h>

#include <std_msgs/Int32.h>
#include <iostream>
#include <sstream>

int main (int argc, char **argv)
{
	int pin = 2;  //gpio2 pin 3
	int pin_on = 3;

    ros::init(argc, argv, "gpio_read_publisher");
    ros::NodeHandle nh;
    ros::Publisher pin_status = nh.advertise<std_msgs::Int32>("/pinNumber",10);

    wiringPiSetupGpio();
    pinMode(pin, INPUT);
    std::cout<<"Pin "<<pin<<" Set as INPUT"<<"\n";
    ros::Rate loop_rate(3);

    while (ros::ok())
    {
    	std_msgs::Int32 status;
        int pulse = digitalRead(pin);
        if(pulse==1){
        	status.data = pin_on;
        	pin_status.publish(status);
        	 std::cout<<"Pin READ "<<pin_on<<"\n";
        }


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
