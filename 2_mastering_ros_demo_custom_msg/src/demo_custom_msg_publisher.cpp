/* This code will publish a integers from 0 to n with a delay of 2-hz

 */

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "mastering_ros_demo_custom_msg/demo_msg.h"
#include <iostream>
#include <sstream>


int main(int argc, char **argv)

{

	//Initializing ROS node with a name of demo_topic_publisher
	ros::init(argc, argv,"demo_custom_msg_publisher");
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	//Create a publisher object
	ros::Publisher number_publisher = node_obj.advertise<mastering_ros_demo_custom_msg::demo_msg>("/demo_custom_msg_topic",10);
	//Create a rate object
	ros::Rate loop_rate(2);
	//Variable of the number initializing as zero
	int number_count = 0;

	//While loop for incrementing number and publishing to topic /numbers
	while (ros::ok()) {

		//Created a Int32 message
		mastering_ros_demo_custom_msg::demo_msg msg;
		//Inserted data to message header

		std::string txt;
		txt = "hello_world_"+std::to_string(number_count);
		msg.id = txt;
		msg.number = number_count;
		//Printing message data
		std::cout<<" custom msg: "<<msg.id<<" -- "<<msg.number<<"\n";

		//Publishing the message
		number_publisher.publish(msg);
		//Spining once for doing the  all operation once
		ros::spinOnce();
		//Setting the loop rate
		loop_rate.sleep();
		//Increamenting the count
		++number_count;
		if(number_count == 10)number_count=0;

	}
	
	return 0;
}


