/*
 * This code will publish a integers from 0 to n with a delay of 100ms
 */

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <sstream>


int limit = 10;

int main(int argc, char **argv)

{
	//Initializing ROS node with a name of demo_topic_publisher
	ros::init(argc, argv,"demo_std_msg_publisher");
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	//Create a publisher object
	ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/demo_std_msg_topic",10);
	//Create a rate object
	ros::Rate loop_rate(2);
	//Variable of the number initializing as zero
	int number_count = 0;

	//While loop for incrementing number and publishing to topic /numbers
	while (ros::ok())
	{

		//Created a Int32 message
		std_msgs::Int32 num_msg;
		num_msg.data=number_count;
		//Inserted data to message header

  
	 	//Printing message data
		//ROS_INFO("%d",num); >> I dont like this
		std::cout<<" Number: "<<number_count<<"\n";


		//Publishing the message
		number_publisher.publish(num_msg);
		//Spining once for doing the  all operation once
		ros::spinOnce();
		//Setting the loop rate
		loop_rate.sleep();
		//Increamenting the count
		++number_count;

		// Comparing
		if(number_count==11){
			number_count = 0;
		}

	}
	
	return 0;
}


