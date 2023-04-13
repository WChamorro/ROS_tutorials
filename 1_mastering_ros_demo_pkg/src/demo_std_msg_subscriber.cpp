/*
 *  This code will subscriber integer values from demo_topic_publisher
*/


#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <sstream>


void number_callback(const std_msgs::Int32::ConstPtr & msg)
{
	int num = msg->data;
	std::cout<<" Recibido: "<<num<<"\n";
}

int main(int argc, char **argv)

{

	//Initializing ROS node with a name of demo_topic_subscriber
	ros::init(argc, argv,"demo_std_msg_subscriber");
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	//Create a publisher object
	ros::Subscriber number_subscriber = node_obj.subscribe("/demo_std_msg_topic",10,number_callback);
	//Spinning the node
	ros::spin();
	return 0;
}


