/* This code will subscriber integer values from demo_topic_publisher

*/


#include "ros/ros.h"
#include "mastering_ros_demo_custom_msg/demo_msg.h"
#include <iostream>
#include <sstream>


void number_callback(const mastering_ros_demo_custom_msg::demo_msg::ConstPtr& msg)
{
	std::cout<<" Recibido: "<<msg->id<<" -- "<<msg->number<<"\n";
}

int main(int argc, char **argv)

{

	//Initializing ROS node with a name of demo_topic_subscriber
	ros::init(argc, argv,"demo_custom_msg_subscriber");
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	//Create a publisher object
	ros::Subscriber number_subscriber = node_obj.subscribe("/demo_custom_msg_topic",10,number_callback);
	//Spinning the node
	ros::spin();
	return 0;
}


