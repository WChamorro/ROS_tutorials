/*
 *  This code will subscriber integer values from demo_topic_publisher
*/


#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

bool data_received = false;
float num = 0;
void topic_callback(const std_msgs::Float32::ConstPtr & msg)
{
	 num = msg->data;
       data_received = true;
}

int main(int argc, char **argv)

{

	//Initializing ROS node with a name of demo_topic_subscriber
	ros::init(argc, argv,"record");
	//Created a nodehandle object
	ros::NodeHandle node_obj;
	//Create a publisher object
	std::string topic_name;
	std::string data_file;
	
	//topico para subscribirse
	node_obj.param<std::string>("topic_name",topic_name,"");
	
	//archivo donde se va a guardar los datos
	node_obj.param<std::string>("file_name",data_file,"");
	
	ros::Subscriber number_subscriber = node_obj.subscribe(topic_name,10,topic_callback);
	std::ofstream file;
	
	
	file.open(data_file, std::ofstream::trunc);
	file.close();
	file.open(data_file, std::ofstream::out | std::ofstream::app );
	while(ros::ok()){
		if(data_received){
			ros::Time t =ros::Time::now();
			file<<t.toSec()<< "\t" << num << std::endl;
		data_received = false;
		}
		ros::spinOnce();
	}
	file.close();
	return 0;
}


