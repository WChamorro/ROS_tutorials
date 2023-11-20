#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <sstream>

bool state_ready_to_pub = false;
bool control_ready_to_pub=false;
std_msgs::Float64 vel;
std_msgs::Float32 control_action;

void vel_callback(const std_msgs::Float32::ConstPtr & msg)
{
	double v = msg->data;
	
	vel.data = v;
	state_ready_to_pub = true;
	
}

void act_callback(const std_msgs::Float64::ConstPtr & msg)
{
	double a = msg->data;
	
	control_action.data = a;
	control_ready_to_pub = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_description");
  ros::NodeHandle n;
  
  //The node advertises the joint values of the pan-tilt
  ros::Publisher state_pub = n.advertise<std_msgs::Float64>("/state", 10);
  ros::Publisher control_pub = n.advertise<std_msgs::Float32>("dc_motor/command", 10);
  ros::Subscriber sub_vel = n.subscribe("dc_motor/velocity",10,vel_callback);
  ros::Subscriber sub_eff = n.subscribe("/control_effort",10,act_callback);
  std::cout<<" Ready link controller"<<std::endl; 
 
 while(ros::ok()){
 if(state_ready_to_pub){
 	state_pub.publish(vel);
 	state_ready_to_pub = false;
 }
  if(control_ready_to_pub){
 	control_pub.publish(control_action);
 	control_ready_to_pub = false;
 }
 ros::spinOnce();
 }
  return 0;
}



