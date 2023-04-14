/* 
 * This code is the ROS service server code which will receive the integer from service client

*/


#include "ros/ros.h"
#include "mastering_ros_demo_services/demo_srv.h"
#include <iostream>
#include <sstream>




bool demo_service_callback(mastering_ros_demo_services::demo_srv::Request  &req,
		mastering_ros_demo_services::demo_srv::Response &res)
{


  float a = req.x;
  float b = req.y;
  float m = std::sqrt(a*a + b*b);

  res.mod = m;

  std::cout<<" Module requested: "<<res.mod<<"\n";

  return true;


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_service_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("demo_service", demo_service_callback);
  ROS_INFO("Ready to receive from client.");
  ros::spin();

  return 0;
}



