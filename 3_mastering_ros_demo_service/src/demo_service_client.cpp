/* 
* This code is the ROS service server code which will receive the integer from service client

*/


#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

#include "mastering_ros_demo_services/demo_srv.h"
#include <iostream>
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_service_client");


  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  ros::ServiceClient client = n.serviceClient<mastering_ros_demo_services::demo_srv>("demo_service");
  float a,b;
  n.param<float>("num_1",a,3);
  n.param<float>("num_2",b,3);

	while (ros::ok())
	{


		mastering_ros_demo_services::demo_srv srv;
	  srv.request.x = a;
	  srv.request.y = b;


	  if (client.call(srv))
	  {

		  std::cout<<" Requested service: a=  "<<srv.request.x<<" x= "<<srv.request.y<<" mod = "<<srv.response.mod<<"\n";
	  }
	  else
	  {
	    ROS_ERROR("Failed to call service");
	    return 1;
	  }

	ros::spinOnce();
	//Setting the loop rate
	loop_rate.sleep();

	}
  return 0;
}
