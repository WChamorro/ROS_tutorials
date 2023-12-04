#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/LinkStates.h>


bool singularity = false;
double w_left,w_right;
ros::Time T;
bool got_w=false;

void gazebo_callback(const gazebo_msgs::LinkStates::ConstPtr msg){
	w_left=msg->twist[2].angular.y; //left
	w_right=msg->twist[3].angular.y; //right
	got_w = true;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

 

  ros::Publisher wheel_pub = node.advertise<geometry_msgs::TwistStamped>("wheel_vel", 10);
  ros::Subscriber gazebon = node.subscribe("/gazebo/link_states",10,gazebo_callback);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  double dt = 0;
  double t_ant = -1;
  geometry_msgs::TwistStamped wr_ant ;
  geometry_msgs::TwistStamped wl_ant ;

  while (node.ok()){

	  if(got_w){
		    geometry_msgs::TwistStamped wheel;
		        	wheel.header.stamp = ros::Time::now();
		        	wheel.twist.angular.x = 0;
		        	wheel.twist.angular.y = w_left;
		        	wheel.twist.angular.z = w_right;
		        	wheel_pub.publish(wheel);
		        	got_w=false;
	  }

    ros::spinOnce();
  }
  return 0;
};
