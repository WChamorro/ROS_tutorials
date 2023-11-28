#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#define b  0.16
#define r  0.033;

double wr =0;
double wl = 0;
bool got_wr = false;
bool got_wl = false;
double t = 0;
double t_ant = -1;

double v,w,x,y,theta;
double x_ant = 0;
double y_ant = 0;
double theta_ant = 0;

ros::Time ros_t;

void right_callback(geometry_msgs::TwistStamped::ConstPtr msg){
	wr = msg->twist.angular.z;
	ros_t = msg->header.stamp;
	got_wr = true;
}

void left_callback(geometry_msgs::TwistStamped::ConstPtr msg){
	wl = msg->twist.angular.z;
	ros_t = msg->header.stamp;
	got_wl = true;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

 

  ros::Subscriber right_sub = node.subscribe("right_wheel",10,right_callback);
  ros::Subscriber left_sub  = node.subscribe("left_wheel",10,left_callback);
  ros::Publisher odom = node.advertise<nav_msgs::Odometry>("kinematics_odom",10);




  while (node.ok()){
	 if(got_wl && got_wr){
		 if(t_ant<0){
			 t = ros_t.toSec();
			 t_ant = t;
		 }else{




			 t = ros_t.toSec();
			 double dt = t - t_ant;
			 double vr = wr * r;
			 double vl = wl * r;

			 v = (vr + vl)/2;
			 w = (vr - vl)/b;

			 theta = theta_ant + w*dt;
			 x = x_ant + v * cos(theta)*dt;
			 y = y_ant + v * sin(theta)*dt;


			 x_ant = x;
			 y_ant = y;
			 theta_ant =theta;
			 t_ant = ros_t.toSec();


			 nav_msgs::Odometry pos;
			 pos.child_frame_id="base_footprint";
			 pos.header.frame_id="kinematic_pos";
			 pos.header.stamp = ros_t;
			 pos.pose.pose.position.x = x;
			 pos.pose.pose.position.y = y;


			 tf2::Quaternion q_;
			 q_.setRPY(0, 0, theta);

			 pos.pose.pose.orientation.x = q_.x();
			 pos.pose.pose.orientation.y = q_.y();
			 pos.pose.pose.orientation.z = q_.z();
			 pos.pose.pose.orientation.w = q_.w();

			 pos.twist.twist.angular.z = w;
			 pos.twist.twist.linear.x = v;
			 odom.publish(pos);

			 got_wr = false;
			 got_wl = false;

			   /*tf2_ros::TransformBroadcaster br;
			   geometry_msgs::TransformStamped transformStamped;

			   transformStamped.header.stamp = ros_t;
			   transformStamped.header.frame_id = "base_footprint";
			   transformStamped.child_frame_id = "kinematic_pos";
			   transformStamped.transform.translation.x = x;
			   transformStamped.transform.translation.y = y;
			   transformStamped.transform.translation.z = 0.0;

			   transformStamped.transform.rotation.x = q.x();
			   transformStamped.transform.rotation.y = q.y();
			   transformStamped.transform.rotation.z = q.z();
			   transformStamped.transform.rotation.w = q.w();

			   br.sendTransform(transformStamped);*/

		 }
	 }
	 ros::spinOnce();

  }
  return 0;
};
