#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
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


void wheel_callback(geometry_msgs::TwistStamped::ConstPtr msg){
	wl = msg->twist.angular.y;
	wr = msg->twist.angular.z;
	ros_t = msg->header.stamp;
		got_wr = true;

}
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  ros::Subscriber left_sub  = node.subscribe("wheel_vel",10,wheel_callback);
  ros::Publisher odom = node.advertise<nav_msgs::Odometry>("kinematics_odom",10);

  while (node.ok()){
	 if(got_wr){
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

			 theta = theta_ant + w*dt; // aqui poner theta del magnetometro en lugar de la operacion matematica
			 x = x_ant + v * cos(theta)*dt;
			 y = y_ant + v * sin(theta)*dt;


			 x_ant = x;
			 y_ant = y;
			 theta_ant =theta;
			 t_ant = ros_t.toSec();


			 nav_msgs::Odometry pos;
			 pos.child_frame_id="kinematics_odom";
			 pos.header.frame_id="base_footprint";
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
			 pos.twist.twist.linear.x = v * cos(theta);
			 pos.twist.twist.linear.y = v * sin(theta);
			 odom.publish(pos);

			 got_wr = false;
			 got_wl = false;

			   /*tf2_ros::TransformBroadcaster br;
			   geometry_msgs::TransformStamped transformStamped;

			   transformStamped.header.stamp = ros_t;
			   transformStamped.header.frame_id = "base_footprint";
			   transformStamped.child_frame_id = "kinematic_frame";
			   transformStamped.transform.translation.x = x;
			   transformStamped.transform.translation.y = y;
			   transformStamped.transform.translation.z = 0.0;



			   transformStamped.transform.rotation.x = q_.x();
			   transformStamped.transform.rotation.y = q_.y();
			   transformStamped.transform.rotation.z = q_.z();
			   transformStamped.transform.rotation.w = q_.w();

			   br.sendTransform(transformStamped);*/

		 }
	 }
	 ros::spinOnce();

  }
  return 0;
};
