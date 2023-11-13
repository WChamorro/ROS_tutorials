#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>



 std::string name;
 Eigen::Vector3f r, r_ant;
 Eigen::Quaterniond q, q_exp;
 Eigen::Quaterniond q_ant(1,0,0,0);
 Eigen::Vector3d v , v_ant;
 Eigen::Vector3d g(0,0,-9.81);
 double t_ant;



void poseCallback(const sensor_msgs::Imu::ConstPtr  &msg){

	Eigen::Vector3d a,w;
	a<< msg->linear_acceleration.x, msg->linear_acceleration.y,msg->linear_acceleration.z;
	w<< msg->angular_velocity.x, msg->angular_velocity.y,msg->angular_velocity.z;

	double t  = msg->header.stamp.toSec();
	double dt = t - t_ant;

	v = v_ant + (q*a +g)*dt;
	r = r_ant + v*dt;

	// Exp map
	double th = (w*dt).norm();
	Eigen::Vector3d  u = (w*dt).normalized();
    q_exp.w() = cos(th/2);
    q_exp.x() = u(0)*sin(th/2);
    q_exp.y() = u(1)*sin(th/2);
    q_exp.z() = u(2)*sin(th/2);

    q = q_ant * q_exp;

    r_ant = r;
    v_ant = v;
    q_ant = q;
    t_ant = t;

    std::cout<<" pose "<< r(0)<<" "<<r(1)<<" "<<r(2)<<" orientation "<<q.w()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<std::endl;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = msg->header.stamp;
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "imu";
  transformStamped.transform.translation.x = r(0);
  transformStamped.transform.translation.y = r(1);
  transformStamped.transform.translation.z = r(2);


  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pose_integration");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/mpu_imu", 10, &poseCallback);

  ros::spin();
  return 0;
};
