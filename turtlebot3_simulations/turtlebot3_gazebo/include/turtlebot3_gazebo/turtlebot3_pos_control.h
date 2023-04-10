
#ifndef TURTLEBOT3_DRIVE_H_
#define TURTLEBOT3_DRIVE_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)


class Turtlebot3Drive
{
 public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();
  bool init();
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber odom_sub_;
  // Variables

  const double L = 0.08; //body radious
  const double r = 0.033; //wheel radious
  const double v_max = 0.22; // max linear velocity
  const double w_max = 2.84; // max angular velocity
  std::vector<double> x;
  double xd=0;
  double yd=0;
  double kv = 0;
  double kw = 0;



  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};
#endif // TURTLEBOT3_DRIVE_H_
