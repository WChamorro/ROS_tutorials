
#include "turtlebot3_gazebo/turtlebot3_pos_control.h"

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  updatecommandVelocity(0.0, 0.0);

}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");
  nh_.param<double>("x_setpoint",xd,0);
  nh_.param<double>("y_setpoint",yd,0);
  nh_.param<double>("kv",kv,0);
  nh_.param<double>("kw",kw,0);

  std::cout<<"set point: "<<xd<<" , "<<yd<<"\n";
  //state size
  x.resize(3);

  // velocity publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // odometry subscribers
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
	//get yaw angle from quaternion
	double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
	double theta = atan2(siny, cosy);

	//get position from odometry
  x[0] = msg->pose.pose.position.x;
  x[1] = msg->pose.pose.position.y;
  x[2] = theta;
  controlLoop();
  std::cout<<"odom vx "<<msg->twist.twist.linear.x<<" vy "<<msg->twist.twist.linear.y<<" th "<<msg->twist.twist.angular.z<<"\n";


}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.linear.y  = linear;
  cmd_vel.linear.z  = linear;

  cmd_vel.angular.x = angular;
  cmd_vel.angular.y = angular;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
  std::cout<<"stopping"<<"\n";
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Drive::controlLoop()
{

	//error
	double ex = xd - x[0];
	double ey = yd - x[1];



	//control law
	//double v = vr*cos(et) + kv*ex;
	//double w = wr + ky*vr*ey + kt*vr*sin(et);
	 double th_g = atan2(ey,ex);
	 double v = kv*sqrt(ex*ex + ey*ey);
	 double w = kw*(th_g - x[2]);

	 geometry_msgs::Twist cmd_vel;
	 cmd_vel.linear.x = v;
	 cmd_vel.angular.z = w;
	 cmd_vel_pub_.publish(cmd_vel);
	 std::cout<<" x: "<<x[0]<<" y: "<<x[1]<<" th: "<<x[2]<<" v "<<v<<" vx: "<<cmd_vel.linear.x<<" vy "<<cmd_vel.linear.y  <<" w "<<w<<"\n";



  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;

 ros::spin();

  return 0;
}
