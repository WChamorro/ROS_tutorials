
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

std::vector<double> x;
double xd=0;
double yd=0;
double kv;
double kw;
ros::Publisher cmd_vel_pub_ ;

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool controlLoop()
{

	//error
	double ex = xd - x[0];
	double ey = yd - x[1];




	 double th_g = atan2(ey,ex);
	 double v = kv*sqrt(ex*ex + ey*ey);
	 double w = kw*(th_g - x[2]);

	 //accion de control
	 //aqui se deberia calcular y publicar wr y wl
	 // wr = (v + w*b/2)/r
	 // wl = (v - w*b/2)/r

	 //la simulación me permite enviar directamente velocidad lineal y angular
	 geometry_msgs::Twist cmd_vel;
	 cmd_vel.linear.x = v;
	 cmd_vel.angular.z = w;
	 cmd_vel_pub_.publish(cmd_vel);
	 std::cout<<" x: "<<x[0]<<" y: "<<x[1]<<" th: "<<x[2]<<" v "<<v<<" vx: "<<cmd_vel.linear.x<<" vy "<<cmd_vel.linear.y  <<" w "<<w<<"\n";

  return true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
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




/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  // initialize ROS parameter
  ros::NodeHandle nh_("~");

   nh_.param<double>("x_setpoint",xd,0);
   nh_.param<double>("y_setpoint",yd,0);
   nh_.param<double>("kv",kv,0);
   nh_.param<double>("kw",kw,0);

   std::cout<<"set point: "<<xd<<" , "<<yd<<"\n";
   //state size
   x.resize(3);

   // velocity publishers
   cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

   // odometry subscribers
   ros::Subscriber odom_sub_ = nh_.subscribe("/odometry/filtered", 10, odomCallback);

 ros::spin();

  return 0;
}
