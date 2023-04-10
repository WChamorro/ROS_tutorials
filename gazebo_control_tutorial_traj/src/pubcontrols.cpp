// This program publishes randomly-generated velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <std_msgs/Float64.h>  

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_controls");
  ros::NodeHandle nh;

  // Create a publisher objects
  ros::Publisher pub1 = nh.advertise<std_msgs::Float64>(
    "seven_dof_arm/joint1_position_controller/command", 1000);
  ros::Publisher pub2 = nh.advertise<std_msgs::Float64>(
    "seven_dof_arm/joint2_position_controller/command", 1000);
  ros::Publisher pub4 = nh.advertise<std_msgs::Float64>(
    "seven_dof_arm/joint4_position_controller/command", 1000);
  ros::Publisher pub6 = nh.advertise<std_msgs::Float64>(
    "seven_dof_arm/joint6_position_controller/command", 1000);
  
  std_msgs::Float64 msg1, msg2, msg4, msg6;
  msg2.data = -0.9;
  msg4.data = 1.9;
  msg6.data = 0.5;
  
  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(100);
  int i=0;
  while(ros::ok()) {
    // Create and fill in the message.  
    msg1.data = 0.3*sin(i/100.0);
    i++;

    // Publish the message.
    pub1.publish(msg1);
    pub2.publish(msg2);
    pub4.publish(msg4);
    pub6.publish(msg6);

    // Wait until it's time for another iteration.
    rate.sleep();
  }
}
