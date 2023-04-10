//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html
//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_controls");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub1 = nh.advertise<trajectory_msgs::JointTrajectory>(
    "seven_dof_arm/arm_joint_trajectory_controller/command", 1000);


  trajectory_msgs::JointTrajectory traj_msg;
  
  traj_msg.header.stamp = ros::Time::now();

  traj_msg.joint_names.resize(7);       
  traj_msg.joint_names[0] = "shoulder_pan_joint";
  traj_msg.joint_names[1] = "shoulder_pitch_joint";
  traj_msg.joint_names[2] = "elbow_roll_joint";
  traj_msg.joint_names[3] = "elbow_pitch_joint";
  traj_msg.joint_names[4] = "wrist_roll_joint";
  traj_msg.joint_names[5] = "wrist_pitch_joint";
  traj_msg.joint_names[6] = "gripper_roll_joint";

  double cycletime = 3.0;//3 seconds
  int trajpoints = 100;
  
  traj_msg.points.resize(trajpoints);
  for(int i=0; i<trajpoints;i++)
  {
      traj_msg.points[i].positions.resize(7);
      traj_msg.points[i].positions[0] = 0.3*sin(6.28*i/trajpoints);
      traj_msg.points[i].positions[1] = -0.9;
      traj_msg.points[i].positions[2] = 0.0;
      traj_msg.points[i].positions[3] = 1.9;
      traj_msg.points[i].positions[4] = 0.0;
      traj_msg.points[i].positions[5] = 0.5;
      traj_msg.points[i].positions[6] = 0.0;
      traj_msg.points[i].time_from_start = ros::Duration(i*cycletime/trajpoints);
  }


  ros::Rate rate(1/cycletime);

  while(ros::ok()) {
    // Publish the message.
    pub1.publish(traj_msg);
 
    // Wait until it's time for another iteration.
    rate.sleep();
  }
}
