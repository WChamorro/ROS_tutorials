//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html
//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>






bool moveRobot(std::array<double,7> &conf, double moveduration, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &rClient) 
{
  control_msgs::FollowJointTrajectoryGoal goal;
  
  goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(1);

  goal.trajectory.joint_names.resize(7);       
  goal.trajectory.joint_names[0] = "shoulder_pan_joint";
  goal.trajectory.joint_names[1] = "shoulder_pitch_joint";
  goal.trajectory.joint_names[2] = "elbow_roll_joint";
  goal.trajectory.joint_names[3] = "elbow_pitch_joint";
  goal.trajectory.joint_names[4] = "wrist_roll_joint";
  goal.trajectory.joint_names[5] = "wrist_pitch_joint";
  goal.trajectory.joint_names[6] = "gripper_roll_joint";
  
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(7);
  goal.trajectory.points[0].positions[0] = conf[0];
  goal.trajectory.points[0].positions[1] = conf[1];
  goal.trajectory.points[0].positions[2] = conf[2];
  goal.trajectory.points[0].positions[3] = conf[3];
  goal.trajectory.points[0].positions[4] = conf[4];
  goal.trajectory.points[0].positions[5] = conf[5];
  goal.trajectory.points[0].positions[6] = conf[6];
  goal.trajectory.points[0].time_from_start = ros::Duration(moveduration);
  
  //Send goal
  rClient.sendGoal(goal);

  //Wait for the action to return
  bool finished_before_timeout = rClient.waitForResult(
                goal.trajectory.points.back().time_from_start+ros::Duration(2*moveduration));
  
  actionlib::SimpleClientGoalState state = rClient.getState();
  if (finished_before_timeout) {
      ROS_INFO("Robot action finished: %s",state.toString().c_str());
  } else {
      ROS_ERROR("Robot action did not finish before the timeout: %s",
                state.toString().c_str());
  }

  return (state == actionlib::SimpleClientGoalState::SUCCEEDED);
}


int main(int argc, char **argv) 
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_controls");
  ros::NodeHandle nh;
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robotClient("/seven_dof_arm/arm_joint_trajectory_controller/follow_joint_trajectory");

  if(!robotClient.waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR("action server not available");
  };
  
  double cycletime = 3.0;//3 seconds
  int trajpoints = 100;
  std::array<double,7> positions;
  
  ros::Rate rate(1/cycletime);
  while(ros::ok()) {
    for(int i=0; i<trajpoints;i++)
    {
      positions[0] = 0.3*sin(6.28*i/trajpoints);
      positions[1] = -0.9;
      positions[2] = 0.0;
      positions[3] = 1.9;
      positions[4] = 0.0;
      positions[5] = 0.5;
      positions[6] = 0.0;
      moveRobot(positions, cycletime/trajpoints, robotClient);
    }  
    // Wait until it's time for another iteration.
    rate.sleep();
  }
}
