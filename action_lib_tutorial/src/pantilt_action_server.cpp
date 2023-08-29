#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h>
#include "actions_tutorial/pantilt_actionAction.h"
#include <iostream>
#include <sstream>
#include <sensor_msgs/JointState.h>


const double degree2rad = M_PI/180;
sensor_msgs::JointState joint_state;

class pantilt_actionAction
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<actions_tutorial::pantilt_actionAction> act_server;
  // create messages that are used to published feedback/result
  actions_tutorial::pantilt_actionFeedback feedback;
  actions_tutorial::pantilt_actionResult result;

  std::string action_name;
  double currentvalue;
  double scale;

public:
  pantilt_actionAction(std::string name) :
    act_server(nh_, name, boost::bind(&pantilt_actionAction::executeCB, this, boost::placeholders::_1), false),
    action_name(name)
  {
	act_server.registerPreemptCallback(boost::bind(&pantilt_actionAction::preemptCB, this));
        act_server.start();
        scale=1;
  }

  ~pantilt_actionAction(void)
  {
  }

  void preemptCB()
  {
    ROS_WARN("%s got preempted!", action_name.c_str());
    result.final_count = currentvalue;
    act_server.setPreempted(result,"I got Preempted");
  }
  void executeCB(const actions_tutorial::pantilt_actionGoalConstPtr &goal)
  {
    if(!act_server.isActive() || act_server.isPreemptRequested()) return;

    ROS_INFO("%s is processing the goal %f for joint %s", action_name.c_str(), goal->count, goal->name.c_str());

    //Set the joint to be controlled
    int jointnumber;
    std::string jointname;
    if(goal->name=="pan")
    {
      jointnumber=0;
      jointname = "pan_joint";
    }
    else
    {
      jointnumber=1;
      jointname = "tilt_joint";
    }

    //detrmine the motion direction
    currentvalue = joint_state.position[jointnumber];
    int sign;
    if(currentvalue-goal->count < 0)
        sign=1;
    else
        sign=-1;

    //loop: keep increasing/decreasing the joint value until the goal has been reached or timeout.
    ros::Rate rate(10);
    while(ros::ok())
    {
      //update joint_state - moving one degree*scale
      joint_state.name[jointnumber] = jointname;
      joint_state.position[jointnumber] = currentvalue + sign * degree2rad * scale;
      currentvalue = joint_state.position[jointnumber];

       if(!act_server.isActive() || act_server.isPreemptRequested())
      {
        return;
      }

      //ROS_INFO("%f %f",abs(currentvalue-goal->count),degree2rad * scale);
      if(fabs(currentvalue-goal->count)< degree2rad * scale)
      {
        ROS_INFO("%s Succeeded at getting joint %s  to goal %f", action_name.c_str(), goal->name.c_str(), goal->count);
        result.final_count = currentvalue;
        act_server.setSucceeded(result);
      }
      else
      {
        ROS_INFO("Setting to goal %f / %f",feedback.current_number,goal->count);
        feedback.current_number = currentvalue;
        act_server.publishFeedback(feedback);
      }

      rate.sleep();
    }
    // not ros::ok()
    result.final_count = currentvalue;
    act_server.setAborted(result,"I failed !");
    ROS_INFO("%s Shutting down",action_name.c_str());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pantilt_action_server");
  ros::NodeHandle nh;
  ROS_INFO("Starting pantilt Action Server");
  pantilt_actionAction pantilt_action_obj(ros::this_node::getName());

  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.header.stamp = ros::Time::now();
  joint_state.name[0] ="pan_joint";
  joint_state.position[0] = 0.0;
  joint_state.name[1] ="tilt_joint";
  joint_state.position[1] = 0.0;

  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Rate loop_rate(30);//set equal to the loop_rate in the CB
  while (ros::ok())
  {
      //update joint_state time stamp - joint values changes in the CB
      joint_state.header.stamp = ros::Time::now();
      joint_pub.publish(joint_state);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
