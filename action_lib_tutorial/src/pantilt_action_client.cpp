
#include "ros/ros.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "actions_tutorial/pantilt_actionAction.h"


const double degree2rad = M_PI/180;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const actions_tutorial::pantilt_actionResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: Final angle is %f", result->final_count);
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const actions_tutorial::pantilt_actionFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback: current angle is %f", feedback->current_number);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "pantilt_action_client");

   if(argc != 4){
	ROS_INFO("%d",argc);
        ROS_WARN("Usage: pantilt_action_client <joint value in degrees> <joint name> <time_to_preempt_in_sec>");
	return 1;
  }

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actions_tutorial::pantilt_actionAction> ac("pantilt_action_server", true);
  ROS_INFO("Waiting for action server to start...");

  // wait for the action server to start
  if(!ac.waitForServer(ros::Duration(10, 0)))
  {
          ROS_INFO("Action server not active - tired of waiting...");
          return 1;
  }

  ROS_INFO("Action server started, sending goal.");

  // send a goal to the action
  actions_tutorial::pantilt_actionGoal goal;
  goal.count = atof(argv[1]) * degree2rad; //joint value in degrees
  goal.name = argv[2]; //joint name
 
  ROS_INFO("Sending Goal: Radians [%f], Joint Name [%s] and Preempt time of [%f]",goal.count, goal.name.c_str(), atof(argv[3]));
 
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(atof(argv[3])));
  //Preempting task
  ac.cancelGoal();

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
