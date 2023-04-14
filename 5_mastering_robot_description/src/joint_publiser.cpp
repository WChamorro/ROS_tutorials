#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


double deltaPan;
double deltaTilt;
double scale;
const double degree2rad = M_PI/180;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_description");
  ros::NodeHandle n;
  
  //The node advertises the joint values of the pan-tilt
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    
  ros::Rate loop_rate(30);

  // message declarations
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(2);  // joint_state is a vector should be sized with the number of joints
  joint_state.position.resize(2);
  double pan = 0.0;
  double tilt = 0.0;

  deltaPan = 0.0;
  deltaTilt = 0.0;
  scale =2;
  

  while (ros::ok())
  {
      //moving one degree
      deltaPan =  degree2rad * scale;
      deltaTilt = degree2rad * scale;
            
      pan = pan + deltaPan;
      tilt = tilt + deltaTilt;
      
      //update joint_state
      joint_state.header.stamp = ros::Time::now();
      joint_state.name[0] ="pan_joint";
      joint_state.position[0] = pan;
      joint_state.name[1] ="tilt_joint";
      joint_state.position[1] = tilt;

      //send the joint state 
      joint_pub.publish(joint_state);

      loop_rate.sleep();
  }
  return 0;
}



