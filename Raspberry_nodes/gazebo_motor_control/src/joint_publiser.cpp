#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

double deltaPan;
double deltaTilt;
double scale;
const double degree2rad = M_PI/180;
float vel = 0;
void vel_callback(const std_msgs::Float32::ConstPtr & msg)
{
	vel = msg->data;
	std::cout<<" vel "<<vel<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_description");
  ros::NodeHandle n;
  
  //The node advertises the joint values of the pan-tilt
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  ros::Subscriber sub = n.subscribe("dc_motor/velocity",10,vel_callback);
    
  ros::Rate loop_rate(100);

  // message declarations
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(1);  // joint_state is a vector should be sized with the number of joints
  joint_state.position.resize(1);
joint_state.velocity.resize(1);
joint_state.effort.resize(1);
  

  while (ros::ok())
  {

      
      //update joint_state
      joint_state.header.stamp = ros::Time::now();
      joint_state.name[0] ="dc_motor";
      joint_state.position[0] = 0;
      joint_state.effort[0] = 0.1;
      joint_state.velocity[0] = vel;

      //send the joint state 
      joint_pub.publish(joint_state);

      loop_rate.sleep();
      ros::spinOnce();
  }
  return 0;
}



