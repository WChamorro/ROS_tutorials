/**
 * @author  Dwindra Sulistyoutomo
 */
#include <ros/ros.h>
#include <iostream>	
#include <wiringPi.h>
#include <sensor_msgs/Imu.h>

long encoder;
int stateA=0;
int stateB=0;
float angle=0;

int wireA=0;
int wireB=0;

void A(){
	if(stateA==stateB){
		encoder++;
	}
	stateA = digitalRead(wireA);
	std::cout<<"count A: "<<encoder<<" -- "<<stateA;
	return;
}

void B(){
	if(stateA==stateB){
		encoder--;
	}
	stateB = digitalRead(wireB);
	std::cout<<"count B: "<<encoder<<" -- "<<stateB;
	return;
}

int main (int argc, char **argv)
{


    ros::init(argc, argv, "encoder");
    ros::NodeHandle nh;
    //ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_mpu6050",10);
    wiringPiSetup();
    wiringPiISR(wireA,INT_EDGE_FALLING , &A);
    wiringPiISR(wireB,INT_EDGE_FALLING , &B);

    while(ros::ok()){

    }

return 0;
}
