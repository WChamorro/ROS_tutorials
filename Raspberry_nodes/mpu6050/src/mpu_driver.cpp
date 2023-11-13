/**
 * @author  Dwindra Sulistyoutomo
 */
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <MPU6050Pi.h>

#define g   9.81



int main(int argc, char **argv) {

	ros::init(argc, argv, "gpio_read_mpu6050");
	ros::NodeHandle nh;
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/mpu_imu", 10);

	// Connect to device with default setting
	MPU6050Pi mpu;
	// Enable interrupt
	mpu.SetIntEnabled(1);


	std::cout << "Current configuration: Accel " << mpu.GetAccelSensitivity()<< " Gyro: " << mpu.GetGyroSensitivity() << std::endl;
	float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
	float deg_to_rad = M_PI / 180;
	Eigen::Matrix3f Ac;
	Eigen::Vector3f Ao,A,G,Go, A_cal,G_cal;

	Ac<< -0.9884,   -0.0054,    0.0690,
		 -0.0444,   -0.9815,   -0.0085,
		  0.0270,    0.0042,   -0.9768;

	Ao<< 0.0120,
		 0.0128,
	    -0.0007;

	Go<<-0.28589312977099235, 0.8312366412213742, 0.22204580152671755;
	ros::Rate loop_rate(200);

			// Publish in loop.
	while (ros::ok()) {
		std::cout<<"status"<<mpu.GetIntStatus()<<std::endl;

		
			// Get gyroscope data.
					mpu.GetGyroFloat(&G(0), &G(1), &G(2));
					// Get accelerometer values.
					mpu.GetAccelFloat(&A(0), &A(1), &A(2));


					// acceleration calibracion
					A_cal = Ac * A + Ao;

					std::cout<<"accel "<<A_cal(0)<<" "<<A_cal(1)<<" "<<A_cal(1)<<std::endl;
					sensor_msgs::Imu imu_data;
					imu_data.header.stamp = ros::Time::now();
					imu_data.header.frame_id = "world";

					imu_data.linear_acceleration.x = A_cal(0)*g; // unit: m/s2
					imu_data.linear_acceleration.y = A_cal(1)*g;
					imu_data.linear_acceleration.z = A_cal(2)*g;

					// gyroscope calibracion
					G_cal = G - Go;

					imu_data.angular_velocity.x = G_cal(0)*deg_to_rad; //unit: rad /s
					imu_data.angular_velocity.y = G_cal(1)*deg_to_rad;
					imu_data.angular_velocity.z = G_cal(2)*deg_to_rad;

					imu_pub.publish(imu_data);
		  loop_rate.sleep();
                               ros::spinOnce();


	}

	return 0;
}
