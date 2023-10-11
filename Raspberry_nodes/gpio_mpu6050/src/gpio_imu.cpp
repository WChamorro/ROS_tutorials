#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <sstream>

#define MPU6050_ADDRESS (0x68)         //mpu6050 default address
#define MPU6050_REG_PWR_MGMT_1 (0x6b)  // power managment register
#define MPU6050_ACC_X (0x3b) //ACCELERATION HIGH
#define MPU6050_ACC_Y (0x3d)
#define MPU6050_ACC_Z (0x3f)

#define MPU6050_time   (0x41)//TIME HIGH

#define MPU6050_GYRO_X (0x43) //GYRO HIGH
#define MPU6050_GYRO_Y (0x45)
#define MPU6050_GYRO_Z (0x47)

void checkRC(int rc, char *text) {
  if (rc < 0) {
    printf("Error: %s - %d\n");
    exit(-1);
  }
}

int main (int argc, char **argv)
{


    ros::init(argc, argv, "gpio_read_mpu6050");
    ros::NodeHandle nh;
    ros::Publisher pin_status = nh.advertise<sensor_msgs::Imu>("/mpu6050",10);

    wiringPiSetup();
    std::cout<<"Starting to read mpu6050 "<<"\n";
    // Open an I2C connection
    int fd = wiringPiI2CSetup(MPU6050_ADDRESS);
    checkRC(fd, "wiringPiI2CSetup");

    // Perform I2C work
    wiringPiI2CWriteReg8(fd, MPU6050_REG_PWR_MGMT_1, 0);  //not reset mpu and 8MHz internal oscilator

    while (ros::ok())
    {
    	uint8_t msb = wiringPiI2CReadReg8(fd, MPU6050_ACC_X);
    	uint8_t lsb = wiringPiI2CReadReg8(fd, MPU6050_ACC_X+1);
    	short accelX = msb << 8 | lsb;

    	msb = wiringPiI2CReadReg8(fd, MPU6050_ACC_Y);
    	lsb = wiringPiI2CReadReg8(fd, MPU6050_ACC_Y+1);
    	short accelY = msb << 8 | lsb;

    	msb = wiringPiI2CReadReg8(fd, MPU6050_ACC_Z);
    	lsb = wiringPiI2CReadReg8(fd, MPU6050_ACC_Z+1);
    	short accelZ = msb << 8 | lsb;

    	msb = wiringPiI2CReadReg8(fd, MPU6050_time);
    	lsb = wiringPiI2CReadReg8(fd, MPU6050_time+1);
    	short time = msb << 8 | lsb;

    	msb = wiringPiI2CReadReg8(fd, MPU6050_GYRO_X);
    	lsb = wiringPiI2CReadReg8(fd, MPU6050_GYRO_X+1);
    	short gyroX = msb << 8 | lsb;

    	msb = wiringPiI2CReadReg8(fd, MPU6050_GYRO_Y);
    	lsb = wiringPiI2CReadReg8(fd, MPU6050_GYRO_Y+1);
    	short gyroY = msb << 8 | lsb;

    	msb = wiringPiI2CReadReg8(fd, MPU6050_GYRO_Z);
    	lsb = wiringPiI2CReadReg8(fd, MPU6050_GYRO_Z+1);
    	short gyroZ = msb << 8 | lsb;

    	std::cout<<"accX: "<<accelX<<" accY: "<<accelY<<" accZ: "<<accelZ<<" gyrX: "<<gyroX<<" gyrY: "<<gyroY<<" gyrZ: "<<gyroZ<<" time: "<<time<<"\n";
    	sleep(1);
    	ros::spinOnce();

    }
    return 0;
}
