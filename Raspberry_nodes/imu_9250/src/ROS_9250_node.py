#!/usr/bin/env python
import RPi.GPIO as gpio
import time
import sys
import signal
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import numpy as np
sys.path.append("/home/pi/ROS_ws/src/imu_9250/src")
t0 = time.time()
start_bool = False # if IMU start fails - stop calibration
while time.time()-t0<5:
    try: 
        from mpu9250_i2c import *
        start_bool = True
        break
    except:
        continue
acc_coefs  = np.load('/home/pi/ROS_ws/src/imu_9250/src/acc_coefs.npy') #m b
gyro_coefs = np.load('/home/pi/ROS_ws/src/imu_9250/src/gyro_coefs.npy')

def imu_publisher():
    pub = rospy.Publisher('imu_9250', Imu, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    print("Ready to receive IMU")
    while not rospy.is_shutdown():
        
        #mx,my,mz = AK8963_conv()

        #try:
        ax,ay,az,wx,wy,wz = mpu6050_conv()

         
            #apply calibration
        ax_cal = ax * acc_coefs[0,0] + acc_coefs[0,1]
        ay_cal = ay * acc_coefs[1,0] + acc_coefs[1,1]
        az_cal = az * acc_coefs[2,0] + acc_coefs[2,1]
        
        wx_cal = wx -  gyro_coefs[0]
        wy_cal = wy -  gyro_coefs[1]
        wz_cal = wz -  gyro_coefs[2]
        
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'map'
        
        imu_msg.linear_acceleration.x = ax_cal*9.81
        imu_msg.linear_acceleration.y = ay_cal*9.81
        imu_msg.linear_acceleration.z = az_cal*9.81
        
        imu_msg.angular_velocity.x = wx_cal
        imu_msg.angular_velocity.y = wy_cal
        imu_msg.angular_velocity.z = wz_cal
        
        pub.publish(imu_msg)
            
  
        rate.sleep()
        

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass