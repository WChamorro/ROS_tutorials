#!/usr/bin/env python
import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import Float32

import numpy as np
sys.path.append("/home/pi/ROS_ws/src/magnetometer/src")
t0 = time.time()
start_bool = False # if IMU start fails - stop calibration
while time.time()-t0<5:
    try: 
        from mpu9250_i2c import *
        start_bool = True
        break
    except:
        continue
cal_coefs = np.load('/home/pi/ROS_ws/src/magnetometer/src/mag_coefs.npy')

def mag_publisher():
    pub = rospy.Publisher('theta', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    theta_ref = 0
    theta = 0
    got_first_ref=False
    print("Ready to receive")
    while not rospy.is_shutdown():
        
        #mx,my,mz = AK8963_conv()

        try:
            mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data

         
            #apply calibration
            mx_cal = mx - cal_coefs[0]
            my_cal = my - cal_coefs[1]
            mz_cal = mz - cal_coefs[2]
        
        
            if got_first_ref==False:  
                theta_ref = np.arctan2(my_cal,mx_cal)
                theta_no_cal_ref= np.arctan2(my,mx)
                got_first_ref=True
            if abs(mx_cal) <=1e-5:
            
                if my_cal > 0:
                    theta = np.pi / 2
                elif my_cal < 0:
                    theta = -np.pi / 2
                else:
            # Handle the case where both x and y are zero, or any other special case
                    theta = 0;
            else:
                theta = np.arctan2(my_cal,mx_cal) - theta_ref
        #print(theta)


        except:
            pass
         #   continue 
        float_msg = Float32()
        float_msg.data = theta
        pub.publish(float_msg)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        mag_publisher()
    except rospy.ROSInterruptException:
        pass