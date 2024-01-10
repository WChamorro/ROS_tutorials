######################################################
# Copyright (c) 2020 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU9250/MPU9265 board
# (MPU6050 - accel/gyro, AK8963 - mag) to verify its
# correct wiring to a Raspberry Pi and the functionality
# of the MPU9250_i2c.py library
#
#
######################################################
#

import time,sys
import numpy as np
sys.path.append("../")
t0 = time.time()
start_bool = False # if IMU start fails - stop calibration
while time.time()-t0<5:
    try: 
        from mpu9250_i2c import *
        start_bool = True
        break
    except:
        continue
imu_devs   = ["MAGNETOMETER"]
imu_labels = ["x-dir","y-dir","z-dir","x-dir-cal","y-dir-cal","z-dir-cal",'theta_cal','theta_no_cal']
imu_units  = ["uT","uT","uT","uT","uT","uT",'rad','rad']
#
#############################
# Main Loop to Test IMU
#############################
#
cal_coefs = np.load('/home/pi/mpu92-calibration/mag/mag_coefs.npy')
got_first_ref = False
BH_ref = 0
theta_ref = 0
theta = 0
theta_no_cal = 0
theta_no_cal_ref = 0

while True:
    #if start_bool==False: # make sure the IMU was started
    #    print("IMU not Started, Check Wiring") # check wiring if error
    #    break
    ##################################
    # Reading and Printing IMU values
    ##################################
    #
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
        
        theta = np.arctan2(my_cal,mx_cal) - theta_ref
        theta_no_cal = np.arctan2(my,mx) - theta_no_cal_ref


    except:
        continue 
    #
    ##################################
    # Reading and Printing IMU values
    ##################################
    #
    print(50*"-")
    for imu_ii,imu_val in enumerate([mx,my,mz,mx_cal,my_cal,mz_cal,theta,theta_no_cal]):
        if imu_ii%8==0:
            print(20*"_"+"\n"+imu_devs[int(imu_ii/8)]) # print sensor header
        #
        ###############
        # Print Data
        ###############
        #
        print("{0}: {1:3.2f} {2}".format(imu_labels[imu_ii%8],imu_val,imu_units[imu_ii]))
        
    time.sleep(1) # wait between prints
    
