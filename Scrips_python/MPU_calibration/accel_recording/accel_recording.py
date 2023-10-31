######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU6050 board,
# (accelerometer/gyroscope)
# and solves for calibration coefficients for the
# accelerometer
#
#
######################################################
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

from mpu6050 import mpu6050


def accel_cal():
    print("-" * 50)
    print("Accelerometer Calibration")
    mpu_offsets = [[], [], []]  # offset array to be printed
    axis_vec = ['z', 'y', 'x']  # axis labels
    cal_directions = ["upward", "downward"]  # direction for IMU cal
    cal_indices = [2, 1, 0]  # axis indices
    mpu_array = []
    for qq, ax_qq in enumerate(axis_vec):
        ax_offsets = [[], [], []]
        print("-" * 50)
        
        
        for direc_ii, direc in enumerate(cal_directions):
            input("-" * 8 + " Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -" +
                  ax_qq + "-axis pointed " + direc)
            
            [mpu.get_accel_data() for ii in range(0, cal_size)]  # clear buffer between readings
            
            count = 0
            while count < cal_size:
                try:
                    acceleration = mpu.get_accel_data()
                    ax = acceleration['x']/9.81
                    ay = acceleration['y']/9.81
                    az = acceleration['z']/9.81
                    
                    mpu_array.append([ax, ay, az])  # append to array
                    count = count + 1
                except:
                    continue
            
            
    print('Aquiring data for Calibration Complete')
    return mpu_array


if __name__ == '__main__':
    mpu = mpu6050(0x68)  # IMU (Accelerometer, Gyroscope)
    mpu.set_accel_range(mpu.ACCEL_RANGE_4G)
    
    cal_size = 600  # number of points to use for calibration

    # Accelerometer Gravity Calibration
    accel_labels = ['a_x', 'a_y', 'a_z']  # gyro labels for plots
    accel_data = accel_cal()  # grab accel coefficients
    
    
    with open("accel_recording.txt", "w") as file:
        for i in range(0,len(accel_data)):
            file.write(f"{accel_data[i][0]} {accel_data[i][1]} {accel_data[i][2]}\n")
    