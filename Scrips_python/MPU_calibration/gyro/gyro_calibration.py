######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU6050 board
# (accelerometer/gyroscope)
# and solves for calibration coefficients for the
# gyroscope
#
#
######################################################
import numpy as np
import matplotlib.pyplot as plt

from mpu6050 import mpu6050

cal_size = 1000  # points to use for calibration
def gyro_cal():
    print("-" * 50)
    print('Gyro Calibrating - Keep the IMU Steady')
    [mpu.get_gyro_data() for ii in range(0, cal_size)]  # clear buffer before calibration
    mpu_array = []
    gyro_offsets = [0.0, 0.0, 0.0]
    while True:
        try:
            gyro = mpu.get_gyro_data()  # get gyro values
        except:
            continue

        wx = gyro['x']
        wy = gyro['y']
        wz = gyro['z']
        mpu_array.append([wx, wy, wz])  # gyro vector append
        if np.shape(mpu_array)[0] == cal_size:
            a= np.array(mpu_array)
            
            for qq in range(0, 3):
                gyro_offsets[qq] = np.mean(np.array(mpu_array)[:, qq])  # average
            break
    print('Gyro Calibration Complete: ',str(gyro_offsets[0]),' ' ,str(gyro_offsets[1]),' ',str(gyro_offsets[2]))

    return gyro_offsets


if __name__ == '__main__':
    mpu = mpu6050(0x68)  # IMU (Accelerometer, Gyroscope)
    mpu.set_gyro_range(mpu.GYRO_RANGE_500DEG)

    

    # Gyroscope Offset Calculation
    gyro_labels = ['w_x', 'w_y', 'w_z']  # gyro labels for plots
    gyro_offsets = gyro_cal()  # calculate gyro offsets

    # record new data
    data = np.array([mpu.get_gyro_data() for ii in range(0, cal_size)])

    # plot with and without offsets
    plt.style.use('ggplot')
    fig, axs = plt.subplots(2, 1, figsize=(12, 9))
    axis_labels = ['x', 'y', 'z']
    with open("gyro_offset.txt", "w") as file:
        file.write(f"x: {gyro_offsets[0]}\n")
        file.write(f"y: {gyro_offsets[1]}\n")
        file.write(f"z: {gyro_offsets[2]}\n")
        
    

    for ii in range(0, 3):
        raw_val = [data[j][axis_labels[ii]] for j in range(0,500)]

        axs[0].plot(raw_val,
                    label='${}$, Uncalibrated'.format(gyro_labels[ii]))
        axs[1].plot(raw_val - gyro_offsets[ii],
                    label='${}$, Calibrated'.format(gyro_labels[ii]))
    axs[0].legend(fontsize=14)
    axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$w_{x, y, z}$ [$^{\circ}/s$]', fontsize=18)  # noqa: W605
    axs[1].set_ylabel('$w_{x, y, z}$ [$^{\circ}/s$]', fontsize=18)  # noqa: W605
    axs[1].set_xlabel('Sample', fontsize=18)
    axs[0].set_ylim([-2, 2])
    axs[1].set_ylim([-2, 2])
    axs[0].set_title('Gyroscope Calibration Offset Correction', fontsize=22)
    fig.savefig('gyro_calibration_output.png', dpi=300,
                bbox_inches='tight', facecolor='#FCFCFC')
    fig.show()
