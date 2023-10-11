/**
 * @author  Dwindra Sulistyoutomo
 */
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>       // Required to sleep
#include <chrono>       // Required to sleep
#include <sys/stat.h>   // Required to check file existence

#include <MPU6050Pi.h>


MPU6050Pi mpu;
int16_t ax, ay, az, gx, gy, gz;
int ax_mean, ay_mean, az_mean, gx_mean, gy_mean, gz_mean;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

void GetAverage(int* ax_mean, int* ay_mean, int* az_mean, int* gx_mean, int* gy_mean, int* gz_mean) {
    int i;
    int buffer_size = 1000;

    int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    int ax_buffer=0, ay_buffer=0, az_buffer=0, gx_buffer=0, gy_buffer=0, gz_buffer=0;

    for (i=0; i < buffer_size+100; i++) {
        mpu.GetMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i >=100) {
            ax_buffer += ax;
            ay_buffer += ay;
            az_buffer += az;
            gx_buffer += gx;
            gy_buffer += gy;
            gz_buffer += gz;
        }
        std::this_thread::sleep_for (std::chrono::milliseconds(2));
    }

    *ax_mean = ax_buffer/buffer_size;
    *ay_mean = ay_buffer/buffer_size;
    *az_mean = az_buffer/buffer_size;
    *gx_mean = gx_buffer/buffer_size;
    *gy_mean = gy_buffer/buffer_size;
    *gz_mean = gz_buffer/buffer_size;
}

void Calibrate(int* ax_mean, int* ay_mean, int* az_mean, int* gx_mean, int* gy_mean, int* gz_mean){  
    // Threshold for allower error. Reduce to reduce error, but will be more difficult to converge 
    int accel_error_threshold = 8;
    int gyro_error_threshold = 1;

    ax_offset = - *ax_mean/8;
    ay_offset = - *ay_mean/8;
    az_offset = - *az_mean/8;
    gx_offset = - *gx_mean/4;
    gy_offset = - *gy_mean/4;
    gz_offset = - *gz_mean/4;

    // Handle gravity axis (default when calibrating on flat position = Z-axis)
    az_offset += mpu.GetAccelSensitivity() / 8;

    bool ready = false;
    int error;
    while (error>0) {
        error = 0;

        mpu.SetAccelXOffset(ax_offset);
        mpu.SetAccelYOffset(ay_offset);
        mpu.SetAccelZOffset(az_offset);

        mpu.SetGyroXOffset(gx_offset);
        mpu.SetGyroYOffset(gy_offset);
        mpu.SetGyroZOffset(gz_offset);

        GetAverage(&(*ax_mean), &(*ay_mean), &(*az_mean), &(*gx_mean), &(*gy_mean), &(*gz_mean));

        std::cout << "Mean: ";
        std::cout << *ax_mean << " " << *ay_mean << " " << *az_mean << " " << *gx_mean << " " << *gy_mean << " " << *gz_mean << std::endl;

        // Correcting accelerometer offset
        if (abs(*ax_mean) > accel_error_threshold) {
            ax_offset -= *ax_mean / accel_error_threshold;
            error ++;
        }
        if (abs(*ay_mean) > accel_error_threshold) {
            ay_offset -= *ay_mean / accel_error_threshold;
            error ++;
        }
        // if (abs(*az_mean) > accel_error_threshold) {
        //     az_offset -= *az_mean / accel_error_threshold;
        //     error ++;
        // }
        // Need to retain +1g for gravity
        if (abs(*az_mean - mpu.GetAccelSensitivity()) > accel_error_threshold) {
            az_offset -= *az_mean / accel_error_threshold;
            az_offset += mpu.GetAccelSensitivity() / accel_error_threshold;
            error ++;
        }

        // Correcting gyroscope offset
        if (abs(*gx_mean) > gyro_error_threshold) {
            gx_offset -= *gx_mean / (gyro_error_threshold+1);
            error ++;
        }
        if (abs(*gy_mean) > gyro_error_threshold) {
            gy_offset -= *gy_mean / (gyro_error_threshold+1);
            error ++;
        }
        if (abs(*gz_mean) > gyro_error_threshold) {
            gz_offset -= *gz_mean / (gyro_error_threshold+1);
            error ++;
        }
    }
}

int main (int argc, char **argv)
{


    ros::init(argc, argv, "imu offset removal");
    ros::NodeHandle nh;

    std::cout << "Calibrating MPU6050." << std::endl;
    std::cout << "Please do this calibration with MPU6050 on flat surface (gravity on Z-axis)." << std::endl;
    std::cout << std::endl;

    // Check whether previously created calibration file exists
    std::string file_name = "calibration.csv";
    struct stat buf;
    if (stat(file_name.c_str(), &buf) == 0) {
        char u_input;
        int MAX_RETRY = 5;
        int try_count = 0;
        do {
            std::cout << "There is previously created calibration file. Replace with new calibration (y/n)? ";
            std::cin >> u_input;
            try_count++;
        }
        while(!std::cin.fail() && try_count < MAX_RETRY && u_input!='y' && u_input!='Y' && u_input!='n' && u_input!='N');

        if (try_count >= MAX_RETRY)
            return 0;

        if ((u_input == 'n') || (u_input == 'N'))
            return 0;
    }
    else {
        std::cout << "No existing calibration file. Creating a new one." << std::endl;
    }

    // Get the average from sensor reading
    std::cout << "\nGet the average from sensors" << std::endl;
    GetAverage(&ax_mean, &ay_mean, &az_mean, &gx_mean, &gy_mean, &gz_mean);

    std::cout << "Mean: ";
    std::cout << ax_mean << " " << ay_mean << " " << az_mean << " " << gx_mean << " " << gy_mean << " " << gz_mean << std::endl;

    std::this_thread::sleep_for (std::chrono::seconds(1));

    // Calibration
    std::cout << "\nCalculating offset" << std::endl;
    Calibrate(&ax_mean, &ay_mean, &az_mean, &gx_mean, &gy_mean, &gz_mean);

    std::cout << "\nCalibration finished" << std::endl;
    std::cout << "Offset:" << std::endl;
    std::cout << ax_offset << " " << ay_offset << " " << az_offset << " " << gx_offset << " " << gy_offset << " " << gz_offset << std::endl;

    // Saving to external file
    std::ofstream ofile;
    ofile.open(file_name);
    ofile << "AccelXOffset" << "," << "AccelYOffset" << "," << "AccelZOffset" << ",";
    ofile << "GyroXOffset" << "," << "GyroYOffset" << "," << "GyroZOffset" << std::endl;
    ofile << ax_offset << "," << ay_offset << "," << az_offset << "," << gx_offset << "," << gy_offset << "," << gz_offset << std::endl;
    ofile.close();

    return 0;
}
