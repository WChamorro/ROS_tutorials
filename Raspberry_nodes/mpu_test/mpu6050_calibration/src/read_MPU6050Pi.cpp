/**
 * @author  Dwindra Sulistyoutomo
 */
#include <ros/ros.h>
#include <iostream>	
#include <iomanip>
#include <fstream>
#include <chrono>       // Required to get time

#include <MPU6050Pi.h>

#define COMPLEMENTARY_FILTER_CONSTANT   0.98

float ComplementaryFilter(float angle, float angle_comp) {
    return COMPLEMENTARY_FILTER_CONSTANT * angle + (1- COMPLEMENTARY_FILTER_CONSTANT) * angle_comp;
}

int16_t* GetCalibrationData (char *filename) {
    int16_t *offsets = new int16_t[6];

    // Getting the file extension
    char *file_ext = strrchr(filename, '.');
    if (file_ext) {
        // offsets = GetCalibrationData(argv[1]);
        // Make sure the calibration file is CSV
        if (strcmp(file_ext, ".csv") == 0) {
            std::ifstream ifile (filename);
            std::string header;
            std::string data_str;
            if (ifile.is_open()){
                // Getting data line by line
                // 1st line should be Header
                std::getline(ifile,header);

                // Data
                std::getline(ifile,data_str);

                ifile.close();

                // Parse csv
                int i=0;
                size_t pos = 0;
                std::string token;
                std::string delimiter = ",";
                while ((pos = data_str.find(delimiter)) != std::string::npos) {
                    offsets[i] = stoi(data_str.substr(0, pos));
                    data_str.erase(0, pos + delimiter.length());
                    i++;
                }
                // Handle last data if not ending with comma
                if (data_str.length() > 0){
                    offsets[i] = stoi(data_str);
                }
            }
            return offsets;
        }
        else {
            std::cout << "Wrong input file" << std::endl;
            return 0;
        }
    }
    else {
        std::cout << "Wrong input file" << std::endl;
        return 0;
    }
}

int main (int argc, char **argv)
{


    ros::init(argc, argv, "gpio_read_mpu6050");
    ros::NodeHandle nh;

    // Connect to device with default setting
    MPU6050Pi mpu;

    if (argc > 1){
        int16_t *offsets;

        offsets = GetCalibrationData(argv[1]);
        if (offsets){
            mpu.SetOffset(offsets);
        }
        else {
            std::cout << "Proceed with default offsets" << std::endl;
        }
    }

    int16_t ax, ay, az, gx, gy, gz;
    float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

    // DMP variables
    int dev_status;
    uint8_t mpu_int_status;
    uint16_t packet_size;
    uint16_t fifo_count;
    uint8_t fifo_buffer[64];
    Quaternion q;
    Vector gravity;
    Vector v_gyro, v_accel;
    int16_t dmp_accel[3];
    int16_t dmp_gyro[3];
    float euler[3];
    float ypr[3]; // yaw pitch roll
    float ypr_comp[3];  // complementary yaw pitch roll
    float yaw, pitch, roll;
    float psi, theta, phi;

    auto t_now = std::chrono::steady_clock::now();
    auto t_last = std::chrono::steady_clock::now();

    //
    uint8_t i,j;

    std::cout << "----------------------------" << std::endl;
    std::cout << "1. Raw Sensor Data (int)" << std::endl;
    std::cout << "2. Float Sensor Data" << std::endl;
    std::cout << "3. Orientation (Complementary filter)" << std::endl;
    std::cout << "4. DMP (Yaw, Pitch, Roll)" << std::endl;
    std::cout << "5. DMP (Euler angles)" << std::endl;
    std::cout << "6. DMP (Quaternion)" << std::endl;
    std::cout << "7. DMP (Raw sensor data)" << std::endl;
    std::cout << "8. DMP (Float sensor data)" << std::endl;
    std::cout << "----------------------------" << std::endl;
    std::cout << "Choose mode to choose: ";
    
    int mode;
    std::cin >> mode;

    switch(mode){
        case 1:
            // ====== Raw Sensor Data ======
            std::cout << "Raw Sensor Data\n";
            std::cout << std::fixed << std::setprecision(6) << std::setfill(' ');
            std::cout << std::setw(9) << 'X' << std::setw(9) << 'Y' << std::setw(9) << 'Z';
            std::cout << std::setw(9) << 'X' << std::setw(9) << 'Y' << std::setw(9) << 'Z';
            std::cout << std::endl;

            // Publish in loop.
            while(1) {
                // Choose between two methods here:
                // 1. Get from one single function for both accelerometer and gyroscope
                mpu.GetMotion6(&ax, &ay, &az, &gx, &gy, &gz);

                // 2. Get separated gyroscope and accelerometer
                // Get gyroscope data.
                // mpu.GetGyro(&gx, &gy, &gz);
                // Get accelerometer values.
                // mpu.GetAccel(&ax, &ay, &az);

                std::cout << std::setw(9) << ax << std::setw(9) << ay << std::setw(9) << az;
                std::cout << std::setw(9) << gx << std::setw(9) << gy << std::setw(9) << gz;
                std::cout << "\r";
            }
            break;
        case 2:
            // ====== Float Sensor Data ======
            std::cout << "Float Sensor Data\n";
            std::cout << std::fixed << std::setprecision(6) << std::setfill(' ');
            std::cout << std::setw(10) << "X(g)" << std::setw(10) << "Y(g)" << std::setw(10) << "Z(g)";
            std::cout << std::setw(10) << "X(deg/s)" << std::setw(10) << "Y(deg/s)" << std::setw(10) << "Z(deg/s)";
            std::cout << std::endl;

            // Publish in loop.
            while(1) {
                // Get gyroscope data.
                mpu.GetGyroFloat(&gyro_x, &gyro_y, &gyro_z);
                // Get accelerometer values.
                mpu.GetAccelFloat(&accel_x, &accel_y, &accel_z);

                std::cout << std::setw(10) << accel_x << std::setw(10) << accel_y << std::setw(10) << accel_z;
                std::cout << std::setw(10) << gyro_x << std::setw(10) << gyro_y << std::setw(10) << gyro_z;
                std::cout << "\r";
            }
            break;
        
        case 3:
            // ====== Orientation using Complementary Filter ======
            std::cout << "Orientation\n";
            std::cout << std::fixed << std::setprecision(6) << std::setfill(' ');
            std::cout << std::setw(12) << "yaw(deg)" << std::setw(12) << "pitch(deg)" << std::setw(12) << "roll(deg)";
            std::cout << std::endl;

            // Publish in loop.
            // Start time
            t_last = std::chrono::steady_clock::now();
            
            // Initialize yaw pitch roll
            std::fill(std::begin(ypr), std::end(ypr), 0);
            std::fill(std::begin(ypr_comp), std::end(ypr_comp), 0);
            
            while(1) {
                // Get IMU data
                mpu.GetGyroFloat(&gyro_x, &gyro_y, &gyro_z);
                mpu.GetAccelFloat(&accel_x, &accel_y, &accel_z);

                // Collect the time
                t_now = std::chrono::steady_clock::now();
                std::chrono::duration<double> dt = t_now - t_last;
                t_last = t_now;

                // Calculate the angles
                // 1. Roll
                //    From gyro. gyro data is in deg/s.
                ypr[2] += gyro_x * dt.count();
                //    From accel. atan or atan2f return is in rad
                ypr_comp[2] = atan2f(accel_y, sqrt(accel_x*accel_x + accel_z*accel_z)) * 180/M_PI;
                // Complementary Filter
                ypr[2] = ComplementaryFilter(ypr[2], ypr_comp[2]);

                // 2. Pitch
                //    From gyro. gyro data is in deg/s.
                ypr[1] += gyro_y * dt.count();
                //    From accel. atan or atan2f return is in rad
                ypr_comp[1] = atan2f(accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180/M_PI;
                // Complementary Filter
                ypr[1] = ComplementaryFilter(ypr[1], ypr_comp[1]);

                // 3. Yaw
                //    From gyro. gyro data is in deg/s.
                ypr[0] += gyro_z * dt.count();
                
                std::cout << std::setw(12) << ypr[0] << std::setw(12) << ypr[1] << std::setw(12) << ypr[2];
                std::cout << "\r";
            }
            break;
        
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
            std::cout << "Initalize DMP..\n";
            dev_status = mpu.DMPInitalize();

            // make sure it worked (returns 0 if so)
            if (dev_status == 0) {
                // turn on the DMP, now that it's ready
                std::cout << "Enabling DMP...\n";
                mpu.SetDMPEnabled(true);

                // get expected DMP packet size for later comparison
                packet_size = mpu.DMPGetFIFOPacketSize();
                std::cout << "Packet size: " << (int)packet_size << std::endl;
            } else {
                // ERROR!
                // 1 = initial memory load failed
                // 2 = DMP configuration updates failed
                // (if it's going to break, usually the code will be 1)
                std::cout << "DMP Initialization failed.\n Error code: ";
                std::cout << dev_status << "\n";
                break;
            }

            // Publish in loop.
            std::cout << "DMP Data\n";
            std::cout << std::fixed << std::setprecision(6) << std::setfill(' ');
            if (mode==4)
                std::cout << std::setw(12) << "yaw(deg)" << std::setw(12) << "pitch(deg)" << std::setw(12) << "roll(deg)";
            else if (mode==5)
                std::cout << std::setw(12) << "psi(deg)" << std::setw(12) << "theta(deg)" << std::setw(12) << "phi(deg)";
            else if (mode==6)
                std::cout << std::setw(12) << "x" << std::setw(12) << "y" << std::setw(12) << "z" << std::setw(12) << "w";
            else if (mode==7) {
                std::cout << std::setw(10) << "X" << std::setw(10) << "Y" << std::setw(10) << "Z";
                std::cout << std::setw(10) << "X" << std::setw(10) << "Y" << std::setw(10) << "Z";
            }
            else if (mode==8) {
                std::cout << std::fixed << std::setprecision(3) << std::setfill(' ');
                std::cout << std::setw(10) << "X(g)" << std::setw(10) << "Y(g)" << std::setw(10) << "Z(g)";
                std::cout << std::setw(10) << "X(deg/s)" << std::setw(10) << "Y(deg/s)" << std::setw(10) << "Z(deg/s)";
            }
                std::cout << "\n";
            while(1) {
                // Clear the buffer so as we can get fresh values
                // The sensor is running a lot faster than our sample period
                mpu.ResetFIFO();
                
                // Waiting until FIFO full on MPU6050 side
                // std::cout << "Waiting until FIFO full...\n";
                while (fifo_count < packet_size) {
                    fifo_count = mpu.GetFIFOCount();
                    // std::cout << "FIFO count: " << fifo_count << "\n";
                }
                if (fifo_count == 1024) {
                    mpu.ResetFIFO();
                    std::cout << "FIFO overflow!\n";
                    break;
                }
                else{
                    // std::cout << "FIFO ready! FIFO count: " << fifo_count << "\n";

                    // Read FIFO buffer
                    while (fifo_count >= packet_size) {
                        mpu.GetFIFOBytes(fifo_buffer,packet_size);
                        fifo_count -= packet_size;

                        // Debug
                        // std::cout << "FIFO count remaining: " << fifo_count << "\n";
                        // std::cout << "FIFO buffer: \n";
                        // for (j=0; j<(sizeof(fifo_buffer)/sizeof(*fifo_buffer));) {
                        //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (uint16_t) fifo_buffer[j] << " ";
                        //     j++;
                        //     if (j%16 == 0) std::cout << std::endl;
                        // }
                    }

                    // Process FIFO buffer
                    mpu.DMPGetQuaternion(&q, fifo_buffer);

                    if (mode==4){
                        mpu.DMPGetGravity(&gravity, &q);
                        mpu.DMPGetYawPitchRoll(ypr, &q, &gravity);
                        yaw = ypr[0] * 180/M_PI;
                        pitch = ypr[1] * 180/M_PI;
                        roll = ypr[2] * 180/M_PI;

                        std::cout << std::setw(12) << yaw << std::setw(12) << pitch << std::setw(12) << roll;
                        std::cout << "\r";
                    }
                    else if (mode==5) {
                        mpu.DMPGetEuler(euler, &q);
                        psi = euler[0] * 180/M_PI;
                        theta = euler[1] * 180/M_PI;
                        phi = euler[2] * 180/M_PI;
                        
                        std::cout << std::setw(12) << psi << std::setw(12) << theta << std::setw(12) << phi;
                        std::cout << "\r";
                    }
                    else if (mode==6) {
                        std::cout << std::setw(12) << q.x << std::setw(12) << q.y << std::setw(12) << q.z << std::setw(12) << q.w;
                        std::cout << "\r";
                    }
                    else if (mode==7) {
                        // Get gyroscope data.
                        mpu.DMPGetGyro(dmp_gyro, fifo_buffer);
                        // Get accelerometer data.
                        mpu.DMPGetAccel(dmp_accel, fifo_buffer);

                        std::cout << std::setw(10) << dmp_accel[0] << std::setw(10) << dmp_accel[1] << std::setw(10) << dmp_accel[2];
                        std::cout << std::setw(10) << dmp_gyro[0] << std::setw(10) << dmp_gyro[1] << std::setw(10) << dmp_gyro[2];
                        std::cout << "\r";
                    }
                    else if (mode==8) {
                        // Get gyroscope data.
                        mpu.DMPGetGyro(&v_gyro, fifo_buffer);
                        // Get accelerometer data.
                        mpu.DMPGetAccel(&v_accel, fifo_buffer);

                        std::cout << std::setw(10) << v_accel.x << std::setw(10) << v_accel.y << std::setw(10) << v_accel.z;
                        std::cout << std::setw(10) << v_gyro.x << std::setw(10) << v_gyro.y << std::setw(10) << v_gyro.z;
                        std::cout << "\r";
                    }
                }
            }


            break;

    }
    return 0;
}
