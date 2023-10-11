/**
 * @author  Dwindra Sulistyoutomo
 * 
 * The library is written by referring to I2Cdev library code writtern by Jeff Rowberg
 * https://github.com/jrowberg/i2cdevlib
 * 
 * The class is written specifically for Raspberry Pi.
 */

#ifndef _MPU6050PI_H
#define _MPU6050PI_H

#include <cstring>      // Required to verify memory block using memcmp
#include <thread>       // Required to sleep
#include <chrono>       // Required to sleep

#include "I2CPi.h"      // Main I2C Communication Library
#include "math_3d.h"    // Classes library for DMP functions

#define G_FORCE 9.80665

#define MPU6050_ADDRESS         0x68 // Default I2C address for MPU6050

/**
 * Register Map
 */
// Undocumented DMP Register
#define XG_OFFS_TC              0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define YG_OFFS_TC              0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define ZG_OFFS_TC              0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define X_FINE_GAIN             0x03 //[7:0] X_FINE_GAIN
#define Y_FINE_GAIN             0x04 //[7:0] Y_FINE_GAIN
#define Z_FINE_GAIN             0x05 //[7:0] Z_FINE_GAIN

// Self test
#define SELF_TEST_X             0x0D
#define SELF_TEST_Y             0x0E
#define SELF_TEST_Z             0x0F
#define SELF_TEST_A             0x10

// Offset
#define XA_OFFS_H               0x06
#define XA_OFFS_L               0x07
#define YA_OFFS_H               0x08
#define YA_OFFS_L               0x09
#define ZA_OFFS_H               0x0A
#define ZA_OFFS_L               0x0B
#define XG_OFFSET_H             0x13
#define XG_OFFSET_L             0x14
#define YG_OFFSET_H             0x15
#define YG_OFFSET_L             0x16
#define ZG_OFFSET_H             0x17
#define ZG_OFFSET_L             0x18

// Measurement Config
#define SMPLRT_DIV              0x19    // Sample rate divider
#define CONFIG                  0x1A
#define GYRO_CONFIG             0x1B
#define ACCEL_CONFIG            0x1C

// Undocumented
#define FF_THR                  0x1D
#define FF_DUR                  0x1E
#define MOT_THR                 0x1F
#define MOT_DUR                 0x20
#define ZRMOT_THR               0x21
#define ZRMOT_DUR               0x22

// FIFO config
#define FIFO_EN                 0x23

// I2C Master Slave Control
#define I2C_MST_CTRL            0x24
#define I2C_SLV0_ADDR           0x25
#define I2C_SLV0_REG            0x26
#define I2C_SLV0_CTRL           0x27
#define I2C_SLV1_ADDR           0x28
#define I2C_SLV1_REG            0x29
#define I2C_SLV1_CTRL           0x2A
#define I2C_SLV2_ADDR           0x2B
#define I2C_SLV2_REG            0x2D
#define I2C_SLV2_CTRL           0x2C
#define I2C_SLV3_ADDR           0x2E
#define I2C_SLV3_REG            0x2F
#define I2C_SLV3_CTRL           0x30
#define I2C_SLV4_ADDR           0x31
#define I2C_SLV4_REG            0x32
#define I2C_SLV4_DO             0x33
#define I2C_SLV4_CTRL           0x34
#define I2C_SLV4_DI             0x35
#define I2C_MST_STATUS          0x36

// Interrupt
#define INT_PIN_CFG             0x37
#define INT_ENABLE              0x38
#define DMP_INT_STATUS          0x39
#define INT_STATUS              0x3A

// Measurement Output
#define ACCEL_XOUT_H            0x3B
#define ACCEL_XOUT_L            0x3C
#define ACCEL_YOUT_H            0x3D
#define ACCEL_YOUT_L            0x3E
#define ACCEL_ZOUT_H            0x3F
#define ACCEL_ZOUT_L            0x40
#define TEMP_OUT_H              0x41
#define TEMP_OUT_L              0x42
#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44
#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46
#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48

// External Sensor Data
#define EXT_SENS_DATA_00        0x49
#define EXT_SENS_DATA_01        0x4A
#define EXT_SENS_DATA_02        0x4B
#define EXT_SENS_DATA_03        0x4C
#define EXT_SENS_DATA_04        0x4D
#define EXT_SENS_DATA_05        0x4E
#define EXT_SENS_DATA_06        0x4F
#define EXT_SENS_DATA_07        0x50
#define EXT_SENS_DATA_08        0x51
#define EXT_SENS_DATA_09        0x52
#define EXT_SENS_DATA_10        0x53
#define EXT_SENS_DATA_11        0x54
#define EXT_SENS_DATA_12        0x55
#define EXT_SENS_DATA_13        0x56
#define EXT_SENS_DATA_14        0x57
#define EXT_SENS_DATA_15        0x58
#define EXT_SENS_DATA_16        0x59
#define EXT_SENS_DATA_17        0x5A
#define EXT_SENS_DATA_18        0x5B
#define EXT_SENS_DATA_19        0x5C
#define EXT_SENS_DATA_20        0x5D
#define EXT_SENS_DATA_21        0x5E
#define EXT_SENS_DATA_22        0x5F
#define EXT_SENS_DATA_23        0x60

// I2C Slave Data Output
#define I2C_SLV0_DO             0x63
#define I2C_SLV1_DO             0x64
#define I2C_SLV2_DO             0x65
#define I2C_SLV3_DO             0x66

// Control config
#define I2C_MST_DELAY_CTRL      0x67
#define SIGNAL_PATH_RESET       0x68
#define ACCEL_INTEL_CTRL        0x69
#define USER_CTRL               0x6A

#define PWR_MGMT_1              0x6B
#define PWR_MGMT_2              0x6C

// Undocumented DMP register
#define BANK_SEL                0x6D
#define MEM_START_ADDR          0x6E
#define MEM_R_W                 0x6F
#define DMP_CFG_1               0x70
#define DMP_CFG_2               0x71

// FIFO Data
#define FIFO_COUNT_H            0x72
#define FIFO_COUNT_L            0x73
#define FIFO_R_W                0x74

// Identifier
#define WHO_AM_I                0x75

/**
 * Bit selector
 */
// CONFIG
#define CONFIG_DLPF_CFG_START       0
#define CONFIG_DLPF_CFG_LENGTH      3
#define EXT_SYNC_SET_START          3
#define EXT_SYNC_SET_LENGTH         3            

// GYRO_CONFIG
#define FS_SEL_START                3
#define FS_SEL_LENGTH               2

// ACCEL_CONFIG
#define AFS_SEL_START               3
#define AFS_SEL_LENGTH              2

// INT_ENABLE
#define DATA_RDY_INT_BIT            0
#define DMP_INT_BIT                 1
#define PLL_RDY_INT_BIT             2
#define I2C_MST_INT_BIT             3
#define FIFO_OFLOW_INT_BIT          4
#define ZMOT_INT_BIT                5
#define MOT_INT_BIT                 6
#define FF_INT_BIT                  7

// USER_CNTRL
#define USERCTRL_SIG_COND_RESET_BIT 0
#define USERCTRL_I2C_MST_RESET_BIT  1
#define USERCTRL_FIFO_RESET_BIT     2
#define USERCTRL_DMP_RESET_BIT      3
#define USERCTRL_I2C_IF_DIS_BIT     4
#define USERCTRL_I2C_MST_EN_BIT     5
#define USERCTRL_FIFO_EN_BIT        6
#define USERCTRL_DMP_EN_BIT         7

// PWR_MGMT_1
#define PWR_MGMT_1_CLKSEL_START     0
#define PWR_MGMT_1_CLKSEL_LENGTH    3
#define PWR_MGMT_1_TEMP_DIS_BIT     3
#define PWR_MGMT_1_CYCLE_BIT        5
#define PWR_MGMT_1_SLEEP_BIT        6
#define PWR_MGMT_1_RESET_BIT        7

// Undocumented DMP Register
#define TC_OTP_BNK_VLD_BIT          0
#define TC_OFFSET_LENGTH            6
#define TC_OFFSET_BIT               6
#define TC_PWR_MODE_BIT             7

/**
 * Parameter Settings
 */
// GYRO_CONFIG
/* 
 * Gyroscope sensitivity 
 * | FS_SEL  | Full Scale Range  |   LSB Sensitivity   |
 * |    0    |       250 deg/s   |     131 LSB/deg/s   |
 * |    1    |       500 deg/s   |    65.5 LSB/deg/s   |
 * |    2    |      1000 deg/s   |    32.8 LSB/deg/s   |
 * |    3    |      2000 deg/s   |    16.4 LSB/deg/s   |
 */
#define FS_SEL_250              0x00
#define FS_SEL_500              0x01
#define FS_SEL_1000             0x02
#define FS_SEL_2000             0x03

#define GYRO_LSB_250            131.0
#define GYRO_LSB_500            65.5
#define GYRO_LSB_1000           32.8
#define GYRO_LSB_2000           16.4

// ACCEL_CONFIG
/* 
 * Accelerometer sensitivity
 * | AFS_SEL  | Full Scale Range |   LSB Sensitivity  |
 * |    0     |      2 g         |     16384 LSB/g    |
 * |    1     |      4 g         |      8192 LSB/g    |
 * |    2     |      8 g         |      4096 LSB/g    |
 * |    3     |     16 g         |      2048 LSB/g    |
 */
#define AFS_SEL_2               0x00
#define AFS_SEL_4               0x01
#define AFS_SEL_8               0x02
#define AFS_SEL_16              0x03

#define ACCEL_LSB_2             16384.0
#define ACCEL_LSB_4             8192.0
#define ACCEL_LSB_8             4096.0
#define ACCEL_LSB_16            2048.0

#define EXT_SYNC_DISABLED       0x00
#define EXT_SYNC_TEMP_OUT_L     0x01
#define EXT_SYNC_GYRO_XOUT_L    0x02
#define EXT_SYNC_GYRO_YOUT_L    0x03
#define EXT_SYNC_GYRO_ZOUT_L    0x04
#define EXT_SYNC_ACCEL_XOUT_L   0x05
#define EXT_SYNC_ACCEL_YOUT_L   0x06
#define EXT_SYNC_ACCEL_ZOUT_L   0x07

#define DLPF_BW_260             0x00
#define DLPF_BW_256             0x00
#define DLPF_BW_184             0x01
#define DLPF_BW_188             0x01
#define DLPF_BW_94              0x02
#define DLPF_BW_98              0x02
#define DLPF_BW_44              0x03
#define DLPF_BW_42              0x03
#define DLPF_BW_21              0x04
#define DLPF_BW_20              0x04
#define DLPF_BW_10              0x05
#define DLPF_BW_5               0x06

#define CLOCK_INTERNAL          0x00
#define CLOCK_PLL_XGYRO         0x01
#define CLOCK_PLL_YGYRO         0x02
#define CLOCK_PLL_ZGYRO         0x03
#define CLOCK_PLL_EXT32K        0x04
#define CLOCK_PLL_EXT19M        0x05
#define CLOCK_KEEP_RESET        0x07

#define SLEEP_DISABLED          0
#define SLEEP_ENABLED           1

/**
 *  DMP define and constants
 */
#define DMP_CODE_SIZE           1929    // dmpMemory[]
#define DMP_CONFIG_SIZE         192     // dmpConfig[]
#define DMP_UPDATES_SIZE        47      // dmpUpdates[]

#define DMP_PACKET_SIZE         42
/**
 * Default MotionApps v2.0 42-byte FIFO packet structure
 *
 * |    QUAT W     |    QUAT X     |    QUAT Y     |     QUAT Z    |
 * | 0   1   2   3 | 4   5   6   7 | 8   9  10  11 |12  13  14  15 | 
 *
 * |    GYRO X     |    GYRO Y     |    GYRO Z     |
 * |16  17  18  19 |20  21  22  23 |24  25  26  27 | 
 * 
 * |     ACC X     |     ACC Y     |     ACC Z     |       |
 * |28  29  30  31 |32  33  34  35 |36  37  38  39 |40  41 | 
 */

#define DMP_MEMORY_BANKS        8
#define DMP_MEMORY_BANK_SIZE    256
#define DMP_MEMORY_CHUNK_SIZE   16

#define DMP_FIFO_RATE_DIVISOR   0x01 

/**
 * Static class for MPU6050 Motion Apps
 */ 
class DMP {
    public:
        static const unsigned char memory[];
};

/**
 * Class for MPU6050 sensor reading using Raspberry Pi GPIO.
 */ 
class MPU6050Pi {
    private:
        uint8_t I2C_address_;           // I2C address for the device
        int fd_;                        // I2C device file handler for the class

        float gyro_sensitivity_;        // Gyroscope sensitivity setting
        float accel_sensitivity_;       // Accelerometer sensivity settting
        float accel_scale_range_;       // Accelerometer full scale range in g
        float gyro_rate_;               // Gyroscope Output Rate setting
        float sample_rate_;             // Sample Rate setting

        // DMP
        uint8_t *dmp_packet_buffer_;    // DMP Packet buffer
        uint16_t dmp_packet_size_;      // DMP Packet size setting

    public:
        /** ============================================================
         *      CONSTRUCTOR
         *  ============================================================
         */
        /**
         * Constructor of the class using all default settings
         */
        MPU6050Pi();
        /**
         * Constructor of the class
         * 
         * @param offsets {int16_t*} Array of offsets: {AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ}
         */
        MPU6050Pi(int16_t *offsets);

        /** ============================================================
         *      CONFIGURATION
         *  ============================================================
         */
        /**
         * Set Sample Rate Divider (SMPLRT_DIV) for Gyroscope Output Rate.
         * 
         * @param rate {uint8_t} Sample rate divider 
         */
        void SetSampleRateDivider(uint8_t rate);
        /**
         * Return current sample rate
         */
        float GetSampleRate();

        // ---------- CONFIG registers ----------
        /**
         * Set the external frame synchronization settings
         * EXT_SYNC_SET |   FSYNC Bit Location
         *      0           Input disabled
         *      1           TEMP_OUT_L  
         *      2           GYRO_XOUT_L 
         *      3           GYRO_YOUT_L 
         *      4           GYRO_ZOUT_L 
         *      5           ACCEL_XOUT_L
         *      6           ACCEL_YOUT_L
         *      7           ACCEL_ZOUT_L
         * 
         * @param sync {uint8_t} 
         */
        void SetExternalFrameSync(uint8_t sync);
        /**
         * Set Digital Low Pass Filter (DLPF) mode. See register map datasheet for DLPF_CFG.
         * 
         * @param mode {uint8_t} Value for DLPF_CFG
         */
        void SetDLPFMode(uint8_t mode);
        /**
         * Return the current DLPF_CFG from CONFIG register
         */
        uint8_t GetDLPFMode();

        // ---------- GYRO_CONFIG registers ----------
        /**
         * Set Gyroscope FullScale Range
         *  FS_SEL  | Full Scale Range  |   LSB Sensitivity
         *      0           250 deg/s           131 LSB/deg/s
         *      1           500 deg/s          65.5 LSB/deg/s
         *      2          1000 deg/s          32.8 LSB/deg/s
         *      3          2000 deg/s          16.4 LSB/deg/s
         * 
         * @param range {uint8_t} Value for FS_SEL. Default to 0.
         */
        void SetFullScaleGyroRange(uint8_t range=FS_SEL_250);
        /**
         * Return current gyroscope sensitivity per Least Significant Bit (LSB)
         */
        float GetGyroSensitivity();

        // ---------- ACCEL_CONFIG registers ----------
        /**
         * Set Accelerometer FullScale Range
         *  AFS_SEL  | Full Scale Range |   LSB Sensitivity
         *      0           2 g              16384 LSB/g
         *      1           4 g               8192 LSB/g
         *      2           8 g               4096 LSB/g
         *      3          16 g               2048 LSB/g
         * 
         * @param range {uint8_t} Value for AFS_SEL. Default to 0.
         */
        void SetFullScaleAccelRange(uint8_t range=AFS_SEL_2);
        /**
         * Return current accerelerometer sensitivity per Least Significant Bit (LSB)
         */
        float GetAccelSensitivity();

        // ---------- FF* registers ----------
        /**
         * Set the acceleration threshold for Free-fall detection in mg to FF_THR register
         * 
         * @param threshold {uint8_t} acceleration threshold in 1mg steps
         */
        void SetFreefallDetectionThreshold(uint8_t threshold);
        /**
         * Return the current acceleration threshold for Free-fall detection in mg
         */
        uint8_t GetFreefallDetectionThreshold();

        /**
         * Set the duration threshold of counter for Free-fall detection in ms to FF_DUR register
         * 
         * @param duration {uint8_t} duration threshold in 1ms steps
         */
        void SetFreefallDetectionDuration(uint8_t duration);
        /**
         * Return the current duration threshold for Free-fall detection in mg
         */
        uint8_t GetFreefallDetectionDuration();

        // ---------- MOT* registers ----------
        /**
         * Set the acceleration threshold for motion detection in mg to MOT_THR register
         * 
         * @param threshold {uint8_t} acceleration threshold in 1mg steps
         */
        void SetMotionDetectionThreshold(uint8_t threshold);
        /**
         * Return the current acceleration threshold for motion detection in mg
         */
        uint8_t GetMotionDetectionThreshold();

        /**
         * Set the duration threshold of counter for motion detection in ms to MOT_DUR register
         * 
         * @param duration {uint8_t} duration threshold in 1ms steps
         */
        void SetMotionDetectionDuration(uint8_t duration);
        /**
         * Return the current duration threshold for motion detection in mg
         */
        uint8_t GetMotionDetectionDuration();

        // ---------- ZRMOT* registers ----------
        /**
         * Set the acceleration threshold for Zero motion detection in mg to ZRMOT_THR register
         * 
         * @param threshold {uint8_t} acceleration threshold in 1mg steps
         */
        void SetZeroMotionDetectionThreshold(uint8_t threshold);
        /**
         * Return the current acceleration threshold for Zero motion detection in mg
         */
        uint8_t GetZeroMotionDetectionThreshold();

        /**
         * Set the duration threshold of counter for Zero motion detection in ms to ZRMOT_DUR register
         * 
         * @param duration {uint8_t} duration threshold in 1ms steps
         */
        void SetZeroMotionDetectionDuration(uint8_t duration);
        /**
         * Return the current duration threshold for Zero motion detection in mg
         */
        uint8_t GetZeroMotionDetectionDuration();

        // ---------- I2C_SLV* registers ----------
        /**
         * Set address for specified slave
         * 
         * @param num       {uint8_t} Slave number (0-3)
         * @param address   {uint8_t} address to assign
         */
        void SetSlaveAddress(uint8_t num, uint8_t address);
        /**
         * Return the current address for specified slave
         * 
         * @param num       {uint8_t} Slave number (0-3)
         */
        uint8_t GetSlaveAddress(uint8_t num);

        // ---------- INT_ENABLE registers ----------
        /**
         * Set full interrupt enabled status on device to INT_ENABLE
         *  Bit |       Flag
         *   0      DATA_RDY_INT
         *   1      DMP_INT_BIT    
         *   2      PLL_RDY_INT_BIT
         *   3      I2C_MST_INT
         *   4      FIFO_OFLOW_INT
         *   5      ZMOT_INT_BIT
         *   6      MOT_INT_BIT 
         *   7      FF_INT_BIT   
         * 
         * @param enabled {uint8_t} for each bit, 0:disable, 1:enable
         */
        void SetIntEnabled(uint8_t enabled);
        /**
         * Return full interrupt enabled status
         */
        uint8_t GetIntEnabled();

        // ---------- INT_STATUS registers ----------
        /**
         * Return full set of interrupt status bits from INT_STATUS
         * The bits clear to 0 after read.
         * The bit order is the same with INT_ENABLE
         */
        uint8_t GetIntStatus();

        // ---------- *OFFS* registers ----------
        /**
         * Set all offsets for accelerometer and gyroscope
         * 
         * @param offset {int16_t*} Array of offsets: {AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ}
         */
        void SetOffset(int16_t *offset);
        /**
         * Set all offsets for accelerometer
         * 
         * @param offset {int16_t*} Array of offsets: {AccelX, AccelY, AccelZ}
         */
        void SetAccelOffset(int16_t *offset);
        /**
         * Set X-axis offsets of accelerometer
         * 
         * @param offset {int16_t} offset
         */
        void SetAccelXOffset(int16_t offset);
        /**
         * Set Y-axis offsets of accelerometer
         * 
         * @param offset {int16_t} offset
         */
        void SetAccelYOffset(int16_t offset);
        /**
         * Set Z-axis offsets of accelerometer
         * 
         * @param offset {int16_t} offset
         */
        void SetAccelZOffset(int16_t offset);

        /**
         * Set all offsets for gyroscope
         * 
         * @param offset {int16_t*} Array of offsets: {GyroX, GyroY, GyroZ}
         */
        void SetGyroOffset(int16_t *offset);
        /**
         * Set X-axis offsets of gyroscope
         * 
         * @param offset {int16_t} offset
         */
        void SetGyroXOffset(int16_t offset);
        /**
         * Set Y-axis offsets of gyroscope
         * 
         * @param offset {int16_t} offset
         */
        void SetGyroYOffset(int16_t offset);
        /**
         * Set Z-axis offsets of gyroscope
         * 
         * @param offset {int16_t} offset
         */
        void SetGyroZOffset(int16_t offset);

        // ---------- USER_CTRL registers ----------
        /**
         * Set FIFO enabl status 
         * 
         * @param enabled {bool} 0:disabled, 1:enabled
         */
        void SetFIFOEnabled(bool enabled);
        /**
         * Enable I2C Master mode
         * 
         * @param enabled {bool}
         */
        void SetI2CMasterModeEnabled(bool enabled);
        /**
         * Reset FIFO buffer
         */
        void ResetFIFO();
        /**
         * Reset I2C master
         */
        void ResetI2CMaster();
        /**
         * Reset the signal paths for all sensors, and clear sensor registers
         */
        void ResetSensors();

        // ---------- PWR_MGMT_1 registers ----------
        /**
         * Set Clock source. Upon pwer up, by default CLK_SEL=0 for internal oscillator.
         * CLK_SEL  |   Clock Source
        *     0     |   Internal oscillator
        *     1     |   PLL with X Gyro reference
        *     2     |   PLL with Y Gyro reference
        *     3     |   PLL with Z Gyro reference
        *     4     |   PLL with external 32.768kHz reference
        *     5     |   PLL with external 19.2MHz reference
        *     6     |   Reserved
        *     7     |   Stops the clock and keeps the timing generator in reset
         * 
         * @param offset {int16_t} offset
         */
        void SetClockSource(uint8_t clk_sel);

        /**
         * Set Sleep mode
         * 
         * @param mode {uint8_t} 0: disabled, 1: sleep mode
         */
        void SetSleepMode(uint8_t mode);

        /**
         * Reset Device using Power Management
         */
        void ResetDevice();

        // ---------- FIFO_COUNT_* registers ----------
        /**
         * Return the number of bytes stored in the FIFO buffer
         */
        uint16_t GetFIFOCount();

        // ---------- FIFO_R_W registers ----------
        /** 
         * Write byte to the FIFO buffer.
         * 
         * @param data {uint8_t} data to write
         */
        void SetFIFOByte(uint8_t data);

        /** 
         * Read one byte from the FIFO buffer.
         */
        uint8_t GetFIFOByte();
        /** 
         * Read bytes from the FIFO buffer.
         * 
         * @param data {uint8_t}    data read from the buffer
         * @param length {uint8_t}  number of bytes to read
         */
        void GetFIFOBytes(uint8_t *data, uint8_t length);

        /** ============================================================
         *      DATA
         *  ============================================================
         */
        // ---------- *OUT* registers ----------
        /**
         * Return 16bit signed raw sensor data for accelerometer and gyroscope
         * 
         * @param ax {int16_t} accelerometer X-axis
         * @param ay {int16_t} accelerometer Y-axis
         * @param az {int16_t} accelerometer Z-axis
         * @param gx {int16_t} gyroscope X-axis
         * @param gy {int16_t} gyroscope Y-axis
         * @param gz {int16_t} gyroscope Z-axis
         */
        void GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

        /**
         * Return 16bit signed raw sensor data for accelerometer
         * 
         * @param ax {int16_t} accelerometer X-axis
         * @param ay {int16_t} accelerometer Y-axis
         * @param az {int16_t} accelerometer Z-axis
         */
        void GetAccel(int16_t* ax, int16_t* ay, int16_t* az);
        /**
         * Return 16bit signed raw sensor data for accelerometer X-axis
         * 
         * @param ax {int16_t} data
         */
        void GetAccelX(int16_t* ax);
        /**
         * Return 16bit signed raw sensor data for accelerometer Y-axis
         * 
         * @param ay {int16_t} data
         */
        void GetAccelY(int16_t* ay);
        /**
         * Return 16bit signed raw sensor data for accelerometer Z-axis
         * 
         * @param az {int16_t} data
         */
        void GetAccelZ(int16_t* az);
        /**
         * Return float-converted sensor data for accelerometer in g (gravity)
         * 
         * @param ax {float} accelerometer X-axis
         * @param ay {float} accelerometer Y-axis
         * @param az {float} accelerometer Z-axis
         */
        void GetAccelFloat(float* ax, float* ay, float* az);

        /**
         * Return 16bit signed raw sensor data for gyroscope
         * 
         * @param gx {int16_t} gyroscope X-axis
         * @param gy {int16_t} gyroscope Y-axis
         * @param gz {int16_t} gyroscope Z-axis
         */
        void GetGyro(int16_t* gx, int16_t* gy, int16_t* gz);
        /**
         * Return float-converted sensor data for gyroscope in deg/s
         * 
         * @param gx {float} gyroscope X-axis
         * @param gy {float} gyroscope Y-axis
         * @param gz {float} gyroscope Z-axis
         */
        void GetGyroFloat(float* gx, float* gy, float* gz);


        /** ============================================================
         *      UNDOCUMENTED DMP METHODS
         *  ============================================================
         */
        /**
         * Enable DMP
         * 
         * @param enabled {bool} 0:disabled, 1:enabled
         */
        void SetDMPEnabled(bool enabled);
        /**
         * Reset DMP
         */
        void ResetDMP();

        // ---------- XG_OFFS_TC registers ----------
        /**
         * Set OTP Bank to be valid
         * 
         * @param enabled {bool} 0: disabled/invalid, 1:enabled/valid
         */
        void SetOTPBankValid(bool enabled);
        /**
         * Return the validity of OTP Bank
         */
        uint8_t GetOTPBankValid();
        
        // ---------- BANK_SEL registers ----------
        /**
         * Set Memory Bank Selection.
         * This register is not documented.
         * 
         * @param bank              {uint8_t}   register bank selection
         * @param prefetch_enabled  {bool}      
         * @param user_bank         {bool}      
         */
        void SetMemoryBank(uint8_t bank, bool prefetch_enabled=false, bool user_bank=false);

        // ---------- MEM_START_ADDR registers ----------
        /**
         * Set starting address for memory
         * 
         * @param address {uint8_t} address
         */
        void SetMemoryStartAddress(uint8_t address);

        // ---------- MEM_R_W registers ----------
        /**
         * Write MEM_R_W register
         * 
         * @param data {uint8_t} data to write
         */
        void WriteMemoryByte(uint8_t data);
        /**
         * Return the data from MEM_R_W register
         */
        uint8_t ReadMemoryByte();

        /**
         * Write bytes of data to MEM_R_W register
         * 
         * @param data {uint8_t}        array of data to write
         * @param data_size {uint16_t}  size of the data in number of bytes
         * @param bank {uint8_t}        register bank to use
         * @param address {uint8_t}     starting address for memory
         * @param verify {bool}         flag to verify if writing is successful or not
         * @param use_progmem {bool}    flag to use program memory (flash)
         */
        bool WriteMemoryBlock(const uint8_t *data, uint16_t data_size, uint8_t bank=0, uint8_t address=0, bool verify=true, bool use_progmem=false);
        /**
         * Write bytes of data to MEM_R_W register using program memory by default
         * 
         * @param data {uint8_t}        array of data to write
         * @param data_size {uint16_t}  size of the data in number of bytes
         * @param bank {uint8_t}        register bank to use
         * @param address {uint8_t}     starting address for memory
         * @param verify {bool}         flag to verify if writing is successful or not
         */
        bool WriteProgMemoryBlock(const uint8_t *data, uint16_t data_size, uint8_t bank=0, uint8_t address=0, bool verify=true);

        /**
         * Read bytes of data from MEM_R_W register
         * 
         * @param data {uint8_t}        array of data read from register
         * @param data_size {uint16_t}  size of the data in number of bytes
         * @param bank {uint8_t}        register bank to use
         * @param address {uint8_t}     starting address for memory
         */
        void ReadMemoryBlock(uint8_t *data, uint16_t data_size, uint8_t bank, uint8_t address);

        // ---------- DMP_CFG* registers ----------
        /**
         * Write config to DMP_CFG_1 register
         * 
         * @param config {uint8_t} config to write
         */
        void SetDMPConfig1(uint8_t config);
        /**
         * Return the current DMP_CFG_1 register
         */
        uint8_t GetDMPConfig1();

        /**
         * Write config to DMP_CFG_2 register
         * 
         * @param config {uint8_t} config to write
         */
        void SetDMPConfig2(uint8_t config);
        /**
         * Return the current DMP_CFG_2 register
         */
        uint8_t GetDMPConfig2();

        /** ============================================================
         *      MOTION PROCESSING
         *  ============================================================
         */
        /**
         * Initialize MPU6050 for DMP
         * 
         * @return {uint8_t} error codes. 0: no error
         */
        uint8_t DMPInitalize();
        
        /**
         * Check if the DMP packet is already available on FIFO buffer based on
         * the packet size set for DMP
         * 
         * @return {bool} True if DMP packet is available
         */
        bool DMPPacketAvailable();

        /**
         * Return the size setting for DMP packet
         */
        uint16_t DMPGetFIFOPacketSize();
        
        /**
         * Parse and return quaternion data from one DMP packet
         * 
         * @param data {int32_t, int16_t, or Quaternion} quaternion data
         * @param packet {uint8_t}  DMP packet
         */
        uint8_t DMPGetQuaternion(int32_t *data, const uint8_t* packet);
        uint8_t DMPGetQuaternion(int16_t *data, const uint8_t* packet);
        uint8_t DMPGetQuaternion(Quaternion *q, const uint8_t* packet);

        /**
         * Parse and return gyroscope data from one DMP packet
         * 
         * @param data {int32_t, int16_t, or Vector} gyroscope data
         * @param packet {uint8_t}  DMP packet
         */
        uint8_t DMPGetGyro(int32_t *data, const uint8_t* packet);
        uint8_t DMPGetGyro(int16_t *data, const uint8_t* packet);
        uint8_t DMPGetGyro(Vector *v, const uint8_t* packet);

        /**
         * Parse and return accelerometer data from one DMP packet
         * 
         * @param data {int32_t, int16_t, or Vector} accelerometer data
         * @param packet {uint8_t}  DMP packet
         */
        uint8_t DMPGetAccel(int32_t *data, const uint8_t* packet);
        uint8_t DMPGetAccel(int16_t *data, const uint8_t* packet);
        uint8_t DMPGetAccel(Vector *v, const uint8_t* packet);

        /**
         * Calculate euler angles from quaternion
         * 
         * @param data {float}      array of euler angles in radians
         * @param q {Quaternion}    quaternion data
         */
        uint8_t DMPGetEuler(float *data, Quaternion *q);

        /**
         * Calculate gravity from DMP packet.
         * The quaternion data will be parsed from the DMP packet.
         * 
         * @param data {int16_t}    array of gravity in X-Y-Z axis
         * @param packet {uint8_t}  DMP packet
         */
        uint8_t DMPGetGravity(int16_t *data, const uint8_t* packet);
        /**
         * Calculate euler angles from quaternion data
         * 
         * @param data {Vector}    vector of gravity
         * @param q {Quaternion}   quaternion data
         */
        uint8_t DMPGetGravity(Vector *v, Quaternion *q);

        /**
         * Calculate yaw, pitch, and roll from quaternion data and vector of gravity
         * 
         * @param data {float}      array of {yaw, pitch, roll} in radians
         * @param q {Quaternion}    quaternion data
         * @param gravity {Vector}  vector of gravity
         */
        uint8_t DMPGetYawPitchRoll(float *data, Quaternion *q, Vector *gravity);

        /**
         * Get the linear acceleration of the device without including gravity
         * 
         * @param v {Vector}        vector of linear acceleration
         * @param v_raw {Vector}    vector of raw acceleration from DMP packet
         * @param gravity {Vector}  vector of gravity
         */
        uint8_t DMPGetLinearAccel(Vector *v, Vector *v_raw, Vector *gravity);

        /**
         * Rotate measured 3D acceleration vector into original state 
         * frame of reference based on orientation quaternion
         * 
         * @param v {Vector}        vector of linear acceleration in world
         * @param v_raw {Vector}    vector of linear acceleration
         * @param q {Quaternion}    quaternion as reference frame
         */
        uint8_t DMPGetLinearAccelInWorld(Vector *v, Vector *v_real, Quaternion *q);

};

#endif