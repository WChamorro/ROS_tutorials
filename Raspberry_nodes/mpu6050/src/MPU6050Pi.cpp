/**
 * @author  Dwindra Sulistyoutomo
 */

#include "MPU6050Pi.h"

/** ============================================================
 *      CONSTRUCTOR
 *  ============================================================
 */
MPU6050Pi::MPU6050Pi() {
    // Set default the I2C address of the device
    I2C_address_ = MPU6050_ADDRESS;

    // Initialize the I2C device file handler
    fd_ = I2CPi::Setup(MPU6050_ADDRESS);

    // Set Clock Source. Better to use X gyro as reference
    MPU6050Pi::SetClockSource(CLOCK_PLL_XGYRO);

    // Disable SLEEP mode
    MPU6050Pi::SetSleepMode(SLEEP_DISABLED);

    // Set DLPF (Digital Low Pass Filter) to 44Hz. Check table in Register Map.
    MPU6050Pi::SetDLPFMode(DLPF_BW_44);

    // Set sample rate divider to 200Hz, div=5.
    MPU6050Pi::SetSampleRateDivider(0x05);

    // Configure gyroscope setting with default range
    MPU6050Pi::SetFullScaleGyroRange(FS_SEL_500);

    // Configure accelerometer setting with default range
    MPU6050Pi::SetFullScaleAccelRange(AFS_SEL_4);

    // Set offsets with default zeros
    MPU6050Pi::SetAccelXOffset(0x0000);
    MPU6050Pi::SetAccelYOffset(0x0000);
    MPU6050Pi::SetAccelZOffset(0x0000);
    MPU6050Pi::SetGyroXOffset(0x0000);
    MPU6050Pi::SetGyroYOffset(0x0000);
    MPU6050Pi::SetGyroZOffset(0x0000);
}

MPU6050Pi::MPU6050Pi(int16_t *offsets) {
    // Set default the I2C address of the device
    I2C_address_ = MPU6050_ADDRESS;

    // Initialize the I2C device file handler
    fd_ = I2CPi::Setup(MPU6050_ADDRESS);

    // Set Clock Source. Better to use X gyro as reference
    MPU6050Pi::SetClockSource(CLOCK_PLL_XGYRO);

    // Disable SLEEP mode
    MPU6050Pi::SetSleepMode(SLEEP_DISABLED);

    // Set DLPF (Digital Low Pass Filter) to 44Hz. Check table in Register Map.
    MPU6050Pi::SetDLPFMode(DLPF_BW_44);

    // Set sample rate divider to 200Hz, div=5.
    MPU6050Pi::SetSampleRateDivider(0x05);

    // Configure gyroscope setting with default range
    MPU6050Pi::SetFullScaleGyroRange(FS_SEL_500);

    // Configure accelerometer setting with default range
    MPU6050Pi::SetFullScaleAccelRange(AFS_SEL_4);

    // Set offsets
    MPU6050Pi::SetAccelXOffset(offsets[0]);
    MPU6050Pi::SetAccelYOffset(offsets[1]);
    MPU6050Pi::SetAccelZOffset(offsets[2]);
    MPU6050Pi::SetGyroXOffset(offsets[4]);
    MPU6050Pi::SetGyroYOffset(offsets[5]);
    MPU6050Pi::SetGyroZOffset(offsets[6]);
}

/** ============================================================
 *      CONFIGURATION
 *  ============================================================
 */
void MPU6050Pi::SetSampleRateDivider(uint8_t rate) {
    I2CPi::WriteByte(fd_, SMPLRT_DIV, rate);

    sample_rate_ = gyro_rate_/(1+rate);
}

float MPU6050Pi::GetSampleRate() {
    return sample_rate_;
}

// ---------- CONFIG registers ----------
void MPU6050Pi::SetExternalFrameSync(uint8_t sync) {
    I2CPi::WriteBits(fd_, CONFIG, EXT_SYNC_SET_START, EXT_SYNC_SET_LENGTH, sync);
}

void MPU6050Pi::SetDLPFMode(uint8_t mode) {
    // Set sample rate based on DLPF_CFG
    if ((mode == DLPF_BW_260) || (mode > DLPF_BW_5)){
        gyro_rate_ = 8; // 8 kHz
    }
    else {
        gyro_rate_ = 1; // 1 kHz
    }

    // Set only the DLPF_CFG on bit 0,1,2
    I2CPi::WriteBits(fd_, CONFIG, CONFIG_DLPF_CFG_START, CONFIG_DLPF_CFG_LENGTH, mode);
}

// ---------- GYRO_CONFIG registers ----------
void MPU6050Pi::SetFullScaleGyroRange(uint8_t range) {
    uint8_t gyro_config_val = range << FS_SEL_START;
    switch (range) {
        case FS_SEL_250:         // 250 deg/s full scale range
            gyro_sensitivity_ = GYRO_LSB_250;
            break;
        case FS_SEL_500:         // 500 deg/s full scale range
            gyro_sensitivity_ = GYRO_LSB_500;
            break;
        case FS_SEL_1000:         // 1000 deg/s full scale range
            gyro_sensitivity_ = GYRO_LSB_1000;
            break;
        case FS_SEL_2000:         // 2000 deg/s full scale range
            gyro_sensitivity_ = GYRO_LSB_2000;
            break;
    }
    I2CPi::WriteByte(fd_, GYRO_CONFIG, gyro_config_val);
}

float MPU6050Pi::GetGyroSensitivity() {
    return gyro_sensitivity_;
}

// ---------- ACCEL_CONFIG registers ----------
void MPU6050Pi::SetFullScaleAccelRange(uint8_t range) {
    uint8_t accel_config_val = range << AFS_SEL_START;
    switch (range) {
        case AFS_SEL_2:      // 2g full scale range
            accel_sensitivity_ = ACCEL_LSB_2;
            accel_scale_range_ = 2;
            break;
        case AFS_SEL_4:      // 4g full scale range
            accel_sensitivity_ = ACCEL_LSB_4;
            accel_scale_range_ = 4;
            break;
        case AFS_SEL_8:      // 8g full scale range
            accel_sensitivity_ = ACCEL_LSB_8;
            accel_scale_range_ = 8;
            break;
        case AFS_SEL_16:      // 16g full scale range
            accel_sensitivity_ = ACCEL_LSB_16;
            accel_scale_range_ = 16;
            break;
    }
    I2CPi::WriteByte(fd_, ACCEL_CONFIG, accel_config_val);
}

float MPU6050Pi::GetAccelSensitivity() {
    return accel_sensitivity_;
}

// ---------- FF* registers ----------
void MPU6050Pi::SetFreefallDetectionThreshold(uint8_t threshold){
    I2CPi::WriteByte(fd_, FF_THR, threshold);
}
uint8_t MPU6050Pi::GetFreefallDetectionThreshold(){
    return I2CPi::ReadByte(fd_, FF_THR);
}

void MPU6050Pi::SetFreefallDetectionDuration(uint8_t duration){
    I2CPi::WriteByte(fd_, FF_DUR, duration);
}
uint8_t MPU6050Pi::GetFreefallDetectionDuration(){
    return I2CPi::ReadByte(fd_, FF_DUR);
}

// ---------- MOT* registers ----------
void MPU6050Pi::SetMotionDetectionThreshold(uint8_t threshold){
    I2CPi::WriteByte(fd_, MOT_THR, threshold);
}
uint8_t MPU6050Pi::GetMotionDetectionThreshold(){
    return I2CPi::ReadByte(fd_, MOT_THR);
}

void MPU6050Pi::SetMotionDetectionDuration(uint8_t duration){
    I2CPi::WriteByte(fd_, MOT_DUR, duration);
}
uint8_t MPU6050Pi::GetMotionDetectionDuration(){
    return I2CPi::ReadByte(fd_, MOT_DUR);
}

// ---------- ZRMOT* registers ----------
void MPU6050Pi::SetZeroMotionDetectionThreshold(uint8_t threshold){
    I2CPi::WriteByte(fd_, ZRMOT_THR, threshold);
}
uint8_t MPU6050Pi::GetZeroMotionDetectionThreshold(){
    return I2CPi::ReadByte(fd_, ZRMOT_THR);
}

void MPU6050Pi::SetZeroMotionDetectionDuration(uint8_t duration){
    I2CPi::WriteByte(fd_, ZRMOT_DUR, duration);
}
uint8_t MPU6050Pi::GetZeroMotionDetectionDuration(){
    return I2CPi::ReadByte(fd_, ZRMOT_DUR);
}

// ---------- I2C_SLV* registers ----------
void MPU6050Pi::SetSlaveAddress(uint8_t num, uint8_t address) {
    if (num > 3) return;
    I2CPi::ReadByte(fd_, I2C_SLV0_ADDR + num*3);
}

uint8_t MPU6050Pi::GetSlaveAddress(uint8_t num) {
    if (num > 3) return 0;
    return I2CPi::ReadByte(fd_, I2C_SLV0_ADDR + num*3);
}

// ---------- INT_ENABLE registers ----------
void MPU6050Pi::SetIntEnabled(uint8_t enabled) {
    I2CPi::WriteByte(fd_, INT_ENABLE, enabled);
}
uint8_t MPU6050Pi::GetIntEnabled(){
    return I2CPi::ReadByte(fd_, INT_ENABLE);
}

// ---------- INT_STATUS registers ----------
uint8_t MPU6050Pi::GetIntStatus() {
    return I2CPi::ReadByte(fd_, INT_STATUS);
}

// ---------- *OFFS* registers ----------
void MPU6050Pi::SetOffset(int16_t *offset){
    I2CPi::WriteWord(fd_, XA_OFFS_H, offset[0]);
    I2CPi::WriteWord(fd_, YA_OFFS_H, offset[1]);
    I2CPi::WriteWord(fd_, ZA_OFFS_H, offset[2]);
    I2CPi::WriteWord(fd_, XG_OFFSET_H, offset[3]);
    I2CPi::WriteWord(fd_, YG_OFFSET_H, offset[4]);
    I2CPi::WriteWord(fd_, ZG_OFFSET_H, offset[5]);
}
void MPU6050Pi::SetAccelOffset(int16_t *offset){
    I2CPi::WriteWord(fd_, XA_OFFS_H, offset[0]);
    I2CPi::WriteWord(fd_, YA_OFFS_H, offset[1]);
    I2CPi::WriteWord(fd_, ZA_OFFS_H, offset[2]);
}
void MPU6050Pi::SetAccelXOffset(int16_t offset) {
    I2CPi::WriteWord(fd_, XA_OFFS_H, offset);
}
void MPU6050Pi::SetAccelYOffset(int16_t offset) {
    I2CPi::WriteWord(fd_, YA_OFFS_H, offset);
}
void MPU6050Pi::SetAccelZOffset(int16_t offset) {
    I2CPi::WriteWord(fd_, ZA_OFFS_H, offset);
}

void MPU6050Pi::SetGyroOffset(int16_t *offset){
    I2CPi::WriteWord(fd_, XG_OFFSET_H, offset[0]);
    I2CPi::WriteWord(fd_, YG_OFFSET_H, offset[1]);
    I2CPi::WriteWord(fd_, ZG_OFFSET_H, offset[2]);
}
void MPU6050Pi::SetGyroXOffset(int16_t offset){
    I2CPi::WriteWord(fd_, XG_OFFSET_H, offset);
}
void MPU6050Pi::SetGyroYOffset(int16_t offset) {
    I2CPi::WriteWord(fd_, YG_OFFSET_H, offset);
}
void MPU6050Pi::SetGyroZOffset(int16_t offset) {
    I2CPi::WriteWord(fd_, ZG_OFFSET_H, offset);
}

// ---------- USER_CTRL registers ----------

void MPU6050Pi::SetFIFOEnabled(bool enabled){
    I2CPi::WriteBit(fd_, USER_CTRL, USERCTRL_FIFO_EN_BIT, enabled);
}

void MPU6050Pi::SetI2CMasterModeEnabled(bool enabled){
    I2CPi::WriteBit(fd_, USER_CTRL, USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050Pi::ResetFIFO(){
    I2CPi::WriteBit(fd_, USER_CTRL, USERCTRL_FIFO_RESET_BIT, true);
}
void MPU6050Pi::ResetI2CMaster(){
    I2CPi::WriteBit(fd_, USER_CTRL, USERCTRL_I2C_MST_RESET_BIT, true);
}
void MPU6050Pi::ResetSensors(){
    I2CPi::WriteBit(fd_, USER_CTRL, USERCTRL_SIG_COND_RESET_BIT, true);
}

// ---------- PWR_MGMT_1 registers ----------
void MPU6050Pi::SetClockSource(uint8_t clk_sel) {
    // Set only the PWR_MGMT_1 on bit 0,1,2
    I2CPi::WriteBits(fd_, PWR_MGMT_1, PWR_MGMT_1_CLKSEL_START, PWR_MGMT_1_CLKSEL_LENGTH, clk_sel);
}

void MPU6050Pi::SetSleepMode(uint8_t mode){
    // Set only the PWR_MGMT_1 on bit PWR_MGMT_1_SLEEP_BIT
    I2CPi::WriteBit(fd_, PWR_MGMT_1, PWR_MGMT_1_SLEEP_BIT, mode);
}

void MPU6050Pi::ResetDevice(){
    // Set only the PWR_MGMT_1 on bit PWR_MGMT_1_SLEEP_BIT
    I2CPi::WriteBit(fd_, PWR_MGMT_1, PWR_MGMT_1_RESET_BIT, 1);
}

// ---------- FIFO_COUNT_* registers ----------
uint16_t MPU6050Pi::GetFIFOCount() {
    return I2CPi::ReadWord(fd_, FIFO_COUNT_H);
}

// ---------- FIFO_R_W registers ----------
void MPU6050Pi::SetFIFOByte(uint8_t data) {
    I2CPi::WriteByte(fd_, FIFO_R_W, data);
}

uint8_t MPU6050Pi::GetFIFOByte() {
    return I2CPi::ReadByte(fd_, FIFO_R_W);
}
void MPU6050Pi::GetFIFOBytes(uint8_t *data, uint8_t length) {
    if(length > 0){
        int8_t i;
        for (i=0; i < length; i++){
             data[i] = I2CPi::ReadByte(fd_, FIFO_R_W);
        }
    } else {
    	*data = 0;
    }
}

/** ============================================================
 *      DATA
 *  ============================================================
 */
// ---------- *OUT* registers ----------
void MPU6050Pi::GetMotion6(int16_t* ax, int16_t* ay, int16_t* az,
                           int16_t* gx, int16_t* gy, int16_t* gz) {
    *ax = I2CPi::ReadWord(fd_, ACCEL_XOUT_H);
    *ay = I2CPi::ReadWord(fd_, ACCEL_YOUT_H);
    *az = I2CPi::ReadWord(fd_, ACCEL_ZOUT_H);
    *gx = I2CPi::ReadWord(fd_, GYRO_XOUT_H);
    *gy = I2CPi::ReadWord(fd_, GYRO_YOUT_H);
    *gz = I2CPi::ReadWord(fd_, GYRO_ZOUT_H);
}

void MPU6050Pi::GetAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    *ax = I2CPi::ReadWord(fd_, ACCEL_XOUT_H);
    *ay = I2CPi::ReadWord(fd_, ACCEL_YOUT_H);
    *az = I2CPi::ReadWord(fd_, ACCEL_ZOUT_H);
}

void MPU6050Pi::GetAccelY(int16_t* ay) {
    *ay = I2CPi::ReadWord(fd_, ACCEL_YOUT_H);
}

void MPU6050Pi::GetAccelFloat(float* ax, float* ay, float* az) {
    int16_t x, y, z;
    MPU6050Pi::GetAccel(&x, &y, &z);
    *ax = (float) x / accel_sensitivity_;
    *ay = (float) y / accel_sensitivity_;
    *az = (float) z / accel_sensitivity_;
}

void MPU6050Pi::GetGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    *gx = I2CPi::ReadWord(fd_, GYRO_XOUT_H);
    *gy = I2CPi::ReadWord(fd_, GYRO_YOUT_H);
    *gz = I2CPi::ReadWord(fd_, GYRO_ZOUT_H);
}

void MPU6050Pi::GetGyroFloat(float* gx, float* gy, float* gz) {
    int16_t x, y, z;
    MPU6050Pi::GetGyro(&x, &y, &z);
    *gx = (float) x / gyro_sensitivity_;
    *gy = (float) y / gyro_sensitivity_;
    *gz = (float) z / gyro_sensitivity_;
}

/** ============================================================
 *      UNDOCUMENTED DMP METHODS
 *  ============================================================
 */

void MPU6050Pi::SetDMPEnabled(bool enabled){
    I2CPi::WriteBit(fd_, USER_CTRL, USERCTRL_DMP_EN_BIT, enabled);
}
void MPU6050Pi::ResetDMP(){
    I2CPi::WriteBit(fd_, USER_CTRL, USERCTRL_DMP_RESET_BIT, true);
}

// ---------- XG_OFFS_TC registers ----------
void MPU6050Pi::SetOTPBankValid(bool enabled){
    I2CPi::WriteBit(fd_, XG_OFFS_TC, TC_OTP_BNK_VLD_BIT, enabled);
}

uint8_t MPU6050Pi::GetOTPBankValid(){
    return I2CPi::ReadBit(fd_, XG_OFFS_TC, TC_OTP_BNK_VLD_BIT);
}

// ---------- BANK_SEL registers ----------
void MPU6050Pi::SetMemoryBank(uint8_t bank, bool prefetch_enabled, bool user_bank){
    bank &= 0x1F;
    if (user_bank) bank |= 0x20;
    if (prefetch_enabled) bank |= 0x40;
    I2CPi::WriteByte(fd_, BANK_SEL, bank);
}

// ---------- MEM_START_ADDR registers ----------
void MPU6050Pi::SetMemoryStartAddress(uint8_t address){
    I2CPi::WriteByte(fd_, MEM_START_ADDR, address);
}

// ---------- MEM_R_W registers ----------
void MPU6050Pi::WriteMemoryByte(uint8_t data){
    I2CPi::WriteByte(fd_, MEM_R_W, data);
}

uint8_t MPU6050Pi::ReadMemoryByte(){
    return I2CPi::ReadByte(fd_, MEM_R_W);
}

bool MPU6050Pi::WriteMemoryBlock(const uint8_t *data, uint16_t data_size, uint8_t bank, uint8_t address, bool verify, bool use_progmem){
    MPU6050Pi::SetMemoryBank(bank);
    MPU6050Pi::SetMemoryStartAddress(address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer=0;
    uint8_t *progBuffer=0;
    uint16_t i;
    uint8_t j;
    if (verify) verifyBuffer = (uint8_t *)malloc(DMP_MEMORY_CHUNK_SIZE);
    // if (use_progmem) progBuffer = (uint8_t *)malloc(DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < data_size;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > data_size) chunkSize = data_size - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        
        progBuffer = (uint8_t *)data + i;

        // Need to write one by one based on MEM_START_ADDR as I2CPi::WriteBytes doesn't work well
        // I2CPi::WriteBytes(fd_, MEM_R_W, progBuffer, chunkSize);
        for (j=0; j < chunkSize; j++){
            MPU6050Pi::SetMemoryStartAddress(address+j);
            I2CPi::WriteByte(fd_, MEM_R_W, progBuffer[j]);
        }

        // verify data if needed
        if (verify && verifyBuffer) {
            MPU6050Pi::SetMemoryBank(bank);

            // Need to read one by one based on MEM_START_ADDR as I2CPi::ReadBytes doesn't work well
            for (j=0; j < chunkSize; j++){
                MPU6050Pi::SetMemoryStartAddress(address+j);
                verifyBuffer[j] = I2CPi::ReadByte(fd_, MEM_R_W);
            }

            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                free(verifyBuffer);
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < data_size) {
            if (address == 0) bank++;
            MPU6050Pi::SetMemoryBank(bank);
            MPU6050Pi::SetMemoryStartAddress(address);
        }
    }
    if (verify) free(verifyBuffer);
    return true;

}

bool MPU6050Pi::WriteProgMemoryBlock(const uint8_t *data, uint16_t data_size, uint8_t bank, uint8_t address, bool verify){
    return MPU6050Pi::WriteMemoryBlock(data, data_size, bank, address, verify, true);
}

void MPU6050Pi::ReadMemoryBlock(uint8_t *data, uint16_t data_size, uint8_t bank, uint8_t address){
    MPU6050Pi::SetMemoryBank(bank);
    MPU6050Pi::SetMemoryStartAddress(address);
    uint8_t chunkSize;
    for (uint16_t i = 0; i < data_size;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > data_size) chunkSize = data_size - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        data = I2CPi::ReadBytes(fd_, MEM_R_W, chunkSize);
        
        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < data_size) {
            if (address == 0) bank++;
            MPU6050Pi::SetMemoryBank(bank);
            MPU6050Pi::SetMemoryStartAddress(address);
        }
    }
}

// ---------- DMP_CFG* registers ----------
void MPU6050Pi::SetDMPConfig1(uint8_t config) {
    I2CPi::WriteByte(fd_, DMP_CFG_1, config);
}

uint8_t MPU6050Pi::GetDMPConfig1() {
    return I2CPi::ReadByte(fd_, DMP_CFG_1);
}

void MPU6050Pi::SetDMPConfig2(uint8_t config) {
    I2CPi::WriteByte(fd_, DMP_CFG_2, config);
}

uint8_t MPU6050Pi::GetDMPConfig2() {
    return I2CPi::ReadByte(fd_, DMP_CFG_2);
}

/** ============================================================
 *      MOTION PROCESSING
 *  ============================================================
 */

const unsigned char DMP::memory[DMP_CODE_SIZE] = {
	/* bank # 0 */
	0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
	0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
	0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
	0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCB, 0x47, 0xA2, 0x20, 0x00, 0x00, 0x00,
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
	0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
	0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
	0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
	0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,
	/* bank # 1 */
	0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
	0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
	0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x09, 0x23, 0xA1, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
	0x80, 0x00, 0xFF, 0xFF, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00,
	0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,
	/* bank # 2 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x01, 0x00, 0x05, 0x8B, 0xC1, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* bank # 3 */
	0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
	0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
	0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
	0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
	0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
	0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
	0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
	0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0x4C, 0xCD, 0x6C, 0xA9, 0x0C,
	0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
	0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
	0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
	0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
	0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
	0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
	0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
	0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,
	/* bank # 4 */
	0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
	0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
	0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
	0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
	0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
	0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
	0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
	0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
	0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
	0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
	0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
	0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
	0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
	0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
	0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
	0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,
	/* bank # 5 */
	0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
	0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
	0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
	0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
	0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
	0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
	0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
	0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
	0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
	0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
	0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
	0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
	0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
	0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
	0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
	0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,
	/* bank # 6 */
	0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
	0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
	0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
	0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
	0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
	0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
	0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
	0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
	0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
	0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
	0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
	0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
	0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
	0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
	0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
	0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,
	/* bank # 7 */
	0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
	0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
	0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
	0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
	0xDD, 0xF1, 0x20, 0x28, 0x30, 0x38, 0x9A, 0xF1, 0x28, 0x30, 0x38, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
	0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
	0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0x28, 0x30, 0x38,
	0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0x30, 0xDC,
	0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xFE, 0xD8, 0xFF,
};

uint8_t MPU6050Pi::DMPInitalize() {
    // Reset device
    MPU6050Pi::ResetDevice();
    std::this_thread::sleep_for (std::chrono::milliseconds(30));

    MPU6050Pi::SetSleepMode(SLEEP_DISABLED);

    // Check Hardware Revision
	MPU6050Pi::SetMemoryBank(0x10, true, true);
	MPU6050Pi::SetMemoryStartAddress(0x06);
	// std::cout << "Checking hardware revision... ";
	// std::cout << "Revision @ user[16][6] = " << (int)MPU6050Pi::ReadMemoryByte() << std::endl;
	MPU6050Pi::SetMemoryBank(0, false, false);

	// check OTP bank valid
	// std::cout << "Reading OTP bank valid flag... ";
	// std::cout << "OTP bank is ";
    if (MPU6050Pi::GetOTPBankValid())
        // std::cout << "valid." << std::endl;
        ;
    else
        // std::cout << "invalid." << std::endl;
        return 1;

	// Set Slave
	// std::cout << "Setting slave 0 address to 0x7F..." << std::endl;
	MPU6050Pi::SetSlaveAddress(0, 0x7F);
	// std::cout << "Disabling I2C Master mode..." << std::endl;
	MPU6050Pi::SetI2CMasterModeEnabled(false);
	// std::cout << "Setting slave 0 address to 0x68..." << std::endl;
	MPU6050Pi::SetSlaveAddress(0, MPU6050_ADDRESS);
	// std::cout << "Resetting I2C Master control..." << std::endl;
	MPU6050Pi::ResetI2CMaster();
    std::this_thread::sleep_for (std::chrono::milliseconds(20));
	// std::cout << "Setting clock source to Z Gyro..." << std::endl;
	MPU6050Pi::SetClockSource(CLOCK_PLL_ZGYRO);

	// std::cout << "Setting DMP and FIFO_OFLOW interrupts enabled..." << std::endl;
	MPU6050Pi::SetIntEnabled(1<<FIFO_OFLOW_INT_BIT | 1<<DMP_INT_BIT);

	// std::cout << "Setting sample rate to 200Hz..." << std::endl;
	MPU6050Pi::SetSampleRateDivider(4); // 1khz / (1 + 4) = 200 Hz


	// std::cout << "Setting external frame sync to TEMP_OUT_L[0]..." << std::endl;
	MPU6050Pi::SetExternalFrameSync(EXT_SYNC_TEMP_OUT_L);

	// std::cout << "Setting DLPF bandwidth to 42Hz..." << std::endl;
	MPU6050Pi::SetDLPFMode(DLPF_BW_42);

	// std::cout << "Setting gyro sensitivity to +/- 2000 deg/sec..." << std::endl;
	MPU6050Pi::SetFullScaleGyroRange(FS_SEL_2000);

	// Load DMP code into memory banks
	// std::cout << "Writing DMP code to MPU memory banks (" << DMP_CODE_SIZE << " bytes)... " << std::endl;
    // std::cout << dmpMemory;
    if (!MPU6050Pi::WriteProgMemoryBlock(DMP::memory, DMP_CODE_SIZE))
        return 1;
	// std::cout << "Writing DMP code successful." << std::endl;

    // Set the FIFO rate divisor
    unsigned char dmpUpdate[] = {0x00, DMP_FIFO_RATE_DIVISOR};
    // Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16
	MPU6050Pi::WriteMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16);

    //Write start address MSB and LSB into register
	MPU6050Pi::SetDMPConfig1(0x03); //MSB
	MPU6050Pi::SetDMPConfig2(0x00); //LSB

	// std::cout << "Clearing OTP Bank flag..." << std::endl;
	MPU6050Pi::SetOTPBankValid(false);

    // Setting motion detection threshold and duration
	// std::cout << "Setting motion detection     : threshold =   2, duration = 80..." << std::endl;
	MPU6050Pi::SetMotionDetectionThreshold(2);
	MPU6050Pi::SetMotionDetectionDuration(80);

	// Setting Zero motion detection threshold and duration
	// std::cout << "Setting zero-motion detection: threshold = 156, duration =  0..." << std::endl;
	MPU6050Pi::SetZeroMotionDetectionThreshold(156);
	MPU6050Pi::SetZeroMotionDetectionDuration(0);

	// std::cout << "Enabling FIFO..." << std::endl;
	MPU6050Pi::SetFIFOEnabled(true);

	// std::cout << "Resetting and disabling DMP..." << std::endl;
	MPU6050Pi::ResetDMP();
	MPU6050Pi::SetDMPEnabled(false);

	// std::cout << "Setting up internal 42-byte (default) DMP packet buffer..." << std::endl;
	dmp_packet_size_ = DMP_PACKET_SIZE;

	// std::cout << "Resetting FIFO and clearing INT status one last time..." << std::endl;
	MPU6050Pi::ResetFIFO();
	MPU6050Pi::GetIntStatus();

    return 0;
}

bool MPU6050Pi::DMPPacketAvailable() {
    return MPU6050Pi::GetFIFOCount() >= MPU6050Pi::DMPGetFIFOPacketSize();
}

uint16_t MPU6050Pi::DMPGetFIFOPacketSize() {
    return dmp_packet_size_;
}

uint8_t MPU6050Pi::DMPGetQuaternion(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmp_packet_buffer_;
    data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
    data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
    data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
    data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
    return 0;
}
uint8_t MPU6050Pi::DMPGetQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmp_packet_buffer_;
    data[0] = ((packet[0] << 8) | packet[1]);
    data[1] = ((packet[4] << 8) | packet[5]);
    data[2] = ((packet[8] << 8) | packet[9]);
    data[3] = ((packet[12] << 8) | packet[13]);
    return 0;
}
uint8_t MPU6050Pi::DMPGetQuaternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = MPU6050Pi::DMPGetQuaternion(qI, packet);
    if (status == 0) {
        q->w = (float)qI[0] / accel_sensitivity_;
        q->x = (float)qI[1] / accel_sensitivity_;
        q->y = (float)qI[2] / accel_sensitivity_;
        q->z = (float)qI[3] / accel_sensitivity_;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

uint8_t MPU6050Pi::DMPGetGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmp_packet_buffer_;
    data[0] = (((uint32_t)packet[16] << 24) | ((uint32_t)packet[17] << 16) | ((uint32_t)packet[18] << 8) | packet[19]);
    data[1] = (((uint32_t)packet[20] << 24) | ((uint32_t)packet[21] << 16) | ((uint32_t)packet[22] << 8) | packet[23]);
    data[2] = (((uint32_t)packet[24] << 24) | ((uint32_t)packet[25] << 16) | ((uint32_t)packet[26] << 8) | packet[27]);
    return 0;
}
uint8_t MPU6050Pi::DMPGetGyro(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmp_packet_buffer_;
    data[0] = (packet[16] << 8) | packet[17];
    data[1] = (packet[20] << 8) | packet[21];
    data[2] = (packet[24] << 8) | packet[25];
    return 0;
}
uint8_t MPU6050Pi::DMPGetGyro(Vector *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t vI[3];
    uint8_t status = MPU6050Pi::DMPGetGyro(vI, packet);
    if (status == 0) {
        v->x = (float)vI[0] / gyro_sensitivity_;
        v->y = (float)vI[1] / gyro_sensitivity_;
        v->z = (float)vI[2] / gyro_sensitivity_;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

uint8_t MPU6050Pi::DMPGetAccel(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmp_packet_buffer_;
    data[0] = (((uint32_t)packet[28] << 24) | ((uint32_t)packet[29] << 16) | ((uint32_t)packet[30] << 8) | packet[31]);
    data[1] = (((uint32_t)packet[32] << 24) | ((uint32_t)packet[33] << 16) | ((uint32_t)packet[34] << 8) | packet[35]);
    data[2] = (((uint32_t)packet[36] << 24) | ((uint32_t)packet[37] << 16) | ((uint32_t)packet[38] << 8) | packet[39]);
    return 0;
}
uint8_t MPU6050Pi::DMPGetAccel(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmp_packet_buffer_;
    data[0] = (packet[28] << 8) | packet[29];
    data[1] = (packet[32] << 8) | packet[33];
    data[2] = (packet[36] << 8) | packet[37];
    return 0;
}
uint8_t MPU6050Pi::DMPGetAccel(Vector *v, const uint8_t* packet) {
    // // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t vI[3];
    uint8_t status = MPU6050Pi::DMPGetAccel(vI, packet);
    if (status == 0) {
        v->x = (float)vI[0] * accel_scale_range_ / accel_sensitivity_;
        v->y = (float)vI[1] * accel_scale_range_ / accel_sensitivity_;
        v->z = (float)vI[2] * accel_scale_range_ / accel_sensitivity_;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

uint8_t MPU6050Pi::DMPGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q->x * q->y - 2*q->w * q->z, 2*q->w * q->w + 2*q->x * q->x - 1);   // psi
    data[1] = -asin(2*q->x * q->z + 2*q->w * q->y);                              // theta
    data[2] = atan2(2*q->y * q->z - 2*q->w * q->x, 2*q->w * q->w + 2*q->z *q->z - 1);   // phi
    return 0;
}

uint8_t MPU6050Pi::DMPGetGravity(int16_t *data, const uint8_t* packet) {
    /* +1g corresponds to +8192, sensitivity is 2g. */
    int16_t qI[4];
    uint8_t status = MPU6050Pi::DMPGetQuaternion(qI, packet);
    data[0] = ((int32_t)qI[1] * qI[3] - (int32_t)qI[0] * qI[2]) / accel_sensitivity_;
    data[1] = ((int32_t)qI[0] * qI[1] + (int32_t)qI[2] * qI[3]) / accel_sensitivity_;
    data[2] = ((int32_t)qI[0] * qI[0] - (int32_t)qI[1] * qI[1]
	       - (int32_t)qI[2] * qI[2] + (int32_t)qI[3] * qI[3]) / (2 * accel_sensitivity_);
    return status;
}
uint8_t MPU6050Pi::DMPGetGravity(Vector *v, Quaternion *q) {
    v->x = 2 * (q->x * q->z - q->w * q->y);
    v->y = 2 * (q->w * q->x + q->y * q->z);
    v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
    return 0;
}

uint8_t MPU6050Pi::DMPGetYawPitchRoll(float *data, Quaternion *q, Vector *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q->x * q->y - 2*q->w * q->z, 2*q->w * q->w + 2*q->x * q->x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity->x , sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity->y , gravity->z);
    if (gravity->z < 0) {
        if(data[1] > 0) {
            data[1] = M_PI - data[1]; 
        } else { 
            data[1] = -M_PI - data[1];
        }
    }
    return 0;
}

uint8_t MPU6050Pi::DMPGetLinearAccel(Vector *v, Vector *v_raw, Vector *gravity) {
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet)
    float divider;
    if (accel_sensitivity_ == ACCEL_LSB_2) divider = accel_sensitivity_/2;
    else if (accel_sensitivity_ == ACCEL_LSB_4) divider = accel_sensitivity_/4;
    else if (accel_sensitivity_ == ACCEL_LSB_8) divider = accel_sensitivity_/8;
    else if (accel_sensitivity_ == ACCEL_LSB_16) divider = accel_sensitivity_/16;

    v->x = v_raw->x - gravity->x * divider;
    v->y = v_raw->y - gravity->y * divider;
    v->z = v_raw->z - gravity->z * divider;
    return 0;
}

uint8_t MPU6050Pi::DMPGetLinearAccelInWorld(Vector *v, Vector *v_real, Quaternion *q) {
    memcpy(v, v_real, sizeof(Vector));
    v->Rotate(*q);
    return 0;
}
