#if defined(M5_STACK)

#include "IMU_M5_STACK.h"
#include <M5Stack.h>
#include <MPU_6886.h>
#include <xyz_int16_type.h>

IMU_M5_STACK::IMU_M5_STACK(void* i2cMutex) :
    IMU_Base(i2cMutex)
{
    // Set up FIFO for IMU
    // IMU data frequency is 500Hz
    i2cSemaphoreTake();
    M5.IMU.setFIFOEnable(true);
    M5.IMU.RestFIFO();
    i2cSemaphoreGive();
}

void IMU_M5_STACK::setAccOffset(const xyz_int16_t& accOffset)
{
    _accOffset = accOffset;
}

void IMU_M5_STACK::setGyroOffset(const xyz_int16_t& gyroOffset)
{
    _gyroOffset = gyroOffset;
}

xyz_int16_t IMU_M5_STACK::readAccRaw() const
{
    xyz_int16_t acc {};

    i2cSemaphoreTake();
    M5.IMU.getAccelAdc(&acc.x, &acc.y, &acc.z);
    i2cSemaphoreGive();

    return acc;
}

xyz_int16_t IMU_M5_STACK::readGyroRaw() const
{
    xyz_int16_t gyro {};

    i2cSemaphoreTake();
    M5.IMU.getGyroAdc(&gyro.x, &gyro.y, &gyro.z);
    i2cSemaphoreGive();

    return gyro;
}

bool IMU_M5_STACK::readAccGyroRadians(xyz_t& acc, xyz_t& gyroRadians) const
{
    i2cSemaphoreTake();
    M5.IMU.getAccelData(&acc.x, &acc.y, &acc.z);
    M5.IMU.getGyroData(&gyroRadians.x, &gyroRadians.y, &gyroRadians.z);
    i2cSemaphoreGive();

    constexpr float degreesToRadians {M_PI / 180.0};
    gyroRadians.x *= degreesToRadians;
    gyroRadians.y *= degreesToRadians;
    gyroRadians.z *= degreesToRadians;

    return true;
}

int IMU_M5_STACK::readFIFO_ToBuffer()
{
    i2cSemaphoreTake();
    const uint16_t fifoCount = M5.IMU.ReadFIFOCount();
    if (fifoCount != 0) {
        M5.IMU.ReadFIFOBuff(&_fifoBuffer[0], fifoCount);
    }
    i2cSemaphoreGive();
    return fifoCount  / mpu_6886_data_t::DATA_SIZE;
}

void IMU_M5_STACK::readFIFO_Item(xyz_t& acc, xyz_t& gyroRadians, size_t index)
{
    const mpu_6886_data_t* imu_data = reinterpret_cast<mpu_6886_data_t*>(&_fifoBuffer[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    const mpu_6886_data_t& imuData = imu_data[index]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const MPU_6886::acc_gyroRadians_t accGyro = MPU_6886::accGyroRadiansFromData(imuData, _accOffset, _gyroOffset);
    acc = accGyro.acc;
    gyroRadians = accGyro.gyroRadians;
}

#endif