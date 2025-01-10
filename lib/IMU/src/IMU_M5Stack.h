#pragma once

#include <IMU_Base.h>
#include <IMU_MPU6886.h>

class IMU_M5_STACK : public IMU_Base {
public:
    explicit IMU_M5_STACK(void* i2cMutex);
public:
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual void setAccOffset(const xyz_int16_t& accOffset) override;
    virtual xyz_int16_t readGyroRaw() const override;
    virtual xyz_int16_t readAccRaw() const override;
    virtual bool readGyroRadiansAcc(xyz_t& gyroRadians, xyz_t& acc) const override;
    virtual int readFIFO_ToBuffer() override;
    virtual void readFIFO_Item(xyz_t& gyroRadians, xyz_t& acc, size_t index) override;
    static gyroRadiansAcc_t gyroRadiansAccFromData(const IMU_MPU6886::acc_temp_gyro_data_t& data, const xyz_int16_t& gyroOffset, const xyz_int16_t& accOffset);
private:
    xyz_int16_t _gyroOffset {};
    xyz_int16_t _accOffset {};
    uint8_t _fifoBuffer[1024] {};
};

