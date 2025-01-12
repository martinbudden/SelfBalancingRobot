#pragma once

#include <IMU_Base.h>

class IMU_M5_UNIFIED : public IMU_Base {
public:
    explicit IMU_M5_UNIFIED(void* i2cMutex);
public:
    virtual void setAccOffset(const xyz_int16_t& accOffset) override;
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual xyz_int16_t readAccRaw() const override;
    virtual xyz_int16_t readGyroRaw() const override;
    virtual bool readGyroRadiansAcc(xyz_t& gyroRadians, xyz_t& acc) const override;
    virtual int readFIFO_ToBuffer() override;
    virtual void readFIFO_Item(xyz_t& gyroRadians, xyz_t& acc, size_t index) override;
private:
    xyz_int16_t _gyroOffset {};
    xyz_int16_t _accOffset {};
    uint8_t _fifoBuffer[1024] {};
};
