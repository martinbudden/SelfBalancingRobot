#pragma once

#include <IMU_Base.h>

class IMU_M5_STACK : public IMU_Base {
public:
    explicit IMU_M5_STACK(void* i2cMutex);
public:
    virtual void setAccOffset(const xyz_int16_t& accOffset) override;
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual xyz_int16_t readAccRaw() const override;
    virtual xyz_int16_t readGyroRaw() const override;
    virtual bool readAccGyroRadians(xyz_t& acc, xyz_t& gyroRadians) const override;
    virtual int readFIFO_ToBuffer() override;
    virtual void readFIFO_Item(xyz_t& acc, xyz_t& gyroRadians, size_t index) override;
private:
    xyz_int16_t _accOffset {};
    xyz_int16_t _gyroOffset {};
    uint8_t _fifoBuffer[1024] {};
};

