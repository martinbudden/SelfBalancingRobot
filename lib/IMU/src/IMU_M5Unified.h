#pragma once

#include <IMU_Base.h>

class IMU_M5_UNIFIED : public IMU_Base {
public:
    IMU_M5_UNIFIED(axis_order_t axisOrder, void* i2cMutex);
public:
    virtual xyz_int16_t readGyroRaw() const override;
    virtual xyz_int16_t readAccRaw() const override;

    virtual xyz_t readGyroRPS() const override;
    virtual xyz_t readGyroDPS() const override;
    virtual xyz_t readAcc() const override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() const override;

private:
    uint8_t _fifoBuffer[1024] {};
};
