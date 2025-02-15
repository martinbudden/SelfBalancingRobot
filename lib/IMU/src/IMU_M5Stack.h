#pragma once

#include <IMU_Base.h>
#include <IMU_MPU6886.h>

class IMU_M5_STACK : public IMU_Base {
public:
    IMU_M5_STACK(axis_order_t axisOrder, void* i2cMutex);
public:
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() override;

    virtual size_t readFIFO_ToBuffer() override;
    virtual gyroRPS_Acc_t readFIFO_Item(size_t index) override;
private:
    gyroRPS_Acc_t gyroRPS_AccFromRaw(const IMU_MPU6886::acc_temperature_gyro_data_t& data) const;
private:
    uint8_t _fifoBuffer[1024] {};
};

