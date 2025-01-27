#pragma once

#include <IMU_Base.h>
#include <IMU_MPU6886.h>

class IMU_M5_STACK : public IMU_Base {
public:
    IMU_M5_STACK(axis_order_t axisOrder, void* i2cMutex);
public:
    virtual xyz_int32_t readGyroRaw() const override;
    virtual xyz_int32_t readAccRaw() const override;

    virtual xyz_t readGyroRPS() const override;
    virtual xyz_t readGyroDPS() const override;
    virtual xyz_t readAcc() const override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() const override;

    virtual size_t readFIFO_ToBuffer() override;
    virtual gyroRPS_Acc_t  readFIFO_Item(size_t index) override;
private:
    gyroRPS_Acc_t gyroRPS_AccFromRaw(const IMU_MPU6886::acc_temperature_gyro_data_t& data) const;
private:
    uint8_t _fifoBuffer[1024] {};
};

