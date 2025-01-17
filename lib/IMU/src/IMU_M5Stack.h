#pragma once

#include <IMU_Base.h>
#include <IMU_MPU6886.h>

class IMU_M5_STACK : public IMU_Base {
public:
    explicit IMU_M5_STACK(void* i2cMutex);
public:
#pragma pack(push, 1)
    struct mems_sensor_data_t {
        enum { DATA_SIZE = 6 };
        uint8_t x_h;
        uint8_t x_l;
        uint8_t y_h;
        uint8_t y_l;
        uint8_t z_h;
        uint8_t z_l;
    };
#pragma pack(pop)
public:
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual void setAccOffset(const xyz_int16_t& accOffset) override;
    virtual xyz_int16_t readGyroRaw() const override;
    virtual xyz_int16_t readAccRaw() const override;

    virtual xyz_t readGyroRPS() const override;
    virtual xyz_t readGyroDPS() const override;
    virtual xyz_t readAcc() const override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() const override;

    virtual int readFIFO_ToBuffer() override;
    virtual gyroRPS_Acc_t  readFIFO_Item(size_t index) override;
private:
    static gyroRPS_Acc_t gyroRPS_AccFromRaw(const IMU_MPU6886::acc_temperature_gyro_data_t& data, const xyz_int16_t& gyroOffset, const xyz_int16_t& accOffset);
private:
    xyz_int16_t _gyroOffset {};
    xyz_int16_t _accOffset {};
    uint8_t _fifoBuffer[1024] {};
};

