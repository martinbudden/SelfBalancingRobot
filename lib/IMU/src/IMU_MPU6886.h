#pragma once

#include "I2C.h"
#include <IMU_Base.h>


class IMU_MPU6886 : public IMU_Base {
public:
    IMU_MPU6886(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex);
    IMU_MPU6886(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin) : IMU_MPU6886(axisOrder, SDA_pin, SCL_pin, nullptr) {}
    void init();
public:
    enum acc_scale_t { AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G };
    enum gyro_scale_t { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };

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
    struct acc_temperature_gyro_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 14 };
        uint8_t acc_x_h;
        uint8_t acc_x_l;
        uint8_t acc_y_h;
        uint8_t acc_y_l;
        uint8_t acc_z_h;
        uint8_t acc_z_l;
        uint8_t temperature_h;
        uint8_t temperature_l;
        uint8_t gyro_x_h;
        uint8_t gyro_x_l;
        uint8_t gyro_y_h;
        uint8_t gyro_y_l;
        uint8_t gyro_z_h;
        uint8_t gyro_z_l;
    };
    union acc_temperature_gyro_array_t {
        enum { DATA_SIZE = 1036 };
        acc_temperature_gyro_data_t accTemperatureGyro[74];
        uint8_t data[DATA_SIZE];
    };
#pragma pack(pop)
public:
    virtual void setGyroOffset(const xyz_int32_t& gyroOffset) override;
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() override;

    virtual size_t readFIFO_ToBuffer() override;
    virtual gyroRPS_Acc_t readFIFO_Item(size_t index) override;

    float readTemperature() const;
    int32_t readTemperatureRaw() const;

    void setFIFOEnable(bool enableflag);
    void resetFIFO();
    static mems_sensor_data_t gyroOffsetFromXYZ(const xyz_int32_t& data);
private:
    xyz_t gyroRPS_FromRaw(const mems_sensor_data_t& data) const;
    xyz_t accFromRaw(const mems_sensor_data_t& data) const;
    gyroRPS_Acc_t gyroRPS_AccFromRaw(const acc_temperature_gyro_data_t& data) const;
private:
    I2C _bus;
    acc_temperature_gyro_array_t _fifoBuffer {};
    uint8_t _imuID {0};
};
