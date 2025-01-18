#pragma once

#include "I2C.h"
#include <IMU_Base.h>
#include <xyz_int16_type.h>


class IMU_MPU6886 : public IMU_Base {
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
    struct acc_temp_gyro_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 14 };
        uint8_t acc_x_h;
        uint8_t acc_x_l;
        uint8_t acc_y_h;
        uint8_t acc_y_l;
        uint8_t acc_z_h;
        uint8_t acc_z_l;
        uint8_t temp_h;
        uint8_t temp_l;
        uint8_t gyro_x_h;
        uint8_t gyro_x_l;
        uint8_t gyro_y_h;
        uint8_t gyro_y_l;
        uint8_t gyro_z_h;
        uint8_t gyro_z_l;
    };
    union acc_temp_gyro_array_t {
        enum { DATA_SIZE = 1036 };
        acc_temp_gyro_data_t accTempGyro[74];
        uint8_t data[DATA_SIZE];
    };
#pragma pack(pop)
public:
    IMU_MPU6886(uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex);
    IMU_MPU6886(uint8_t SDA_pin, uint8_t SCL_pin) : IMU_MPU6886(SDA_pin, SCL_pin, nullptr) {}
    void init();
public:
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual void setAccOffset(const xyz_int16_t& accOffset) override;
    virtual xyz_int16_t readGyroRaw() const override;
    virtual xyz_int16_t readAccRaw() const override;
    virtual bool readGyroRPS_Acc(xyz_t& gyroRPS, xyz_t& acc) const override;
    virtual int readFIFO_ToBuffer() override;
    virtual void readFIFO_Item(xyz_t& gyroRPS, xyz_t& acc, size_t index) override;

    float readTemperature() const;
    int16_t readTemperatureRaw() const;

    void setFIFOEnable(bool enableflag);
    void resetFIFO();

    static gyroRPS_Acc_t gyroRPS_AccFromData(const acc_temp_gyro_data_t& data, const xyz_int16_t& gyroOffset, const xyz_int16_t& accOffset);
    static mems_sensor_data_t gyroOffsetFromXYZ(const xyz_int16_t& data);
private:
    gyroRPS_Acc_t readGyroRPS_Acc() const;
    xyz_t readGyroDPS() const;
    xyz_t readGyroRPS() const;
    xyz_t readAcc() const;
private:
    I2C _bus;
    xyz_int16_t _accOffset {};
    xyz_int16_t _gyroOffset {};
    acc_temp_gyro_array_t _fifoBuffer {};
    uint8_t _imuID {0};
};
