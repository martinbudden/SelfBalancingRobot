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
    virtual bool readGyroRadiansAcc(xyz_t& gyroRadians, xyz_t& acc) const override;
    virtual int readFIFO_ToBuffer() override;
    virtual void readFIFO_Item(xyz_t& gyroRadians, xyz_t& acc, size_t index) override;

    float readTemperature() const;
    int16_t readTemperatureRaw() const;

    void setFIFOEnable(bool enableflag);
    void readFIFO(uint8_t* data, size_t len) const;
    uint16_t readFIFO_count() const;
    void resetFIFO();

    void setGyroFSR(gyro_scale_t gyroScale);
    void setAccFSR(acc_scale_t accScale);

    static gyroRadiansAcc_t gyroRadiansAccFromData(const acc_temp_gyro_data_t& data, const xyz_int16_t& gyroOffset, const xyz_int16_t& accOffset);
    static mems_sensor_data_t gyroOffsetFromXYZ(const xyz_int16_t& data);
private:
    gyroRadiansAcc_t readGyroRadiansAcc() const;
    xyz_t readGyro() const;
    xyz_t readAcc() const;
    xyz_t readGyroRadians() const;
private:
    I2C _bus;
    float _accResolution {0.0};
    float _gyroResolution {0.0};
    xyz_int16_t _accOffset {};
    xyz_int16_t _gyroOffset {};
    uint8_t _imuID {0};
};
