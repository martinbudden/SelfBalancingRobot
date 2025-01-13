#pragma once

#include <I2C.h>
#include <IMU_Base.h>
#include <xyz_int16_type.h>


class  IMU_BMI270 : public IMU_Base {
public:
    static constexpr uint8_t I2C_ADDRESS=0x19;

#pragma pack(push, 1)
    struct mems_sensor_data_t {
        enum { DATA_SIZE = 6 };
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct acc_gyro_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 12 };
        xyz_int16_t acc;
        xyz_int16_t gyro;
    };
#pragma pack(pop)
public:
    IMU_BMI270(uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex);
    IMU_BMI270(uint8_t SDA_pin, uint8_t SCL_pin) :  IMU_BMI270(SDA_pin, SCL_pin, nullptr) {}
    void init();
public:
    virtual void setAccOffset(const xyz_int16_t& accOffset) override;
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual xyz_int16_t readAccRaw() const override;
    virtual xyz_int16_t readGyroRaw() const override;
    virtual bool readGyroRadiansAcc(xyz_t& gyroRadians, xyz_t& acc) const override;
    virtual int readFIFO_ToBuffer() override;
    virtual void readFIFO_Item(xyz_t& gyroRadians, xyz_t& acc, size_t index) override;
    static gyroRadiansAcc_t gyroRadiansAccFromData(const acc_gyro_data_t& data, const xyz_int16_t& gyroOffset, const xyz_int16_t& accOffset);
private:
    gyroRadiansAcc_t readGyroRadiansAcc() const;
private:
    I2C _bus; //!< Serial Communication Bus interface, can be either I2C or SPI
    xyz_int16_t _accOffset {};
    xyz_int16_t _gyroOffset {};
};
