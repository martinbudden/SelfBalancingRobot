#pragma once

#include <I2C.h>
#include <IMU_Base.h>


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
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    };
#pragma pack(pop)
public:
    IMU_BMI270(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex);
    IMU_BMI270(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin) :  IMU_BMI270(axisOrder, SDA_pin, SCL_pin, nullptr) {}
    void init();
public:
    virtual xyz_int32_t readGyroRaw() const override;
    virtual xyz_int32_t readAccRaw() const override;
    virtual int32_t getAccOneG_Raw() const override;

    virtual gyroRPS_Acc_t readGyroRPS_Acc() const override;
private:
    gyroRPS_Acc_t gyroRPS_AccFromRaw(const acc_gyro_data_t& data) const;
private:
    I2C _bus; //!< Serial Communication Bus interface, can be either I2C or SPI
};
