#pragma once

#include <BUS_I2C.h>
#include <BUS_SPI.h>
#include <IMU_Base.h>


class IMU_LSM6DS3TR_C : public IMU_Base {
public:
    static constexpr uint8_t I2C_ADDRESS=0x6A;
#pragma pack(push, 1)
    struct mems_sensor_data_t {
        enum { DATA_SIZE = 6 };
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct acc_gyro_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 12 };
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
    };
#pragma pack(pop)
public:
    explicit IMU_LSM6DS3TR_C(axis_order_t axisOrder);
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex);
    IMU_LSM6DS3TR_C(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin) : IMU_LSM6DS3TR_C(axisOrder, SDA_pin, SCL_pin, nullptr) {}
public:
    virtual void init() override;
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;
    virtual int32_t getAccOneG_Raw() const override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() override;
private:
    gyroRPS_Acc_t gyroRPS_AccFromRaw(const acc_gyro_data_t& data) const;
private:
#if defined(USE_IMU_LSM6DS3TR_C_I2C)
    BUS_I2C _bus; //!< I2C bus interface
#else
    BUS_SPI _bus; //!< SPI bus interface,
#endif
};
