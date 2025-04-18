#pragma once

#include <BUS_I2C.h>
#include <BUS_SPI.h>
#include <IMU_Base.h>


class IMU_LSM303AGR : public IMU_Base {
public:
    static constexpr uint8_t I2C_ADDRESS=0x19;

#pragma pack(push, 1)
    struct mems_sensor_data_t {
        enum { DATA_SIZE = 6 };
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct gyro_acc_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 12 };
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
    };
    union gyro_acc_array_t {
        enum { DATA_SIZE = 1032 };
        std::array<gyro_acc_data_t, 86> gyroAcc;
        std::array<uint8_t, DATA_SIZE> data;
    };
#pragma pack(pop)

public:
    IMU_LSM303AGR(axis_order_t axisOrder, uint8_t CS_pin);
    IMU_LSM303AGR(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex);
    IMU_LSM303AGR(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin) : IMU_LSM303AGR(axisOrder, SDA_pin, SCL_pin, nullptr) {}
public:
    virtual void init() override;
    virtual xyz_int32_t readAccRaw() override;
    virtual xyz_int32_t readGyroRaw() override;

    virtual gyroRPS_Acc_t readGyroRPS_Acc() override;

    virtual size_t readFIFO_ToBuffer() override;
    virtual gyroRPS_Acc_t readFIFO_Item(size_t index) override;
private:
    gyroRPS_Acc_t gyroRPS_AccFromRaw(const gyro_acc_data_t& data) const;
private:
#if defined(USE_IMU_LSM303AGR_I2C)
    BUS_I2C _bus; //!< I2C bus interface
#else
    BUS_SPI _bus; //!< SPI bus interface,
#endif
    gyro_acc_array_t _fifoBuffer {};
};
