#pragma once

#include <I2C.h>
#include <IMU_Base.h>


class  IMU_LSM303AGR : public IMU_Base {
public:
    IMU_LSM303AGR(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex);
    IMU_LSM303AGR(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin) :  IMU_LSM303AGR(axisOrder, SDA_pin, SCL_pin, nullptr) {}
    void init();
public:
    static constexpr uint8_t I2C_ADDRESS=0x19;

#pragma pack(push, 1)
    struct mems_sensor_data_t {
        enum { DATA_SIZE = 6 };
        uint8_t x_l;
        uint8_t x_h;
        uint8_t y_l;
        uint8_t y_h;
        uint8_t z_l;
        uint8_t z_h;
    };
    struct gyro_acc_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 12 };
        uint16_t gyro_x;
        uint16_t gyro_y;
        uint16_t gyro_z;
        uint16_t acc_x;
        uint16_t acc_y;
        uint16_t acc_z;
    };
    union gyro_acc_array_t {
        enum { DATA_SIZE = 1032 };
        gyro_acc_data_t gyroAcc[86];
        uint8_t data[DATA_SIZE];
    };
#pragma pack(pop)
public:
    virtual xyz_int16_t readAccRaw() const override;
    virtual xyz_int16_t readGyroRaw() const override;

    virtual gyroRPS_Acc_t readGyroRPS_Acc() const override;

    virtual int readFIFO_ToBuffer() override;
    virtual gyroRPS_Acc_t  readFIFO_Item(size_t index) override;
private:
    gyroRPS_Acc_t gyroRPS_AccFromRaw(const gyro_acc_data_t& data) const;
private:
    I2C _bus; //!< Serial Communication Bus interface, can be either I2C or SPI
    gyro_acc_array_t _fifoBuffer {};
};
