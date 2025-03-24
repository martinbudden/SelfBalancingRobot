#if defined(USE_IMU_LSM303AGR_I2C) || defined(USE_IMU_LSM303AGR_SPI)

#include "IMU_LSM303AGR.h"
#include <array>
#include <cassert>
//#include <delay.h>

// see https://github.com/STMicroelectronics/lsm6ds3tr-c-pid

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_8G_RES { 8.0 / 32768.0 };
} // end namespace


// 0x00 Reserved
constexpr uint8_t REG_FUNC_CFG_ACCESS       = 0x01;
// 0x02 Reserved
// 0x03 Reserved
constexpr uint8_t REG_SENSOR_SYNC_TIME_FRAME= 0x04;
constexpr uint8_t REG_SENSOR_SYNC_RES_RATIO = 0x05;
constexpr uint8_t REG_FIFO_CTRL1            = 0x06;
constexpr uint8_t REG_FIFO_CTRL2            = 0x07;
constexpr uint8_t REG_FIFO_CTRL3            = 0x08;
constexpr uint8_t REG_FIFO_CTRL4            = 0x09;
constexpr uint8_t REG_FIFO_CTRL5            = 0x0A;
constexpr uint8_t REG_DRDY_PULSE_CFG_G      = 0x0B;
// 0x0C Reserved
 constexpr uint8_t REG_INT1_CTRL            = 0x0D;
constexpr uint8_t REG_INT2_CTRL             = 0x0E;
constexpr uint8_t REG_WHO_AM_I              = 0x0F;

constexpr uint8_t REG_CTRL1_XL              = 0x10;
constexpr uint8_t REG_CTRL2_G               = 0x11;
// bit values for REG_CTRL1_XL and REG_CTRL2_G
    constexpr uint8_t ODR_416_HZ  = 0b01100000;
    constexpr uint8_t ODR_833_HZ  = 0b01110000;
    constexpr uint8_t ODR_1660_HZ = 0b10000000;
    constexpr uint8_t ODR_3330_HZ = 0b10010000;
    constexpr uint8_t ODR_6660_HZ = 0b10100000;
    constexpr uint8_t FS_XL_8G = 0b00001100;
    constexpr uint8_t FS_G_2000_DPS = 0b00000011;


constexpr uint8_t REG_CTRL3_C               = 0x12;
constexpr uint8_t REG_CTRL4_C               = 0x13;
constexpr uint8_t REG_CTRL5_C               = 0x14;
constexpr uint8_t REG_CTRL6_C               = 0x15;
constexpr uint8_t REG_CTRL7_G               = 0x16;
constexpr uint8_t REG_CTRL8_XL              = 0x17;
constexpr uint8_t REG_CTRL9_XL              = 0x18;
constexpr uint8_t REG_CTRL10_C              = 0x19;
constexpr uint8_t REG_MASTER_CONFIG         = 0x1A;
constexpr uint8_t REG_WAKE_UP_SRC           = 0x1B;
constexpr uint8_t REG_TAP_SRC               = 0x1C;
constexpr uint8_t REG_D6D_SRC               = 0x1D;
constexpr uint8_t REG_STATUS_REG            = 0x1E;

constexpr uint8_t REG_OUT_TEMP_L            = 0x20;
constexpr uint8_t REG_OUTX_L_G              = 0x22;
constexpr uint8_t REG_OUTX_H_G              = 0x23;
constexpr uint8_t REG_OUTY_L_G              = 0x24;
constexpr uint8_t REG_OUTY_H_G              = 0x25;
constexpr uint8_t REG_OUTZ_L_G              = 0x26;
constexpr uint8_t REG_OUTZ_H_G              = 0x27;
constexpr uint8_t REG_OUTX_L_XL             = 0x28;
constexpr uint8_t REG_OUTX_H_XL             = 0x29;
constexpr uint8_t REG_OUTY_L_XL             = 0x2A;
constexpr uint8_t REG_OUTY_H_XL             = 0x2B;
constexpr uint8_t REG_OUTZ_L_XL             = 0x2C;
constexpr uint8_t REG_OUTZ_H_XL             = 0x2D;


#if defined(USE_IMU_LSM303AGR_I2C)
IMU_LSM303AGR::IMU_LSM303AGR(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex) :
    IMU_Base(axisOrder, i2cMutex),
    _bus(I2C_ADDRESS, SDA_pin, SCL_pin)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(gyro_acc_data_t) == gyro_acc_data_t::DATA_SIZE);
    static_assert(sizeof(gyro_acc_array_t) == gyro_acc_array_t::DATA_SIZE);
    init();
}
#else
IMU_LSM303AGR::IMU_LSM303AGR(axis_order_t axisOrder) :
    IMU_Base(axisOrder)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(gyro_acc_data_t) == gyro_acc_data_t::DATA_SIZE);
    static_assert(sizeof(gyro_acc_array_t) == gyro_acc_array_t::DATA_SIZE);
    init();
}
#endif

void IMU_LSM303AGR::init()
{
    // Clear all interrupt settings, this is the default
    _bus.writeRegister(REG_INT1_CTRL, 0x00);
    _bus.writeRegister(REG_INT2_CTRL, 0x00);

    //_bus.writeRegister(REG_CTRL1_XL, ODR_416_HZ | FS_XL_8G); // need to check other bits

    //_bus.writeRegister(REG_CTRL2_G, ODR_416_HZ | FS_G_2000_DPS); // need to check other bits

    struct setting_t {
        uint8_t reg;
        uint8_t value;
    };
    static constexpr std::array<setting_t, 4> settings = {
        REG_INT1_CTRL,      0x00,
        REG_INT2_CTRL,      0x00,
        REG_CTRL1_XL,       ODR_416_HZ | FS_XL_8G,
        REG_CTRL2_G,        ODR_416_HZ | FS_G_2000_DPS,
    };

    for (setting_t setting : settings) {
        //delay(1);
        int retryCount = 4;
        while (_bus.readRegister(setting.reg) != setting.value && --retryCount) {
            _bus.writeRegister(setting.reg, setting.value);
        }
    }
    _gyroResolutionDPS = GYRO_2000DPS_RES;
    _gyroResolutionRPS = GYRO_2000DPS_RES * degreesToRadians;
    _accResolution = ACC_8G_RES;
}

IMU_Base::xyz_int32_t IMU_LSM303AGR::readGyroRaw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_G, reinterpret_cast<uint8_t*>(&gyro), sizeof(gyro)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return xyz_int32_t {
        .x = gyro.x,
        .y = gyro.y,
        .z = gyro.z
    };
}

IMU_Base::xyz_int32_t IMU_LSM303AGR::readAccRaw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_XL, reinterpret_cast<uint8_t*>(&acc), sizeof(acc)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return xyz_int32_t {
        .x = acc.x,
        .y = acc.y,
        .z = acc.z
    };
}

IMU_Base::gyroRPS_Acc_t IMU_LSM303AGR::readGyroRPS_Acc()
{
    gyro_acc_data_t data; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_G, reinterpret_cast<uint8_t*>(&data), sizeof(data)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyroRPS_AccFromRaw(data);
}

IMU_Base::gyroRPS_Acc_t IMU_LSM303AGR::gyroRPS_AccFromRaw(const gyro_acc_data_t& data) const
{
#if defined(IMU_BUILD_YNEG_XPOS_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x = -static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .y =  static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .y =  static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_YPOS_XNEG_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x =  static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .y = -static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .y = -static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_XPOS_ZPOS_YNEG)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x =  static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS,
            .z = -static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution,
            .z = -static_cast<float>(data.acc_y - _accOffset.y)* _accResolution
        }
    };
#elif defined(IMU_BUILD_XPOS_YPOS_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x = static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y = static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .z = static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x  = static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y  = static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .z  = static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
        }
    };
#else
    // Axis order mapping done at run-time
    const gyroRPS_Acc_t gyroRPS_Acc {
        .gyroRPS = {
            .x = static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y = static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .z = static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x  = static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y  = static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .z  = static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
        }
    };
    switch (_axisOrder) {
    case XPOS_YPOS_ZPOS:
        return gyroRPS_Acc;
        break;
    case YNEG_XPOS_ZPOS:
        return gyroRPS_Acc_t {
            .gyroRPS = {
                .x = -gyroRPS_Acc.gyroRPS.y,
                .y =  gyroRPS_Acc.gyroRPS.x,
                .z =  gyroRPS_Acc.gyroRPS.z
            },
            .acc = {
                .x = -gyroRPS_Acc.acc.y,
                .y =  gyroRPS_Acc.acc.x,
                .z =  gyroRPS_Acc.acc.z
            }
        };
        break;
    case XNEG_YNEG_ZPOS:
        return gyroRPS_Acc_t {
            .gyroRPS = {
                .x = -gyroRPS_Acc.gyroRPS.x,
                .y = -gyroRPS_Acc.gyroRPS.y,
                .z =  gyroRPS_Acc.gyroRPS.z
            },
            .acc = {
                .x = -gyroRPS_Acc.acc.x,
                .y = -gyroRPS_Acc.acc.y,
                .z =  gyroRPS_Acc.acc.z
            }
        };
        break;
    case YPOS_XNEG_ZPOS:
        return gyroRPS_Acc_t {
            .gyroRPS = {
                .x =  gyroRPS_Acc.gyroRPS.y,
                .y = -gyroRPS_Acc.gyroRPS.x,
                .z =  gyroRPS_Acc.gyroRPS.z
            },
            .acc = {
                .x =  gyroRPS_Acc.acc.y,
                .y = -gyroRPS_Acc.acc.x,
                .z =  gyroRPS_Acc.acc.z
            }
        };
        break;
    case XPOS_ZPOS_YNEG:
        return gyroRPS_Acc_t {
            .gyroRPS = {
                .x =  gyroRPS_Acc.gyroRPS.x,
                .y =  gyroRPS_Acc.gyroRPS.z,
                .z = -gyroRPS_Acc.gyroRPS.y
            },
            .acc = {
                .x = -gyroRPS_Acc.acc.z,
                .y =  gyroRPS_Acc.acc.z,
                .z = -gyroRPS_Acc.acc.y
            }
        };
        break;
    default:
        assert(false && "IMU axis order not implemented");
        break;
    } // end switch

    return gyroRPS_Acc;
#endif
}

/*!
It seems the LSM303AGR does not properly support bulk reading from the FIFO.
*/
size_t IMU_LSM303AGR::readFIFO_ToBuffer()
{
    //std::array<uint8_t, 2> lengthData;

    i2cSemaphoreTake();

    i2cSemaphoreGive();

     // return the number of acc_temp_gyro_data_t items read
    const size_t fifoLength = 0;
    return fifoLength  / gyro_acc_array_t::DATA_SIZE;
}


IMU_Base::gyroRPS_Acc_t IMU_LSM303AGR::readFIFO_Item(size_t index)
{
    (void)index;

    gyroRPS_Acc_t gyroAcc {};
    return gyroAcc;
}

#endif
