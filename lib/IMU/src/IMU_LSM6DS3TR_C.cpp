#if defined(USE_IMU_LSM6DS3TR_C_I2C) || defined(USE_IMU_LSM6DS3TR_C_SPI)

#include "IMU_LSM6DS3TR_C.h"
#include <array>
#include <cassert>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_16G_RES { 16.0 / 32768.0 };
} // end namespace


constexpr uint8_t REG_RESERVED1             = 0x00;
constexpr uint8_t REG_FUNC_CFG_ACCESS       = 0x01;
constexpr uint8_t REG_RESERVED2             = 0x02;
constexpr uint8_t REG_RESERVED3             = 0x03;
constexpr uint8_t REG_SENSOR_SYNC_TIME_FRAME= 0x04;
constexpr uint8_t REG_SENSOR_SYNC_RES_RATIO = 0x05;
constexpr uint8_t REG_FIFO_CTRL1            = 0x06;
constexpr uint8_t REG_FIFO_CTRL2            = 0x07;
constexpr uint8_t REG_FIFO_CTRL3            = 0x08;
constexpr uint8_t REG_FIFO_CTRL4            = 0x09;
constexpr uint8_t REG_FIFO_CTRL5            = 0x0A;
constexpr uint8_t REG_DRDY_PULSE_CFG_G      = 0x0B;
constexpr uint8_t REG_RESERVED4             = 0x0C;
constexpr uint8_t REG_INT1_CTRL             = 0x0D;
constexpr uint8_t REG_INT2_CTRL             = 0x0E;
constexpr uint8_t REG_WHO_AM_I              = 0x0F;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE = 0x6A;

constexpr uint8_t REG_CTRL1_XL              = 0x10;
    constexpr uint8_t ACC_RANGE_2G  = 0b0000;
    constexpr uint8_t ACC_RANGE_4G  = 0b1000;
    constexpr uint8_t ACC_RANGE_8G  = 0b1100;
    constexpr uint8_t ACC_RANGE_16G = 0b0100;
    constexpr uint8_t ODR_416_HZ =  0b01100000;
    constexpr uint8_t ODR_833_HZ =  0b01110000;
    constexpr uint8_t ODR_1660_HZ = 0b10000000;
    constexpr uint8_t ODR_3330_HZ = 0b10010000;
    constexpr uint8_t ODR_6660_HZ = 0b10100000;
constexpr uint8_t REG_CTRL2_G               = 0x11;
    constexpr uint8_t GYRO_RANGE_245_DPS    = 0b0000;
    constexpr uint8_t GYRO_RANGE_500_DPS    = 0b0100;
    constexpr uint8_t GYRO_RANGE_1000_DPS   = 0b1000;
    constexpr uint8_t GYRO_RANGE_2000_DPS   = 0b1100;
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
constexpr uint8_t REG_RESERVED5             = 0x1F;

constexpr uint8_t REG_OUT_TEMP_L            = 0x20;
constexpr uint8_t REG_OUT_TEMP_H            = 0x22;
constexpr uint8_t REG_OUTX_L_G              = 0x22;
constexpr uint8_t REG_OUTX_H_G              = 0x23;
constexpr uint8_t REG_OUTY_L_G              = 0x24;
constexpr uint8_t REG_OUTY_H_G              = 0x25;
constexpr uint8_t REG_OUTZ_L_G              = 0x26;
constexpr uint8_t REG_OUTZ_H_G              = 0x27;
constexpr uint8_t REG_OUTX_L_ACC            = 0x28;
constexpr uint8_t REG_OUTX_H_ACC            = 0x29;
constexpr uint8_t REG_OUTY_L_ACC            = 0x2A;
constexpr uint8_t REG_OUTY_H_ACC            = 0x2B;
constexpr uint8_t REG_OUTZ_L_ACC            = 0x2C;
constexpr uint8_t REG_OUTZ_H_ACC            = 0x2D;


/*!
Gyroscope data rates up to 6.4 kHz, accelerometer up to 1.6 kHz
*/
#if defined(USE_IMU_LSM6DS3TR_C_I2C)
IMU_LSM6DS3TR_C::IMU_LSM6DS3TR_C(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex) :
    IMU_Base(axisOrder, i2cMutex),
    _bus(I2C_ADDRESS, SDA_pin, SCL_pin)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);
    init();
}
#else
IMU_LSM6DS3TR_C::IMU_LSM6DS3TR_C(axis_order_t axisOrder) :
    IMU_Base(axisOrder)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);
    init();
}
#endif

void IMU_LSM6DS3TR_C::init()
{
    _bus.writeRegister(REG_CTRL3_C, 0x01); // software reset
    delayMs(100);

    const uint8_t chipID = _bus.readRegister(REG_WHO_AM_I);
    assert(chipID == REG_WHO_AM_I_RESPONSE);
    delayMs(1);

    struct setting_t {
        uint8_t reg;
        uint8_t value;
    };
    static constexpr std::array<setting_t, 8> settings = {
        // cppcheck-suppress-begin badBitmaskCheck // so we can OR with zero values without a warning
        REG_INT1_CTRL,          0x02, // Enable gyro data ready on INT1 pin
        REG_INT2_CTRL,          0x02, // Enable gyro data ready on INT1 pin
        REG_CTRL1_XL,           ODR_6660_HZ | ACC_RANGE_16G, // bandwidth selection bits 00
        REG_CTRL2_G,            ODR_6660_HZ | GYRO_RANGE_2000_DPS,
        REG_CTRL3_C,            0,
        REG_CTRL4_C,            0,
        REG_CTRL6_C,            0,
        REG_CTRL9_XL,           0
        // cppcheck-suppress-end badBitmaskCheck
    };

    for (const setting_t setting : settings) {
        _bus.writeRegister(setting.reg, setting.value);
        delayMs(1);
    }
    _gyroResolutionDPS = GYRO_2000DPS_RES;
    _gyroResolutionRPS = GYRO_2000DPS_RES * degreesToRadians;
    _accResolution = ACC_16G_RES;
}

IMU_Base::xyz_int32_t IMU_LSM6DS3TR_C::readGyroRaw()
{
    xyz_int32_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_G, reinterpret_cast<uint8_t*>(&gyro), sizeof(gyro)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyro;
}

IMU_Base::xyz_int32_t IMU_LSM6DS3TR_C::readAccRaw()
{
    xyz_int32_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_ACC, reinterpret_cast<uint8_t*>(&acc), sizeof(acc)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return acc;
}

int32_t IMU_LSM6DS3TR_C::getAccOneG_Raw() const
{
    return 2048;
}

IMU_Base::gyroRPS_Acc_t IMU_LSM6DS3TR_C::readGyroRPS_Acc()
{
    acc_gyro_data_t data; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_G, reinterpret_cast<uint8_t*>(&data), sizeof(data)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyroRPS_AccFromRaw(data);
}

IMU_Base::gyroRPS_Acc_t IMU_LSM6DS3TR_C::gyroRPS_AccFromRaw(const acc_gyro_data_t& data) const
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

#endif
