#if defined(USE_IMU_BMI270)

#include "IMU_BMI270.h"
#include <array>
#include <cassert>
#include <cmath>
#include <esp32-hal.h>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_16G_RES { 16.0 / 32768.0 };
} // end namespace


constexpr uint8_t I2C_ADDRESS               = 0x68;

constexpr uint8_t REG_CHIP_ID               = 0x00;


constexpr uint8_t REG_OUTX_L_ACC            = 0x0C;
constexpr uint8_t REG_OUTX_H_ACC            = 0x0D;
constexpr uint8_t REG_OUTY_L_ACC            = 0x0E;
constexpr uint8_t REG_OUTY_H_ACC            = 0x0F;
constexpr uint8_t REG_OUTZ_L_ACC            = 0x10;
constexpr uint8_t REG_OUTZ_H_ACC            = 0x11;
constexpr uint8_t REG_OUTX_L_G              = 0x12;
constexpr uint8_t REG_OUTX_H_G              = 0x13;
constexpr uint8_t REG_OUTY_L_G              = 0x14;
constexpr uint8_t REG_OUTY_H_G              = 0x15;
constexpr uint8_t REG_OUTZ_L_G              = 0x16;
constexpr uint8_t REG_OUTZ_H_G              = 0x17;

constexpr uint8_t REG_SENSORTIME_0          = 0x18;
constexpr uint8_t REG_SENSORTIME_1          = 0x19;
constexpr uint8_t REG_SENSORTIME_3          = 0x1A;

constexpr uint8_t REG_EVENT                 = 0x1B;
constexpr uint8_t REG_INT_STATUS_0          = 0x1C;
constexpr uint8_t REG_INT_STATUS_1          = 0x1D;
constexpr uint8_t REG_SC_OUT_0              = 0x1E;
constexpr uint8_t REG_SC_OUT_1              = 0x1F;
constexpr uint8_t REG_WR_GEST_ACT           = 0x20;
constexpr uint8_t REG_INTERNAL_STATUS       = 0x21;
constexpr uint8_t REG_TEMPERATURE_0         = 0x22;
constexpr uint8_t REG_TEMPERATURE_1         = 0x23;
constexpr uint8_t REG_FIFO_LENGTH_0         = 0x24;
constexpr uint8_t REG_FIFO_LENGTH_1         = 0x25;
constexpr uint8_t REG_FIFO_DATA             = 0x26;


constexpr uint8_t REG_ACC_CONF              = 0x40;
    constexpr uint8_t PERFORMANCE_OPTIMIZED = 0b10000000;
    constexpr uint8_t ODR_800_HZ = 0x0B;
    constexpr uint8_t ODR_1600_HZ = 0x0C;
    constexpr uint8_t ACC_OSR4_AVG1 = 0b00000000; // no averaging
    constexpr uint8_t ACC_OSR4_AVG2 = 0b00001000; // average 2 samples
constexpr uint8_t REG_ACC_RANGE             = 0x41;
    constexpr uint8_t ACC_RANGE_16G = 0x03;
constexpr uint8_t REG_GYR_CONF              = 0x42;
    constexpr uint8_t GYRO_ODR_3200_HZ = 0x0D; // for gyro only, not for acc
    constexpr uint8_t GYRO_OSR4_AVG = 0x00;
constexpr uint8_t REG_GYR_RANGE             = 0x43;
    constexpr uint8_t GYRO_RANGE_2000 = 0x00;
constexpr uint8_t REG_AUX_CONF              = 0x44;
constexpr uint8_t REG_FIFO_DOWNS            = 0x45;
constexpr uint8_t REG_FIFO_WTM_0            = 0x46;
constexpr uint8_t REG_FIFO_WTM_1            = 0x47;
constexpr uint8_t REG_FIFO_CONFIG_0         = 0x48;
constexpr uint8_t REG_FIFO_CONFIG_1         = 0x49;
    constexpr uint8_t FIFO_HEADER_DISABLE   = 0b00000000; // requires output data rates for gyro and acc to be the same
    constexpr uint8_t FIFO_HEADER_ENABLE    = 0b00001000;
    constexpr uint8_t FIFO_AUX_ENABLE       = 0b01000000;
    constexpr uint8_t FIFO_ACC_ENABLE       = 0b01000000;
    constexpr uint8_t FIFO_GYRO_ENABLE      = 0b10000000;
constexpr uint8_t REG_FIFO_SATURATION       = 0x4A;
constexpr uint8_t REG_AUX_DEV_ID_ADDR       = 0x4B;
constexpr uint8_t REG_AUX_IF_CONF_ADDR      = 0x4C;

constexpr uint8_t REG_IF_CONF_ADDR          = 0x6B;
constexpr uint8_t REG_PWR_CONF_ADDR         = 0x7C;
constexpr uint8_t REG_PWR_CTRL_ADDR         = 0x7D;

/*!
Gyroscope data rates up to 6.4 kHz, accelerometer up to 1.6 kHz
*/
IMU_BMI270::IMU_BMI270(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex) :
    IMU_Base(axisOrder, i2cMutex),
    _bus(I2C_ADDRESS, SDA_pin, SCL_pin)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);
    init();
}

void IMU_BMI270::init()
{
    _bus.writeByte(REG_PWR_CONF_ADDR, 0x00); // Power save disabled.
    delay(1); // 450us is minimum required

    const uint8_t chipID = _bus.readByte(REG_CHIP_ID);
    assert(chipID == 0x24);
    delay(1);

    struct setting_t {
        uint8_t reg;
        uint8_t value;
    };
    static constexpr std::array<setting_t, 6> settings = {
        // cppcheck-suppress-begin badBitmaskCheck // so we can OR with zero values without a warning
        REG_PWR_CTRL_ADDR,          0x00, // Enable acquisition of acc, gyro and temperature sensor data.
        // cppcheck-suppress badBitmaskCheck
        REG_ACC_CONF,               PERFORMANCE_OPTIMIZED | ACC_OSR4_AVG1 | ODR_1600_HZ,
        REG_ACC_RANGE,              ACC_RANGE_16G,
        // cppcheck-suppress badBitmaskCheck
        REG_GYR_CONF,               PERFORMANCE_OPTIMIZED | GYRO_OSR4_AVG | ODR_1600_HZ,
        REG_GYR_RANGE,              GYRO_RANGE_2000,
        // cppcheck-suppress badBitmaskCheck
        REG_FIFO_CONFIG_1,          FIFO_GYRO_ENABLE | FIFO_ACC_ENABLE | FIFO_HEADER_DISABLE
        // cppcheck-suppress-end badBitmaskCheck
    };

    for (const setting_t setting : settings) {
        _bus.writeByte(setting.reg, setting.value);
        delay(1);
    }
    _gyroResolutionDPS = GYRO_2000DPS_RES;
    _gyroResolutionRPS = GYRO_2000DPS_RES * degreesToRadians;
    _accResolution = ACC_16G_RES;
}

xyz_int16_t IMU_BMI270::readGyroRaw() const
{
    xyz_int16_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readBytes(REG_OUTX_L_G, reinterpret_cast<uint8_t*>(&gyro), sizeof(gyro)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyro;
}

xyz_int16_t IMU_BMI270::readAccRaw() const
{
    xyz_int16_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readBytes(REG_OUTX_L_ACC, reinterpret_cast<uint8_t*>(&acc), sizeof(acc)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return acc;
}

IMU_Base::gyroRPS_Acc_t IMU_BMI270::readGyroRPS_Acc() const
{
    acc_gyro_data_t data; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readBytes(REG_OUTX_L_ACC, reinterpret_cast<uint8_t*>(&data), sizeof(data)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyroRPS_AccFromRaw(data);
}

IMU_BMI270::gyroRPS_Acc_t IMU_BMI270::gyroRPS_AccFromRaw(const acc_gyro_data_t& data) const
{
#if defined(IMU_BUILD_YNEG_XPOS_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x = -(data.gyro.y - _gyroOffset.y) * _gyroResolutionRPS,
            .y =  (data.gyro.x - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  (data.gyro.z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -(data.acc.y - _accOffset.y)* _accResolution,
            .y =  (data.acc.x - _accOffset.x)* _accResolution,
            .z =  (data.acc.z - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_YPOS_XNEG_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x =  (data.gyro.y - _gyroOffset.y) * _gyroResolutionRPS,
            .y = -(data.gyro.x - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  (data.gyro.z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  (data.acc.y - _accOffset.y)* _accResolution,
            .y = -(data.acc.x - _accOffset.x)* _accResolution,
            .z =  (data.acc.z - _accOffset.z)* _accResolution)
        }
    };
#elif defined(IMU_BUILD_XPOS_ZPOS_YNEG)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x =  (data.gyro.x - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  (data.gyro.z - _gyroOffset.z) * _gyroResolutionRPS,
            .z = -(data.gyro.y - _gyroOffset.y) * _gyroResolutionRPS
        },
        .acc = {
            .x =  (data.acc.x - _accOffset.x)* _accResolution,
            .y =  (data.acc.z - _accOffset.z)* _accResolution,
            .z = -(data.acc.y - _accOffset.y)* _accResolution
        }
    };
#elif defined(IMU_BUILD_XPOS_YPOS_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x = (data.gyro.x - _gyroOffset.x) * _gyroResolutionRPS,
            .y = (data.gyro.y - _gyroOffset.y) * _gyroResolutionRPS,
            .z = (data.gyro.z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x  = (data.acc.x - _accOffset.x)* _accResolution,
            .y  = (data.acc.y - _accOffset.y)* _accResolution,
            .z  = (data.acc.z - _accOffset.z)* _accResolution
        }
    };
#else
    // Axis order mapping done at run-time
    const gyroRPS_Acc_t gyroRPS_Acc {
        .gyroRPS = {
            .x =  (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x) * _accResolution,
            .y =  (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y) * _accResolution,
            .z =  (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z) * _accResolution
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
