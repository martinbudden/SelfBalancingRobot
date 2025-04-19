#if defined(USE_IMU_ISM330DHCX_I2C) || defined(USE_IMU_ISM330DHCX_SPI)

#include "IMU_ISM330DHCX.h"
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif
#include <cassert>

// see https://github.com/STMicroelectronics/lsm6ds3tr-c-pid
// see https://github.com/STMicroelectronics/ism330dhcx-pid

namespace { // use anonymous namespace to make items local to this translation unit

constexpr uint8_t REG_RESERVED_00           = 0x00;
constexpr uint8_t REG_FUNC_CFG_ACCESS       = 0x01;

#if defined(USE_IMU_ISM330DHCX_I2C) || defined(USE_IMU_ISM330DHCX_SPI)

constexpr uint8_t REG_PIN_CTRL              = 0x02;
constexpr uint8_t REG_RESERVED_03           = 0x03;
constexpr uint8_t REG_RESERVED_04           = 0x04;
constexpr uint8_t REG_RESERVED_05           = 0x05;
constexpr uint8_t REG_RESERVED_06           = 0x06;
constexpr uint8_t REG_FIFO_CTRL1            = 0x07;
constexpr uint8_t REG_FIFO_CTRL2            = 0x08;
constexpr uint8_t REG_FIFO_CTRL3            = 0x09;
constexpr uint8_t REG_FIFO_CTRL4            = 0x0A;
constexpr uint8_t REG_COUNTER_BDR_REG1      = 0x0B;
constexpr uint8_t REG_COUNTER_BDR_REG2      = 0x0C;
constexpr uint8_t REG_ALL_INT_SRC           = 0x1A;

#elif defined(USE_IMU_LSM6DSOX_I2C) || defined(USE_IMU_LSM6DSOX_SPI)

constexpr uint8_t REG_PIN_CTRL              = 0x02;
constexpr uint8_t REG_RESERVED_03           = 0x03;
constexpr uint8_t REG_S4S_TPH_L             = 0x04;
constexpr uint8_t REG_S4S_TPH_H             = 0x05;
constexpr uint8_t REG_S4S_RR                = 0x06;
constexpr uint8_t REG_FIFO_CTRL1            = 0x07;
constexpr uint8_t REG_FIFO_CTRL2            = 0x08;
constexpr uint8_t REG_FIFO_CTRL3            = 0x09;
constexpr uint8_t REG_FIFO_CTRL4            = 0x0A;
constexpr uint8_t REG_COUNTER_BDR_REG1      = 0x0B;
constexpr uint8_t REG_COUNTER_BDR_REG2      = 0x0C;
constexpr uint8_t REG_ALL_INT_SRC           = 0x1A;

#endif

constexpr uint8_t REG_INT1_CTRL             = 0x0D;
    constexpr uint8_t INT1_DRDY_G           =0b00000010;
constexpr uint8_t REG_INT2_CTRL             = 0x0E;
    constexpr uint8_t INT2_DRDY_G           =0b00000010;
constexpr uint8_t REG_WHO_AM_I              = 0x0F;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C = 0x6A;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE_ISM330DHCX = 0x6B;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE_LSM6DSOX = 0x6C;
constexpr uint8_t REG_CTRL1_XL              = 0x10;
    constexpr uint8_t ACC_RANGE_2G    =     0b0000;
    constexpr uint8_t ACC_RANGE_4G    =     0b1000;
    constexpr uint8_t ACC_RANGE_8G    =     0b1100;
    constexpr uint8_t ACC_RANGE_16G   =     0b0100;
    constexpr uint8_t ACC_ODR_12p5_HZ = 0b00010000;
    constexpr uint8_t ACC_ODR_26_HZ   = 0b00100000;
    constexpr uint8_t ACC_ODR_52_HZ   = 0b00110000;
    constexpr uint8_t ACC_ODR_104_HZ  = 0b01000000;
    constexpr uint8_t ACC_ODR_208_HZ  = 0b010100000;
    constexpr uint8_t ACC_ODR_416_HZ  = 0b01100000;
    constexpr uint8_t ACC_ODR_833_HZ  = 0b01110000;
    constexpr uint8_t ACC_ODR_1666_HZ = 0b10000000;
    constexpr uint8_t ACC_ODR_3332_HZ = 0b10010000;
    constexpr uint8_t ACC_ODR_6664_HZ = 0b10100000;
constexpr uint8_t REG_CTRL2_G               = 0x11;
    constexpr uint8_t GYRO_RANGE_125_DPS   = 0b0010;
    constexpr uint8_t GYRO_RANGE_245_DPS   = 0b0000; // LSM6DS3TR_C
    constexpr uint8_t GYRO_RANGE_250_DPS   = 0b0000; // ISM330DHCX, LSM6DSOX
    constexpr uint8_t GYRO_RANGE_500_DPS   = 0b0100;
    constexpr uint8_t GYRO_RANGE_1000_DPS  = 0b1000;
    constexpr uint8_t GYRO_RANGE_2000_DPS  = 0b1100;
    constexpr uint8_t GYRO_RANGE_4000_DPS  = 0b0001; // ISM330DHCX only
    constexpr uint8_t GYRO_ODR_12p5_HZ = 0b00010000;
    constexpr uint8_t GYRO_ODR_26_HZ   = 0b00100000;
    constexpr uint8_t GYRO_ODR_52_HZ   = 0b00110000;
    constexpr uint8_t GYRO_ODR_104_HZ  = 0b01000000;
    constexpr uint8_t GYRO_ODR_208_HZ  = 0b010100000;
    constexpr uint8_t GYRO_ODR_416_HZ  = 0b01100000;
    constexpr uint8_t GYRO_ODR_833_HZ  = 0b01110000;
    constexpr uint8_t GYRO_ODR_1666_HZ = 0b10000000;
    constexpr uint8_t GYRO_ODR_3332_HZ = 0b10010000;
    constexpr uint8_t GYRO_ODR_6664_HZ = 0b10100000;
constexpr uint8_t REG_CTRL3_C               = 0x12;
    constexpr uint8_t BDU                   = 0b01000000;
    constexpr uint8_t IF_INC                = 0b00000100;
    constexpr uint8_t SW_RESET              = 0b00000001;
constexpr uint8_t REG_CTRL4_C               = 0x13;
    constexpr uint8_t I2C_DISABLE           = 0b00000100;
    constexpr uint8_t LPF1_SEL_G            = 0b00000010;
constexpr uint8_t REG_CTRL5_C               = 0x14;
constexpr uint8_t REG_CTRL6_C               = 0x15;
    constexpr uint8_t XL_HM_MODE_DISABLE    = 0b00010000;
    constexpr uint8_t LPF1_MHI              = 0x00;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz - LSM6DS3TR_C: 351Hz, ISM330DHCX: 297Hz, LSM6DSOX: 335.5Hz
    constexpr uint8_t LPF1_MLO              = 0x01;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz - LSM6DS3TR_C: 237Hz, ISM330DHCX: 223Hz, LSM6DSOX: 232.0Hz
    constexpr uint8_t LPF1_LO               = 0x02;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz - LSM6DS3TR_C: 172Hz, ISM330DHCX: 154Hz, LSM6DSOX: 171.1Hz
    constexpr uint8_t LPF1_HI               = 0x03;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz   LSM6DS3TR_C: 937Hz, ISM330DHCX: 470Hz, LSM6DSOX: 609.0Hz
constexpr uint8_t REG_CTRL7_G               = 0x16;
constexpr uint8_t REG_CTRL8_XL              = 0x17;
constexpr uint8_t REG_CTRL9_XL              = 0x18;
constexpr uint8_t REG_CTRL10_C              = 0x19;
constexpr uint8_t REG_WAKE_UP_SRC           = 0x1B;
constexpr uint8_t REG_TAP_SRC               = 0x1C;
constexpr uint8_t REG_D6D_SRC               = 0x1D;
constexpr uint8_t REG_STATUS_REG            = 0x1E;
constexpr uint8_t REG_RESERVED_1F           = 0x1F;

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

} // end namespace

/*!
Gyroscope data rates up to 6.4 kHz, accelerometer up to 1.6 kHz
*/
#if defined(USE_IMU_ISM330DHCX_I2C)
IMU_ISM330DHCX::IMU_ISM330DHCX(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex) :
    IMU_Base(axisOrder, i2cMutex),
    _bus(I2C_ADDRESS, SDA_pin, SCL_pin)
{
}
#else
IMU_ISM330DHCX::IMU_ISM330DHCX(axis_order_t axisOrder, uint8_t CS_pin) :
    IMU_Base(axisOrder),
    _bus(CS_pin)
{
}
#endif

void IMU_ISM330DHCX::init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity) // NOLINT(readability-function-cognitive-complexity)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);

    _bus.writeRegister(REG_CTRL3_C, SW_RESET); // software reset
    delayMs(100);

    const uint8_t chipID = _bus.readRegisterWithTimeout(REG_WHO_AM_I, 100);
#if defined(SERIAL_OUTPUT)
    Serial.printf("IMU init, chipID=%02x\r\n", chipID);
#else
    (void)chipID;
#endif
    //assert(chipID == REG_WHO_AM_I_RESPONSE);
    delayMs(1);

    struct setting_t {
        uint8_t reg;
        uint8_t value;
    };
    static constexpr std::array<setting_t, 6> settings = {
        // Suppress badBitmaskCheck so we can OR with zero values without a warning
        // cppcheck-suppress-begin badBitmaskCheck
        REG_INT1_CTRL,          INT1_DRDY_G, // Enable gyro data ready on INT1 pin
        REG_INT2_CTRL,          INT2_DRDY_G, // Enable gyro data ready on INT2 pin
        REG_CTRL3_C,            BDU | IF_INC, // Block Data Update and automatically increment registers when read via serial interface (I2C or SPI)
#if defined(USE_IMU_ISM330DHCX_I2C)
        REG_CTRL4_C,            LPF1_SEL_G, // enable gyro LPF
#else
        REG_CTRL4_C,            LPF1_SEL_G | I2C_DISABLE,
#endif
        REG_CTRL6_C,            LPF1_HI
        // cppcheck-suppress-end badBitmaskCheck
    };

    for (const setting_t setting : settings) {
        _bus.writeRegister(setting.reg, setting.value);
        delayMs(1);
    }

    const uint8_t gyroOutputDataRate = 
        outputDataRateHz == 0 ? GYRO_ODR_6664_HZ :
        outputDataRateHz > 3332 ? GYRO_ODR_6664_HZ :
        outputDataRateHz > 1666 ? GYRO_ODR_3332_HZ :
        outputDataRateHz > 833 ? GYRO_ODR_1666_HZ :
        outputDataRateHz > 416 ? GYRO_ODR_833_HZ :
        outputDataRateHz > 208 ? GYRO_ODR_416_HZ :
        outputDataRateHz > 104 ? GYRO_ODR_208_HZ :
        outputDataRateHz > 52 ? GYRO_ODR_104_HZ :
        outputDataRateHz > 26 ? GYRO_ODR_52_HZ :
        outputDataRateHz > 13 ? GYRO_ODR_26_HZ : GYRO_ODR_12p5_HZ;
    switch (gyroSensitivity) {
    case GYRO_FULL_SCALE_125_DPS:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_125_DPS | gyroOutputDataRate);
        _gyroResolutionDPS = 245.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_250_DPS:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_245_DPS | gyroOutputDataRate); // cppcheck-suppress badBitmaskCheck
        _gyroResolutionDPS = 245.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_500_DPS:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_500_DPS | gyroOutputDataRate);
        _gyroResolutionDPS = 500.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_1000_DPS:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_1000_DPS | gyroOutputDataRate);
        _gyroResolutionDPS = 1000.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_2000_DPS:
        [[fallthrough]];
    default:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_2000_DPS | gyroOutputDataRate);
        _gyroResolutionDPS = 2000.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_4000_DPS:
        _bus.writeRegister(REG_CTRL2_G, GYRO_RANGE_4000_DPS | gyroOutputDataRate);
        _gyroResolutionDPS = 4000.0F / 32768.0F;
        break;
    }
    _gyroResolutionRPS = _gyroResolutionDPS * degreesToRadians;
    delayMs(1);

    const uint8_t accOutputDataRate = 
        outputDataRateHz == 0 ? ACC_ODR_6664_HZ :
        outputDataRateHz > 3332 ? ACC_ODR_6664_HZ :
        outputDataRateHz > 1666 ? ACC_ODR_3332_HZ :
        outputDataRateHz > 833 ? ACC_ODR_1666_HZ :
        outputDataRateHz > 416 ? ACC_ODR_833_HZ :
        outputDataRateHz > 208 ? ACC_ODR_416_HZ :
        outputDataRateHz > 104 ? ACC_ODR_208_HZ :
        outputDataRateHz > 52 ? ACC_ODR_104_HZ :
        outputDataRateHz > 26 ? ACC_ODR_52_HZ :
        outputDataRateHz > 13 ? ACC_ODR_26_HZ : ACC_ODR_12p5_HZ;
    switch (accSensitivity) {
    case ACC_FULL_SCALE_2G:
        _bus.writeRegister(REG_CTRL1_XL, ACC_RANGE_2G | accOutputDataRate); // cppcheck-suppress badBitmaskCheck
        _accResolution = 2.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_4G:
        _bus.writeRegister(REG_CTRL1_XL, ACC_RANGE_4G | accOutputDataRate);
        _accResolution = 4.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_8G:
        _bus.writeRegister(REG_CTRL1_XL, ACC_RANGE_8G | accOutputDataRate);
        _accResolution = 8.0F / 32768.0F;
        break;
    default:
        _bus.writeRegister(REG_CTRL1_XL, ACC_RANGE_16G | accOutputDataRate);
        _accResolution = 16.0F / 32768.0F;
        break;
    }
    delayMs(1);
}

IMU_Base::xyz_int32_t IMU_ISM330DHCX::readGyroRaw()
{
    xyz_int32_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_G, reinterpret_cast<uint8_t*>(&gyro), sizeof(gyro)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyro;
}

IMU_Base::xyz_int32_t IMU_ISM330DHCX::readAccRaw()
{
    xyz_int32_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_ACC, reinterpret_cast<uint8_t*>(&acc), sizeof(acc)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return acc;
}

int32_t IMU_ISM330DHCX::getAccOneG_Raw() const
{
    return 2048;
}

IMU_Base::gyroRPS_Acc_t IMU_ISM330DHCX::readGyroRPS_Acc()
{
    i2cSemaphoreTake();
    _bus.readRegister(REG_OUTX_L_G, reinterpret_cast<uint8_t*>(&_accGyroData), sizeof(_accGyroData)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyroRPS_AccFromRaw(_accGyroData);
}

IMU_Base::gyroRPS_Acc_t IMU_ISM330DHCX::gyroRPS_AccFromRaw(const acc_gyro_data_t& data) const
{
#if defined(IMU_BUILD_XPOS_YPOS_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x =  static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
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
#elif defined(IMU_BUILD_XNEG_YNEG_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x = -static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y = -static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y = -static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_YNEG_XPOS_ZPOS)
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
#else
    // Axis order mapping done at run-time
    const gyroRPS_Acc_t gyroRPS_Acc {
        .gyroRPS = {
            .x =  static_cast<float>(data.gyro_x - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(data.gyro_y - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(data.gyro_z - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(data.acc_x - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(data.acc_y - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(data.acc_z - _accOffset.z)* _accResolution
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
                .x = -gyroRPS_Acc.acc.x,
                .y =  gyroRPS_Acc.acc.z,
                .z = -gyroRPS_Acc.acc.y
            }
        };
        break;
    default:
        return gyroRPS_Acc_t {
            .gyroRPS = mapAxes(gyroRPS_Acc.gyroRPS),
            .acc = mapAxes(gyroRPS_Acc.acc)
        };
        break;
    } // end switch

    return gyroRPS_Acc;
#endif
}
#endif
