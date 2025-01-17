#if defined(USE_IMU_BMI270)

#include "IMU_BMI270.h"
#include <array>
#include <cmath>
#include <esp32-hal.h>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float degreesToRadians {M_PI / 180.0};
constexpr float ACC_16G_RES { 16.0 / 32768.0 };
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float GYRO_2000DPS_RES_RADIANS { degreesToRadians * GYRO_2000DPS_RES };
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
IMU_BMI270::IMU_BMI270(uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex) :
    IMU_Base(i2cMutex),
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
}

void IMU_BMI270::setGyroOffset(const xyz_int16_t& gyroOffset)
{
    _gyroOffset = gyroOffset;
}

void IMU_BMI270::setAccOffset(const xyz_int16_t& accOffset)
{
    _accOffset = accOffset;
}

xyz_int16_t IMU_BMI270::readGyroRaw() const
{
    xyz_int16_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readBytes(REG_OUTX_L_G, reinterpret_cast<uint8_t*>(&gyro), sizeof(gyro)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyro;
}

xyz_t IMU_BMI270::readGyroRPS() const
{
    const gyroRPS_Acc_t gyroAcc = readGyroRPS_Acc();
    return gyroAcc.gyroRPS;
}

xyz_t IMU_BMI270::readGyroDPS() const
{
    const gyroRPS_Acc_t gyroAcc = readGyroRPS_Acc();
    return gyroAcc.gyroRPS * radiansToDegrees;
}

xyz_int16_t IMU_BMI270::readAccRaw() const
{
    xyz_int16_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readBytes(REG_OUTX_L_ACC, reinterpret_cast<uint8_t*>(&acc), sizeof(acc)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return acc;
}

xyz_t IMU_BMI270::readAcc() const
{
    const gyroRPS_Acc_t gyroAcc = readGyroRPS_Acc();
    return gyroAcc.acc;
}

IMU_Base::gyroRPS_Acc_t IMU_BMI270::readGyroRPS_Acc() const
{
    acc_gyro_data_t data; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readBytes(REG_OUTX_L_ACC, reinterpret_cast<uint8_t*>(&data), sizeof(data)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyroRPS_AccFromRaw(data, _gyroOffset, _accOffset);
}

IMU_BMI270::gyroRPS_Acc_t IMU_BMI270::gyroRPS_AccFromRaw(const acc_gyro_data_t& data, const xyz_int16_t& gyroOffset, const xyz_int16_t& accOffset)
{
    return gyroRPS_Acc_t {
// NOLINTBEGIN(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions) avoid "narrowing conversion from int to float" warnings
#if defined(IMU_X_AXIS_FRONT_Y_AXIS_LEFT)
        .gyroRPS = {
            .x = -(data.gyro.y - gyroOffset.y) * GYRO_2000DPS_RES_RADIANS,
            .y =  (data.gyro.x - gyroOffset.x) * GYRO_2000DPS_RES_RADIANS,
            .z =  (data.gyro.z - gyroOffset.z) * GYRO_2000DPS_RES_RADIANS
        },
        .acc = {
            .x = -(data.acc.y - accOffset.y) * ACC_16G_RES,
            .y =  (data.acc.x - accOffset.x) * ACC_16G_RES,
            .z =  (data.acc.z - accOffset.z) * ACC_16G_RES
        }
#elif defined(IMU_X_AXIS_BACK_Y_AXIS_RIGHT)
        .gyroRPS = {
            .x =  (data.gyro.y - gyroOffset.y) * GYRO_2000DPS_RES_RADIANS,
            .y = -(data.gyro.x - gyroOffset.x) * GYRO_2000DPS_RES_RADIANS,
            .z =  (data.gyro.z - gyroOffset.z) * GYRO_2000DPS_RES_RADIANS
        },
        .acc = {
            .x =  (data.acc.y - accOffset.y) * ACC_16G_RES,
            .y = -(data.acc.x - accOffset.x) * ACC_16G_RES,
            .z =  (data.acc.z - accOffset.z) * ACC_16G_RES)
        }
#elif defined(IMU_X_AXIS_RIGHT_Y_AXIS_DOWN)
        .gyroRPS = {
            .x =  (data.gyro.x - gyroOffset.x) * GYRO_2000DPS_RES_RADIANS,
            .y =  (data.gyro.z - gyroOffset.z) * GYRO_2000DPS_RES_RADIANS,
            .z = -(data.gyro.y - gyroOffset.y) * GYRO_2000DPS_RES_RADIANS
        },
        .acc = {
            .x =  (data.acc.x - accOffset.x) * ACC_16G_RES,
            .y =  (data.acc.z - accOffset.z) * ACC_16G_RES,
            .z = -(data.acc.y - accOffset.y) * ACC_16G_RES
        }
#elif defined(IMU_X_AXIS_RIGHT_Y_AXIS_FRONT)
        .gyroRPS = {
            .x = (data.gyro.x - gyroOffset.x) * GYRO_2000DPS_RES_RADIANS,
            .y = (data.gyro.y - gyroOffset.y) * GYRO_2000DPS_RES_RADIANS,
            .z = (data.gyro.z - gyroOffset.z) * GYRO_2000DPS_RES_RADIANS
        },
        .acc = {
            .x  = (data.acc.x - accOffset.x) * ACC_16G_RES,
            .y  = (data.acc.y - accOffset.y) * ACC_16G_RES,
            .z  = (data.acc.z - accOffset.z) * ACC_16G_RES
        }
#else
    static_assert(false && "IMU orientation not implemented for BMI270.");
#endif
// NOLINTEND(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    };
}

/*!
It seems the LSM303AGR does not properly support bulk reading from the FIFO.
*/
int  IMU_BMI270::readFIFO_ToBuffer()
{
    return 0;
}


IMU_Base::gyroRPS_Acc_t IMU_BMI270::readFIFO_Item(size_t index)
{
    (void)index;

    gyroRPS_Acc_t gyroAcc {};
    return gyroAcc;
}

#endif
