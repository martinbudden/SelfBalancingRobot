#if defined(USE_IMU_MPU6886_DIRECT)

#include "IMU_MPU6886.h"

#include <array>
#if defined(UNIT_TEST_BUILD)
void delay(int) {}
#else
#include <esp32-hal.h>
#endif
#include <cmath>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float degreesToRadians {M_PI / 180.0};
constexpr float ACC_8G_RES { 8.0 / 32768.0 };
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float GYRO_2000DPS_RES_RADIANS { degreesToRadians * GYRO_2000DPS_RES };
} // end namespace


constexpr uint8_t I2C_ADDRESS               = 0x68;

constexpr uint8_t REG_XG_OFFS_TC_H          = 0x04;
constexpr uint8_t REG_XG_OFFS_TC_L          = 0x05;
constexpr uint8_t REG_YG_OFFS_TC_H          = 0x07;
constexpr uint8_t REG_YG_OFFS_TC_L          = 0x08;
constexpr uint8_t REG_ZG_OFFS_TC_H          = 0x0A;
constexpr uint8_t REG_ZG_OFFS_TC_L          = 0x0B;

constexpr uint8_t REG_SELF_TEST_X_ACCEL     = 0x0D;
constexpr uint8_t REG_SELF_TEST_Y_ACCEL     = 0x0E;
constexpr uint8_t REG_SELF_TEST_Z_ACCEL     = 0x0F;

//  GYRO OFFSET ADJUSTMENT REGISTERS
constexpr uint8_t REG_XG_OFFS_USRH          = 0x13;
constexpr uint8_t REG_XG_OFFS_USRL          = 0x14;
constexpr uint8_t REG_YG_OFFS_USRH          = 0x15;
constexpr uint8_t REG_YG_OFFS_USRL          = 0x16;
constexpr uint8_t REG_ZG_OFFS_USRH          = 0x17;
constexpr uint8_t REG_ZG_OFFS_USRL          = 0x18;

constexpr uint8_t REG_SAMPLE_RATE_DIVIDER   = 0x19;
    constexpr uint8_t DIVIDE_BY_1 = 0x00;
    constexpr uint8_t DIVIDE_BY_2 = 0x01;
constexpr uint8_t REG_CONFIG                = 0x1A;
    // Update rate: 1kHz, Filter:176 3-DB BW (Hz), default value used by M5Stack, the least filtered 1kHz update variant
    constexpr uint8_t DLPF_CFG_1 = 0x01;
    // Update rate: 8kHz, Filter:3281 3-DB BW (Hz) - only non-32kHz variant less filtered than DLPF_CFG_1
    constexpr uint8_t DLPF_CFG_7 = 0x07;
constexpr uint8_t REG_GYRO_CONFIG           = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG          = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2         = 0x1D;

constexpr uint8_t REG_FIFO_ENABLE           = 0x23;
    constexpr uint8_t GYRO_FIFO_EN = 0b00001000;
    constexpr uint8_t ACC_FIFO_EN  = 0b00000100;

constexpr uint8_t REG_INT_PIN_CFG           = 0x37;
constexpr uint8_t REG_INT_ENABLE            = 0x38;
constexpr uint8_t FIFO_WM_INT_STATUS        = 0x39; // FIFO watermark interrupt status

constexpr uint8_t REG_ACCEL_XOUT_H          = 0x3B;
constexpr uint8_t REG_ACCEL_XOUT_L          = 0x3C;
constexpr uint8_t REG_ACCEL_YOUT_H          = 0x3D;
constexpr uint8_t REG_ACCEL_YOUT_L          = 0x3E;
constexpr uint8_t REG_ACCEL_ZOUT_H          = 0x3F;
constexpr uint8_t REG_ACCEL_ZOUT_L          = 0x40;

constexpr uint8_t REG_TEMP_OUT_H            = 0x41;
constexpr uint8_t REG_TEMP_OUT_L            = 0x42;

// ACCELEROMETER OFFSET REGISTERS
constexpr uint8_t REG_GYRO_XOUT_H           = 0x43;
constexpr uint8_t REG_GYRO_XOUT_L           = 0x44;
constexpr uint8_t REG_GYRO_YOUT_H           = 0x45;
constexpr uint8_t REG_GYRO_YOUT_L           = 0x46;
constexpr uint8_t REG_GYRO_ZOUT_H           = 0x47;
constexpr uint8_t REG_GYRO_ZOUT_L           = 0x48;

constexpr uint8_t REG_FIFO_WM_TH1           = 0x60; // FIFO watermark threshold in number of bytes
constexpr uint8_t REG_FIFO_WM_TH2           = 0x61;

constexpr uint8_t REG_SIGNAL_PATH_RESET     = 0x68;
constexpr uint8_t REG_ACCEL_INTEL_CTRL      = 0x69;
constexpr uint8_t REG_USER_CTRL             = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1            = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2            = 0x6C;

constexpr uint8_t REG_FIFO_COUNT_H          = 0x72;
constexpr uint8_t REG_FIFO_COUNT_L          = 0x73;
constexpr uint8_t REG_FIFO_R_W              = 0x74;

constexpr uint8_t REG_WHOAMI                = 0x75;

constexpr uint8_t REG_XA_OFFSET_H           = 0x77;
constexpr uint8_t REG_XA_OFFSET_L           = 0x78;
constexpr uint8_t REG_YA_OFFSET_H           = 0x7A;
constexpr uint8_t REG_YA_OFFSET_L           = 0x7B;
constexpr uint8_t REG_ZA_OFFSET_H           = 0x7D;
constexpr uint8_t REG_ZA_OFFSET_L           = 0x7E;

IMU_MPU6886::IMU_MPU6886(uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex) :
    IMU_Base(i2cMutex),
    _bus(I2C_ADDRESS, SDA_pin, SCL_pin)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_temp_gyro_data_t) == acc_temp_gyro_data_t::DATA_SIZE);
    static_assert(sizeof(acc_temp_gyro_array_t) == acc_temp_gyro_array_t::DATA_SIZE);
    init();
}

void IMU_MPU6886::init()
{
    i2cSemaphoreTake();

    _imuID = _bus.readByte(REG_WHOAMI);
    delay(1);

    _bus.writeByte(REG_PWR_MGMT_1, 0); // clear the power management register
    delay(10);

    constexpr uint8_t DEVICE_RESET = 0x01 << 7;
    _bus.writeByte(REG_PWR_MGMT_1, DEVICE_RESET); // reset the device
    delay(10);

    constexpr uint8_t CLKSEL_1 = 0x01;
    _bus.writeByte(REG_PWR_MGMT_1, CLKSEL_1); // CLKSEL must be set to 001 to achieve full gyroscope performance.
    delay(10);

    // Gyro scale is fixed at 2000DPS, the maximum supported.
    constexpr uint8_t GYRO_FCHOICE_B = 0x00; // enables gyro update rate and filter configuration using REG_CONFIG
    _bus.writeByte(REG_GYRO_CONFIG, (GFS_2000DPS << 3) | GYRO_FCHOICE_B); // cppcheck-suppress badBitmaskCheck
    delay(1);

    // Accelerometer scale is fixe at 8G, the maximum supported.
    _bus.writeByte(REG_ACCEL_CONFIG, AFS_8G << 3);
    delay(1);

    constexpr uint8_t ACC_FCHOICE_B = 0x00; // Filter:218.1 3-DB BW (Hz), least filtered 1kHz update variant
    _bus.writeByte(REG_ACCEL_CONFIG2, ACC_FCHOICE_B);
    delay(1);

    constexpr uint8_t FIFO_MODE_OVERWRITE = 0b01000000;
    _bus.writeByte(REG_CONFIG, DLPF_CFG_1 | FIFO_MODE_OVERWRITE);
    delay(1);

    // M5Stack default divider is two, giving 500Hz output rate
    _bus.writeByte(REG_SAMPLE_RATE_DIVIDER, DIVIDE_BY_2);
    delay(1);

    _bus.writeByte(REG_FIFO_ENABLE, 0x00); // FIFO disabled
    delay(1);

    // M5 Unified settings
    //_bus.writeByte(REG_INT_PIN_CFG, 0b11000000); // Active low, open drain 50us pulse width, clear on read
    _bus.writeByte(REG_INT_PIN_CFG, 0x22);
    delay(1);

    constexpr uint8_t DATA_RDY_INT_EN = 0x01;
    _bus.writeByte(REG_INT_ENABLE, DATA_RDY_INT_EN); // data ready interrupt enabled
    delay(10);

    _bus.writeByte(REG_USER_CTRL, 0x00);

    i2cSemaphoreGive();
    delay(1);
}

void IMU_MPU6886::setAccOffset(const xyz_int16_t& accOffset)
{
    _accOffset = accOffset;
}

/*!
Gyro offset adjustment.
These values are used to remove DC bias from the sensor output. The values are 
added to the gyroscope sensor value before going into the sensor register.
So the offset value is negated.
*/
IMU_MPU6886::mems_sensor_data_t IMU_MPU6886::gyroOffsetFromXYZ(const xyz_int16_t& data)
{
    return mems_sensor_data_t {
        .x_h = static_cast<uint8_t>((-data.x) >> 8),
        .x_l = static_cast<uint8_t>((-data.x) & 0xFF),
        .y_h = static_cast<uint8_t>((-data.y) >> 8),
        .y_l = static_cast<uint8_t>((-data.y) & 0xFF),
        .z_h = static_cast<uint8_t>((-data.z) >> 8),
        .z_l = static_cast<uint8_t>((-data.z) & 0xFF)
    };
}

void IMU_MPU6886::setGyroOffset(const xyz_int16_t& gyroOffset)
{
#if false
    // Setting the XG_OFFS registers seems to have no effect, so this code disabled
    const mems_sensor_data_t offset = gyroOffsetFromXYZ(gyroOffset);
    _bus.writeBytes(REG_XG_OFFS_USRH, reinterpret_cast<const uint8_t*>(&offset), sizeof(offset)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    delay(1);
#else
    _gyroOffset = gyroOffset;
#endif
}

xyz_int16_t IMU_MPU6886::readAccRaw() const
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readBytes(REG_ACCEL_XOUT_H, reinterpret_cast<uint8_t*>(&acc), sizeof(acc)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return xyz_int16_t {
        .x = static_cast<int16_t>((acc.x_h << 8) | acc.x_l),
        .y = static_cast<int16_t>((acc.y_h << 8) | acc.y_l),
        .z = static_cast<int16_t>((acc.z_h << 8) | acc.z_l)
    };
}

xyz_t IMU_MPU6886::readAcc() const
{
    const xyz_int16_t acc = readAccRaw();

    return xyz_t {
        .x = static_cast<float>(acc.x - _accOffset.x) * ACC_8G_RES,
        .y = static_cast<float>(acc.y - _accOffset.y) * ACC_8G_RES,
        .z = static_cast<float>(acc.z - _accOffset.z) * ACC_8G_RES
    };
}

xyz_int16_t IMU_MPU6886::readGyroRaw() const
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readBytes(REG_GYRO_XOUT_H, reinterpret_cast<uint8_t*>(&gyro), sizeof(gyro)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return xyz_int16_t {
        .x = static_cast<int16_t>((gyro.x_h << 8) | gyro.x_l),
        .y = static_cast<int16_t>((gyro.y_h << 8) | gyro.y_l),
        .z = static_cast<int16_t>((gyro.z_h << 8) | gyro.z_l)
    };
}

xyz_t IMU_MPU6886::readGyro() const
{
    const xyz_int16_t gyro = readGyroRaw();

    return xyz_t {
        .x = static_cast<float>(gyro.x - _gyroOffset.x) * GYRO_2000DPS_RES,
        .y = static_cast<float>(gyro.y - _gyroOffset.y) * GYRO_2000DPS_RES,
        .z = static_cast<float>(gyro.z - _gyroOffset.z) * GYRO_2000DPS_RES
    };
}

xyz_t IMU_MPU6886::readGyroRadians() const
{
    const xyz_int16_t gyro = readGyroRaw();

    return xyz_t {
        .x = static_cast<float>(gyro.x - _gyroOffset.x) * GYRO_2000DPS_RES_RADIANS,
        .y = static_cast<float>(gyro.y - _gyroOffset.y) * GYRO_2000DPS_RES_RADIANS,
        .z = static_cast<float>(gyro.z - _gyroOffset.z) * GYRO_2000DPS_RES_RADIANS
    };
}

IMU_Base::gyroRadiansAcc_t IMU_MPU6886::readGyroRadiansAcc() const
{
    acc_temp_gyro_data_t data; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

    i2cSemaphoreTake();
    _bus.readBytes(REG_ACCEL_XOUT_H, reinterpret_cast<uint8_t*>(&data), sizeof(data)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    i2cSemaphoreGive();

    return gyroRadiansAccFromData(data, _gyroOffset, _accOffset);
}

bool IMU_MPU6886::readGyroRadiansAcc(xyz_t& gyroRadians, xyz_t& acc) const
{
    const gyroRadiansAcc_t gyroAcc =  readGyroRadiansAcc();
    gyroRadians = gyroAcc.gyroRadians;
    acc = gyroAcc.acc;

    return true;
}

int16_t IMU_MPU6886::readTemperatureRaw() const
{
    std::array<uint8_t, 2> data;

    i2cSemaphoreTake();
    _bus.readBytes(REG_TEMP_OUT_H, &data[0], sizeof(data));
    i2cSemaphoreGive();

    const int16_t temperature = static_cast<int16_t>((data[0] << 8) | data[1]); // NOLINT(hicpp-use-auto,modernize-use-auto)
    return temperature;

}

float IMU_MPU6886::readTemperature() const
{
    const int16_t temperature = readTemperatureRaw();

    return static_cast<float>(temperature) / 326.8F + 25.0F;
}

void IMU_MPU6886::setFIFOEnable(bool enableflag)
{
    i2cSemaphoreTake();
    _bus.writeByte(REG_FIFO_ENABLE, enableflag ? 0x18 : 0x00);
    delay(1);
    _bus.writeByte(REG_USER_CTRL, enableflag ? 0x40 : 0x00);
    i2cSemaphoreGive();
    delay(1);
}

void IMU_MPU6886::resetFIFO()
{
    i2cSemaphoreTake();

    uint8_t data = _bus.readByte(REG_USER_CTRL);
    data |= 0x04;
    _bus.writeByte(REG_USER_CTRL, data);

    i2cSemaphoreGive();
}

int IMU_MPU6886::readFIFO_ToBuffer()
{
    std::array<uint8_t, 2> lengthData;

    i2cSemaphoreTake();

    _bus.readBytes(REG_FIFO_COUNT_H, &lengthData[0], sizeof(lengthData));
    const uint16_t fifoLength = lengthData[0] << 8 | lengthData[1];

    constexpr size_t chunkSize = 8*sizeof(acc_temp_gyro_data_t);
    const int count = fifoLength / chunkSize;
    for (int ii = 0; ii < count; ++ii) {
        _bus.readBytes(REG_FIFO_R_W, &_fifoBuffer.data[ii * chunkSize], chunkSize); // NOLINT(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-pro-bounds-constant-array-index)
    }
    _bus.readBytes(REG_FIFO_R_W, &_fifoBuffer.data[count * chunkSize], fifoLength - count*chunkSize); // NOLINT(cppcoreguidelines-pro-type-union-access)

    i2cSemaphoreGive();

     // return the number of acc_temp_gyro_data_t items read
    return fifoLength  / acc_temp_gyro_data_t::DATA_SIZE;
}

void IMU_MPU6886::readFIFO_Item(xyz_t& gyroRadians, xyz_t& acc, size_t index)
{
    const acc_temp_gyro_data_t& accTempGyro = _fifoBuffer.accTempGyro[index]; // NOLINT(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-pro-bounds-constant-array-index)
    const gyroRadiansAcc_t gyroRadiansAcc = gyroRadiansAccFromData(accTempGyro, _gyroOffset, _accOffset);

    gyroRadians = gyroRadiansAcc.gyroRadians;
    acc = gyroRadiansAcc.acc;
}

IMU_Base::gyroRadiansAcc_t IMU_MPU6886::gyroRadiansAccFromData(const acc_temp_gyro_data_t& data, const xyz_int16_t& gyroOffset, const xyz_int16_t& accOffset)
{
    return gyroRadiansAcc_t {
// NOLINTBEGIN(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions) avoid "narrowing conversion from int to float" warnings
#if defined(IMU_Y_AXIS_POINTS_LEFT)
        .gyroRadians = {
            .x = -(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y) * GYRO_2000DPS_RES_RADIANS,
            .y =  (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x) * GYRO_2000DPS_RES_RADIANS,
            .z =  (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z) * GYRO_2000DPS_RES_RADIANS
        },
        .acc = {
            .x = -(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y) * ACC_8G_RES,
            .y =  (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x) * ACC_8G_RES,
            .z =  (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z) * ACC_8G_RES
        }
#elif defined(IMU_Y_AXIS_POINTS_RIGHT)
        .gyroRadians = {
            .x =  (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y) * GYRO_2000DPS_RES_RADIANS,
            .y = -(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x) * GYRO_2000DPS_RES_RADIANS,
            .z =  (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z) * GYRO_2000DPS_RES_RADIANS
        },
        .acc = {
            .x =  (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y) * ACC_8G_RES,
            .y = -(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x) * ACC_8G_RES,
            .z =  (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z) * ACC_8G_RES
        }
#elif defined(IMU_Y_AXIS_POINTS_DOWN)
        .gyroRadians = {
            .x =  (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x) * GYRO_2000DPS_RES_RADIANS,
            .y =  (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z) * GYRO_2000DPS_RES_RADIANS,
            .z = -(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y) * GYRO_2000DPS_RES_RADIANS
        },
        .acc = {
            .x =  (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x) * ACC_8G_RES,
            .y =  (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z) * ACC_8G_RES,
            .z = -(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y) * ACC_8G_RES
        }
#else
        .gyroRadians = {
            .x =  (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x) * GYRO_2000DPS_RES_RADIANS,
            .y =  (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y) * GYRO_2000DPS_RES_RADIANS,
            .z =  (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z) * GYRO_2000DPS_RES_RADIANS
        },
        .acc = {
            .x =  (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x) * ACC_8G_RES,
            .y =  (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y) * ACC_8G_RES,
            .z =  (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z) * ACC_8G_RES
        }
#endif
// NOLINTEND(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    };
}
#endif // USE_IMU_MPU6886_DIRECT
