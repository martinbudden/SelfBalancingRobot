#if defined(USE_IMU_MPU6886_DIRECT)

#include "IMU_MPU6886.h"

#include <array>
#if defined(UNIT_TEST_BUILD)
void delay(int) {}
#else
#include <esp32-hal.h>
#endif
#include <cmath>

constexpr float degreesToRadians {M_PI / 180.0};

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
constexpr uint8_t REG_CONFIG                = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG           = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG          = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2         = 0x1D;

constexpr uint8_t REG_INT_PIN_CFG           = 0x37;
constexpr uint8_t REG_INT_ENABLE            = 0x38;

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

constexpr uint8_t REG_FIFO_WM_TH1           = 0x60;
constexpr uint8_t REG_FIFO_WM_TH2           = 0x61;

constexpr uint8_t REG_SIGNAL_PATH_RESET     = 0x68;
constexpr uint8_t REG_ACCEL_INTEL_CTRL      = 0x69;
constexpr uint8_t REG_USER_CTRL             = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1            = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2            = 0x6C;

constexpr uint8_t REG_FIFO_ENABLE           = 0x23;
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
    static_assert(sizeof(mems_sensor_data_t) == 6);
    init();
}

void IMU_MPU6886::init()
{
    static_assert(sizeof(mpu_6886_data_t) == mpu_6886_data_t::DATA_SIZE);

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

    _bus.writeByte(REG_GYRO_CONFIG, GFS_2000DPS << 3);
    _gyroResolution = 2000.0F / 32768.0F;
    delay(1);

    _bus.writeByte(REG_ACCEL_CONFIG, AFS_8G << 3);
    _accResolution = 8.0F / 32768.0F;
    delay(1);

    _bus.writeByte(REG_ACCEL_CONFIG2, 0x00); // no filtering
    delay(1);

    constexpr uint8_t DLPF_CFG_1 = 0x01; // 1khz output
    _bus.writeByte(REG_CONFIG, DLPF_CFG_1);
    delay(1);

    // divider is two, FIFO 500hz out
    _bus.writeByte(REG_SAMPLE_RATE_DIVIDER, 0x01);
    delay(1);

    _bus.writeByte(REG_FIFO_ENABLE, 0x00); // FIFO disabled
    delay(1);

    _bus.writeByte(REG_INT_PIN_CFG, 0x22);
    delay(1);

    constexpr uint8_t DATA_RDY_INT_EN {0x01};
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
        .x = static_cast<float>(acc.x - _accOffset.x) * _accResolution,
        .y = static_cast<float>(acc.y - _accOffset.y) * _accResolution,
        .z = static_cast<float>(acc.z - _accOffset.z) * _accResolution
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
        .x = static_cast<float>(gyro.x - _gyroOffset.x) * _gyroResolution,
        .y = static_cast<float>(gyro.y - _gyroOffset.y) * _gyroResolution,
        .z = static_cast<float>(gyro.z - _gyroOffset.z) * _gyroResolution
    };
}

xyz_t IMU_MPU6886::readGyroRadians() const
{
    const xyz_int16_t gyro = readGyroRaw();

    const float scaleFactor = degreesToRadians * _gyroResolution;
    return xyz_t {
        .x = static_cast<float>(gyro.x - _gyroOffset.x) * scaleFactor,
        .y = static_cast<float>(gyro.y - _gyroOffset.y) * scaleFactor,
        .z = static_cast<float>(gyro.z - _gyroOffset.z) * scaleFactor
    };
}

IMU_Base::gyroRadiansAcc_t IMU_MPU6886::readGyroRadiansAcc() const
{
    mpu_6886_data_t data; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

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

/*!
Set the gyro Force Sensitive Resistor
*/
void IMU_MPU6886::setGyroFSR(gyro_scale_t gyroScale)
{
    i2cSemaphoreTake();
    uint8_t data = _bus.readByte(REG_GYRO_CONFIG);
    delay(1);
    data |= gyroScale << 3;
    _bus.writeByte(REG_GYRO_CONFIG, data);
    i2cSemaphoreGive();
    delay(1);

    switch (gyroScale) {
    case GFS_250DPS:
        _gyroResolution = 250.0F / 32768.0F;
        break;
    case GFS_500DPS:
        _gyroResolution = 500.0F / 32768.0F;
        break;
    case GFS_1000DPS:
        _gyroResolution = 1000.0F / 32768.0F;
        break;
    case GFS_2000DPS:
        _gyroResolution = 2000.0F / 32768.0F;
        break;
    }
}

void IMU_MPU6886::setAccFSR(acc_scale_t accScale)
{
    i2cSemaphoreGive();
    uint8_t data = _bus.readByte(REG_ACCEL_CONFIG);
    delay(1);
    data |= accScale << 3;
    _bus.writeByte(REG_ACCEL_CONFIG, data);
    i2cSemaphoreGive();
    delay(1);

    switch (accScale) {
    case AFS_2G:
        _accResolution = 2.0F / 32768.0F;
        break;
    case AFS_4G:
        _accResolution = 4.0F / 32768.0F;
        break;
    case AFS_8G:
        _accResolution = 8.0F / 32768.0F;
        break;
    case AFS_16G:
        _accResolution = 16.0F / 32768.0F;
        break;
    }
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

void IMU_MPU6886::readFIFO(uint8_t *data, size_t len) const
{
    constexpr size_t chunkSize = 15*sizeof(mpu_6886_data_t);
    const auto count = len / chunkSize;
    i2cSemaphoreTake();
    for(auto ii = 0; ii < count; ++ii) {
        _bus.readBytes(REG_FIFO_R_W, &data[ii * chunkSize], chunkSize); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    _bus.readBytes(REG_FIFO_R_W, &data[count * chunkSize], len % chunkSize); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    i2cSemaphoreGive();
}

void IMU_MPU6886::resetFIFO()
{
    i2cSemaphoreTake();

    uint8_t data = _bus.readByte(REG_USER_CTRL);
    data |= 0x04;
    _bus.writeByte(REG_USER_CTRL, data);

    i2cSemaphoreGive();
}

uint16_t IMU_MPU6886::readFIFO_count() const
{
    std::array<uint8_t, 2> data;

    i2cSemaphoreTake();

    _bus.readBytes(REG_FIFO_COUNT_H, &data[0], sizeof(data));

    i2cSemaphoreGive();

    const uint16_t ret = data[0] << 8 | data[1];
    return ret;
}


int IMU_MPU6886::readFIFO_ToBuffer()
{
    return 0;
}

void IMU_MPU6886::readFIFO_Item(xyz_t& gyroRadians, xyz_t& acc, size_t index)
{
}

IMU_Base::gyroRadiansAcc_t IMU_MPU6886::gyroRadiansAccFromData(const mpu_6886_data_t& data, const xyz_int16_t& gyroOffset, const xyz_int16_t& accOffset)
{
    static constexpr float ACC_8G_RES { 8.0 / 32768.0 };
    static constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };

    return gyroRadiansAcc_t {
// NOLINTBEGIN(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions) avoid "narrowing conversion from int to float" warnings
#if defined(IMU_Y_AXIS_POINTS_LEFT)
        .gyroRadians = {
            .x = -degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y),
            .y =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x),
            .z =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z)
        },
        .acc = {
            .x = -ACC_8G_RES * (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y),
            .y =  ACC_8G_RES * (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x),
            .z =  ACC_8G_RES * (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z)
        }
#elif defined(IMU_Y_AXIS_POINTS_RIGHT)
        .gyroRadians = {
            .x =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y),
            .y = -degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x),
            .z =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z)
        },
        .acc = {
            .x =  ACC_8G_RES * (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y),
            .y = -ACC_8G_RES * (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x),
            .z =  ACC_8G_RES * (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z)
        }
#elif defined(IMU_Y_AXIS_POINTS_DOWN)
        .gyroRadians = {
            .x =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x),
            .y =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z),
            .z = -degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y)
        },
        .acc = {
            .x =  ACC_8G_RES * (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x),
            .y =  ACC_8G_RES * (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z),
            .z = -ACC_8G_RES * (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y)
        }
#else
        .gyroRadians = {
            .x =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x),
            .y =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y),
            .z =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z)
        },
        .acc = {
            .x =  ACC_8G_RES * (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x),
            .y =  ACC_8G_RES * (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y),
            .z =  ACC_8G_RES * (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z)
        }
#endif
// NOLINTEND(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    };
}
#endif // USE_IMU_MPU6886_DIRECT
