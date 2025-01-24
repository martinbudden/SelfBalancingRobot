#if defined(M5_STACK)

#include "IMU_M5Stack.h"
#include <M5Stack.h>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_8G_RES { 8.0 / 32768.0 };
} // end namespace


IMU_M5_STACK::IMU_M5_STACK(axis_order_t axisOrder, void* i2cMutex) :
    IMU_Base(axisOrder, i2cMutex)
{
    // Set up FIFO for IMU
    // IMU data frequency is 500Hz
    i2cSemaphoreTake();
    M5.IMU.setFIFOEnable(true);
    M5.IMU.RestFIFO();
    i2cSemaphoreGive();
    _gyroResolutionDPS = GYRO_2000DPS_RES;
    _gyroResolutionRPS = GYRO_2000DPS_RES * degreesToRadians;
    _accResolution = ACC_8G_RES;
}

xyz_int16_t IMU_M5_STACK::readAccRaw() const
{
    xyz_int16_t acc {};

    i2cSemaphoreTake();
    M5.IMU.getAccelAdc(&acc.x, &acc.y, &acc.z);
    i2cSemaphoreGive();

    return acc;
}

xyz_t IMU_M5_STACK::readAcc() const
{
    xyz_t acc {};

    i2cSemaphoreTake();
    M5.IMU.getAccelData(&acc.x, &acc.y, &acc.z);
    i2cSemaphoreGive();

    return acc;
}

xyz_int16_t IMU_M5_STACK::readGyroRaw() const
{
    xyz_int16_t gyro {};

    i2cSemaphoreTake();
    M5.IMU.getGyroAdc(&gyro.x, &gyro.y, &gyro.z);
    i2cSemaphoreGive();

    return gyro;
}

xyz_t IMU_M5_STACK::readGyroRPS() const
{
    xyz_t gyroRPS {};

    i2cSemaphoreTake();
    M5.IMU.getGyroData(&gyroRPS.x, &gyroRPS.y, &gyroRPS.z);
    i2cSemaphoreGive();

    gyroRPS.x *= degreesToRadians;
    gyroRPS.y *= degreesToRadians;
    gyroRPS.z *= degreesToRadians;

    return gyroRPS;
}

xyz_t IMU_M5_STACK::readGyroDPS() const
{
    xyz_t gyroDPS {};

    i2cSemaphoreTake();
    M5.IMU.getGyroData(&gyroDPS.x, &gyroDPS.y, &gyroDPS.z);
    i2cSemaphoreGive();

    return gyroDPS;
}

IMU_Base::gyroRPS_Acc_t IMU_M5_STACK::readGyroRPS_Acc() const
{
    gyroRPS_Acc_t gyroAcc {};

    i2cSemaphoreTake();
    M5.IMU.getGyroData(&gyroAcc.gyroRPS.x, &gyroAcc.gyroRPS.y, &gyroAcc.gyroRPS.z);
    M5.IMU.getAccelData(&gyroAcc.acc.x, &gyroAcc.acc.y, &gyroAcc.acc.z);
    i2cSemaphoreGive();

    gyroAcc.gyroRPS.x *= degreesToRadians;
    gyroAcc.gyroRPS.y *= degreesToRadians;
    gyroAcc.gyroRPS.z *= degreesToRadians;

    return gyroAcc;
}

int IMU_M5_STACK::readFIFO_ToBuffer()
{
    i2cSemaphoreTake();
    const uint16_t fifoCount = M5.IMU.ReadFIFOCount();
    if (fifoCount != 0) {
        M5.IMU.ReadFIFOBuff(&_fifoBuffer[0], fifoCount);
    }
    i2cSemaphoreGive();
    return fifoCount  / IMU_MPU6886::acc_temperature_gyro_data_t::DATA_SIZE;
}

IMU_Base::gyroRPS_Acc_t IMU_M5_STACK::readFIFO_Item(size_t index)
{
    const IMU_MPU6886::acc_temperature_gyro_data_t* imu_data = reinterpret_cast<IMU_MPU6886::acc_temperature_gyro_data_t*>(&_fifoBuffer[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    const IMU_MPU6886::acc_temperature_gyro_data_t& imuData = imu_data[index]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const gyroRPS_Acc_t gyroAcc = IMU_M5_STACK::gyroRPS_AccFromRaw(imuData);

    return gyroAcc;
}

IMU_Base::gyroRPS_Acc_t IMU_M5_STACK::gyroRPS_AccFromRaw(const IMU_MPU6886::acc_temperature_gyro_data_t& data) const
{
#if defined(IMU_BUILD_YNEG_XPOS_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS {
            .x = - _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y),
            .y =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x),
            .z =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z)
        },
        .acc {
            .x = -_accResolution * static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y),
            .y =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x),
            .z =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)
        }
    };
#elif defined(IMU_BUILD_YPOS_XNEG_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS {
            .x =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y),
            .y = - _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x),
            .z =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z)
        },
        .acc {
            .x =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y),
            .y = -_accResolution * static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x),
            .z =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)
        }
    };
#elif defined(IMU_BUILD_XPOS_ZPOS_YNEG)
    return gyroRPS_Acc_t {
        .gyroRPS {
            .x =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x),
            .y =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z),
            .z = - _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y)
        },
        .acc {
            .x =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x),
            .y =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z),
            .z = -_accResolution * static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)
        }
#elif defined(IMU_BUILD_XPOS_YPOS_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS {
            .x =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x),
            .y =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y),
            .z =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z)
        },
        .acc {
            .x =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x),
            .y =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y),
            .z =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)
        }
    };
#else
    const xyz_t gyroRPS  {
        .x =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x),
        .y =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y),
        .z =   _gyroResolutionRPS * static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z)
    };
    const xyz_t acc {
        .x =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x),
        .y =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y),
        .z =  _accResolution * static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)
    };
    return gyroRPS_Acc_t {
        .gyroRPS = mapAxes(gyroRPS),
        .acc = mapAxes(acc)
    };
#endif
}
#endif