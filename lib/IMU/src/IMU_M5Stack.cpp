#if defined(M5_STACK)

#include "IMU_M5Stack.h"
#include <IMU_MPU6886.h>
#include <M5Stack.h>
#include <xyz_int16_type.h>

static constexpr float degreesToRadians {M_PI / 180.0};


IMU_M5_STACK::IMU_M5_STACK(void* i2cMutex) :
    IMU_Base(i2cMutex)
{
    // Set up FIFO for IMU
    // IMU data frequency is 500Hz
    i2cSemaphoreTake();
    M5.IMU.setFIFOEnable(true);
    M5.IMU.RestFIFO();
    i2cSemaphoreGive();
}

void IMU_M5_STACK::setAccOffset(const xyz_int16_t& accOffset)
{
    _accOffset = accOffset;
}

void IMU_M5_STACK::setGyroOffset(const xyz_int16_t& gyroOffset)
{
    _gyroOffset = gyroOffset;
}

xyz_int16_t IMU_M5_STACK::readAccRaw() const
{
    xyz_int16_t acc {};

    i2cSemaphoreTake();
    M5.IMU.getAccelAdc(&acc.x, &acc.y, &acc.z);
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

bool IMU_M5_STACK::readGyroRPS_Acc(xyz_t& gyroRPS, xyz_t& acc) const
{
    i2cSemaphoreTake();
    M5.IMU.getGyroData(&gyroRPS.x, &gyroRPS.y, &gyroRPS.z);
    M5.IMU.getAccelData(&acc.x, &acc.y, &acc.z);
    i2cSemaphoreGive();

    gyroRPS.x *= degreesToRadians;
    gyroRPS.y *= degreesToRadians;
    gyroRPS.z *= degreesToRadians;

    return true;
}

int IMU_M5_STACK::readFIFO_ToBuffer()
{
    i2cSemaphoreTake();
    const uint16_t fifoCount = M5.IMU.ReadFIFOCount();
    if (fifoCount != 0) {
        M5.IMU.ReadFIFOBuff(&_fifoBuffer[0], fifoCount);
    }
    i2cSemaphoreGive();
    return fifoCount  / IMU_MPU6886::acc_temp_gyro_data_t::DATA_SIZE;
}

void IMU_M5_STACK::readFIFO_Item(xyz_t& gyroRPS, xyz_t& acc, size_t index)
{
    const IMU_MPU6886::acc_temp_gyro_data_t* imu_data = reinterpret_cast<IMU_MPU6886::acc_temp_gyro_data_t*>(&_fifoBuffer[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    const IMU_MPU6886::acc_temp_gyro_data_t& imuData = imu_data[index]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const gyroRPS_Acc_t gyroAcc = IMU_M5_STACK::gyroRPS_AccFromData(imuData, _gyroOffset, _accOffset);
    gyroRPS = gyroAcc.gyroRPS;
    acc = gyroAcc.acc;
}

IMU_Base::gyroRPS_Acc_t IMU_M5_STACK::gyroRPS_AccFromData(const IMU_MPU6886::acc_temp_gyro_data_t& data, const xyz_int16_t& gyroOffset, const xyz_int16_t& accOffset)
{
    static constexpr float ACC_8G_RES { 8.0 / 32768.0 };
    static constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };

    return gyroRPS_Acc_t {
// NOLINTBEGIN(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions) avoid "narrowing conversion from int to float" warnings
#if defined(IMU_X_AXIS_FRONT_Y_AXIS_LEFT)
        .gyroRPS {
            .x = -degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y),
            .y =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x),
            .z =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z)
        },
        .acc {
            .x = -ACC_8G_RES * (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y),
            .y =  ACC_8G_RES * (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x),
            .z =  ACC_8G_RES * (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z)
        }
#elif defined(IMU_X_AXIS_BACK_Y_AXIS_RIGHT)
        .gyroRPS {
            .x =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y),
            .y = -degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x),
            .z =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z)
        },
        .acc {
            .x =  ACC_8G_RES * (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y),
            .y = -ACC_8G_RES * (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x),
            .z =  ACC_8G_RES * (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z)
        }
#elif defined(IMU_X_AXIS_RIGHT_Y_AXIS_DOWN)
        .gyroRPS {
            .x =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x),
            .y =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z),
            .z = -degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y)
        },
        .acc {
            .x =  ACC_8G_RES * (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x),
            .y =  ACC_8G_RES * (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z),
            .z = -ACC_8G_RES * (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y)
        }
#elif defined(IMU_X_AXIS_RIGHT_Y_AXIS_FRONT)
        .gyroRPS {
            .x =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - gyroOffset.x),
            .y =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - gyroOffset.y),
            .z =  degreesToRadians * GYRO_2000DPS_RES * (static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - gyroOffset.z)
        },
        .acc {
            .x =  ACC_8G_RES * (static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - accOffset.x),
            .y =  ACC_8G_RES * (static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - accOffset.y),
            .z =  ACC_8G_RES * (static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - accOffset.z)
        }
#else
    static_assert(false && "IMU orientation not implemented for IMU_M5Stack.");
#endif
// NOLINTEND(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    };
}
#endif