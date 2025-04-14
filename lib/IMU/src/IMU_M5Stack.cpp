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

IMU_Base::xyz_int32_t IMU_M5_STACK::readAccRaw()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake();
    M5.IMU.getAccelAdc(&x, &y, &z);
    i2cSemaphoreGive();

    return xyz_int32_t {.x = x, .y = y, .z = z };
}

xyz_t IMU_M5_STACK::readAcc()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake();
    M5.IMU.getAccelAdc(&x, &y, &z);
    i2cSemaphoreGive();

    const xyz_t acc {
        .x = static_cast<float>(x - _accOffset.x) * _accResolution,
        .y = static_cast<float>(y - _accOffset.y) * _accResolution,
        .z = static_cast<float>(z - _accOffset.z) * _accResolution
    };
    return mapAxes(acc);
}

IMU_Base::xyz_int32_t IMU_M5_STACK::readGyroRaw()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake();
    M5.IMU.getGyroAdc(&x, &y, &z);
    i2cSemaphoreGive();

    return xyz_int32_t {.x = x, .y = y, .z = z };
}

xyz_t IMU_M5_STACK::readGyroRPS()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake();
    M5.IMU.getGyroAdc(&x, &y, &z);
    i2cSemaphoreGive();

    const xyz_t gyroRPS {
        .x = static_cast<float>(x - _gyroOffset.x) * _gyroResolutionRPS,
        .y = static_cast<float>(y - _gyroOffset.y) * _gyroResolutionRPS,
        .z = static_cast<float>(z - _gyroOffset.z) * _gyroResolutionRPS,
    };
    return mapAxes(gyroRPS);
}

xyz_t IMU_M5_STACK::readGyroDPS()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    i2cSemaphoreTake();
    M5.IMU.getGyroAdc(&x, &y, &z);
    i2cSemaphoreGive();

    const xyz_t gyroDPS {
        .x = static_cast<float>(x - _gyroOffset.x) * _gyroResolutionDPS,
        .y = static_cast<float>(y - _gyroOffset.y) * _gyroResolutionDPS,
        .z = static_cast<float>(z - _gyroOffset.z) * _gyroResolutionDPS,
    };
    return mapAxes(gyroDPS);
}

IMU_Base::gyroRPS_Acc_t IMU_M5_STACK::readGyroRPS_Acc()
{
    const gyroRPS_Acc_t gyroAcc {
        .gyroRPS = readGyroRPS(),
        .acc = readAcc()
    };

    return gyroAcc;
}

size_t IMU_M5_STACK::readFIFO_ToBuffer()
{
    i2cSemaphoreTake();
    const uint32_t fifoCount = M5.IMU.ReadFIFOCount();
    if (fifoCount != 0) {
        M5.IMU.ReadFIFOBuff(&_fifoBuffer[0], fifoCount);
    }
    i2cSemaphoreGive();
    return fifoCount / acc_temperature_gyro_data_t::DATA_SIZE;
}

IMU_Base::gyroRPS_Acc_t IMU_M5_STACK::readFIFO_Item(size_t index)
{
    const acc_temperature_gyro_data_t* imu_data = reinterpret_cast<acc_temperature_gyro_data_t*>(&_fifoBuffer[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    const acc_temperature_gyro_data_t& imuData = imu_data[index]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const gyroRPS_Acc_t gyroAcc = IMU_M5_STACK::gyroRPS_AccFromRaw(imuData);

    return gyroAcc;
}

IMU_Base::gyroRPS_Acc_t IMU_M5_STACK::gyroRPS_AccFromRaw(const acc_temperature_gyro_data_t& data) const
{
// NOLINTBEGIN(hicpp-signed-bitwise)
#if defined(IMU_BUILD_XPOS_YPOS_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_YPOS_XNEG_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_XNEG_YNEG_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_YNEG_XPOS_ZPOS)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution
        }
    };
#elif defined(IMU_BUILD_XPOS_ZPOS_YNEG)
    return gyroRPS_Acc_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS,
            .z = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x)* _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z)* _accResolution,
            .z = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y)* _accResolution
        }
    };
#else
    // Axis order mapping done at run-time
    const gyroRPS_Acc_t gyroRPS_Acc {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l) - _gyroOffset.x) * _gyroResolutionRPS,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l) - _gyroOffset.y) * _gyroResolutionRPS,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l) - _gyroOffset.z) * _gyroResolutionRPS
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l) - _accOffset.x) * _accResolution,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l) - _accOffset.y) * _accResolution,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l) - _accOffset.z) * _accResolution
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
// NOLINTEND(hicpp-signed-bitwise)
}
#endif
