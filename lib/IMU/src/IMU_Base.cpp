#include "IMU_Base.h"
#include <cassert>


#if defined(FRAMEWORK_ARDUINO)
#include <Arduino.h>
#elif defined(FRAMEWORK_PICO)
#endif

IMU_Base::IMU_Base(axis_order_t axisOrder) :
    _axisOrder(axisOrder)
{
}

IMU_Base::IMU_Base(axis_order_t axisOrder, [[maybe_unused]] void* i2cMutex) :
#if defined(I2C_MUTEX_REQUIRED)
    _i2cMutex(static_cast<SemaphoreHandle_t>(i2cMutex)),
#endif
    _axisOrder(axisOrder)
{
}

void IMU_Base::delayMs(int ms)
{
#if defined(FRAMEWORK_ARDUINO)
    delay(ms);
#elif defined(FRAMEWORK_PICO)
    //sleep_ms(ms);
#else
    (void)ms;
#endif
}

void IMU_Base::init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity)
{
    (void)outputDataRateHz;
    (void)gyroSensitivity;
    (void)accSensitivity;
}

IMU_Base::xyz_int32_t IMU_Base::getGyroOffset() const
{
    return _gyroOffset;
}

void IMU_Base::setGyroOffset(const xyz_int32_t& gyroOffset)
{
    _gyroOffset = gyroOffset;
}

IMU_Base::xyz_int32_t IMU_Base::getAccOffset() const
{
    return _accOffset;
}

void IMU_Base::setAccOffset(const xyz_int32_t& accOffset)
{
    _accOffset = accOffset;
}

int32_t IMU_Base::getAccOneG_Raw() const
{
    return 4096;
}

xyz_t IMU_Base::readGyroRPS()
{
    const xyz_int32_t gyroRaw = readGyroRaw();
    const xyz_t gyroRPS = {
        .x = static_cast<float>(gyroRaw.x - _gyroOffset.x) * _gyroResolutionRPS,
        .y = static_cast<float>(gyroRaw.y - _gyroOffset.y) * _gyroResolutionRPS,
        .z = static_cast<float>(gyroRaw.z - _gyroOffset.z) * _gyroResolutionRPS
    };
    return mapAxes(gyroRPS);
}

xyz_t IMU_Base::readGyroDPS()
{
    const xyz_int32_t gyroRaw = readGyroRaw();
    const xyz_t gyroDPS = {
        .x = static_cast<float>(gyroRaw.x - _gyroOffset.x) * _gyroResolutionDPS,
        .y = static_cast<float>(gyroRaw.y - _gyroOffset.y) * _gyroResolutionDPS,
        .z = static_cast<float>(gyroRaw.z - _gyroOffset.z) * _gyroResolutionDPS
    };
    return mapAxes(gyroDPS);
}

xyz_t IMU_Base::readAcc()
{
    const xyz_int32_t accRaw = readAccRaw();
    const xyz_t acc = {
        .x = static_cast<float>(accRaw.x - _accOffset.x) * _accResolution,
        .y = static_cast<float>(accRaw.y - _accOffset.y) * _accResolution,
        .z = static_cast<float>(accRaw.z - _accOffset.z) * _accResolution
    };
    return mapAxes(acc);
}

IMU_Base::gyroRPS_Acc_t IMU_Base::readGyroRPS_Acc()
{
    return gyroRPS_Acc_t {
        .gyroRPS = readGyroRPS(),
        .acc = readAcc()
    };
}

Quaternion IMU_Base::readOrientation()
{
    return Quaternion {};
}

xyz_t IMU_Base::mapAxes(const xyz_t& data, axis_order_t axisOrder)
{
// NOLINTBEGIN(bugprone-branch-clone) false positive
    switch (axisOrder) {
    case XPOS_YPOS_ZPOS:
        return data;
        break;
    case YPOS_XNEG_ZPOS:
        return xyz_t {
            .x =  data.y,
            .y = -data.x,
            .z =  data.z
        };
        break;
    case XNEG_YNEG_ZPOS:
        return xyz_t {
            .x = -data.x,
            .y = -data.y,
            .z =  data.z
        };
        break;
    case YNEG_XPOS_ZPOS:
        return xyz_t {
            .x = -data.y,
            .y =  data.x,
            .z =  data.z
        };
        break;
    case XPOS_YNEG_ZNEG:
        return xyz_t {
            .x =  data.x,
            .y = -data.y,
            .z = -data.z
        };
        break;
    case YPOS_XPOS_ZNEG:
        return xyz_t {
            .x =  data.y,
            .y =  data.x,
            .z = -data.z
        };
        break;
    case XNEG_YPOS_ZNEG:
        return xyz_t {
            .x = -data.x,
            .y =  data.y,
            .z = -data.z
        };
        break;
    case YNEG_XNEG_ZNEG:
        return xyz_t {
            .x = -data.y,
            .y = -data.x,
            .z = -data.z
        };
        break;
    case ZPOS_YNEG_XPOS:
        return xyz_t {
            .x =  data.z,
            .y = -data.y,
            .z =  data.x
        };
        break;
    case YPOS_ZPOS_XPOS:
        return xyz_t {
            .x =  data.y,
            .y =  data.z,
            .z =  data.x
        };
        break;
    case ZNEG_YPOS_XPOS:
        return xyz_t {
            .x = -data.z,
            .y =  data.y,
            .z =  data.x
        };
        break;
    case YNEG_ZNEG_XPOS:
        return xyz_t {
            .x = -data.y,
            .y = -data.z,
            .z =  data.x
        };
        break;
    case ZPOS_YPOS_XNEG:
        return xyz_t {
            .x =  data.z,
            .y =  data.y,
            .z = -data.x
        };
        break;
    case YPOS_ZNEG_XNEG:
        return xyz_t {
            .x =  data.y,
            .y = -data.z,
            .z = -data.x
        };
        break;
    case ZNEG_YNEG_XNEG:
        return xyz_t {
            .x = -data.z,
            .y = -data.y,
            .z = -data.x
        };
        break;
    case YNEG_ZPOS_XNEG:
        return xyz_t {
            .x = -data.y,
            .y =  data.z,
            .z = -data.x
        };
        break;
    case ZPOS_XPOS_YPOS:
        return xyz_t {
            .x =  data.z,
            .y =  data.x,
            .z =  data.y
        };
        break;
    case XNEG_ZPOS_YPOS:
        return xyz_t {
            .x = -data.x,
            .y =  data.z,
            .z =  data.y
        };
        break;
    case ZNEG_XNEG_YPOS:
        return xyz_t {
            .x = -data.z,
            .y = -data.x,
            .z =  data.y
        };
        break;
    case XPOS_ZNEG_YPOS:
        return xyz_t {
            .x =  data.x,
            .y = -data.z,
            .z =  data.y
        };
        break;
    case ZPOS_XNEG_YNEG:
        return xyz_t {
            .x =  data.z,
            .y = -data.x,
            .z = -data.y
        };
        break;
    case XNEG_ZNEG_YNEG:
        return xyz_t {
            .x = -data.x,
            .y = -data.z,
            .z = -data.y
        };
        break;
    case ZNEG_XPOS_YNEG:
        return xyz_t {
            .x = -data.z,
            .y =  data.x,
            .z = -data.y
        };
        break;
    case XPOS_ZPOS_YNEG:
        return xyz_t {
            .x =  data.x,
            .y =  data.z,
            .z = -data.y
        };
        break;
    default:
        assert(false && "IMU axis order not supported");
        break;
    } // end switch
// NOLINTEND(bugprone-branch-clone)

    return data;
}

IMU_Base::axis_order_t IMU_Base::axisOrderInverse(axis_order_t axisOrder)
{
    switch (axisOrder) {
    case YPOS_XNEG_ZPOS:
        return YNEG_XPOS_ZPOS;
    case YNEG_XPOS_ZPOS:
        return YPOS_XNEG_ZPOS;
    case YPOS_ZPOS_XPOS:
        return ZPOS_XPOS_YPOS;
    case ZNEG_YPOS_XPOS:
        return ZPOS_YPOS_XNEG;
    case YNEG_ZNEG_XPOS:
        return ZPOS_XNEG_YNEG;
    case ZPOS_YPOS_XNEG:
        return ZNEG_YPOS_XPOS;
    case YPOS_ZNEG_XNEG:
        return ZNEG_XPOS_YNEG;
    case YNEG_ZPOS_XNEG:
        return ZNEG_XNEG_YPOS;
    case ZPOS_XPOS_YPOS:
        return YPOS_ZPOS_XPOS;
    case ZNEG_XNEG_YPOS:
        return YNEG_ZPOS_XNEG;
    case XPOS_ZNEG_YPOS:
        return XPOS_ZPOS_YNEG;
    case ZPOS_XNEG_YNEG:
        return YNEG_ZNEG_XPOS;
    case ZNEG_XPOS_YNEG:
        return YPOS_ZNEG_XNEG;
    case XPOS_ZPOS_YNEG:
        return XPOS_ZNEG_YPOS;
    default:
        // other axis orders are self-inverting
        return axisOrder;
    }
}

size_t IMU_Base::readFIFO_ToBuffer()
{
    return 0;
}

IMU_Base::gyroRPS_Acc_t IMU_Base::readFIFO_Item(size_t index)
{
    (void)index;

    const gyroRPS_Acc_t gyroAcc {};
    return gyroAcc;
}
