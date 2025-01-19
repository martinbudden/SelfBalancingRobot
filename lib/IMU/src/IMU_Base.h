#pragma once

#include <cstddef>

#if defined(I2C_MUTEX_REQUIRED)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

#include <cmath>
#include <xyz_int16_type.h>
#include <xyz_type.h>

/*!
IMU virtual base class.
*/
class IMU_Base {
public:
    struct gyroRPS_Acc_t {
        xyz_t gyroRPS;
        xyz_t acc;
    };
public:
    static constexpr float degreesToRadians {M_PI / 180.0};
    static constexpr float radiansToDegrees {180.0 / M_PI};
public:
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) = 0;
    virtual void setAccOffset(const xyz_int16_t& accOffset) = 0;

    virtual xyz_int16_t readGyroRaw() const = 0;
    virtual xyz_int16_t readAccRaw() const = 0;

    virtual xyz_t readGyroRPS() const = 0;
    virtual xyz_t readGyroDPS() const = 0;
    virtual xyz_t readAcc() const = 0;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() const = 0;

    virtual int readFIFO_ToBuffer() = 0;
    virtual gyroRPS_Acc_t readFIFO_Item(size_t index) = 0;

#if defined(I2C_MUTEX_REQUIRED)
    explicit IMU_Base(void* i2cMutex) : _i2cMutex(static_cast<SemaphoreHandle_t>(i2cMutex)) {}
protected:
    inline void i2cSemaphoreTake() const { xSemaphoreTake(_i2cMutex, portMAX_DELAY); }
    inline void i2cSemaphoreGive() const { xSemaphoreGive(_i2cMutex); }
    SemaphoreHandle_t _i2cMutex;
#else
    explicit IMU_Base(void*) {}
protected:
    inline void i2cSemaphoreTake() const {}
    inline void i2cSemaphoreGive() const {}
#endif
};

