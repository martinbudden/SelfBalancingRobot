#pragma once

#include <cstddef>

#if defined(I2C_MUTEX_REQUIRED)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

#include <xyz_int16_type.h>
#include <xyz_type.h>

/*!
IMU virtual base class.
*/
class IMU_Base {
public:
public:
    virtual void setAccOffset(const xyz_int16_t& accOffset) = 0;
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) = 0;

    virtual xyz_int16_t readAccRaw() const = 0;
    virtual xyz_int16_t readGyroRaw() const = 0;

    virtual bool readAccGyroRadians(xyz_t& acc, xyz_t& gyroRadians) const = 0;

    virtual int readFIFO_ToBuffer() = 0;
    virtual void readFIFO_Item(xyz_t& acc, xyz_t& gyroRadians, size_t index) = 0;

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

