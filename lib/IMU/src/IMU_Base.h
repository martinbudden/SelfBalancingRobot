#pragma once

#include <cstdint>

#if defined(I2C_MUTEX_REQUIRED)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

#include <cmath>
#include <xyz_type.h>

/*!
IMU virtual base class.
*/
class IMU_Base {
public:
    enum axis_order_t {
        XPOS_YPOS_ZPOS,
        YNEG_XPOS_ZPOS,
        XNEG_YNEG_ZPOS,
        YPOS_XNEG_ZPOS,
        XPOS_ZPOS_YNEG,
    };
public:
    IMU_Base(axis_order_t axisOrder, void* i2cMutex);
public:
    struct xyz_int32_t {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    struct gyroRPS_Acc_t {
        xyz_t gyroRPS;
        xyz_t acc;
    };
    static constexpr float degreesToRadians {M_PI / 180.0};
    static constexpr float radiansToDegrees {180.0 / M_PI};
public:
    virtual void setGyroOffset(const xyz_int32_t& gyroOffset);
    virtual void setAccOffset(const xyz_int32_t& accOffset);

    virtual xyz_int32_t readGyroRaw() const = 0;
    virtual xyz_int32_t readAccRaw() const = 0;
    virtual int32_t getAccOneG_Raw() const;

    // read functions have default implementations in the base class for convenience,
    // but should be reimplemented in derived classes for efficiency
    virtual xyz_t readGyroRPS() const;
    virtual xyz_t readGyroDPS() const;
    virtual xyz_t readAcc() const;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() const;

    // by default the FIFO is not enabled
    virtual size_t readFIFO_ToBuffer();
    virtual gyroRPS_Acc_t readFIFO_Item(size_t index);
protected:
    xyz_t mapAxes(const xyz_t& data) const;
#if defined(I2C_MUTEX_REQUIRED)
protected:
    inline void i2cSemaphoreTake() const { xSemaphoreTake(_i2cMutex, portMAX_DELAY); }
    inline void i2cSemaphoreGive() const { xSemaphoreGive(_i2cMutex); }
    SemaphoreHandle_t _i2cMutex {};
#else
protected:
    inline void i2cSemaphoreTake() const {}
    inline void i2cSemaphoreGive() const {}
#endif
protected:
    axis_order_t _axisOrder;
    float _gyroResolutionRPS {};
    float _gyroResolutionDPS {};
    float _accResolution {};
    xyz_int32_t _gyroOffset {};
    xyz_int32_t _accOffset {};
};

