#pragma once

#include <Quaternion.h>
#include <cstdint>

#if defined(I2C_MUTEX_REQUIRED)
#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif
#endif

#include <xyz_type.h>


/*!
IMU virtual base class.
*/
class IMU_Base {
public:
    enum axis_order_t {
        XPOS_YPOS_ZPOS,
        YPOS_XNEG_ZPOS,
        XNEG_YNEG_ZPOS,
        YNEG_XPOS_ZPOS,
        XPOS_YNEG_ZNEG,
        YPOS_XPOS_ZNEG,
        XNEG_YPOS_ZNEG,
        YNEG_XNEG_ZNEG,
        ZPOS_YNEG_XPOS,
        YPOS_ZPOS_XPOS,
        ZNEG_YPOS_XPOS,
        YNEG_ZNEG_XPOS,
        ZPOS_YPOS_XNEG,
        YPOS_ZNEG_XNEG,
        ZNEG_YNEG_XNEG,
        YNEG_ZPOS_XNEG,
        ZPOS_XPOS_YPOS,
        XNEG_ZPOS_YPOS,
        ZNEG_XNEG_YPOS,
        XPOS_ZNEG_YPOS,
        ZPOS_XNEG_YNEG,
        XNEG_ZNEG_YNEG,
        ZNEG_XPOS_YNEG,
        XPOS_ZPOS_YNEG
    };
public:
    explicit IMU_Base(axis_order_t axisOrder);
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
    static constexpr float degreesToRadians = static_cast<float>(M_PI / 180.0);
    static constexpr float radiansToDegrees = static_cast<float>(180.0 / M_PI);
public:
    virtual xyz_int32_t getGyroOffset() const;
    virtual void setGyroOffset(const xyz_int32_t& gyroOffset);
    virtual xyz_int32_t getAccOffset() const;
    virtual void setAccOffset(const xyz_int32_t& accOffset);

    virtual xyz_int32_t readGyroRaw() = 0;
    virtual xyz_int32_t readAccRaw() = 0;
    virtual int32_t getAccOneG_Raw() const;

    // read functions have default implementations in the base class for convenience,
    // but should be reimplemented in derived classes for efficiency
    virtual xyz_t readGyroRPS();
    virtual xyz_t readGyroDPS();
    virtual xyz_t readAcc();
    virtual gyroRPS_Acc_t readGyroRPS_Acc();
    virtual Quaternion readOrientation();

    // by default the FIFO is not enabled
    virtual size_t readFIFO_ToBuffer();
    virtual gyroRPS_Acc_t readFIFO_Item(size_t index);
protected:
    xyz_t mapAxes(const xyz_t& data) const;
#if defined(I2C_MUTEX_REQUIRED)
#if defined(USE_FREERTOS)
    inline void i2cSemaphoreTake() const { xSemaphoreTake(_i2cMutex, portMAX_DELAY); }
    inline void i2cSemaphoreGive() const { xSemaphoreGive(_i2cMutex); }
    SemaphoreHandle_t _i2cMutex {};
#endif
#else
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

