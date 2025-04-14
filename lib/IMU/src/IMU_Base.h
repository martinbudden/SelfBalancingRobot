#pragma once

#include <Quaternion.h>
#include <array>
#include <cstdint>

#if defined(I2C_MUTEX_REQUIRED)
#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif
#endif


/*!
IMU virtual base class.

Base class for an IMU (Inertial Management Unit) including a gyroscope and accelerometer.

Gyroscope readings can be returned as raw values, in RPS (Radians Per Second), or in DPS (Degrees Per Second).

Accelerometer readings are returned in units of standard gravity (g - 9.80665 meters per second squared).

The gyro and accelerometer can be read together using readGyroRPS_Acc. For typical IMUs this involves reading 12 bytes of data:
if the IMU is read via a 10 MHz  SPI bus this takes approximately 10 microseconds,
if the IMU is read via a 400 kHz I2C bus this takes approximately 270 microseconds.
*/
class IMU_Base {
public:
    /*!
    Axes order describing the sensor axes relative to the body axes.
    For example, if the sensor is rotated relative to the body so that the
    sensor X-axis points right, the sensor Z-axis points forward, and the sensor Y-axis points down,
    then the axis order is XPOS_ZPOS_YNEG.

    For example, if the sensor is rotated relative to the body so that the
    sensor y-axis points right, the sensor X-axis points back, and the sensor Z-axis points up,
    then the axis order is YPOS_XNEG_ZPOS
    */
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
    enum gyro_sensitivity_t {
        GYRO_FULL_SCALE_MAX,
        GYRO_FULL_SCALE_125_DPS,
        GYRO_FULL_SCALE_250_DPS,
        GYRO_FULL_SCALE_500_DPS,
        GYRO_FULL_SCALE_1000_DPS,
        GYRO_FULL_SCALE_2000_DPS,
        GYRO_FULL_SCALE_4000_DPS,
    };
    enum acc_sensitivity_t {
        ACC_FULL_SCALE_MAX,
        ACC_FULL_SCALE_1G,
        ACC_FULL_SCALE_2G,
        ACC_FULL_SCALE_4G,
        ACC_FULL_SCALE_8G,
        ACC_FULL_SCALE_16G,
        ACC_FULL_SCALE_32G,
    };
    static constexpr float sin45f = 0.7071067811865475F;
    const std::array<Quaternion, 24> axisOrientations = {
        Quaternion(  1.0F,    0.0F,    0.0F,    0.0F ),
        Quaternion(  sin45f,  0.0F,    0.0F,    sin45f ),
        Quaternion(  0.0F,    0.0F,    0.0F,    1.0F ),
        Quaternion(  sin45f,  0.0F,    0.0F,   -sin45f ),
        Quaternion(  0.0F,    0.0F,   -1.0F,    0.0F ),
        Quaternion(  0.0F,   -sin45f, -sin45f,  0.0F ),
        Quaternion(  0.0F,   -1.0F,    0.0F,    0.0F ),
        Quaternion(  0.0F,   -sin45f,  sin45f,  0.0F ),
        Quaternion(  0.0F,    0.0F,   -sin45f, sin45f ),
        Quaternion(  0.5F,   -0.5F,   -0.5F,    0.5F ),
        Quaternion(  sin45f, -sin45f,  0.0F,    0.0F ),
        Quaternion(  0.5F,   -0.5F,    0.5F,   -0.5F ),
        Quaternion(  sin45f, -sin45f,  0.0F,    0.0F ),
        Quaternion( -0.5F,   -0.5F,   -0.5F,   -0.5F ),
        Quaternion(  0.0F,    0.0F,   -sin45f, -sin45f ),
        Quaternion(  0.5F,    0.5F,   -0.5F,   -0.5F ),
        Quaternion( -0.5F,   -0.5F,   -0.5F,    0.5F ),
        Quaternion(  0.0F,   -sin45f,  0.0F,    sin45f ),
        Quaternion(  0.5F,   -0.5F,    0.5F,    0.5F ),
        Quaternion( -sin45f,  0.0F,   -sin45f,  0.0F ),
        Quaternion(  0.5F,    0.5F,   -0.5F,    0.5F ),
        Quaternion(  0.0F,   -sin45f,  0.0F,   -sin45f ),
        Quaternion(  0.5F,   -0.5F,   -0.5F,   -0.5F ),
        Quaternion(  sin45f,  0.0F,   -sin45f,  0.0F )
    };
public:
    virtual ~IMU_Base() = default;
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
    static void delayMs(int ms);
    virtual void init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity);
    void init(uint32_t outputDataRateHz) { init(outputDataRateHz, GYRO_FULL_SCALE_MAX, ACC_FULL_SCALE_MAX); }
    void init() { init(0, GYRO_FULL_SCALE_MAX, ACC_FULL_SCALE_MAX); }
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
