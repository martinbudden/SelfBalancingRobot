#pragma once

#include <cstdint>

#if defined(I2C_MUTEX_REQUIRED)
#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif
#endif

/*!
Motor pair virtual base class.

Allows the Motor Controller to work with different kinds of motors.
*/
class MotorPairBase {
public:
    enum can_accurately_estimate_speed_t { CANNOT_ACCURATELY_ESTIMATE_SPEED = 0,  CAN_ACCURATELY_ESTIMATE_SPEED = 1};
public:
    inline MotorPairBase(float stepsPerRevolution, can_accurately_estimate_speed_t canAccuratelyEstimateSpeed) :
        _stepsPerRevolution(stepsPerRevolution),
        _canAccuratelyEstimateSpeed(canAccuratelyEstimateSpeed)
        {}
public:
#if defined(I2C_MUTEX_REQUIRED)
#if defined(USE_FREERTOS)
    inline void setMutex(SemaphoreHandle_t i2cMutex) { _i2cMutex = i2cMutex; }
#endif
#endif
    inline int32_t getLeftEncoder() const { return _leftEncoder - _leftEncoderOffset; }
    inline int32_t getRightEncoder() const { return _rightEncoder - _rightEncoderOffset; }
    inline float getStepsPerRevolution() const { return _stepsPerRevolution; }
    inline void resetEncodersToZero() { _leftEncoderOffset = _leftEncoder; _rightEncoderOffset = _rightEncoder; }
    inline bool canAccuratelyEstimateSpeed() const { return _canAccuratelyEstimateSpeed; }
    inline float getLeftSpeed() const { return _leftSpeed; }
    inline float getRightSpeed() const { return _rightSpeed; }
    inline float scalePower(float power) const;
    float getDeadbandPower() const { return _deadbandPower; }
    void setDeadbandPower(float deadbandPower) { _deadbandPower = deadbandPower; }
public:
    virtual void readEncoder() = 0;
    virtual void setPower(float leftPower, float rightPower) = 0;
public:
    static float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
protected:
#if defined(I2C_MUTEX_REQUIRED)
#if defined(USE_FREERTOS)
    inline void i2cSemaphoreTake() const { xSemaphoreTake(_i2cMutex, portMAX_DELAY); }
    inline void i2cSemaphoreGive() const { xSemaphoreGive(_i2cMutex); }
    SemaphoreHandle_t _i2cMutex {nullptr};
#endif
#else
    inline void i2cSemaphoreTake() const {}
    inline void i2cSemaphoreGive() const {}
#endif
protected:
    float _stepsPerRevolution {0.0};
    int32_t _leftEncoder {0};
    int32_t _rightEncoder {0};
    int32_t _leftEncoderOffset {0};
    int32_t _rightEncoderOffset {0};
    float _leftSpeed  {0.0}; // revolutions/second
    float _rightSpeed {0.0};
    float _deadbandPower {0.0};
    int _canAccuratelyEstimateSpeed {CANNOT_ACCURATELY_ESTIMATE_SPEED};
};

inline float MotorPairBase::scalePower(float power) const
{
    power = clip(power, -1.0, 1.0);
    if (power < 0.0F) {
        power = -_deadbandPower + power * (1.0F - _deadbandPower);
    } else if (power > 0.0F) {
        power = _deadbandPower + power * (1.0F - _deadbandPower);
    }
    return power;
}

