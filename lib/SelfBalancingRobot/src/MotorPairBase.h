#pragma once

#include <cstdint>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <semphr.h>
#endif
#endif


/*!
Motor pair virtual base class.

Allows the Motor Pair Controller to work with different kinds of motor pairs.
*/
class MotorPairBase {
public:
    enum { I2C_FREQUENCY = 400000 }; // 400 kHz
public:
    enum can_accurately_estimate_speed_e { CANNOT_ACCURATELY_ESTIMATE_SPEED = 0,  CAN_ACCURATELY_ESTIMATE_SPEED = 1};
public:
    virtual ~MotorPairBase() = default;
    inline MotorPairBase(float stepsPerRevolution, can_accurately_estimate_speed_e canAccuratelyEstimateSpeed, float deadbandPower) :
        _stepsPerRevolution(stepsPerRevolution),
        _canAccuratelyEstimateSpeed(canAccuratelyEstimateSpeed),
        _deadbandPower(deadbandPower)
        {}
    inline MotorPairBase(float stepsPerRevolution, can_accurately_estimate_speed_e canAccuratelyEstimateSpeed) :
        MotorPairBase(stepsPerRevolution, canAccuratelyEstimateSpeed, 0.0F)
        {}
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    inline void setMutex(SemaphoreHandle_t i2cMutex) { _i2cMutex = i2cMutex; }
#endif
    inline int32_t getLeftEncoder() const { return _leftEncoder - _leftEncoderOffset; }
    inline int32_t getRightEncoder() const { return _rightEncoder - _rightEncoderOffset; }
    inline float getStepsPerRevolution() const { return _stepsPerRevolution; }
    inline void resetAllEncoders() { _leftEncoderOffset = _leftEncoder; _rightEncoderOffset = _rightEncoder; }
    inline bool canAccuratelyEstimateSpeed() const { return _canAccuratelyEstimateSpeed; }
    inline float getLeftSpeed() const { return _leftSpeed; }
    inline float getRightSpeed() const { return _rightSpeed; }
    inline float scalePower(float power) const;
    float getDeadbandPower() const { return _deadbandPower; }
    void setDeadbandPower(float deadbandPower) { _deadbandPower = deadbandPower; }
public:
    virtual void readEncoder() {};
    virtual void setPower(float leftPower, float rightPower) = 0;
public:
    static float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
protected:
#if defined(FRAMEWORK_USE_FREERTOS)
    inline void i2cSemaphoreTake() const { if (_i2cMutex) { xSemaphoreTake(_i2cMutex, portMAX_DELAY); } }
    inline void i2cSemaphoreGive() const { if (_i2cMutex) { xSemaphoreGive(_i2cMutex); } }
    SemaphoreHandle_t _i2cMutex {nullptr};
#else
    inline void i2cSemaphoreTake() const {}
    inline void i2cSemaphoreGive() const {}
#endif
protected:
    const float _stepsPerRevolution;
    const int _canAccuratelyEstimateSpeed;
    float _deadbandPower;
    int32_t _leftEncoder {0};
    int32_t _rightEncoder {0};
    int32_t _leftEncoderOffset {0};
    int32_t _rightEncoderOffset {0};
    float _leftSpeed  {0.0F}; // revolutions/second
    float _rightSpeed {0.0F};
};

inline float MotorPairBase::scalePower(float power) const
{
    power = clip(power, -1.0F, 1.0F);
    if (power < 0.0F) {
        power = -_deadbandPower + power * (1.0F - _deadbandPower);
    } else if (power > 0.0F) {
        power = _deadbandPower + power * (1.0F - _deadbandPower);
    }
    return power;
}

