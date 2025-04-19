#if defined(MOTORS_GPIO)

#include "MotorsGPIO.h"
#include <cmath>
#if defined(USE_ESP32)
#include <esp32-hal-ledc.h>
#endif

namespace {
// Motor channels
constexpr int motorLeft  = 0;
constexpr int motorRight = 1;
constexpr int servoLeft  = 2;
constexpr int servoRight = 3;
}; // end namespace


MotorsGPIO::MotorsGPIO(const pins_t& pins) :
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED)
{
#if defined(USE_ESP32)
    // Motor PWM Frequency
    constexpr int freq = 150000;

    // PWM Resolution
    constexpr int resolution = 8;

    ledcSetup(motorLeft, freq, resolution);
    ledcSetup(motorRight, freq, resolution);
    ledcSetup(servoLeft, freq, resolution);
    ledcSetup(servoRight, freq, resolution);

    ledcAttachPin(pins.motorLeft, motorLeft);
    ledcAttachPin(pins.motorRight, motorRight);
    ledcAttachPin(pins.servoLeft, servoLeft);
    ledcAttachPin(pins.servoRight, servoRight);
#else
    (void)pins;
#endif
}

void MotorsGPIO::readEncoder()
{
}

void MotorsGPIO::setPower(float leftPower, float rightPower)
{
    leftPower = scalePower(leftPower) * MAX_POWER;
    rightPower = scalePower(rightPower) * MAX_POWER;

    // set signs so positive power moves motor in a forward direction
    const int8_t leftOutput =   static_cast<int8_t>(roundf(leftPower)); // NOLINT(hicpp-use-auto,modernize-use-auto)
    const int8_t rightOutput = -static_cast<int8_t>(roundf(rightPower));

#if defined(USE_ESP32)
    ledcWrite(motorLeft,  leftOutput);
    ledcWrite(motorRight, rightOutput);
#else
    (void)leftOutput;
    (void)rightOutput;
#endif
}
#endif
