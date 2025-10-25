#include "MotorsGPIO.h"
#include <cmath>

#if defined(FRAMEWORK_RPI_PICO)
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#elif defined(FRAMEWORK_ESPIDF)
#include <driver/ledc.h>
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO

#if defined(FRAMEWORK_ARDUINO_ESP32)
#if !defined(TARGET_M5STACK_STICKC_BALAC)
#include <esp32-hal-ledc.h>
#else
#include <Arduino.h>
#endif

#endif
#endif // FRAMEWORK

namespace {
// Motor channels
static constexpr int motorLeft  = 0;
static constexpr int motorRight = 1;
static constexpr int servoLeft  = 2;
static constexpr int servoRight = 3;
}; // end namespace


MotorsGPIO::MotorsGPIO(const pins_t& pins) :
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED),
    _pins(pins)
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    // Motor PWM Frequency
    constexpr int frequency = 150000;

    // PWM Resolution
    constexpr int resolution = 8;

#if defined(ESPRESSIF32_6_11_0)
    ledcSetup(motorLeft, frequency, resolution);
    ledcSetup(motorRight, frequency, resolution);
    ledcSetup(servoLeft, frequency, resolution);
    ledcSetup(servoRight, frequency, resolution);

    ledcAttachPin(pins.motorLeft, motorLeft);
    ledcAttachPin(pins.motorRight, motorRight);
    ledcAttachPin(pins.servoLeft, servoLeft);
    ledcAttachPin(pins.servoRight, servoRight);
#else
    (void)frequency;
    (void)resolution;
    ledcAttach(pins.motorLeft, frequency, resolution);
    ledcAttach(pins.motorRight, frequency, resolution);
#endif // ESPRESSIF32_6_11_0

#else
    (void)pins;
#endif
}

void MotorsGPIO::setPower(float leftPower, float rightPower)
{
    leftPower = scalePower(leftPower) * MAX_POWER;
    rightPower = scalePower(rightPower) * MAX_POWER;

    // set signs so positive power moves motor in a forward direction
    const auto leftOutput =  static_cast<int8_t>( roundf(leftPower)); // NOLINT(hicpp-use-auto,modernize-use-auto)
    const auto rightOutput = static_cast<int8_t>(-roundf(rightPower));

#if defined(FRAMEWORK_ARDUINO_ESP32)
    ledcWrite(_pins.motorLeft,  leftOutput);
    ledcWrite(_pins.motorRight, rightOutput);
#else
    (void)leftOutput;
    (void)rightOutput;
#endif
}
