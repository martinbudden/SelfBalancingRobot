#if defined(MOTORS_BALA_C)

#include "MotorsBalaC.h"
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <cmath>


MotorsBalaC::MotorsBalaC(float deadbandPower) :
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED, deadbandPower)
{
    enum { SDA_PIN = 0, SCL_PIN = 26 };
#if !defined(FRAMEWORK_TEST)
    Wire.begin(SDA_PIN, SCL_PIN);  // SDA,SCL
#endif
}

void MotorsBalaC::readEncoder()
{
}

void MotorsBalaC::setPower(float leftPower, float rightPower)
{
    leftPower = scalePower(leftPower) * MAX_POWER;
    rightPower = scalePower(rightPower) * MAX_POWER;

    // set signs so positive power moves motor in a forward direction
    const int8_t leftOutput =   static_cast<int8_t>(std::roundf(leftPower)); // NOLINT(hicpp-use-auto,modernize-use-auto)
    const int8_t rightOutput = -static_cast<int8_t>(std::roundf(rightPower));

    i2cSemaphoreTake();

#if defined(FRAMEWORK_TEST)
    (void)leftOutput;
    (void)rightOutput;
#else
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(MOTOR_LEFT);
    Wire.write(leftOutput);
    Wire.endTransmission();

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(MOTOR_RIGHT);
    Wire.write(rightOutput);
    Wire.endTransmission();
#endif

    i2cSemaphoreGive();
}

#endif
