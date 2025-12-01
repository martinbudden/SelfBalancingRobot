#include "MotorsBalaC.h"
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <cmath>


MotorsBalaC::MotorsBalaC(float deadbandPower) :
    MotorPairBase(0, CANNOT_REPORT_SPEED, deadbandPower)
{
    enum { SDA_PIN = 0, SCL_PIN = 26 };
#if defined(M5_UNIFIED)
    Wire.begin(SDA_PIN, SCL_PIN);  // SDA,SCL
#endif
}

void MotorsBalaC::setPower(float leftPower, float rightPower)
{
    leftPower = scalePower(leftPower) * MAX_POWER;
    rightPower = scalePower(rightPower) * MAX_POWER;

    // set signs so positive power moves motor in a forward direction
    const auto leftOutput =  static_cast<int8_t>( std::roundf(leftPower));
    const auto rightOutput = static_cast<int8_t>(-std::roundf(rightPower));

    i2cSemaphoreTake();

#if defined(M5_UNIFIED)
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(MOTOR_LEFT);
    Wire.write(leftOutput);
    Wire.endTransmission();

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(MOTOR_RIGHT);
    Wire.write(rightOutput);
    Wire.endTransmission();
#else
    (void)leftOutput;
    (void)rightOutput;
#endif

    i2cSemaphoreGive();
}
