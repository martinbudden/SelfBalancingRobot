#if defined(MOTORS_BALA_C)

#include "MotorsBalaC.h"
#include <cmath>


MotorsBalaC::MotorsBalaC() :
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED),
    _I2C(I2C_ADDRESS, SDA_PIN, SCL_PIN)
{
}

void MotorsBalaC::readEncoder()
{
}

void MotorsBalaC::setPower(float leftPower, float rightPower)
{
    leftPower = scalePower(leftPower) * MAX_POWER;
    rightPower = scalePower(rightPower) * MAX_POWER;

    // set signs so positive power moves motor in a forward direction
    const int8_t leftOutput =   static_cast<int8_t>(round(leftPower)); // NOLINT(hicpp-use-auto,modernize-use-auto)
    const int8_t rightOutput = -static_cast<int8_t>(round(rightPower));

    i2cSemaphoreTake();

    _I2C.writeByte(MOTOR_LEFT, leftOutput);
    _I2C.writeByte(MOTOR_RIGHT, rightOutput);

    i2cSemaphoreGive();
}

#endif
