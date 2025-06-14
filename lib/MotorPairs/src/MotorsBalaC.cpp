#if defined(MOTORS_BALA_C)

#include "MotorsBalaC.h"
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <cmath>


MotorsBalaC::MotorsBalaC() :
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED)
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

#if defined(M5_UNIFIED)
    M5.Ex_I2C.writeRegister8(I2C_ADDRESS, MOTOR_LEFT, leftOutput, I2C_FREQUENCY);
    M5.Ex_I2C.writeRegister8(I2C_ADDRESS, MOTOR_RIGHT, rightOutput, I2C_FREQUENCY);
#else
    (void)leftOutput;
    (void)rightOutput;
#endif

    i2cSemaphoreGive();
}

#endif
