#if defined(MOTORS_GO_PLUS_2)

#include "MotorsGoPlus2.h"
#include <cmath>


MotorsGoPlus2::MotorsGoPlus2(uint8_t SDA_pin, uint8_t SCL_pin) :
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED),
    _I2C(I2C_ADDRESS, SDA_pin, SCL_pin)
{
}

void MotorsGoPlus2::readEncoder()
{
}


void MotorsGoPlus2::setPower(float leftPower, float rightPower)
{
    const float leftClipped = clip(leftPower, -1.0, 1.0);
    const float rightClipped = clip(rightPower, -1.0, 1.0);
    // set signs so positive power moves motor in a forward direction
    const int8_t left = -static_cast<int8_t>(round(leftClipped * MAX_POWER));
    const int8_t right = -static_cast<int8_t>(round(rightClipped * MAX_POWER));

    i2cSemaphoreTake();

    _I2C.writeRegister(MOTOR_LEFT, left);
    _I2C.writeRegister(MOTOR_RIGHT, right);

    i2cSemaphoreGive();
}

#endif
