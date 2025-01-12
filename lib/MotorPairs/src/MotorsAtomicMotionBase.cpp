#if defined(MOTORS_ATOMIC_MOTION_BASE)

#include "MotorsAtomicMotionBase.h"
#include <cmath>


// see https://github.com/m5stack/M5Atomic-Motion/blob/master/src/M5AtomicMotion.cpp
// and https://github.com/m5stack/M5AtomS3/blob/main/examples/AtomicBase/AtomicMotion/AtomicMotion.ino
// and https://github.com/m5stack/M5AtomS3/blob/main/src/M5AtomS3.h


MotorsAtomicMotionBase::MotorsAtomicMotionBase(uint8_t SDA_pin, uint8_t SCL_pin) :
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED),
    _I2C(I2C_ADDRESS, SDA_pin, SCL_pin)
{
}

void MotorsAtomicMotionBase::readEncoder()
{
}

void MotorsAtomicMotionBase::setPower(float leftPower, float rightPower)
{
    leftPower = scalePower(leftPower) * MAX_POWER;
    rightPower = scalePower(rightPower) * MAX_POWER;
    // set signs so positive power moves motor in a forward direction
    const int8_t leftOutput = -static_cast<int8_t>(round(leftPower));
    const int8_t rightOutput = static_cast<int8_t>(round(rightPower)); // NOLINT(hicpp-use-auto,modernize-use-auto)

    i2cSemaphoreTake();

    _I2C.writeByte(MOTOR_LEFT, leftOutput);
    _I2C.writeByte(MOTOR_RIGHT, rightOutput);

    i2cSemaphoreGive();
}

#endif
