#if defined(MOTORS_ATOMIC_MOTION_BASE)

#include "MotorsAtomicMotionBase.h"
#include <cmath>


// see https://github.com/m5stack/M5Atomic-Motion/blob/master/src/M5AtomicMotion.cpp
// and https://github.com/m5stack/M5AtomS3/blob/main/examples/AtomicBase/AtomicMotion/AtomicMotion.ino
// and https://github.com/m5stack/M5AtomS3/blob/main/src/M5AtomS3.h


MotorsAtomicMotionBase::MotorsAtomicMotionBase(float deadbandPower, uint8_t SDA_pin, uint8_t SCL_pin) :
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED, deadbandPower),
    _I2C(I2C_ADDRESS, BUS_I2C::pins_t{.sda=SDA_pin, .scl=SCL_pin, .irq=BUS_I2C::IRQ_NOT_SET, .irqLevel=0})
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
    const int8_t leftOutput = -static_cast<int8_t>(std::roundf(leftPower));
    const int8_t rightOutput = static_cast<int8_t>(std::roundf(rightPower)); // NOLINT(hicpp-use-auto,modernize-use-auto)

    i2cSemaphoreTake();

    _I2C.writeRegister(MOTOR_LEFT, leftOutput);
    _I2C.writeRegister(MOTOR_RIGHT, rightOutput);

    i2cSemaphoreGive();
}

#endif
