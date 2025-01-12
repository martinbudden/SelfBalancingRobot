#pragma once

#include "I2C.h"
#include "MotorPairBase.h"

class MotorsAtomicMotionBase final : public MotorPairBase {
public:
    MotorsAtomicMotionBase(uint8_t SDA_pin, uint8_t SCL_pin);
public:
    virtual void setPower(float leftPower, float rightPower) override;
private:
    virtual void readEncoder() override;
    bool exists();
private:
    enum { MIN_POWER = -127, MAX_POWER = 127 };

    enum : uint8_t { I2C_ADDRESS = 0x38 };
    enum : uint8_t { REGISTER_MOTOR_0 = 0x20, REGISTER_MOTOR_1 = 0x21 };

    // map left and right motors to REGISTER_MOTOR_0 and REGISTER_MOTOR_1
    enum : uint8_t { MOTOR_LEFT = REGISTER_MOTOR_0, MOTOR_RIGHT = REGISTER_MOTOR_1 };
private:
    I2C _I2C;
};
