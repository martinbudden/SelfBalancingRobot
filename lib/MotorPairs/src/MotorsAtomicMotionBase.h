#pragma once

#include "MotorPairBase.h"
#include <BUS_I2C.h>

class MotorsAtomicMotionBase final : public MotorPairBase {
public:
    MotorsAtomicMotionBase(float deadbandPower, uint8_t SDA_pin, uint8_t SCL_pin);
public:
    virtual void setPower(float leftPower, float rightPower) override;
private:
    static constexpr float MIN_POWER = -127.0F;
    static constexpr float MAX_POWER =  127.0F;

    enum : uint8_t { I2C_ADDRESS = 0x38 };
    enum : uint8_t { REGISTER_MOTOR_0 = 0x20, REGISTER_MOTOR_1 = 0x21 };

    // map left and right motors to REGISTER_MOTOR_0 and REGISTER_MOTOR_1
    enum : uint8_t { MOTOR_LEFT = REGISTER_MOTOR_0, MOTOR_RIGHT = REGISTER_MOTOR_1 };
private:
    BUS_I2C _I2C;
};
