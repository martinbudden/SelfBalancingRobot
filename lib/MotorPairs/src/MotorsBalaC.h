#pragma once

#include "MotorPairBase.h"
#include <BUS_I2C.h>

class MotorsBalaC final : public MotorPairBase {
public:
    explicit MotorsBalaC(float deadbandPower);
public:
    static constexpr float MIN_POWER = -127.0F;
    static constexpr float MAX_POWER =  127.0F;
public:
    virtual void setPower(float leftPower, float rightPower) override;
private:
    enum : uint8_t { I2C_ADDRESS = 0x38 };
    enum : uint8_t { REGISTER_MOTOR_0 = 0x00, REGISTER_MOTOR_1 = 0x01 };

    // map left and right motors to MOTOR_1 and MOTOR_0
    enum : uint8_t { MOTOR_LEFT = REGISTER_MOTOR_1, MOTOR_RIGHT = REGISTER_MOTOR_0 };
};

