#pragma once

#include "MotorPairBase.h"
#include <BUS_I2C.h>

class MotorsGoPlus2 final : public MotorPairBase {
public:
    MotorsGoPlus2(uint8_t SDA_pin, uint8_t SCL_pin);
public:
    virtual void setPower(float leftPower, float rightPower) override;
private:
    static constexpr float MIN_POWER = -127.0F;
    static constexpr float MAX_POWER =  127.0F;

    enum : uint8_t { I2C_ADDRESS = 0x38 };
    enum : uint8_t { REGISTER_MOTOR_A = 0x30, REGISTER_MOTOR_B = 0x31 };

    // map left and right motors to MOTOR_A and MOTOR_B
    enum : uint8_t { MOTOR_LEFT = REGISTER_MOTOR_A, MOTOR_RIGHT = REGISTER_MOTOR_B };
private:
    BUS_I2C _I2C;
};
