#pragma once

#include "I2C.h"
#include "MotorPairBase.h"
#include <cstdint>

class MotorsBalaC final : public MotorPairBase {
public:
    MotorsBalaC(uint8_t SDA_pin, uint8_t SCL_pin);
public:
    enum { MIN_POWER = -127, MAX_POWER = 127 };
public:
    virtual void setPower(float leftPower, float rightPower) override;
private:
    virtual void readEncoder() override;
private:
    enum : uint8_t { I2C_ADDRESS = 0x38 };
    enum : uint8_t { REGISTER_MOTOR_0 = 0x00, REGISTER_MOTOR_1 = 0x01 };

    // map left and right motors to MOTOR_1 and MOTOR_0
    enum : uint8_t { MOTOR_LEFT = REGISTER_MOTOR_1, MOTOR_RIGHT = REGISTER_MOTOR_0 };
private:
    I2C _I2C;
};

