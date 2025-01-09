#pragma once

#include "I2C.h"
#include "MotorPairBase.h"

class MotorsGoPlus2 final : public MotorPairBase {
public:
    MotorsGoPlus2();
public:
    virtual void readEncoder() override;
    virtual void setPower(float leftPower, float rightPower) override;
private:
    enum { MIN_POWER = -127, MAX_POWER = 127 };

    enum { SDA_PIN = 21, SCL_PIN = 22 };
    enum : uint8_t { I2C_ADDRESS = 0x38 };
    enum : uint8_t { REGISTER_MOTOR_A = 0x30, REGISTER_MOTOR_B = 0x31 };

    // map left and right motors to MOTOR_A and MOTOR_B
    enum : uint8_t { MOTOR_LEFT = REGISTER_MOTOR_A, MOTOR_RIGHT = REGISTER_MOTOR_B };
private:
    I2C _I2C;
};
