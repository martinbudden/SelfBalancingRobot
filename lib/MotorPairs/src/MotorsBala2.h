#pragma once

#include "I2C.h"
#include "MotorPairBase.h"


class MotorsBala2 final : public MotorPairBase {
public:
    MotorsBala2(uint8_t SDA_pin, uint8_t SCL_pin);
public:
    virtual void readEncoder() override;
    virtual void setPower(float leftPower, float rightPower) override;
private:
    enum { MIN_POWER = -1023, MAX_POWER = 1023 };
    enum { ENCODER_STEPS_PER_REVOLUTION = 420 };

    enum : uint8_t { I2C_ADDRESS = 0x3A };
    enum : uint8_t { REGISTER_SPEED = 0x00, REGISTER_ENCODER = 0x10 };
private:
    //I2C _I2C;
};
