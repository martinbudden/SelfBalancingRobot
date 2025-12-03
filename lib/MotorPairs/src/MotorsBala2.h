#pragma once

#include "MotorPairBase.h"


class MotorsBala2 final : public MotorPairBase {
public:
    MotorsBala2(uint8_t SDA_pin, uint8_t SCL_pin);
public:
    virtual void readAllEncoders() override;
    virtual void setPower(float leftPower, float rightPower) override;
private:
    static constexpr float MIN_POWER = -1023.0F;
    static constexpr float MAX_POWER =  1023.0F;
    enum { ENCODER_STEPS_PER_REVOLUTION = 420 };

    enum : uint8_t { I2C_ADDRESS = 0x3A };
    enum : uint8_t { REGISTER_SPEED = 0x00, REGISTER_ENCODER = 0x10 };
};
