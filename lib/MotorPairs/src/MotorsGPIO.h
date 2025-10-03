#pragma once

#include "MotorPairBase.h"


class MotorsGPIO final : public MotorPairBase {
public:
    struct pins_t {
        uint8_t motorLeft;
        uint8_t motorRight;
        uint8_t servoLeft;
        uint8_t servoRight;
    };
    explicit MotorsGPIO(const pins_t& pins);
public:
    virtual void setPower(float leftPower, float rightPower) override;
private:
    static constexpr float MIN_POWER = -127.0F;
    static constexpr float MAX_POWER =  127.0F;
    pins_t _pins {};
};
