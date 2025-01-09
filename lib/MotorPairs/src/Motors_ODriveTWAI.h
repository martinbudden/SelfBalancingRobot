#pragma once

#include "Motors_ODrive.h"

class Motors_ODriveTWAI final : public Motors_ODrive {
public:
    explicit Motors_ODriveTWAI(float stepsPerRevolution) : Motors_ODrive(stepsPerRevolution) {}
    bool setup();
    virtual void setPower(float leftPower, float rightPower) override;
private:
    bool setupTWAI();
};
