#pragma once

#include "Motors_ODrive.h"

class Motors_ODriveCAN final : public Motors_ODrive {
public:
    explicit Motors_ODriveCAN(float stepsPerRevolution) : Motors_ODrive(stepsPerRevolution) {}
    bool setup();
    virtual void setPower(float leftPower, float rightPower) override;
};
