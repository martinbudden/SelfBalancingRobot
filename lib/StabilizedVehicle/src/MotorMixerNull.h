#pragma once

#include <MotorMixerBase.h>


class MotorMixerNull : public MotorMixerBase {
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
};
