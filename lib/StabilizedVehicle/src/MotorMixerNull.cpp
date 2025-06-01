#include "MotorMixerNull.h"

void MotorMixerNull::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void) commands;
    (void) deltaT;
    (void) tickCount;
}
