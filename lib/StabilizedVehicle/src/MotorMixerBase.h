#pragma once

#include <cstdint>


class MotorMixerBase {
public:
    struct output_t {
        float speed;
        float roll;
        float pitch;
        float yaw;
    };
public:
    inline bool motorsIsOn() const { return _motorsIsOn; }
    inline void motorsSwitchOn() { _motorsIsOn = true; }
    inline void motorsSwitchOff() { _motorsIsOn = false; }
    inline bool motorsIsDisabled() const { return _motorsIsDisabled; }

    virtual void outputToMotors(const output_t& outputs, float deltaT, uint32_t tickCount) = 0;
public:
    static float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
protected:
    int32_t _motorsIsOn {false};
    int32_t _motorsIsDisabled {false};
};

