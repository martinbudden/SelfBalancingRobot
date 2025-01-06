# pragma once

#include "TaskBase.h"
#include <cfloat>
#include <cstdint>

class MotorControllerBase : public TaskBase {
public:
    inline bool motorsIsOn() const { return _motorsIsOn; }
    inline void motorsSwitchOn() { _motorsIsOn = true; }
    inline void motorsSwitchOff() { _motorsIsOn = false; }
    inline void motorsToggleOnOff() { _motorsIsOn = _motorsIsOn ? false : true; }

    void setSetpoints(int32_t throttleStickQ4dot12, int32_t rollStickQ4dot12, int32_t pitchStickQ4dot12, int32_t yawStickQ4dot12);

    inline float getPitchAngleDegreesRaw() const { return _pitchAngleDegreesRaw; } // not offset by balance angle
    inline float getRollAngleDegreesRaw() const { return _rollAngleDegreesRaw; } // not offset by balance angle
    inline float getYawAngleDegreesRaw() const { return _yawAngleDegreesRaw; } // not offset by balance angle
    inline static float Q4dot12_to_float(int32_t q4dot12) { return static_cast<float>(q4dot12) / 2048.0F; } //<! convert Q4dot12 fixed point number to floating point

protected:
    uint32_t _motorSwitchOffTickCount {0};
    int _motorsIsOn {false};
    int32_t _throttleStickQ4dot12 {0}; //<! unscaled throttle value from receiver as Q4.12 fixed point integer, ie in range [-2048, 2047]
    int32_t _rollStickQ4dot12 {0};
    int32_t _pitchStickQ4dot12 {0};
    int32_t _yawStickQ4dot12 {0};
    float _pitchAngleDegreesRaw {0.0};
    float _rollAngleDegreesRaw {0.0};
    float _yawAngleDegreesRaw {0.0};
};

