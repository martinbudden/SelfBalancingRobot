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

    inline void newStickValuesReceived() { _newStickValuesAvailable = true; }
    inline float getPitchAngleDegreesRaw() const { return _pitchAngleDegreesRaw; } // not offset by balance angle
    inline float getRollAngleDegreesRaw() const { return _rollAngleDegreesRaw; } // not offset by balance angle
    inline float getYawAngleDegreesRaw() const { return _yawAngleDegreesRaw; } // not offset by balance angle
    inline uint32_t getOutputPowerTimeMicroSeconds() const { return _outputPowerTimeMicroSeconds; }
protected:
    uint32_t _motorSwitchOffTickCount {0};
    uint32_t _outputPowerTimeMicroSeconds {0}; //!< time taken to set the motor pair power
    int _motorsIsOn {false};
    int _newStickValuesAvailable {false};
    // stick values scaled to the range [-1,0, 1.0]
    float _throttleStick {0};
    float _rollStick {0};
    float _pitchStick {0};
    float _yawStick {0};
    float _pitchAngleDegreesRaw {0.0};
    float _rollAngleDegreesRaw {0.0};
    float _yawAngleDegreesRaw {0.0};
};
