# pragma once

#include "TaskBase.h"
#include <cfloat>
#include <cstdint>

class Quaternion;
struct xyz_t;

/*!
Abstract base class defining a motor controller for a stabilized vehicle.
*/
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

    virtual void updatePIDs(const xyz_t& gyroRPS, const xyz_t& acc, const Quaternion& orientation, float deltaT) = 0;
protected:
    int _motorsIsOn {false};
    int _newStickValuesAvailable {false};
    float _pitchAngleDegreesRaw {0.0};
    float _rollAngleDegreesRaw {0.0};
    float _yawAngleDegreesRaw {0.0};
};
