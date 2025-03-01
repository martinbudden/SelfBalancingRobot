# pragma once

#include "TaskBase.h"
#include <cstdint>

class Quaternion;
struct xyz_t;

/*!
Abstract base class defining a motor controller for a stabilized vehicle.
*/
class MotorControllerBase : public TaskBase {
public:
    inline void newStickValuesReceived() { _newStickValuesAvailable = true; }
    inline float getPitchAngleDegreesRaw() const { return _pitchAngleDegreesRaw; }
    inline float getRollAngleDegreesRaw() const { return _rollAngleDegreesRaw; }
    inline float getYawAngleDegreesRaw() const { return _yawAngleDegreesRaw; }

    virtual void updateOutputsUsingPIDs(const xyz_t& gyroRPS, const xyz_t& acc, const Quaternion& orientation, float deltaT) = 0;
protected:
    int _newStickValuesAvailable {false};
    float _pitchAngleDegreesRaw {0.0};
    float _rollAngleDegreesRaw {0.0};
    float _yawAngleDegreesRaw {0.0};
};
