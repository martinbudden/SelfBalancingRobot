# pragma once

#include "TaskBase.h"

class Quaternion;
struct xyz_t;

/*!
Abstract base class defining a controller for a stabilized vehicle.
*/
class VehicleControllerBase : public TaskBase {
public:
    inline void packetReceived() { _packetReceived = true; }
    inline void newStickValuesReceived() { _newStickValuesAvailable = true; }
    inline float getPitchAngleDegreesRaw() const { return _pitchAngleDegreesRaw; }
    inline float getRollAngleDegreesRaw() const { return _rollAngleDegreesRaw; }
    inline float getYawAngleDegreesRaw() const { return _yawAngleDegreesRaw; }

    virtual void updateOutputsUsingPIDs(const xyz_t& gyroRPS, const xyz_t& acc, const Quaternion& orientation, float deltaT) = 0;
    virtual void motorsToggleOnOff() = 0;
protected:
    int32_t _packetReceived {false};
    int32_t _newStickValuesAvailable {false};
    float _pitchAngleDegreesRaw {0.0};
    float _rollAngleDegreesRaw {0.0};
    float _yawAngleDegreesRaw {0.0};
};
