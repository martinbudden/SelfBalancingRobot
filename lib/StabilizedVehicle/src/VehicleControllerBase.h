# pragma once

#include "TaskBase.h"

class Quaternion;
struct xyz_t;

/*!
Abstract base class defining a controller for a stabilized vehicle.
*/
class VehicleControllerBase {
public:
    struct PIDF_uint8_t {
        uint8_t kp;
        uint8_t ki;
        uint8_t kd;
        uint8_t kf;
    };
public:
    inline const TaskBase* getTask() const { return _task; }
    inline void setTask(const TaskBase* task) { _task = task; }
    inline float getPitchAngleDegreesRaw() const { return _pitchAngleDegreesRaw; }
    inline float getRollAngleDegreesRaw() const { return _rollAngleDegreesRaw; }
    inline float getYawAngleDegreesRaw() const { return _yawAngleDegreesRaw; }

    virtual void loop(float deltaT, uint32_t tickCount) = 0;
    virtual void updateOutputsUsingPIDs(const xyz_t& gyroRPS, const xyz_t& acc, const Quaternion& orientation, float deltaT) = 0;
protected:
    const TaskBase* _task {nullptr};
    float _pitchAngleDegreesRaw {0.0};
    float _rollAngleDegreesRaw {0.0};
    float _yawAngleDegreesRaw {0.0};
};
