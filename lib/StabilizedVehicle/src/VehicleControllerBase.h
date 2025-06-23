# pragma once

#include <TaskBase.h>
#include <cstddef>

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
    enum { TYPE_NOT_SET= 0, SELF_BALANCING_ROBOT = 1, AIRCRAFT = 2 };
public:
    VehicleControllerBase(uint32_t type, uint32_t PID_Count) : _type(type),  _PID_Count(PID_Count) {}
public:
    inline uint32_t getType() const { return _type; };
    inline uint32_t getPID_Count() const { return _PID_Count; };
    inline const TaskBase* getTask() const { return _task; }
    inline void setTask(const TaskBase* task) { _task = task; }
    inline float getPitchAngleDegreesRaw() const { return _pitchAngleDegreesRaw; }
    inline float getRollAngleDegreesRaw() const { return _rollAngleDegreesRaw; }
    inline float getYawAngleDegreesRaw() const { return _yawAngleDegreesRaw; }

    virtual void loop(float deltaT, uint32_t tickCount) = 0;
    virtual void updateOutputsUsingPIDs(const xyz_t& gyroRPS, const xyz_t& acc, const Quaternion& orientation, float deltaT) = 0;
    virtual uint32_t getOutputPowerTimeMicroSeconds() const = 0; //<! time taken to write output power to the motors, for instrumentation
    virtual PIDF_uint8_t getPID_MSP(size_t index) const = 0;
protected:
    const uint32_t _type;
    const uint32_t _PID_Count;
    const TaskBase* _task {nullptr};
    float _pitchAngleDegreesRaw {0.0};
    float _rollAngleDegreesRaw {0.0};
    float _yawAngleDegreesRaw {0.0};
};
