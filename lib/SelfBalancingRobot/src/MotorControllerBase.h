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

    /*!
    Use the joystick values provided by the receiver to set the setpoints for motor speed, pitch angle, and yaw rate.

    Note all setpoints are scaled to be in the range [-1.0, 1.0]. This in particular means we can feed back the speedUpdate into the pitch setpoint.

    NOTE: this function runs in the context of the MAIN_LOOP_TASK, so use of the FPU is avoided so that the ESP32 FPU registers don't have to be saved on a context switch.
    The values are converted to floats in the range [-1.0, 1.0] in the MotorPairController::loop() function, which runs in the context of the MPC_TASK.
    */
    void setSetpoints(int32_t throttleStickQ4dot12, int32_t rollStickQ4dot12, int32_t pitchStickQ4dot12, int32_t yawStickQ4dot12) // NOLINT(bugprone-easily-swappable-parameters)
    {
        _throttleStickQ4dot12 = throttleStickQ4dot12;
        _rollStickQ4dot12 = rollStickQ4dot12;
        _pitchStickQ4dot12 = pitchStickQ4dot12;
        _yawStickQ4dot12 = yawStickQ4dot12;
    }

    inline float getPitchAngleDegreesRaw() const { return _pitchAngleDegreesRaw; } // not offset by balance angle
    inline float getRollAngleDegreesRaw() const { return _rollAngleDegreesRaw; } // not offset by balance angle
    inline float getYawAngleDegreesRaw() const { return _yawAngleDegreesRaw; } // not offset by balance angle
    inline static float Q4dot12_to_float(int32_t q4dot12) { return static_cast<float>(q4dot12) * (1.0F / 2048.0F); } //<! convert Q4dot12 fixed point number to floating point

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

