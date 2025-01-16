#pragma once

#include "MotorControllerBase.h"
#include "MotorPairBase.h"
#include <Filters.h>
#include <PIDF.h>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

struct motor_pair_controller_telemetry_t;
class ReceiverBase;
class AHRS;
class Quaternion;


class MotorPairController : public MotorControllerBase {
public:
    MotorPairController(const AHRS& ahrs, const ReceiverBase& receiver, void* i2cMutex);
    MotorPairController(const AHRS& ahrs, ReceiverBase& receiver) : MotorPairController(ahrs, receiver, nullptr) {}
private:
    // MotorPairController is not copyable or moveable
    MotorPairController(const MotorPairController&) = delete;
    MotorPairController& operator=(const MotorPairController&) = delete;
    MotorPairController(MotorPairController&&) = delete;
    MotorPairController& operator=(MotorPairController&&) = delete;
public:
    enum ControlMode_t {
        CONTROL_MODE_SERIAL_PIDS, //!< Serial configuration for pitch and speed PIDs. Output from speed PID is added to the setpoint of the pitch PID.
        CONTROL_MODE_PARALLEL_PIDS, //!< Parallel configuration for pitch and speed PIDs. Pitch and speed are independently set.
        CONTROL_MODE_POSITION //!< The speed PID is used to set position rather than speed. Movement is obtained by incrementing position.
    };
public:
    inline void motorsResetEncodersToZero(void ) { _motors.resetEncodersToZero(); }

    inline ControlMode_t getControlMode() const { return _controlMode; }
    void setControlMode(ControlMode_t controlMode);

    inline float getPitchBalanceAngleDegrees() const { return _pitchBalanceAngleDegrees; }
    inline void setPitchBalanceAngleDegrees(float pitchBalanceAngleDegrees) { _pitchBalanceAngleDegrees = pitchBalanceAngleDegrees; }

    inline PIDF* getPitchPID() { return &_pitchPID; }
    inline PIDF* getSpeedPID() { return &_speedPID; }
    inline PIDF* getPositionPID() { return &_positionPID; }
    inline PIDF* getYawRatePID() { return &_yawRatePID; }

    void setPitchPID(const PIDF::PIDF_t& pid) { _pitchPID.setPID(pid); }
    void setSpeedPID(const PIDF::PIDF_t& pid) { _speedPID.setPID(pid); }
    void setPositionPID(const PIDF::PIDF_t& pid) { _positionPID.setPID(pid); }
    void setYawRatePID(const PIDF::PIDF_t& pid) { _yawRatePID.setPID(pid); }

    inline const PIDF::PIDF_t& getPitchPIDConstants() const { return _pitchPID.getPID(); }
    inline const PIDF::PIDF_t& getSpeedPIDConstants() const { return _speedPID.getPID(); }
    inline const PIDF::PIDF_t& getPositionPIDConstants() const { return _positionPID.getPID(); }
    inline const PIDF::PIDF_t& getYawRatePIDConstants() const { return _yawRatePID.getPID(); }

    inline float getPitchPIDSetpoint() const { return _pitchPID.getSetpoint(); }
    inline float getSpeedPIDSetpoint() const { return _speedPID.getSetpoint(); }
    inline float getPositionPIDSetpoint() const { return _positionPID.getSetpoint(); }
    inline float getYawRatePIDSetpoint() const { return _yawRatePID.getSetpoint(); }

    void getTelemetryData(motor_pair_controller_telemetry_t& telemetry) const;
    inline uint32_t getOutputPowerTimeMicroSeconds() const { return _outputPowerTimeMicroSeconds; } //<! time taken to write output power to the motors, for instrumentation
public:
    struct TaskParameters {
        MotorPairController* motorPairController;
        uint32_t tickIntervalMilliSeconds;
    };
    static void Task(void* arg);
    static MotorPairBase& motors();
    void loop(float deltaT, uint32_t tickCount);
public:
    void updateSetpointsAndMotorSpeedEstimates(float deltaT, uint32_t tickCount);
    void updatePIDs(float deltaT);
    virtual void updatePIDs(const Quaternion& orientation, float deltaT) override;
    void updateMotors();
private:
    void Task(const TaskParameters* taskParameters);
#if defined(USE_FREERTOS)
    inline void YIELD_TASK() const { taskYIELD(); }
#else
    inline void YIELD_TASK() const {}
#endif
private:
    const AHRS& _ahrs;
    const ReceiverBase& _receiver;
    MotorPairBase& _motors;

    int32_t _motorsDisabled {false};
    uint32_t _motorSwitchOffTickCount {0};

    float _powerLeft {0.0};
    float _powerRight {0.0};
    uint32_t _outputPowerTimeMicroSeconds {0}; //!< for instrumentation, time taken to set the motor pair power

    int32_t _encoderLeft {0}; //!< value read from left motor encoder, raw
    int32_t _encoderRight {0}; //!< value read from right motor encoder, raw
    int32_t _encoderLeftPrevious {0};
    int32_t _encoderRightPrevious {0};
    int32_t _encoderLeftDelta {0}; //!< difference between current left motor encoder value and previous value, raw
    int32_t _encoderRightDelta {0}; //!< difference between current right motor encoder value and previous value, raw

    float _speedLeftDPS {0}; //!< rotation speed of left motor, degrees per second
    float _speedRightDPS {0}; //!< rotation speed of right motor, degrees per second
    float _speedDPS {0.0}; //<!< filtered average of left and right motor speeds
    IIR_filter _speedFilter;

    const float _motorMaxSpeedDPS;
    const float _motorMaxSpeedDPS_reciprocal;
    const float _motorStepsPerRevolution; //!< Local copy of the value of _motors->getStepsPerRevolution().
    const float _motorSwitchOffAngleDegrees; //!< Pitch angle at which the motors switch off. So if the robot flips over it won't lie on its back with its motors spinning.

    ControlMode_t _controlMode;

    float _positionSetpointDegrees {0.0}; //!< Position setpoint for CONTROL_MODE_POSITION
    float _positionDegrees {0.0}; //!< Position for CONTROL_MODE_POSITION

    float _pitchBalanceAngleDegrees {0.0};
    float _pitchAngleDegreesPrevious {0.0};
    const float _pitchMaxAngleDegrees {20.0};

    PIDF _pitchPID;
    float _pitchUpdate {0.0};

    PIDF _speedPID;
    float _speedUpdate {0.0};

    PIDF _positionPID;
    float _positionUpdate {0.0};

    float _yawStickMultiplier {1.0};
    PIDF _yawRatePID;
    float _yawRateUpdate {0.0};
};
