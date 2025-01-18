#pragma once

#include "MotorControllerBase.h"
#include <Filters.h>
#include <PIDF.h>
#include <array>
#include <string>

struct motor_pair_controller_telemetry_t;
class AHRS;
class MotorPairBase;
class ReceiverBase;
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
    enum pid_index_t { PITCH_ANGLE=0, SPEED=1, YAW_RATE=2, POSITION=3, PID_COUNT=4, PID_BEGIN=0 };
public:
    inline ControlMode_t getControlMode() const { return _controlMode; }
    void setControlMode(ControlMode_t controlMode);

    std::string getPIDName(pid_index_t pidIndex) const;
    inline const PIDF::PIDF_t& getPIDConstants(pid_index_t pidIndex) const { return _PIDS[pidIndex].getPID(); }
    inline void setPIDConstants(pid_index_t pidIndex, const PIDF::PIDF_t& pid) { _PIDS[pidIndex].setPID(pid); }
    inline void setPID_P(pid_index_t pidIndex, float kp) { _PIDS[pidIndex].setP(kp); }
    inline void setPID_I(pid_index_t pidIndex, float ki) { _PIDS[pidIndex].setI(ki); }
    inline void setPID_D(pid_index_t pidIndex, float kd) { _PIDS[pidIndex].setD(kd); }
    inline void setPID_F(pid_index_t pidIndex, float kf) { _PIDS[pidIndex].setF(kf); }

    inline float getPIDSetpoint(pid_index_t pidIndex) const { return _PIDS[pidIndex].getSetpoint(); }
    void setPIDSetpoint(pid_index_t pidIndex, float setpoint) { _PIDS[pidIndex].setSetpoint(setpoint); }

    std::string getBalanceAngleName() const;
    inline float getPitchBalanceAngleDegrees() const { return _pitchBalanceAngleDegrees; }
    inline void setPitchBalanceAngleDegrees(float pitchBalanceAngleDegrees) { _pitchBalanceAngleDegrees = pitchBalanceAngleDegrees; }

    void getTelemetryData(motor_pair_controller_telemetry_t& telemetry) const;

    void motorsResetEncodersToZero();
    inline uint32_t getOutputPowerTimeMicroSeconds() const { return _mixer.outputPowerTimeMicroSeconds; } //<! time taken to write output power to the motors, for instrumentation
public:
    struct TaskParameters {
        MotorPairController* motorPairController;
        uint32_t tickIntervalMilliSeconds;
    };
    static void Task(void* arg);
    void loop(float deltaT, uint32_t tickCount);
public:
    void updateSetpointsAndMotorSpeedEstimates(float deltaT);
    void updatePIDs(float deltaT);
    virtual void updatePIDs(const xyz_t& gyroRPS, const xyz_t& acc, const Quaternion& orientation, float deltaT) override;
    void updateMotors(uint32_t tickCount);
private:
    void updatePosition(float deltaT);
    MotorPairBase& motors();
    void Task(const TaskParameters* taskParameters);
private:
    const AHRS& _ahrs;
    const ReceiverBase& _receiver;
    MotorPairBase& _motors;
    // stick values scaled to the range [-1,0, 1.0]
    float _throttleStick {0};
    float _rollStick {0};
    float _pitchStick {0};
    float _yawStick {0};

    struct mixer_t {
        float motorSwitchOffAngleDegrees {70.0}; //!< Pitch angle at which the motors switch off. So if the robot flips over it won't lie on its back with its motors spinning.
        int32_t motorsDisabled {false};
        uint32_t motorSwitchOffTickCount {0};
        float powerLeft {0.0};
        float powerRight {0.0};
        uint32_t outputPowerTimeMicroSeconds {0}; //!< for instrumentation, time taken to set the motor pair power
    };
    mixer_t _mixer;
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

    ControlMode_t _controlMode;

    float _positionSetpointDegrees {0.0}; //!< Position setpoint for CONTROL_MODE_POSITION
    float _positionDegrees {0.0}; //!< Position for CONTROL_MODE_POSITION

    float _pitchBalanceAngleDegrees {0.0};
    float _pitchAngleDegreesPrevious {0.0};
    const float _pitchMaxAngleDegrees {20.0};
    float _yawStickMultiplier {1.0};

    std::array<PIDF, PID_COUNT> _PIDS;
    std::array<float, PID_COUNT> _updates {};
};
