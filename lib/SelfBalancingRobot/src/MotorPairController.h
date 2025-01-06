#pragma once

#include "MotorControllerBase.h"
#include "MotorPairBase.h"
#include "MotorPairControllerTelemetry.h"

class AHRS_Base;

class MotorPairController : public MotorControllerBase {
public:
    MotorPairController(const AHRS_Base& ahrsBase, void* i2cMutex);
    explicit MotorPairController(const AHRS_Base& ahrsBase) : MotorPairController(ahrsBase, nullptr) {}
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

    inline bool getPitchRateIsFiltered() const { return _pitchRateIsFiltered; }
    inline void setPitchRateIsFiltered(bool pitchRateIsFiltered) { _pitchRateIsFiltered = pitchRateIsFiltered; }

    inline ControlMode_t getControlMode() const { return _controlMode; }
    void setControlMode(ControlMode_t controlMode);

    inline float getPitchBalanceAngleDegrees() const { return _pitchBalanceAngleDegrees; }
    inline void setPitchBalanceAngleDegrees(float pitchBalanceAngleDegrees) { _pitchBalanceAngleDegrees = pitchBalanceAngleDegrees; }

    inline PIDF* getPitchPID() { return &_pitchPID; }
    inline PIDF* getSpeedPID() { return &_speedPID; }
    inline PIDF* getYawRatePID() { return &_yawRatePID; }

    void setPitchPID(const PIDF::PIDF_t& pid) { _pitchPID.setPID(pid); }
    void setSpeedPID(const PIDF::PIDF_t& pid) { _speedPID.setPID(pid); }
    void setYawRatePID(const PIDF::PIDF_t& pid) { _yawRatePID.setPID(pid); }

    inline const PIDF::PIDF_t& getPitchPIDConstants() const { return _pitchPID.getPID(); }
    inline const PIDF::PIDF_t& getSpeedPIDConstants() const { return _speedPID.getPID(); }
    inline const PIDF::PIDF_t& getYawRatePIDConstants() const { return _yawRatePID.getPID(); }

    inline float getPitchPIDSetpoint() const { return _pitchPID.getSetpoint(); }
    inline float getSpeedPIDSetpoint() const { return _speedPID.getSetpoint(); }
    inline float getYawRatePIDSetpoint() const { return _yawRatePID.getSetpoint(); }

    inline const PIDF::PIDF_t& getPitchPIDTelemetryScaleFactors() const { return _pitchPIDTelemetryScaleFactors; }
    inline const PIDF::PIDF_t& getSpeedPIDTelemetryScaleFactors() const { return _speedPIDTelemetryScaleFactors; }
    inline const PIDF::PIDF_t& getYawRatePIDTelemetryScaleFactors() const { return _yawRatePIDTelemetryScaleFactors; }

    const motor_pair_controller_telemetry_t& getTelemetryData() const;

    static float mapYawStick(float yawStick);
public:
    struct TaskParameters {
        MotorPairController* motorPairController;
        uint32_t tickIntervalMilliSeconds;
    };
    static void Task(void* arg);
    static MotorPairBase& motors();
    void loop(float deltaT, uint32_t tickCount);
private:
    void Task(const TaskParameters* taskParameters);
private:
    const AHRS_Base& _ahrs;
    MotorPairBase& _motors;

    int32_t _encoderLeftPrevious {0};
    int32_t _encoderRightPrevious {0};
    float _positionSetpointDegrees {0.0};
    float _positionDegrees {0.0};
    float _speedDPS {0.0};

    const float _motorMaxSpeedDPS;
    const float _motorMaxSpeedDPS_reciprocal;
    const float _motorStepsPerRevolution; //!< Local copy of the value of _motors->getStepsPerRevolution().
    const float _motorSwitchOffAngleDegrees; //!< Pitch angle at which the motors switch off. So if the robot flips over it won't lie on its back with its motors spinning.

    ControlMode_t _controlMode;

    float _pitchAngleDegreesPrevious {0.0};
    float _pitchBalanceAngleDegrees {0.0};
    const float _pitchMaxAngleDegrees {20.0};
    int _pitchRateIsFiltered {true}; //!< set to true to cause _pitchPID to use the filtered pitch rate in its update function

    PIDF _pitchPID;
    PIDF::PIDF_t _pitchPIDTelemetryScaleFactors;

    PIDF _speedPID;
    PIDF::PIDF_t _speedPIDTelemetryScaleFactors;

    float _yawStickMultiplier {1.0};
    PIDF _yawRatePID;
    PIDF::PIDF_t _yawRatePIDTelemetryScaleFactors;

    motor_pair_controller_telemetry_t _telemetry;
};
