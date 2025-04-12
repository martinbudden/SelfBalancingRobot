#pragma once

#include "MotorMixer.h"
#include <Filters.h>
#include <PIDF.h>
#include <VehicleControllerBase.h>
#include <array>
#include <cfloat>
#include <string>

struct motor_pair_controller_telemetry_t;
class AHRS;
class MotorPairBase;
class Quaternion;
class ReceiverBase;

/*!
The MotorPairController uses the ENU (East North Up) coordinate convention, the same as used by ROS (Robot Operating System).
This is different from the NED (North East Down) convention commonly used by aircraft.

For ENU
positive pitch is nose down
positive roll is left side up
positive yaw is nose left

For NED
positive pitch is nose up
positive roll is left side up
positive yaw is nose right
*/
class MotorPairController : public VehicleControllerBase {
public:
    MotorPairController(const AHRS& ahrs, ReceiverBase& receiver, void* i2cMutex);
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
    enum pid_index_t {
        ROLL_ANGLE_DEGREES=0, // allow possibility of roll control in future implementation
        PITCH_ANGLE_DEGREES=1,
        YAW_RATE_DPS=2,
        SPEED_DPS=3,
        POSITION_DEGREES=4,
        PID_COUNT=5,
        PID_BEGIN=0
    };
    static constexpr float NOT_SET = FLT_MAX;
public:
    inline bool motorsIsOn() const { return _mixer.motorsIsOn(); }
    void motorsSwitchOff();
    void motorsSwitchOn();
    void motorsToggleOnOff();
    inline bool motorsIsDisabled() const { return _mixer.motorsIsDisabled(); }
    inline uint32_t getOutputPowerTimeMicroSeconds() const { return _mixer.getOutputPowerTimeMicroSeconds(); } //<! time taken to write output power to the motors, for instrumentation

    inline ControlMode_t getControlMode() const { return _controlMode; }
    void setControlMode(ControlMode_t controlMode);

    inline void setFailSafeTickCountThreshold(uint32_t failSafeTickCountThreshold) { _failSafeTickCountThreshold = failSafeTickCountThreshold; }
    inline void setFailSafeTickCountSwitchOffThreshold(uint32_t failSafeTickCountSwitchOffThreshold) { _failSafeTickCountSwitchOffThreshold = failSafeTickCountSwitchOffThreshold; }

    std::string getPID_Name(pid_index_t pidIndex) const;
    inline const PIDF::PIDF_t& getPID_Constants(pid_index_t pidIndex) const { return _PIDS[pidIndex].getPID(); }
    inline void setPID_Constants(pid_index_t pidIndex, const PIDF::PIDF_t& pid) { _PIDS[pidIndex].setPID(pid); }
    inline void setPID_P(pid_index_t pidIndex, float kp) { _PIDS[pidIndex].setP(kp); }
    inline void setPID_I(pid_index_t pidIndex, float ki) { _PIDS[pidIndex].setI(ki); }
    inline void setPID_D(pid_index_t pidIndex, float kd) { _PIDS[pidIndex].setD(kd); }
    inline void setPID_F(pid_index_t pidIndex, float kf) { _PIDS[pidIndex].setF(kf); }

    inline float getPID_Setpoint(pid_index_t pidIndex) const { return _PIDS[pidIndex].getSetpoint(); }
    void setPID_Setpoint(pid_index_t pidIndex, float setpoint) { _PIDS[pidIndex].setSetpoint(setpoint); }

    std::string getBalanceAngleName() const;
    inline float getPitchBalanceAngleDegrees() const { return _pitchBalanceAngleDegrees; }
    inline void setPitchBalanceAngleDegrees(float pitchBalanceAngleDegrees) { _pitchBalanceAngleDegrees = pitchBalanceAngleDegrees; }

    void getTelemetryData(motor_pair_controller_telemetry_t& telemetry) const;

    void motorsResetEncodersToZero();
public:
    struct TaskParameters {
        MotorPairController* motorPairController;
        uint32_t tickIntervalMilliSeconds;
    };
    [[noreturn]] static void Task(void* arg);
    void loop(float deltaT, uint32_t tickCount);
public:
    static float mapYawStick(float yawStick);
    void updateSetpoints(float deltaT, uint32_t tickCount);
    void updateMotorSpeedEstimates(float deltaT);
    void updateOutputsUsingPIDs(float deltaT);
    virtual void updateOutputsUsingPIDs(const xyz_t& gyroRPS, const xyz_t& acc, const Quaternion& orientation, float deltaT) override;
    void outputToMotors(float deltaT, uint32_t tickCount);
private:
    void updatePositionOutputs(float deltaT);
    MotorPairBase& allocateMotors();
    [[noreturn]] void Task(const TaskParameters* taskParameters);
private:
    const AHRS& _ahrs;
    ReceiverBase& _receiver;
    MotorPairBase& _motors; //!< The MotorPairController has a reference to the motors for input, ie reading the encoders.
    MotorMixer _mixer;
    ControlMode_t _controlMode;

    int32_t _onOffSwitchPressed {false};
    int32_t _receiverInUse {false};
    int32_t _failSafeOn {false};
    uint32_t _failSafeTickCount {0}; //<! failsafe counter, so the vehicle doesn't run away if it looses contact with the transmitter (for example by going out of range)
    uint32_t _failSafeTickCountThreshold {1500};
    uint32_t _failSafeTickCountSwitchOffThreshold {5000};

    // stick values scaled to the range [-1,0, 1.0]
    float _throttleStick {0};
    float _rollStick {0};
    float _pitchStick {0};
    float _yawStick {0};

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

    float _positionSetpointDegrees {0.0}; //!< Position setpoint for CONTROL_MODE_POSITION
    float _positionDegrees {0.0}; //!< Position for CONTROL_MODE_POSITION
    float _positionDegreesPrevious {0.0}; //!< Previous position for CONTROL_MODE_POSITION

    const float _rollMaxAngleDegrees {45.0};
    float _pitchBalanceAngleDegrees {0.0};
    float _pitchAngleDegreesPrevious {0.0};
    const float _pitchMaxAngleDegrees {20.0};
    float _yawStickMultiplier {1.0};

    std::array<PIDF, PID_COUNT> _PIDS;
    std::array<float, PID_COUNT> _outputs {}; //<! PID outputs, stored since the output from on PID may be used as the input to another
};
