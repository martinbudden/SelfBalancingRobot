#pragma once

#include "MotorPairMixer.h"
#include <CockpitBase.h>
#include <Filters.h>
#include <PIDF.h>
#include <SV_TelemetryData.h>
#include <VehicleControllerBase.h>
#include <cfloat>
#include <string>

class AHRS;
class AHRS_MessageQueue;
class Blackbox;

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
    virtual ~MotorPairController() = default;
    MotorPairController(uint32_t taskIntervalMicroseconds, uint32_t outputToMotorsDenominator, MotorPairBase& motorPair, AHRS_MessageQueue& ahrsMessageQueue, void* i2cMutex);
    MotorPairController(uint32_t taskIntervalMicroseconds, uint32_t outputToMotorsDenominator, MotorPairBase& motorPair, AHRS_MessageQueue& ahrsMessageQueue) :
        MotorPairController(taskIntervalMicroseconds, outputToMotorsDenominator, motorPair, ahrsMessageQueue, nullptr) {}
    const AHRS_MessageQueue& getAHRS_MessageQueue() const { return _ahrsMessageQueue; }
private:
    // MotorPairController is not copyable or moveable
    MotorPairController(const MotorPairController&) = delete;
    MotorPairController& operator=(const MotorPairController&) = delete;
    MotorPairController(MotorPairController&&) = delete;
    MotorPairController& operator=(MotorPairController&&) = delete;
public:
    enum control_mode_e {
        CONTROL_MODE_SERIAL_PIDS, //!< Serial configuration for pitch and speed PIDs. Output from speed PID is added to the setpoint of the pitch PID.
        CONTROL_MODE_PARALLEL_PIDS, //!< Parallel configuration for pitch and speed PIDs. Pitch and speed are independently set.
        CONTROL_MODE_POSITION //!< The speed PID is used to set position rather than speed. Movement is obtained by incrementing position.
    };
    enum pid_index_e {
        ROLL_ANGLE_DEGREES=0, // allow possibility of roll control in future implementation
        PITCH_ANGLE_DEGREES=1,
        YAW_RATE_DPS=2,
        SPEED_SERIAL_DPS=3,
        SPEED_PARALLEL_DPS=4,
        POSITION_DEGREES=5,
        PID_COUNT=6,
        PID_BEGIN=0
    };
    enum output_index_e {
        OUTPUT_ROLL_ANGLE_DEGREES=0, // allow possibility of roll control in future implementation
        OUTPUT_PITCH_ANGLE_DEGREES=1,
        OUTPUT_YAW_RATE_DPS=2,
        OUTPUT_SPEED_DPS=3,
        OUTPUT_POSITION_DEGREES=4,
        OUTPUT_COUNT=5,
        OUTPUT_BEGIN=0
    };
    struct controls_t {
        uint32_t tickCount;
        float throttleStick;
        float rollStickDegrees;
        float pitchStickDegrees;
        float yawStickDPS;
    };
    struct vehicle_t {
        float maxMotorRPM;
        float wheelDiameterMM;
        float wheelTrackMM;
        float pitchBalanceAngleDegrees;
        float motorSwitchOffAngleDegrees;
        float encoderStepsPerRevolution;
    };
    typedef std::array<PIDF::PIDF_t, PID_COUNT> pidf_array_t;
    typedef std::array<PIDF_uint16_t, PID_COUNT> pidf_uint16_array_t;
    static constexpr float NOT_SET = FLT_MAX;
private:
    MotorPairController(uint32_t taskIntervalMicroseconds, uint32_t outputToMotorsDenominator, MotorPairBase& motorPair, AHRS_MessageQueue& ahrsMessageQueue, void* i2cMutex, const vehicle_t& vehicle);
public:
    static MotorPairBase& allocateMotors();

    inline bool motorsIsOn() const { return _motorMixer.motorsIsOn(); }
    void motorsSwitchOff();
    void motorsSwitchOn();
    void motorsToggleOnOff();

    virtual uint32_t getOutputPowerTimeMicroseconds() const override;

    inline control_mode_e getControlMode() const { return _controlMode; }
    void setControlMode(control_mode_e controlMode) { _controlMode = controlMode; resetIntegrals(); }

    const std::string& getPID_Name(pid_index_e pidIndex) const;

    inline const PIDF& getPID(pid_index_e pidIndex) const { return _PIDS[pidIndex]; }
    inline const PIDF::PIDF_t getPID_Constants(pid_index_e pidIndex) const { return _PIDS[pidIndex].getPID(); }

    void setPID_Constants(const pidf_uint16_array_t& pids);
    inline void setPID_Constants(pid_index_e pidIndex, const PIDF::PIDF_t& pid) { _PIDS[pidIndex].setPID(pid); }
    void setPID_Constants(pid_index_e pidIndex, const PIDF_uint16_t& pid16);

    virtual PIDF_uint16_t getPID_MSP(size_t index) const override;
    void setPID_P_MSP(pid_index_e pidIndex, uint16_t kp) { _PIDS[pidIndex].setP(kp * _scaleFactors.kp); }
    void setPID_I_MSP(pid_index_e pidIndex, uint16_t ki) { _PIDS[pidIndex].setI(ki * _scaleFactors.ki); }
    void setPID_D_MSP(pid_index_e pidIndex, uint16_t kd) { _PIDS[pidIndex].setD(kd * _scaleFactors.kd); }
    void setPID_S_MSP(pid_index_e pidIndex, uint16_t ks) { _PIDS[pidIndex].setS(ks * _scaleFactors.ks); }
    void setPID_K_MSP(pid_index_e pidIndex, uint16_t kk) { _PIDS[pidIndex].setK(kk * _scaleFactors.kk); }

    inline float getPID_Setpoint(pid_index_e pidIndex) const { return _PIDS[pidIndex].getSetpoint(); }
    void setPID_Setpoint(pid_index_e pidIndex, float setpoint) { _PIDS[pidIndex].setSetpoint(setpoint); }

    void resetIntegrals() { for (auto& pid : _PIDS) { pid.resetIntegral(); } }

    std::string getBalanceAngleName() const;
    inline float getPitchBalanceAngleDegrees() const { return _pitchBalanceAngleDegrees; }
    inline void setPitchBalanceAngleDegrees(float pitchBalanceAngleDegrees) { _pitchBalanceAngleDegrees = pitchBalanceAngleDegrees; }

    float getPitchAngleDegreesRaw() const { return _pitchAngleDegreesRaw; }
    float getRollAngleDegreesRaw() const { return _rollAngleDegreesRaw; }
    float getYawAngleDegreesRaw() const { return _yawAngleDegreesRaw; }

    motor_pair_controller_telemetry_t getTelemetryData() const;
    const MotorMixerBase& getMotorMixer() const { return _motorMixer; }

    void motorsResetAllEncoders();
public:
    virtual void outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem) override;
public:
    void updateSetpoints(const controls_t& controls);
    void updateMotorSpeedEstimates(float deltaT);
    virtual void updateOutputsUsingPIDs(const AHRS::ahrs_data_t& imuDataNED) override;
private:
    void updatePositionOutputs(float deltaT);
private:
    MotorPairMixer _motorMixer;
    AHRS_MessageQueue& _ahrsMessageQueue;
    const uint32_t _outputToMotorsDenominator;
    uint32_t _taskSignalledCount {0};
    control_mode_e _controlMode {CONTROL_MODE_SERIAL_PIDS};

    float _rollAngleDegreesRaw {NOT_SET};
    float _pitchAngleDegreesRaw {NOT_SET};
    float _yawAngleDegreesRaw {NOT_SET};

    // throttle stick scaled to the range [-1,0, 1.0]
    float _throttleStick {0};
    float _yawStickMultiplier {1.0F};

    int32_t _encoderLeft {0}; //!< value read from left motor encoder, raw
    int32_t _encoderRight {0}; //!< value read from right motor encoder, raw
    int32_t _encoderLeftPrevious {0};
    int32_t _encoderRightPrevious {0};
    int32_t _encoderLeftDelta {0}; //!< difference between current left motor encoder value and previous value, raw
    int32_t _encoderRightDelta {0}; //!< difference between current right motor encoder value and previous value, raw

    float _speedLeftDPS {0}; //!< rotation speed of left motor, degrees per second
    float _speedRightDPS {0}; //!< rotation speed of right motor, degrees per second
    float _speedDPS {0.0F}; //<!< filtered average of left and right motor speeds
    FilterMovingAverage<4> _speedMovingAverageFilter;
    IIR_filter _speedFilter;

    const float _motorMaxSpeedDPS;
    const float _motorMaxSpeedDPS_reciprocal;
    const float _motorPairStepsPerRevolution; //!< Local copy of the value of _motors->getStepsPerRevolution().

    float _positionSetpointDegrees {0.0F}; //!< Position setpoint for CONTROL_MODE_POSITION
    float _positionDegrees {0.0F}; //!< Position for CONTROL_MODE_POSITION
    float _positionDegreesPrevious {0.0F}; //!< Previous position for CONTROL_MODE_POSITION

    float _pitchBalanceAngleDegrees {0.0F};
    PowerTransferFilter2  _pitchAngleDTermFilter {};

    std::array<float, OUTPUT_COUNT> _outputs {};
    std::array<PIDF, PID_COUNT> _PIDS {};
    static constexpr PIDF::PIDF_t _scaleFactors = { 0.0003F, 0.025F, 0.000005F, 0.01F, 0.01F };
};
