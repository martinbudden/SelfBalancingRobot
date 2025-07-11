#pragma once

#include "MotorPairMixer.h"
#include <Filters.h>
#include <PIDF.h>
#include <RadioControllerBase.h>
#include <SV_TelemetryData.h>
#include <VehicleControllerBase.h>
#include <cfloat>
#include <string>

class AHRS;
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
    MotorPairController(uint32_t taskIntervalMicroSeconds, MotorPairBase& motorPair, const AHRS& ahrs, RadioControllerBase& radioController, void* i2cMutex);
    MotorPairController(uint32_t taskIntervalMicroSeconds, MotorPairBase& motorPair, const AHRS& ahrs, RadioControllerBase& radioController) : 
        MotorPairController(taskIntervalMicroSeconds, motorPair, ahrs, radioController, nullptr) {}
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
    struct vehicle_t {
        float maxMotorRPM;
        float wheelDiameterMM;
        float wheelTrackMM;
        float pitchBalanceAngleDegrees;
        float motorSwitchOffAngleDegrees;
        float encoderStepsPerRevolution;
    };
    typedef std::array<PIDF::PIDF_t, PID_COUNT> pidf_array_t;
    typedef std::array<PIDF_uint8_t, PID_COUNT> pidf_uint8_array_t;
    static constexpr float NOT_SET = FLT_MAX;
private:
    MotorPairController(uint32_t taskIntervalMicroSeconds, MotorPairBase& motorPair, const AHRS& ahrs, RadioControllerBase& radioController, void* i2cMutex, const vehicle_t& vehicle, const pidf_array_t& scaleFactors);
public:
    static MotorPairBase& allocateMotors();
    uint32_t getTaskIntervalMicroSeconds() const { return _taskIntervalMicroSeconds; }
    float getMixerThrottle() const { return _mixerThrottle; }

    inline bool motorsIsOn() const { return _motorPairMixer.motorsIsOn(); }
    void motorsSwitchOff();
    void motorsSwitchOn();
    void motorsToggleOnOff();
    inline bool motorsIsDisabled() const { return _motorPairMixer.motorsIsDisabled(); }
    void setBlackbox(Blackbox& blackbox) { _blackbox = &blackbox; }

    virtual uint32_t getOutputPowerTimeMicroSeconds() const override;

    inline control_mode_e getControlMode() const { return _controlMode; }
    void setControlMode(control_mode_e controlMode) { _controlMode = controlMode; resetIntegrals(); }

    inline void setFailSafeTickCountThreshold(uint32_t failsafeTickCountThreshold) { _failsafeTickCountThreshold = failsafeTickCountThreshold; }
    inline void setFailSafeTickCountSwitchOffThreshold(uint32_t failsafeTickCountSwitchOffThreshold) { _failsafeTickCountSwitchOffThreshold = failsafeTickCountSwitchOffThreshold; }

    const std::string& getPID_Name(pid_index_e pidIndex) const;

    inline const PIDF& getPID(pid_index_e pidIndex) const { return _PIDS[pidIndex]; }
    inline const PIDF::PIDF_t getPID_Constants(pid_index_e pidIndex) const { return _PIDS[pidIndex].getPID(); }
    void setPID_Constants(const pidf_uint8_array_t& pids);
    inline void setPID_Constants(pid_index_e pidIndex, const PIDF::PIDF_t& pid) { _PIDS[pidIndex].setPID(pid); }

    virtual PIDF_uint8_t getPID_MSP(size_t index) const override;
    void setPID_P_MSP(pid_index_e pidIndex, uint8_t kp) { _PIDS[pidIndex].setP(kp * _scaleFactors[pidIndex].kp); }
    void setPID_I_MSP(pid_index_e pidIndex, uint8_t ki) { _PIDS[pidIndex].setI(ki * _scaleFactors[pidIndex].ki); }
    void setPID_D_MSP(pid_index_e pidIndex, uint8_t kd) { _PIDS[pidIndex].setD(kd * _scaleFactors[pidIndex].kd); }
    void setPID_F_MSP(pid_index_e pidIndex, uint8_t kf) { _PIDS[pidIndex].setF(kf * _scaleFactors[pidIndex].kf); }

    const std::array<PIDF::PIDF_t, PID_COUNT>&  getScaleFactors() const { return _scaleFactors; }

    inline float getPID_Setpoint(pid_index_e pidIndex) const { return _PIDS[pidIndex].getSetpoint(); }
    void setPID_Setpoint(pid_index_e pidIndex, float setpoint) { _PIDS[pidIndex].setSetpoint(setpoint); }

    void resetIntegrals() { for (auto& pid : _PIDS) { pid.resetIntegral(); } }

    std::string getBalanceAngleName() const;
    inline float getPitchBalanceAngleDegrees() const { return _pitchBalanceAngleDegrees; }
    inline void setPitchBalanceAngleDegrees(float pitchBalanceAngleDegrees) { _pitchBalanceAngleDegrees = pitchBalanceAngleDegrees; }

    motor_pair_controller_telemetry_t getTelemetryData() const;

    void motorsResetEncodersToZero();
public:
    virtual void loop(float deltaT, uint32_t tickCount) override;
public:
    void updateSetpoints(const RadioControllerBase::controls_t& controls);
    void updateMotorSpeedEstimates(float deltaT);
    virtual void updateOutputsUsingPIDs(const xyz_t& gyroRPS, const xyz_t& acc, const Quaternion& orientation, float deltaT) override;
    virtual uint32_t updateBlackbox(uint32_t timeMicroSeconds, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc) override;
private:
    void outputToMotors(float deltaT, uint32_t tickCount);
    void updatePositionOutputs(float deltaT);
private:
    const AHRS& _ahrs;
    RadioControllerBase& _radioController;
    MotorPairBase& _motorPair; //!< The MotorPairController has a reference to the motors for input, ie reading the encoders.
    MotorPairMixer _motorPairMixer;
    Blackbox* _blackbox {nullptr};
    control_mode_e _controlMode {CONTROL_MODE_SERIAL_PIDS};

    uint32_t _taskIntervalMicroSeconds;
    float _mixerThrottle {0.0F};

    int32_t _onOffSwitchPressed {false};
    int32_t _receiverInUse {false};
    uint32_t _failsafeTickCount {0}; //<! failsafe counter, so the vehicle doesn't run away if it looses contact with the transmitter (for example by going out of range)
    uint32_t _failsafeTickCountThreshold {1500};
    uint32_t _failsafeTickCountSwitchOffThreshold {5000};

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
    FilterMovingAverage<4> _speedMovingAverageFilter;
    IIR_filter _speedFilter;

    const float _motorMaxSpeedDPS;
    const float _motorMaxSpeedDPS_reciprocal;
    const float _motorPairStepsPerRevolution; //!< Local copy of the value of _motors->getStepsPerRevolution().

    float _positionSetpointDegrees {0.0}; //!< Position setpoint for CONTROL_MODE_POSITION
    float _positionDegrees {0.0}; //!< Position for CONTROL_MODE_POSITION
    float _positionDegreesPrevious {0.0}; //!< Previous position for CONTROL_MODE_POSITION

    const float _rollMaxAngleDegrees {45.0};
    float _pitchBalanceAngleDegrees {0.0};
    const float _pitchMaxAngleDegrees {20.0};
    //FilterMovingAverage<4> _pitchAngleDTermFilter {};
    PowerTransferFilter2  _pitchAngleDTermFilter {};
    float _yawStickMultiplier {1.0};

    std::array<float, OUTPUT_COUNT> _outputs {};
    std::array<PIDF, PID_COUNT> _PIDS {};
    const std::array<PIDF::PIDF_t, PID_COUNT> _scaleFactors;
};
