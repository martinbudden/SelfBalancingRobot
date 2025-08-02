#pragma once

#include <Filters.h>

class MotorPairBase;

/*!
The MotorMixer takes the outputs from the MotorPairController and "mixes" the values, to set the appropriate power for each motor.
*/
class MotorPairMixer {
public:
    struct commands_t {
        float speed;
        float roll;
        float pitch;
        float yaw;
    };
public:
    enum { MOTOR_COUNT = 2 };
public:
    explicit MotorPairMixer(MotorPairBase& motorPair) : _motorPair(motorPair) {}
public:
    inline bool motorsIsOn() const { return _motorsIsOn; }
    inline void motorsSwitchOn() { _motorsIsOn = true; }
    inline void motorsSwitchOff() { _motorsIsOn = false; }
    inline bool motorsIsDisabled() const { return _motorsIsDisabled; }

    void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount);
    float getMotorOutput(size_t motorIndex) const;

    void setMotorSwitchOffAngleDegrees(float motorSwitchOffAngleDegrees) { _motorSwitchOffAngleDegrees = motorSwitchOffAngleDegrees; }
    inline void setPitchAngleDegreesRaw(float pitchAngleDegreesRaw) { _pitchAngleDegreesRaw = pitchAngleDegreesRaw; }

    float getPowerLeft() const { return _powerLeft; } //!< for telemetry
    float getPowerRight() const { return _powerRight; } //!< for telemetry
    uint32_t getOutputPowerTimeMicroSeconds() const { return _outputPowerTimeMicroSeconds; } //!< for telemetry
private:
    MotorPairBase& _motorPair; //<! The MotorMixer has a reference to the motor pair for output, ie setting the motor power.
    int32_t _motorsIsOn {false};
    int32_t _motorsIsDisabled {false};
    uint32_t _motorSwitchOffTickCount {0}; //<! For switch bounce protection
    float _powerLeft {0.0};
    float _powerRight {0.0};
    uint32_t _outputPowerTimeMicroSeconds {0}; //!< for instrumentation, time taken to set the motor pair power. Can be significant if motors controlled over I2C
    float _motorSwitchOffAngleDegrees {70.0}; //!< Pitch angle at which the motors switch off. So if the robot flips over it won't lie on its back with its motors spinning.
    float _pitchAngleDegreesRaw {}; //<! The pitch angle, compared with _motorSwitchOffAngleDegrees to see if motors should switch off
    FilterMovingAverage<4> _powerLeftFilter;
    FilterMovingAverage<4> _powerRightFilter;
};

