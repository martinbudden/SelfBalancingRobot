#pragma once

#include <Filters.h>
#include <MotorMixerBase.h>

class MotorPairBase;


/*!
The MotorMixer takes the outputs from the MotorPairController and "mixes" the values, to set the appropriate power for each motor.
*/
class MotorPairMixer : public MotorMixerBase {
public:
    enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1, MOTOR_COUNT = 2, SERVO_COUNT = 0 };
public:
    explicit MotorPairMixer(MotorPairBase& motorPair) :
        MotorMixerBase(CUSTOM, MOTOR_COUNT, SERVO_COUNT, nullptr),
        _motorPair(motorPair) {}
public:

    void outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount) override;
    float getMotorOutput(size_t motorIndex) const override;

    bool canReportPosition(size_t motorIndex) const override;
    void resetAllEncoders() override;
    void readEncoder(size_t motorIndex) override;
    int32_t getEncoder(size_t motorIndex) const override;
    uint32_t getStepsPerRevolution(size_t motorIndex) const override;

    bool canReportSpeed(size_t motorIndex) const override;
    int32_t getMotorRPM(size_t motorIndex) const override;
    float getMotorSpeedDPS(size_t motorIndex) const override;

    float getMixerThrottleCommand() const override;

    inline uint32_t getOutputPowerTimeMicroseconds() const { return _outputPowerTimeMicroseconds; } //!< for telemetry
    inline void setMotorSwitchOffAngleDegrees(float motorSwitchOffAngleDegrees) { _motorSwitchOffAngleDegrees = motorSwitchOffAngleDegrees; }
    inline void setPitchAngleDegreesRaw(float pitchAngleDegreesRaw) { _pitchAngleDegreesRaw = pitchAngleDegreesRaw; }
private:
    MotorPairBase& _motorPair; //<! The MotorMixer has a reference to the motor pair for output, ie setting the motor power.

    uint32_t _motorSwitchOffTickCount {0}; //<! For switch bounce protection
    float _throttleCommand {0.0F}; //!< used solely for instrumentation
    float _powerLeft {0.0F};
    float _powerRight {0.0F};
    uint32_t _outputPowerTimeMicroseconds {0}; //!< for instrumentation, time taken to set the motor pair power. Can be significant if motors controlled over I2C
    float _motorSwitchOffAngleDegrees {70.0F}; //!< Pitch angle at which the motors switch off. So if the robot flips over it won't lie on its back with its motors spinning.
    float _pitchAngleDegreesRaw {}; //<! The pitch angle, compared with _motorSwitchOffAngleDegrees to see if motors should switch off
    FilterMovingAverage<4> _powerLeftFilter;
    FilterMovingAverage<4> _powerRightFilter;
};
