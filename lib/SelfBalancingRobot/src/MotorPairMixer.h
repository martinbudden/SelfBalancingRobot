#pragma once

#include <Filters.h>
#include <MotorMixerBase.h>

class MotorPairBase;

/*!
The MotorMixer takes the outputs from the MotorPairController and "mixes" the values, to set the appropriate power for each motor.
*/
class MotorPairMixer : public MotorMixerBase {
public:
    enum { MOTOR_COUNT = 2 };
public:
    explicit MotorPairMixer(MotorPairBase& motorPair) : MotorMixerBase(MOTOR_COUNT), _motorPair(motorPair) {}
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual float getMotorOutput(size_t motorIndex) const override;
    virtual int32_t getMotorRPM(size_t motorIndex) const override;

    void setMotorSwitchOffAngleDegrees(float motorSwitchOffAngleDegrees) { _motorSwitchOffAngleDegrees = motorSwitchOffAngleDegrees; }
    inline void setPitchAngleDegreesRaw(float pitchAngleDegreesRaw) { _pitchAngleDegreesRaw = pitchAngleDegreesRaw; }

    float getPowerLeft() const { return _powerLeft; } //!< for telemetry
    float getPowerRight() const { return _powerRight; } //!< for telemetry
    uint32_t getOutputPowerTimeMicroSeconds() const { return _outputPowerTimeMicroSeconds; } //!< for telemetry
private:
    MotorPairBase& _motorPair; //<! The MotorMixer has a reference to the motor pair for output, ie setting the motor power.
    uint32_t _motorSwitchOffTickCount {0}; //<! For switch bounce protection
    float _powerLeft {0.0};
    float _powerRight {0.0};
    uint32_t _outputPowerTimeMicroSeconds {0}; //!< for instrumentation, time taken to set the motor pair power. Can be significant if motors controlled over I2C
    float _motorSwitchOffAngleDegrees {70.0}; //!< Pitch angle at which the motors switch off. So if the robot flips over it won't lie on its back with its motors spinning.
    float _pitchAngleDegreesRaw {}; //<! The pitch angle, compared with _motorSwitchOffAngleDegrees to see if motors should switch off
    FilterMovingAverage<4> _powerLeftFilter;
    FilterMovingAverage<4> _powerRightFilter;
};

