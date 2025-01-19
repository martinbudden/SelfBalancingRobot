#pragma once

#include <MotorMixerBase.h>

class MotorPairBase;


class MotorMixer : public MotorMixerBase {
public:
    explicit MotorMixer(MotorPairBase& motors) : _motors(motors) {}
public:
    virtual void outputToMotors(const output_t& outputs, float deltaT, uint32_t tickCount) override;

    void setMotorSwitchOffAngleDegrees(float motorSwitchOffAngleDegrees) { _motorSwitchOffAngleDegrees = motorSwitchOffAngleDegrees; }
    inline void setPitchAngleDegreesRaw(float pitchAngleDegreesRaw) { _pitchAngleDegreesRaw = pitchAngleDegreesRaw; }

    float getPowerLeft() const { return _powerLeft; } //!< for telemetry
    float getPowerRight() const { return _powerRight; } //!< for telemetry
    uint32_t getOutputPowerTimeMicroSeconds() const { return _outputPowerTimeMicroSeconds; } //!< for telemetry
private:
    MotorPairBase& _motors;
    uint32_t _motorSwitchOffTickCount {0};
    float _powerLeft {0.0};
    float _powerRight {0.0};
    uint32_t _outputPowerTimeMicroSeconds {0}; //!< for instrumentation, time taken to set the motor pair power
    float _motorSwitchOffAngleDegrees {70.0}; //!< Pitch angle at which the motors switch off. So if the robot flips over it won't lie on its back with its motors spinning.
    float _pitchAngleDegreesRaw {};
};

