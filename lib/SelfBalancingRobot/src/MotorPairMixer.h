#pragma once

#include <Filters.h>

class MotorPairBase;


/*!
The MotorMixer takes the outputs from the MotorPairController and "mixes" the values, to set the appropriate power for each motor.
*/
class MotorPairMixer {
public:
    struct commands_t {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    };
public:
    enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1, MOTOR_COUNT = 2 };
public:
    explicit MotorPairMixer(MotorPairBase& motorPair) : _motorPair(motorPair) {}
public:
    inline bool motorsIsOn() const { return _motorsIsOn; }
    inline void motorsSwitchOn() { _motorsIsOn = true; }
    inline void motorsSwitchOff() { _motorsIsOn = false; }
    inline bool motorsIsDisabled() const { return _motorsIsDisabled; }
    inline float getThrottleCommand() const { return _throttleCommand; }

    void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount);
    float getMotorOutput(size_t motorIndex) const;

    inline void setMotorSwitchOffAngleDegrees(float motorSwitchOffAngleDegrees) { _motorSwitchOffAngleDegrees = motorSwitchOffAngleDegrees; }
    inline void setPitchAngleDegreesRaw(float pitchAngleDegreesRaw) { _pitchAngleDegreesRaw = pitchAngleDegreesRaw; }

    inline uint32_t getOutputPowerTimeMicroseconds() const { return _outputPowerTimeMicroseconds; } //!< for telemetry

    void readEncoder(size_t motorIndex);
    int32_t getEncoder(size_t motorIndex) const;
    float getStepsPerRevolution(size_t motorIndex) const;
    void resetEncoderToZero(size_t motorIndex);
    bool canAccuratelyEstimateSpeed(size_t motorIndex) const;
    float getSpeed(size_t motorIndex) const;
private:
    MotorPairBase& _motorPair; //<! The MotorMixer has a reference to the motor pair for output, ie setting the motor power.
    int32_t _motorsIsOn {false};
    int32_t _motorsIsDisabled {false};
    uint32_t _motorSwitchOffTickCount {0}; //<! For switch bounce protection
    float _throttleCommand {0.0F}; //!< used solely for instrumentation
    float _powerLeft {0.0};
    float _powerRight {0.0};
    uint32_t _outputPowerTimeMicroseconds {0}; //!< for instrumentation, time taken to set the motor pair power. Can be significant if motors controlled over I2C
    float _motorSwitchOffAngleDegrees {70.0}; //!< Pitch angle at which the motors switch off. So if the robot flips over it won't lie on its back with its motors spinning.
    float _pitchAngleDegreesRaw {}; //<! The pitch angle, compared with _motorSwitchOffAngleDegrees to see if motors should switch off
    FilterMovingAverage<4> _powerLeftFilter;
    FilterMovingAverage<4> _powerRightFilter;
};

