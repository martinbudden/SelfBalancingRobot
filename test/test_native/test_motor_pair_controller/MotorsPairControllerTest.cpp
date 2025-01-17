#include "AHRS.h"
#include "MotorPairBase.h"

#include <MotorPairController.h>

class MotorsTest final : public MotorPairBase {
public:
    MotorsTest();
public:
    virtual void setPower(float leftPower, float rightPower) override;
private:
    virtual void readEncoder() override;
};

MotorsTest::MotorsTest() :
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED)
    {}

void MotorsTest::readEncoder()
{
}

void MotorsTest::setPower([[maybe_unused]] float leftPower,[[maybe_unused]] float rightPower)
{
}

constexpr PIDF::PIDF_t pitchPID_Default                       { 0.0240,  0.0,    0.00020, 0.0 };

constexpr PIDF::PIDF_t yawRatePID_Default                     { 0.0,     0.0,    0.0,     1.00 };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                 { 0.020,   0.0,    0.0,     0.0 };

constexpr PIDF::PIDF_t speedPID_DefaultParallel               { 0.010,   0.0,    0.0,     0.0 };

constexpr float maxMotorRPM                 {620.0};
constexpr float wheelDiameterMM             {68.0};
constexpr float wheelTrackMM                {170.0};
constexpr float pitchBalanceAngleDegrees    {0.0};
constexpr float motorSwitchOffAngleDegrees  {70.0};
constexpr float encoderStepsPerRevolution   {1000.0};

MotorPairBase& MotorPairController::motors()
{
    // Statically allocate the MotorPair object
    static MotorsTest motors;
    return motors;
}

/*!
Constructor. Sets member data.
*/
MotorPairController::MotorPairController(const AHRS& ahrs, const ReceiverBase& receiver, [[maybe_unused]] void* i2cMutex) :
    _ahrs(ahrs),
    _receiver(receiver),
    _motors(motors()),
    _motorMaxSpeedDPS(maxMotorRPM * 360 / 60),
    _motorMaxSpeedDPS_reciprocal(1.0F / _motorMaxSpeedDPS),
    _motorStepsPerRevolution(_motors.getStepsPerRevolution()),
    _controlMode(CONTROL_MODE_SERIAL_PIDS),
    _pitchBalanceAngleDegrees(pitchBalanceAngleDegrees)
{
#if defined(I2C_MUTEX_REQUIRED)
    _motors.setMutex(static_cast<SemaphoreHandle_t>(i2cMutex));
#endif

    _mixer.motorSwitchOffAngleDegrees = motorSwitchOffAngleDegrees;
    setControlMode(_controlMode);

    _PIDS[PITCH_ANGLE].setPID(pitchPID_Default);
    _PIDS[PITCH_ANGLE].setIntegralMax(1.0F);
    _PIDS[PITCH_ANGLE].setOutputSaturationValue(1.0F);

    _PIDS[SPEED].setIntegralMax(1.0F);

    _PIDS[YAW_RATE].setPID(yawRatePID_Default);

    // copy of motorMaxSpeedDPS for telemetry, so telemetry viewer can scale motor speed

    const float yawRateDPS_AtMaxPower = _motorMaxSpeedDPS * wheelDiameterMM / wheelTrackMM; // =7200 *45/75 = 4320 DPS, this is insanely fast
    static constexpr float maxDesiredYawRateDPS {720.0};
    _yawStickMultiplier = maxDesiredYawRateDPS / yawRateDPS_AtMaxPower;
}

/*!
Sets the control mode and adjusts the _PIDS[SPEED] constants accordingly.
*/
void MotorPairController::setControlMode(ControlMode_t controlMode)
{
    _PIDS[SPEED].resetIntegral();
    _controlMode = controlMode;
    if (controlMode == CONTROL_MODE_SERIAL_PIDS) {
        _PIDS[SPEED].setPID(speedPID_DefaultSerial);
    } else {
        _PIDS[SPEED].setPID(speedPID_DefaultParallel);
    }
}

