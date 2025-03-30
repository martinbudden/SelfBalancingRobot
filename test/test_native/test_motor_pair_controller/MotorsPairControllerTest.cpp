#include "AHRS.h"
#include "MotorPairBase.h"

#include <MotorPairController.h>

class MotorsTest final : public MotorPairBase {
public:
    MotorsTest();
// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
    virtual void setPower(float leftPower, float rightPower) override;
private:
    virtual void readEncoder() override;
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
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

constexpr PIDF::PIDF_t pitchPID_Default                       { 0.0240F,  0.0F,    0.00020F, 0.0F };

constexpr PIDF::PIDF_t yawRatePID_Default                     { 0.0F,     0.0F,    0.0F,     1.00F };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                 { 0.020F,   0.0F,    0.0F,     0.0F };

constexpr PIDF::PIDF_t speedPID_DefaultParallel               { 0.010F,   0.0F,    0.0F,     0.0F };

constexpr float maxMotorRPM                 {620.0F};
constexpr float wheelDiameterMM             {68.0F};
constexpr float wheelTrackMM                {170.0F};
constexpr float pitchBalanceAngleDegrees    {0.0F};
constexpr float motorSwitchOffAngleDegrees  {70.0F};
constexpr float encoderStepsPerRevolution   {1000.0F};

MotorPairBase& MotorPairController::allocateMotors()
{
    // Statically allocate the MotorPair object
    static MotorsTest motors; // NOLINT(misc-const-correctness) false positive
    return motors;
}

/*!
Constructor. Sets member data.
*/
MotorPairController::MotorPairController(const AHRS& ahrs, const ReceiverBase& receiver, [[maybe_unused]] void* i2cMutex) :
    _ahrs(ahrs),
    _receiver(receiver),
    _motors(allocateMotors()),
    _mixer(_motors),
    _controlMode(CONTROL_MODE_SERIAL_PIDS),
    _motorMaxSpeedDPS(maxMotorRPM * 360 / 60),
    _motorMaxSpeedDPS_reciprocal(1.0F / _motorMaxSpeedDPS),
    _motorStepsPerRevolution(_motors.getStepsPerRevolution()),
    _pitchBalanceAngleDegrees(pitchBalanceAngleDegrees)
{
#if defined(I2C_MUTEX_REQUIRED)
    _motors.setMutex(static_cast<SemaphoreHandle_t>(i2cMutex));
#endif

    setControlMode(_controlMode);
    _mixer.setMotorSwitchOffAngleDegrees(motorSwitchOffAngleDegrees);

    _PIDS[PITCH_ANGLE_DEGREES].setPID(pitchPID_Default);
    _PIDS[PITCH_ANGLE_DEGREES].setIntegralMax(1.0F);
    _PIDS[PITCH_ANGLE_DEGREES].setOutputSaturationValue(1.0F);

    _PIDS[SPEED_DPS].setIntegralMax(1.0F);

    _PIDS[YAW_RATE_DPS].setPID(yawRatePID_Default);

    // copy of motorMaxSpeedDPS for telemetry, so telemetry viewer can scale motor speed

    const float yawRateDPS_AtMaxPower = _motorMaxSpeedDPS * wheelDiameterMM / wheelTrackMM; // =7200 *45/75 = 4320 DPS, this is insanely fast
    static constexpr float maxDesiredYawRateDPS {720.0};
    _yawStickMultiplier = maxDesiredYawRateDPS / yawRateDPS_AtMaxPower;
}

/*!
Sets the control mode and adjusts the _PIDS[SPEED_DPS] constants accordingly.
*/
void MotorPairController::setControlMode(ControlMode_t controlMode)
{
    _PIDS[SPEED_DPS].resetIntegral();
    _controlMode = controlMode;
    if (controlMode == CONTROL_MODE_SERIAL_PIDS) { // NOLINT(bugprone-branch-clone) false positive
        _PIDS[SPEED_DPS].setPID(speedPID_DefaultSerial);
    } else {
        _PIDS[SPEED_DPS].setPID(speedPID_DefaultParallel);
    }
}

