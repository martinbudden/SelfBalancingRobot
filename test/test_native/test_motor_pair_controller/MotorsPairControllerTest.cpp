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
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors         { 0.0001,  0.001,  0.00001, 0.1 };

constexpr PIDF::PIDF_t yawRatePID_Default                     { 0.0,     0.0,    0.0,     1.00 };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors       { 0.1,     1.0,    0.01,    0.01 };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                 { 0.020,   0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial   { 0.001,   0.001,  0.00001, 0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultParallel               { 0.010,   0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel { 0.001,   1.0,    0.00001, 0.1 };

constexpr MotorPairController::ControlMode_t CONTROL_MODE = MotorPairController::CONTROL_MODE_SERIAL_PIDS;
const PIDF::PIDF_t& speedPID_Default = speedPID_DefaultSerial;
const PIDF::PIDF_t& speedPID_TelemetryScaleFactors = speedPID_TelemetryScaleFactorsSerial;

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
    _motorSwitchOffAngleDegrees(motorSwitchOffAngleDegrees),
    _controlMode(CONTROL_MODE),
    // scale factors to bring PIDs into approximately the range [0, 100] for telemetry display and PID tuning
    _pitchPIDTelemetryScaleFactors(pitchPID_TelemetryScaleFactors),
    _speedPIDTelemetryScaleFactors(speedPID_TelemetryScaleFactors),
    _yawRatePIDTelemetryScaleFactors(yawRatePID_TelemetryScaleFactors),
    _pitchBalanceAngleDegrees(pitchBalanceAngleDegrees)
{
#if defined(I2C_MUTEX_REQUIRED)
    _motors.setMutex(static_cast<SemaphoreHandle_t>(i2cMutex));
#endif


    _pitchPID.setPID(pitchPID_Default);
    _pitchPID.setIntegralMax(1.0F);
    _pitchPID.setOutputSaturationValue(1.0F);

    _speedPID.setPID(speedPID_Default);
    _speedPID.setIntegralMax(1.0F);

    _yawRatePID.setPID(yawRatePID_Default);

    // copy of motorMaxSpeedDPS for telemetry, so telemetry viewer can scale motor speed

    const float yawRateDPS_AtMaxPower = _motorMaxSpeedDPS * wheelDiameterMM / wheelTrackMM; // =7200 *45/75 = 4320 DPS, this is insanely fast
    static constexpr float maxDesiredYawRateDPS {720.0};
    _yawStickMultiplier = maxDesiredYawRateDPS / yawRateDPS_AtMaxPower;
}

/*!
Sets the control mode and adjusts the _speedPID constants accordingly.
*/
void MotorPairController::setControlMode(ControlMode_t controlMode)
{
    _speedPID.resetIntegral();
    _controlMode = controlMode;
    if (controlMode == CONTROL_MODE_SERIAL_PIDS) {
        _speedPID.setPID(speedPID_DefaultSerial);
        _speedPIDTelemetryScaleFactors = speedPID_TelemetryScaleFactorsSerial;
    } else {
        _speedPID.setPID(speedPID_DefaultParallel);
        _speedPIDTelemetryScaleFactors = speedPID_TelemetryScaleFactorsParallel;
    }
}

