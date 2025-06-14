#include "AHRS.h"
#include "MotorPairBase.h"

#include <MotorPairController.h>

class MotorPairTest final : public MotorPairBase {
public:
    MotorPairTest();
// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
    virtual void setPower(float leftPower, float rightPower) override;
private:
    virtual void readEncoder() override;
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
};

MotorPairTest::MotorPairTest() : // NOLINT(hicpp-use-equals-default, modernize-use-equals-default)
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED)
    {}

void MotorPairTest::readEncoder()
{
}

void MotorPairTest::setPower([[maybe_unused]] float leftPower,[[maybe_unused]] float rightPower)
{
}

constexpr PIDF::PIDF_t pitchPID_Default                       { 0.0240F,  0.0F,    0.00020F, 0.0F };

constexpr PIDF::PIDF_t yawRatePID_Default                     { 0.0F,     0.0F,    0.0F,     1.00F };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                 { 0.020F,   0.0F,    0.0F,     0.0F };

constexpr PIDF::PIDF_t speedPID_DefaultParallel               { 0.010F,   0.0F,    0.0F,     0.0F };

constexpr MotorPairController::pidf_array_t gScaleFactors {{
    { 0.0001F,  0.001F, 0.000002F,  0.01F },    // ROLL_ANGLE_DEGREES=0,
    { 0.0002F,  0.001F, 0.000002F,  0.01F },    // PITCH_ANGLE_DEGREES
    { 0.01F,    0.01F,  0.01F,      0.01F },    // YAW_RATE_DPS
    { 0.01F,    0.01F,  0.00001F,   0.01F },    // SPEED_SERIAL_DPS
    { 0.001F,   0.01F,  0.0001F,    0.01F },    // SPEED_PARALLEL_DPS
    { 0.10F,    0.01F,  0.001F,     0.01F }     // POSITION_DEGREES
}};

const MotorPairController::vehicle_t gVehicle = {
    .maxMotorRPM                 = 5620.0F,
    .wheelDiameterMM             = 68.0F,
    .wheelTrackMM                = 170.0F,
    .pitchBalanceAngleDegrees    = 0.0F,
    .motorSwitchOffAngleDegrees  = 70.0F,
    .encoderStepsPerRevolution   = 1000.0F
};


MotorPairBase& MotorPairController::allocateMotors()
{
    // Statically allocate the MotorPair object
    static MotorPairTest motorPair; // NOLINT(misc-const-correctness) false positive
    return motorPair;
}

/*!
Constructor. Sets member data.
*/
MotorPairController::MotorPairController(const AHRS& ahrs, ReceiverBase& receiver, [[maybe_unused]] void* i2cMutex) :
    VehicleControllerBase(VehicleControllerBase::SELF_BALANCING_ROBOT, PID_COUNT),
    _ahrs(ahrs),
    _receiver(receiver),
    _motorPair(allocateMotors()),
    _motorPairMixer(_motorPair),
    _controlMode(CONTROL_MODE_SERIAL_PIDS),
    _motorMaxSpeedDPS(gVehicle.maxMotorRPM * 360 / 60),
    _motorMaxSpeedDPS_reciprocal(1.0F / _motorMaxSpeedDPS),
    _motorPairStepsPerRevolution(_motorPair.getStepsPerRevolution()),
    _pitchBalanceAngleDegrees(gVehicle.pitchBalanceAngleDegrees),
    _scaleFactors(gScaleFactors)
{
#if defined(I2C_MUTEX_REQUIRED)
    _motorPair.setMutex(static_cast<SemaphoreHandle_t>(i2cMutex));
#endif

    _motorPairMixer.setMotorSwitchOffAngleDegrees(gVehicle.motorSwitchOffAngleDegrees);

    _PIDS[PITCH_ANGLE_DEGREES].setPID(pitchPID_Default);
    _PIDS[PITCH_ANGLE_DEGREES].setIntegralMax(1.0F);
    _PIDS[PITCH_ANGLE_DEGREES].setOutputSaturationValue(1.0F);

    _PIDS[SPEED_SERIAL_DPS].setIntegralMax(1.0F);
    _PIDS[SPEED_PARALLEL_DPS].setIntegralMax(1.0F);

    _PIDS[YAW_RATE_DPS].setPID(yawRatePID_Default);

    // copy of motorMaxSpeedDPS for telemetry, so telemetry viewer can scale motor speed

    const float yawRateDPS_AtMaxPower = _motorMaxSpeedDPS * gVehicle.wheelDiameterMM / gVehicle.wheelTrackMM; // =7200 *45/75 = 4320 DPS, this is insanely fast
    static constexpr float maxDesiredYawRateDPS {720.0};
    _yawStickMultiplier = maxDesiredYawRateDPS / yawRateDPS_AtMaxPower;
}
