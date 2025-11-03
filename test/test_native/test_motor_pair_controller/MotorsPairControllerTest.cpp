#include <AHRS.h>
#include <MotorPairBase.h>
#include <MotorPairController.h>


class MotorPairTest final : public MotorPairBase {
public:
    MotorPairTest();
// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
    virtual void setPower(float leftPower, float rightPower) override;
private:
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
};

MotorPairTest::MotorPairTest() : // NOLINT(hicpp-use-equals-default, modernize-use-equals-default)
    MotorPairBase(0, CANNOT_ACCURATELY_ESTIMATE_SPEED)
    {}

void MotorPairTest::setPower(float leftPower, float rightPower)
{
    (void)leftPower;
    (void)rightPower;
}

const MotorPairController::vehicle_t gVehicle = {
    .maxMotorRPM                = 200.0F, // this is an estimate of max RPM under load
    .wheelDiameterMM            = 45.0F,
    .wheelTrackMM               = 75.0F,
    .pitchBalanceAngleDegrees   = -2.5F,
    .motorSwitchOffAngleDegrees = 70.0F,
    .encoderStepsPerRevolution  = 420.0F
};
//constexpr PIDF::PIDF_t pitchPID_Default                       { 0.0240F,  0.0F,    0.00020F, 0.0F };
//constexpr PIDF::PIDF_t yawRatePID_Default                     { 0.0F,     0.0F,    0.0F,     1.00F };
//constexpr PIDF::PIDF_t speedPID_DefaultSerial                 { 0.020F,   0.0F,    0.0F,     0.0F };
//constexpr PIDF::PIDF_t speedPID_DefaultParallel               { 0.010F,   0.0F,    0.0F,     0.0F };

constexpr MotorPairController::pidf_array_t gDefaultPIDs {{
    { 0.0F,     0.0F,   0.0F,       0.1F, 0.0F },     // ROLL_ANGLE_DEGREES=0,
    { 0.0300F,  0.0F,   0.00020F,   0.1F, 0.0F },     // PITCH_ANGLE_DEGREES
    { 0.0F,     0.0F,   0.0F,       1.0F, 0.0F },     // YAW_RATE_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F, 0.0F },     // SPEED_SERIAL_DPS
    { 0.030F,   0.0F,   0.0F,       0.0F, 0.0F },     // SPEED_PARALLEL_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F, 0.0F }      // POSITION_DEGREES
}};

constexpr MotorPairController::pidf_array_t gScaleFactors {{
    { 0.0001F,  0.001F, 0.000002F,  0.01F, 0.01F },    // ROLL_ANGLE_DEGREES=0,
    { 0.0002F,  0.001F, 0.000002F,  0.01F, 0.01F },    // PITCH_ANGLE_DEGREES
    { 0.01F,    0.01F,  0.01F,      0.01F, 0.01F },    // YAW_RATE_DPS
    { 0.01F,    0.01F,  0.00001F,   0.01F, 0.01F },    // SPEED_SERIAL_DPS
    { 0.001F,   0.01F,  0.0001F,    0.01F, 0.01F },    // SPEED_PARALLEL_DPS
    { 0.10F,    0.01F,  0.001F,     0.01F, 0.01F }     // POSITION_DEGREES
}};


MotorPairBase& MotorPairController::allocateMotors()
{
    // Statically allocate the MotorPair object
    static MotorPairTest motorPair; // NOLINT(misc-const-correctness) false positive
    return motorPair;
}

/*!
Constructor. Sets member data.
*/
MotorPairController::MotorPairController(uint32_t taskIntervalMicroseconds, uint32_t outputToMotorsDenominator, MotorPairBase& motorPair, AHRS_MessageQueue& ahrsMessageQueue, void* i2cMutex) :
    VehicleControllerBase(SELF_BALANCING_ROBOT, PID_COUNT, taskIntervalMicroseconds),
    _motorPair(motorPair),
    _motorPairMixer(_motorPair),
    _ahrsMessageQueue(ahrsMessageQueue),
    _outputToMotorsDenominator(outputToMotorsDenominator),
    _motorMaxSpeedDPS(gVehicle.maxMotorRPM * 360 / 60),
    _motorMaxSpeedDPS_reciprocal(1.0F / _motorMaxSpeedDPS),
    _motorPairStepsPerRevolution(_motorPair.getStepsPerRevolution()),
    _pitchBalanceAngleDegrees(gVehicle.pitchBalanceAngleDegrees)
{
    (void)motorPair;
    (void)i2cMutex;

    _motorPairMixer.setMotorSwitchOffAngleDegrees(gVehicle.motorSwitchOffAngleDegrees);

    for (size_t ii = PID_BEGIN; ii < PID_COUNT; ++ii) {
        _PIDS[ii].setPID(gDefaultPIDs[ii]);
    }

    _PIDS[PITCH_ANGLE_DEGREES].setIntegralMax(1.0F);
    _PIDS[PITCH_ANGLE_DEGREES].setOutputSaturationValue(1.0F);

    _PIDS[SPEED_SERIAL_DPS].setIntegralMax(1.0F);
    _PIDS[SPEED_PARALLEL_DPS].setIntegralMax(1.0F);


    // copy of motorMaxSpeedDPS for telemetry, so telemetry viewer can scale motor speed

    const float yawRateDPS_AtMaxPower = _motorMaxSpeedDPS * gVehicle.wheelDiameterMM / gVehicle.wheelTrackMM; // =7200 *45/75 = 4320 DPS, this is insanely fast
    static constexpr float maxDesiredYawRateDPS {720.0};
    _yawStickMultiplier = maxDesiredYawRateDPS / yawRateDPS_AtMaxPower;
}
