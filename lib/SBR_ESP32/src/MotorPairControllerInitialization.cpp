#include "MotorPairControllerDefaults.h"

#include <AHRS.h>
#include <MotorPairBase.h>
#include <MotorPairController.h>

#if defined(USE_MOTOR_PAIR_GPIO)
#include "MotorsGPIO.h"
#endif

#include <cfloat>
#if defined(I2C_MUTEX_REQUIRED)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

/*!
Statically allocate the motors according to the build flags.
*/
MotorPairBase& MotorPairController::allocateMotors()
{
    // Statically allocate the MotorPair object as defined by the build flags.
#if defined(USE_MOTOR_PAIR_GPIO)
    const MotorsGPIO::pins_t pins = MOTOR_PINS;
    static MotorsGPIO motors(pins);// NOLINT(misc-const-correctness) false positive
#endif
#if defined(MOTOR_POWER_DEADBAND)
    motors.setDeadbandPower(MOTOR_DEADBAND_POWER);
#endif
    return motors;
}

/*!
Sets the control mode and adjusts the _PIDS[SPEED_DPS] constants accordingly.
*/
void MotorPairController::setControlMode(ControlMode_t controlMode)
{
    _controlMode = controlMode;
    _PIDS[SPEED_DPS].setPID((controlMode == CONTROL_MODE_SERIAL_PIDS) ? speedPID_DefaultSerial : speedPID_DefaultParallel);
    _PIDS[SPEED_DPS].resetIntegral();
}

/*!
Constructor. Sets member data.
*/
MotorPairController::MotorPairController(const AHRS& ahrs, const ReceiverBase& receiver, [[maybe_unused]] void* i2cMutex) :
    _ahrs(ahrs),
    _receiver(receiver),
    _motors(allocateMotors()),
    _mixer(_motors),
    _motorMaxSpeedDPS(maxMotorRPM * 360 / 60),
    _motorMaxSpeedDPS_reciprocal(1.0F / _motorMaxSpeedDPS),
    _motorStepsPerRevolution(_motors.getStepsPerRevolution()),
    _controlMode(controlModeDefault),
    _pitchBalanceAngleDegrees(pitchBalanceAngleDegrees)
{
    static constexpr float NOT_SET = FLT_MAX;
    _rollAngleDegreesRaw = NOT_SET;
    _yawAngleDegreesRaw = NOT_SET;

#if defined(I2C_MUTEX_REQUIRED)
    _motors.setMutex(static_cast<SemaphoreHandle_t>(i2cMutex));
#endif

    setControlMode(_controlMode);
    _mixer.setMotorSwitchOffAngleDegrees(motorSwitchOffAngleDegrees);

    _PIDS[PITCH_ANGLE_DEGREES].setPID(pitchPID_Default);
    _PIDS[PITCH_ANGLE_DEGREES].setIntegralMax(1.0F);
    _PIDS[PITCH_ANGLE_DEGREES].setOutputSaturationValue(1.0F);

    _speedFilter.setAlpha(0.8F);
    //_PIDS[SPEED_DPS].setIntegralMax(1.0F);

    _PIDS[POSITION_DEGREES].setPID(positionPID_Default);

    _PIDS[YAW_RATE_DPS].setPID(yawRatePID_Default);

    const float yawRateDPS_AtMaxPower = _motorMaxSpeedDPS * wheelDiameterMM / wheelTrackMM; // =7200 *45/75 = 4320 DPS, this is insanely fast
    static constexpr float maxDesiredYawRateDPS {720.0};
    _yawStickMultiplier = maxDesiredYawRateDPS / yawRateDPS_AtMaxPower;
}
