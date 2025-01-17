#include "MotorPairController.h"

#include "AHRS.h"
#include "MotorPairBase.h"
#include "MotorPairControllerDefaults.h"

#if defined(MOTORS_4_ENCODER_MOTOR)
#include "Motors4EncoderMotor.h"
#elif defined(MOTORS_ATOMIC_MOTION_BASE)
#include "MotorsAtomicMotionBase.h"
#elif defined(MOTORS_BALA_2)
#include "MotorsBala2.h"
#elif defined(MOTORS_BALA_C)
#include "MotorsBalaC.h"
#elif defined(MOTORS_GO_PLUS_2)
#include "MotorsGoPlus2.h"
#elif defined(MOTORS_PWR_CAN)
#include "MotorsPwrCAN.h"
#elif defined(MOTORS_O_DRIVE_CAN)
#include "Motors_ODriveCAN.h"
#elif defined(MOTORS_O_DRIVE_TWAI)
#include "Motors_ODriveTWAI.h"
#endif

#include <cfloat>
#if defined(I2C_MUTEX_REQUIRED)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

/*!
Statically allocate the motors according to the build flags.
*/
MotorPairBase& MotorPairController::motors()
{
    // Statically allocate the MotorPair object as defined by the build flags.
#if defined(MOTORS_BALA_2)
    static MotorsBala2 motors(MOTOR_SDA_PIN, MOTOR_SCL_PIN);// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_BALA_C)
    static MotorsBalaC motors(MOTOR_SDA_PIN, MOTOR_SCL_PIN);// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_4_ENCODER_MOTOR)
    static Motors4EncoderMotor motors(MOTOR_SDA_PIN, MOTOR_SCL_PIN, encoderStepsPerRevolution);// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_GO_PLUS_2)
    static MotorsGoPlus2 motors(MOTOR_SDA_PIN, MOTOR_SCL_PIN);// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_ATOMIC_MOTION_BASE)
    static MotorsAtomicMotionBase motors(MOTOR_SDA_PIN, MOTOR_SCL_PIN);// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_PWR_CAN)
    static MotorsPwrCAN motors;// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_O_DRIVE_CAN)
    static Motors_ODriveCAN motors(encoderStepsPerRevolution);
    motors.setup();
#elif defined(MOTORS_O_DRIVE_TWAI)
    static Motors_ODriveTWAI motors(encoderStepsPerRevolution);
    motors.setup();
#elif defined(MOTORS_ROLLER_CAN)
#endif

#if defined(MOTOR_POWER_DEADBAND)
    motors.setDeadbandPower(MOTOR_DEADBAND_POWER);
#endif
    return motors;
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
    } else if (controlMode == CONTROL_MODE_PARALLEL_PIDS) {
        _speedPID.setPID(speedPID_DefaultParallel);
    }
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
    _mixer.motorSwitchOffAngleDegrees = motorSwitchOffAngleDegrees;

    _pitchAnglePID.setPID(pitchPID_Default);
    _pitchAnglePID.setIntegralMax(1.0F);
    _pitchAnglePID.setOutputSaturationValue(1.0F);

    _speedFilter.setAlpha(0.8F);
    _speedPID.setIntegralMax(1.0F);

    _positionPID.setPID(positionPID_Default);

    _yawRatePID.setPID(yawRatePID_Default);

    const float yawRateDPS_AtMaxPower = _motorMaxSpeedDPS * wheelDiameterMM / wheelTrackMM; // =7200 *45/75 = 4320 DPS, this is insanely fast
    static constexpr float maxDesiredYawRateDPS {720.0};
    _yawStickMultiplier = maxDesiredYawRateDPS / yawRateDPS_AtMaxPower;
}

