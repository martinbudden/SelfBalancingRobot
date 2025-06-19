#include "MotorPairControllerDefaults.h"

#include <MotorPairBase.h>

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
#elif defined(MOTORS_GPIO)
#include "MotorsGPIO.h"
#else
#error "No MOTORS defined"
#endif

/*!
Statically allocate the motors according to the build flags.
*/
MotorPairBase& MotorPairController::allocateMotors(const vehicle_t& vehicle)
{
    (void)vehicle;
    // Statically allocate the MotorPair object as defined by the build flags.
#if defined(MOTORS_BALA_2)
    static MotorsBala2 motors(MOTOR_SDA_PIN, MOTOR_SCL_PIN);// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_BALA_C)
    //static MotorsBalaC motors(MOTOR_SDA_PIN, MOTOR_SCL_PIN);// NOLINT(misc-const-correctness) false positive
    static MotorsBalaC motors(MOTOR_DEADBAND_POWER);// NOLINT(misc-const-correctness) false positive
    //motors.setDeadbandPower(MOTOR_DEADBAND_POWER);
#elif defined(MOTORS_4_ENCODER_MOTOR)
    static Motors4EncoderMotor motors(MOTOR_SDA_PIN, MOTOR_SCL_PIN, vehicle.encoderStepsPerRevolution);// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_GO_PLUS_2)
    static MotorsGoPlus2 motors(MOTOR_SDA_PIN, MOTOR_SCL_PIN);// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_ATOMIC_MOTION_BASE)
    static MotorsAtomicMotionBase motors(MOTOR_DEADBAND_POWER, MOTOR_SDA_PIN, MOTOR_SCL_PIN);// NOLINT(misc-const-correctness) false positive
    //motors.setDeadbandPower(MOTOR_DEADBAND_POWER);
#elif defined(MOTORS_PWR_CAN)
    static MotorsPwrCAN motors;// NOLINT(misc-const-correctness) false positive
#elif defined(MOTORS_O_DRIVE_CAN)
    static Motors_ODriveCAN motors(vehicle.encoderStepsPerRevolution);
    motors.setup();
#elif defined(MOTORS_O_DRIVE_TWAI)
    static Motors_ODriveTWAI motors(vehicle.encoderStepsPerRevolution);
    motors.setup();
#elif defined(MOTORS_ROLLER_CAN)
#elif defined(MOTORS_GPIO)
    const MotorsGPIO::pins_t pins = MOTOR_GPIO_PINS;
    static MotorsGPIO motors(pins);// NOLINT(misc-const-correctness) false positive
#endif

    return motors;
}

MotorPairController::MotorPairController(uint32_t taskIntervalMicroSeconds, const AHRS& ahrs, ReceiverBase& receiver, void* i2cMutex) :
    MotorPairController(taskIntervalMicroSeconds, ahrs, receiver, i2cMutex, gVehicle, gScaleFactors)
{
}

    

/*!
Constructor. Sets member data.
*/
MotorPairController::MotorPairController(uint32_t taskIntervalMicroSeconds, const AHRS& ahrs, ReceiverBase& receiver, void* i2cMutex, const vehicle_t& vehicle, const pidf_array_t& scaleFactors) :
    VehicleControllerBase(SELF_BALANCING_ROBOT, PID_COUNT),
    _ahrs(ahrs),
    _receiver(receiver),
    _motorPair(allocateMotors(vehicle)),
    _motorPairMixer(_motorPair),
    _motorMaxSpeedDPS(vehicle.maxMotorRPM * 360 / 60),
    _motorMaxSpeedDPS_reciprocal(1.0F / _motorMaxSpeedDPS),
    _motorPairStepsPerRevolution(_motorPair.getStepsPerRevolution()),
    _pitchBalanceAngleDegrees(vehicle.pitchBalanceAngleDegrees),
    _scaleFactors(scaleFactors)
{
    _rollAngleDegreesRaw = NOT_SET;
    _yawAngleDegreesRaw = NOT_SET;

#if defined(I2C_MUTEX_REQUIRED)
    _motorPair.setMutex(static_cast<SemaphoreHandle_t>(i2cMutex));
#else
    (void)i2cMutex;
#endif

    _motorPairMixer.setMotorSwitchOffAngleDegrees(vehicle.motorSwitchOffAngleDegrees);

    for (size_t ii = PID_BEGIN; ii < PID_COUNT; ++ii) {
        _PIDS[ii].setPID(gDefaultPIDs[ii]);
    }

    const float deltaT = static_cast<float>(taskIntervalMicroSeconds) / 1000000.0F;
    _pitchAngleDTermFilter.setCutoffFrequency(50.0F, deltaT);
/*
gain20=0.493995
gain40=0.661307
gain60=0.745469
gain70=0.796129
gain100=0.829970
*/
    _PIDS[PITCH_ANGLE_DEGREES].setIntegralMax(1.0F);
    _PIDS[PITCH_ANGLE_DEGREES].setOutputSaturationValue(1.0F);

    _speedFilter.setAlpha(0.8F);
    //_PIDS[SPEED_DPS].setIntegralMax(1.0F);

    const float yawRateDPS_AtMaxPower = _motorMaxSpeedDPS * vehicle.wheelDiameterMM / vehicle.wheelTrackMM; // =7200 *45/75 = 4320 DPS, this is insanely fast
    static constexpr float maxDesiredYawRateDPS {720.0};
    _yawStickMultiplier = maxDesiredYawRateDPS / yawRateDPS_AtMaxPower;
}
