#pragma once

#include <MotorPairController.h>
#include <PIDF.h>

// use anonymous namespace to make items local to this translation unit
namespace {// NOLINT(clang-diagnostic-implicit-int) false positive

// MOTORS_BALA_2
constexpr MotorPairController::control_mode_e controlModeBala2 = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

const MotorPairController::vehicle_t vehicleBala2 = {
    .maxMotorRPM                = 200.0F, // this is an estimate of max RPM under load
    .wheelDiameterMM            = 45.0F,
    .wheelTrackMM               = 75.0F,
    .pitchBalanceAngleDegrees   = -2.5F,
    .motorSwitchOffAngleDegrees = 70.0F,
    .encoderStepsPerRevolution  = 420.0F
};

constexpr MotorPairController::pidf_array_t scaleFactorsBala2 {{
    { 0.0001F,  0.001F, 0.00001F,   0.01F },    // ROLL_ANGLE_DEGREES=0,
    { 0.0002F,  0.001F, 0.000002F,   0.01F },    // PITCH_ANGLE_DEGREES
    { 0.01F,    0.01F,  0.01F,      0.01F },    // YAW_RATE_DPS
    { 0.01F,    0.01F,  0.00001F,   0.01F },    // SPEED_SERIAL_DPS
    { 0.001F,   0.01F,  0.0001F,    0.01F },    // SPEED_PARALLEL_DPS
    { 0.10F,    0.01F,  0.001F,     0.01F }     // POSITION_DEGREES
}};

constexpr MotorPairController::pidf_array_t defaultPIDsBala2 {{
    { 0.0F,     0.0F,   0.0F,       0.1F },     // ROLL_ANGLE_DEGREES=0,
    { 0.0300F,  0.0F,   0.00020F,   0.1F },     // PITCH_ANGLE_DEGREES
    { 0.0F,     0.0F,   0.0F,       1.0F },     // YAW_RATE_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F },     // SPEED_SERIAL_DPS
    { 0.030F,   0.0F,   0.0F,       0.0F },     // SPEED_PARALLEL_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F }      // POSITION_DEGREES
}};


// MOTORS_BALA_C
constexpr MotorPairController::control_mode_e controlModeBalaC = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

const MotorPairController::vehicle_t vehicleBalaC = {
    .maxMotorRPM                = 80.0F,
    .wheelDiameterMM            = 66.5F,
    .wheelTrackMM               = 70.0F,
    .pitchBalanceAngleDegrees   = -13.0F,
    .motorSwitchOffAngleDegrees = 70.0F,
    .encoderStepsPerRevolution  = 0.0F
};

constexpr MotorPairController::pidf_array_t scaleFactorsBalaC {{
    { 0.0001F,  0.001F, 0.00001F,0.1F },    // ROLL_ANGLE_DEGREES=0,
    { 0.0001F,  1.0F,   0.00001F,0.1F },    // PITCH_ANGLE_DEGREES
    { 0.1F,     1.0F,   0.01F,   0.01F },   // YAW_RATE_DPS
    { 0.01F,    0.01F,  0.0001F, 0.1F },    // SPEED_SERIAL_DPS
    { 0.01F,    0.01F,  0.0001F, 0.01F },   // SPEED_PARALLEL_DPS
    { 0.10F,    0.01F,  0.001F,  0.1F }     // POSITION_DEGREES
}};

constexpr MotorPairController::pidf_array_t defaultPIDsBalaC {{
    { 0.0F,     0.0F,   0.0F,       0.1F },     // ROLL_ANGLE_DEGREES=0,
    { 0.0300F,  0.0F,   0.0F,       0.0F },     // PITCH_ANGLE_DEGREES
    { 0.0F,     0.0F,   0.0F,       1.0F },     // YAW_RATE_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F },     // SPEED_SERIAL_DPS
    { 0.050F,   0.0F,   0.0F,       0.0F },     // SPEED_PARALLEL_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F }      // POSITION_DEGREES
}};


// MOTORS_4_ENCODER_MOTOR
constexpr MotorPairController::control_mode_e controlMode4EncoderMotor = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

const MotorPairController::vehicle_t vehicle4EncoderMotor = {
    .maxMotorRPM                = 170.0F,
    .wheelDiameterMM            = 68.0F,
    .wheelTrackMM               = 180.0F,
    .pitchBalanceAngleDegrees   = -0.5F,
    .motorSwitchOffAngleDegrees = 65.0F,
    .encoderStepsPerRevolution  = 750.0F
};

constexpr MotorPairController::pidf_array_t defaultPIDs4EncoderMotor {{
    { 0.0F,     0.0F,   0.0F,       0.0F },     // ROLL_ANGLE_DEGREES=0,
    { 0.0460F,  0.0F,   0.00130F,   0.0F },     // PITCH_ANGLE_DEGREES
    { 0.0F,     0.0F,   0.0F,       0.5F },     // YAW_RATE_DPS
    { 0.00001F, 0.0F,   0.0F,       0.0F },     // SPEED_SERIAL_DPS
    { 0.650F,   0.0F,   0.0F,       0.0F },     // SPEED_PARALLEL_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F }      // POSITION_DEGREES
}};

const MotorPairController::pidf_array_t& scaleFactors4EncoderMotor = scaleFactorsBala2;

// MOTORS_GO_PLUS_2 and MOTORS_ATOMIC_MOTION_BASE
constexpr MotorPairController::control_mode_e controlModeGoPlus2 = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;
const MotorPairController::vehicle_t vehicleGoPlus2 = {
    .maxMotorRPM                 = 100.0F,
    .wheelDiameterMM             = 56.0F,
    .wheelTrackMM                = 155.0F,
    .pitchBalanceAngleDegrees    = 0.0F,
    .motorSwitchOffAngleDegrees  = 70.0F,
    .encoderStepsPerRevolution   = 0.0F
};

constexpr MotorPairController::pidf_array_t defaultPIDsGoPlus2 {{
    { 0.0F,     0.0F,   0.0F,       0.0F },     // ROLL_ANGLE_DEGREES=0,
    { 0.0000F,  0.0F,   0.00130F,   0.0F },     // PITCH_ANGLE_DEGREES
    { 0.0F,     0.0F,   0.0F,       1.0F },     // YAW_RATE_DPS
    { 0.00000F, 0.0F,   0.0F,       0.0F },     // SPEED_SERIAL_DPS
    { 0.000F,   0.0F,   0.0F,       0.0F },     // SPEED_PARALLEL_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F }      // POSITION_DEGREES
}};

const MotorPairController::pidf_array_t& scaleFactorsGoPlus2 = scaleFactorsBala2;


// OTHER

constexpr MotorPairController::control_mode_e controlModeOther = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;
const MotorPairController::vehicle_t vehicleOther = {
    .maxMotorRPM                 = 100.0F,
    .wheelDiameterMM             = 68.0F,
    .wheelTrackMM                = 170.0F,
    .pitchBalanceAngleDegrees    = 0.0F,
    .motorSwitchOffAngleDegrees  = 70.0F,
    .encoderStepsPerRevolution   = 1000.0F
};

const MotorPairController::pidf_array_t& scaleFactorsOther = scaleFactorsBala2;
const MotorPairController::pidf_array_t& defaultPIDsOther = defaultPIDsBala2;


#if defined(MOTORS_BALA_2)

const MotorPairController::control_mode_e controlMode = controlModeBala2;
const MotorPairController::vehicle_t& vehicle = vehicleBala2;
const MotorPairController::pidf_array_t& scaleFactors = scaleFactorsBala2;
const MotorPairController::pidf_array_t& defaultPIDs = defaultPIDsBala2;

#elif defined(MOTORS_BALA_C)

const MotorPairController::control_mode_e controlMode = controlModeBalaC;
const MotorPairController::vehicle_t& vehicle = vehicleBalaC;
const MotorPairController::pidf_array_t& scaleFactors = scaleFactorsBalaC;
const MotorPairController::pidf_array_t& defaultPIDs = defaultPIDsBala2;

#elif defined(MOTORS_4_ENCODER_MOTOR)

const MotorPairController::control_mode_e controlMode = controlMode4EncoderMotor;
const MotorPairController::vehicle_t& vehicle = vehicle4EncoderMotor;
const MotorPairController::pidf_array_t& scaleFactors = scaleFactors4EncoderMotor;
const MotorPairController::pidf_array_t& defaultPIDs = defaultPIDs4EncoderMotor;

#elif defined(MOTORS_GO_PLUS_2) || defined(MOTORS_ATOMIC_MOTION_BASE)

const MotorPairController::control_mode_e controlMode = controlModeGoPlus2;
const MotorPairController::vehicle_t& vehicle = vehicleGoPlus2;
const MotorPairController::pidf_array_t& scaleFactors = scaleFactorsGoPlus2;
const MotorPairController::pidf_array_t& defaultPIDs = defaultPIDsGoPlus2;

#elif defined(MOTORS_ROLLER_CAN) || defined(MOTORS_PWR_CAN) || defined(MOTORS_O_DRIVE_CAN) || defined(MOTORS_O_DRIVE_TWAI) || defined(MOTORS_GPIO)

const MotorPairController::control_mode_e controlMode = controlModeOther;
const MotorPairController::vehicle_t& vehicle = vehicleOther;
const MotorPairController::pidf_array_t& scaleFactors = scaleFactorsOther;
const MotorPairController::pidf_array_t& defaultPIDs = defaultPIDsOther;

#endif

} // end namespace
