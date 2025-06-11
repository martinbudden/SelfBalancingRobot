#pragma once

#include <MotorPairController.h>
#include <PIDF.h>

// use anonymous namespace to make items local to this translation unit
namespace {// NOLINT(clang-diagnostic-implicit-int) false positive

typedef std::array<PIDF::PIDF_t, MotorPairController::PID_COUNT> scale_factors_t;


constexpr MotorPairController::control_mode_e controlModeBala2 = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

const MotorPairController::vehicle_t vehicleBala2 = {
    .maxMotorRPM                = 200.0F, // this is an estimate of max RPM under load
    .wheelDiameterMM            = 45.0F,
    .wheelTrackMM               = 75.0F,
    .pitchBalanceAngleDegrees   = -2.5F,
    .motorSwitchOffAngleDegrees = 70.0F,
    .encoderStepsPerRevolution  = 420.0F
};

constexpr std::array<PIDF::PIDF_t, MotorPairController::PID_COUNT> scaleFactorsBala2 {{
    { 0.0001F,  0.001F, 0.00001F,   0.1F },     // ROLL_ANGLE_DEGREES=0,
    { 0.0001F,  0.001F, 0.00001F,   0.1F },     // PITCH_ANGLE_DEGREES
    { 0.01F,    0.01F,  0.01F,      0.01F },    // YAW_RATE_DPS
    { 0.01F,    0.01F,  0.00001F,   0.1F },     // SPEED_SERIAL_DPS
    { 0.001F,   0.01F,  0.0001F,    0.01F },    // SPEED_PARALLEL_DPS
    { 0.10F,    0.01F,  0.001F,     0.1F }      // POSITION_DEGREES
}};

constexpr std::array<PIDF::PIDF_t, MotorPairController::PID_COUNT> defaultPIDsBala2 {{
    { 0.0F,     0.0F,   0.0F,       0.1F },     // ROLL_ANGLE_DEGREES=0,
    { 0.0300F,  0.0F,   0.00020F,   0.1F },     // PITCH_ANGLE_DEGREES
    { 0.0F,     0.0F,   0.0F,       1.0F },     // YAW_RATE_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F },     // SPEED_SERIAL_DPS
    { 0.030F,   0.0F,   0.0F,       0.0F },     // SPEED_PARALLEL_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F }      // POSITION_DEGREES
}};


constexpr MotorPairController::control_mode_e controlModeBalaC = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

const MotorPairController::vehicle_t vehicleBalaC = {
    .maxMotorRPM                = 80.0F,
    .wheelDiameterMM            = 66.5F,
    .wheelTrackMM               = 70.0F,
    .pitchBalanceAngleDegrees   = -13.0F,
    .motorSwitchOffAngleDegrees = 70.0F,
    .encoderStepsPerRevolution  = 0.0F
};

constexpr std::array<PIDF::PIDF_t, MotorPairController::PID_COUNT> scaleFactorsBalaC {{
    { 0.0001F,  0.001F, 0.00001F,0.1F },    // ROLL_ANGLE_DEGREES=0,
    { 0.0001F,  1.0F,  0.00001F,0.1F },     // PITCH_ANGLE_DEGREES
    { 0.1F,     1.0F,   0.01F,   0.01F },   // YAW_RATE_DPS
    { 0.01F,    0.01F,  0.0001F, 0.1F },    // SPEED_SERIAL_DPS
    { 0.01F,    0.01F,  0.0001F, 0.01F },   // SPEED_PARALLEL_DPS
    { 0.10F,    0.01F,  0.001F,  0.1F }     // POSITION_DEGREES
}};

constexpr std::array<PIDF::PIDF_t, MotorPairController::PID_COUNT> defaultPIDsBalaC {{
    { 0.0F,     0.0F,   0.0F,       0.1F },     // ROLL_ANGLE_DEGREES=0,
    { 0.0300F,  0.0F,   0.0F,       0.0F },     // PITCH_ANGLE_DEGREES
    { 0.0F,     0.0F,   0.0F,       1.0F },     // YAW_RATE_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F },     // SPEED_SERIAL_DPS
    { 0.050F,   0.0F,   0.0F,       0.0F },     // SPEED_PARALLEL_DPS
    { 0.0F,     0.0F,   0.0F,       0.0F }      // POSITION_DEGREES
}};


const MotorPairController::vehicle_t vehicleBala4EncoderMotor = {
    .maxMotorRPM                = 170.0F,
    .wheelDiameterMM            = 68.0F,
    .wheelTrackMM               = 180.0F,
    .pitchBalanceAngleDegrees   = -0.5F,
    .motorSwitchOffAngleDegrees = 65.0F,
    .encoderStepsPerRevolution  = 750.0F
};


#if defined(MOTORS_BALA_2)

const MotorPairController::control_mode_e controlMode = controlModeBala2;
const MotorPairController::vehicle_t& vehicle = vehicleBala2;
const scale_factors_t& scaleFactors = scaleFactorsBala2;
const scale_factors_t& defaultPIDs = defaultPIDsBala2;

#elif defined(MOTORS_BALA_C)

const MotorPairController::control_mode_e controlMode = controlModeBala2;
const MotorPairController::vehicle_t& vehicle = vehicleBalaC;
const scale_factors_t& scaleFactors = scaleFactorsBalaC;
const scale_factors_t& defaultPIDs = defaultPIDsBala2;

#elif defined(MOTORS_4_ENCODER_MOTOR)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0460F, 0.000F, 0.00130F,0.0F };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001F, 0.001F, 0.00001F,0.1F };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.0F,    0.0F,   0.0F,    0.50F };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.1F,    1.0F,   0.01F,   0.01F };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 0.0001F, 0.0F,   0.0F,    0.0F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.0001F, 0.001F, 0.00001F,0.1F };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.650F,  0.000F, 0.0000F, 0.0F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.001F,  0.001F, 0.0001F, 0.1F };

constexpr PIDF::PIDF_t positionPID_Default                      { 0.650F,  0.00F,  0.0000F, 0.0F };
constexpr PIDF::PIDF_t positionPID_TelemetryScaleFactors        { 0.001F,  0.01F,  0.0001F, 0.1F };

constexpr MotorPairController::control_mode_e controlModeDefault = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;



#elif defined(MOTORS_ROLLER_CAN) || defined(MOTORS_PWR_CAN) || defined(MOTORS_O_DRIVE_CAN) || defined(MOTORS_O_DRIVE_TWAI) || defined(MOTORS_GPIO)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0400F, 0.0F,   0.00000F,0.0F };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001F, 1.0F,   0.00001F,0.1F };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.0F,    0.0F,   0.0F,    1.00F };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.1F,    1.0F,   0.01F,   0.01F };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 0.0001F, 0.0F,   0.0F,    0.0};
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.0001F, 1.0F,   0.00001F,0.1F };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.001F,  0.0F,   0.0F,    0.0F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.001F,  1.0F,   0.0001F, 0.1F };

constexpr PIDF::PIDF_t positionPID_Default                      { 0.001F,  0.0F,   0.0F,    0.0F };
constexpr PIDF::PIDF_t positionPID_TelemetryScaleFactors        { 0.001F,  1.0F,   0.0001F, 0.1F };


const MotorPairController::vehicle_t vehicleNAMEr = {
    .maxMotorRPM                 {100.0};
    .wheelDiameterMM             {68.0};
    .wheelTrackMM                {170.0};
    .pitchBalanceAngleDegrees    {0.0};
    .motorSwitchOffAngleDegrees  {70.0};
    .encoderStepsPerRevolution   {1000.0};
};

#elif defined(MOTORS_GO_PLUS_2) || defined(MOTORS_ATOMIC_MOTION_BASE)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0000F, 0.0F,   0.00000F,0.0F };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001F, 1.0F,   0.00001F,0.1F };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.0F,    0.0F,   0.0F,    1.00F };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.1F,    1.0F,   0.01F,   0.01F };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 0.0000F, 0.0F,   0.0F,    0.0F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.0001F, 0.001F, 0.00001F,0.1F };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.0000F, 0.0F,   0.0F,    0.0F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.0001F, 0.001F, 0.00001F,0.1F };

constexpr PIDF::PIDF_t positionPID_Default                      { 0.0000F, 0.0F,   0.0F,    0.0F };
constexpr PIDF::PIDF_t positionPID_TelemetryScaleFactors        { 0.0001F, 0.001F, 0.00001F,0.1F };

constexpr MotorPairController::control_mode_e controlModeDefault = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

const MotorPairController::vehicle_t vehicleNAMEr = {
    .maxMotorRPM                 {100.0};
    .wheelDiameterMM             {56.0};
    .wheelTrackMM                {155.0};
    .pitchBalanceAngleDegrees    {0.0};
    .motorSwitchOffAngleDegrees  {70.0};
}


#endif

} // end namespace
