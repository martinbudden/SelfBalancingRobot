#pragma once

#include <MotorPairController.h>
#include <PIDF.h>

// use anonymous namespace to make items local to this translation unit
namespace {// NOLINT(clang-diagnostic-implicit-int) false positive

#if defined(MOTORS_BALA_2)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0300F,  0.0F,   0.00020F,0.0F };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001F,  0.001F, 0.00001F,0.1F };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.00F,    0.00F,  0.0F,    1.00F };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.01F,    0.01F,  0.01F,   0.01F };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 1.00F,    0.00F,  0.0F,    0.0F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.01F,    0.01F,  0.00001F,0.1F };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.030F,   1.00F,  0.0F,    0.00F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.001F,   0.01F,  0.0001F, 0.01F };

constexpr PIDF::PIDF_t positionPID_Default                      { 1.00F,    0.00F,  0.0F,    0.0F };
constexpr PIDF::PIDF_t positionPID_TelemetryScaleFactors        { 0.10F,    0.01F,  0.001F,  0.1F };

constexpr MotorPairController::ControlMode_t controlModeDefault = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

constexpr float maxMotorRPM                 {200.0}; // this is an estimate of max RPM under load
constexpr float wheelDiameterMM             {45.0};
constexpr float wheelTrackMM                {75.0};
constexpr float pitchBalanceAngleDegrees    {-2.5};
constexpr float motorSwitchOffAngleDegrees  {70.0};


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

constexpr MotorPairController::ControlMode_t controlModeDefault = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

constexpr float maxMotorRPM                 {170.0};
constexpr float wheelDiameterMM             {68.0};
constexpr float wheelTrackMM                {180.0};
constexpr float pitchBalanceAngleDegrees    {-0.5};
constexpr float motorSwitchOffAngleDegrees  {65.0};
constexpr float encoderStepsPerRevolution   {750.0};


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

constexpr MotorPairController::ControlMode_t controlModeDefault = MotorPairController::CONTROL_MODE_SERIAL_PIDS;

constexpr float maxMotorRPM                 {100.0};
constexpr float wheelDiameterMM             {68.0};
constexpr float wheelTrackMM                {170.0};
constexpr float pitchBalanceAngleDegrees    {0.0};
constexpr float motorSwitchOffAngleDegrees  {70.0};
constexpr float encoderStepsPerRevolution   {1000.0};


#elif defined(MOTORS_BALA_C)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0300F, 0.0F,   0.00000F,0.0F };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001F, 1.0F,   0.00001F,0.1F };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.0F,    0.0F,   0.0F,    1.00F };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.1F,    1.0F,   0.01F,   0.01F };

// Bala C has no encodersF,so set speedPID to zero
constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 0.00F,   0.0F,   0.0F,    0.0F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.01F,   0.001F, 0.00001F,0.1F };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.050F,   0.0F,   0.0F,    0.0F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.001F,   1.0F,   0.001F,  0.1F };

constexpr PIDF::PIDF_t positionPID_Default                      { 0.050F,   0.0F,   0.0F,    0.0F };
constexpr PIDF::PIDF_t positionPID_TelemetryScaleFactors        { 0.001F,   1.0F,   0.001F,  0.1F };

constexpr MotorPairController::ControlMode_t controlModeDefault = MotorPairController::CONTROL_MODE_SERIAL_PIDS;

constexpr float maxMotorRPM                 {80.0};
constexpr float wheelDiameterMM             {66.5};
constexpr float wheelTrackMM                {70.0};
constexpr float pitchBalanceAngleDegrees    {-13.0};
constexpr float motorSwitchOffAngleDegrees  {70.0};


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

constexpr MotorPairController::ControlMode_t controlModeDefault = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

constexpr float maxMotorRPM                 {100.0};
constexpr float wheelDiameterMM             {56.0};
constexpr float wheelTrackMM                {155.0};
constexpr float pitchBalanceAngleDegrees    {0.0};
constexpr float motorSwitchOffAngleDegrees  {70.0};


#endif

} // end namespace
