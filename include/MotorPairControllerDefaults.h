#pragma once

#include <PIDF.h>

// use anonymous namespace to make items local to this translation unit
namespace {// NOLINT(clang-diagnostic-implicit-int) false positive

#if defined(MOTORS_BALA_2)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0300,   0.0,    0.00020, 0.0 };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001,   0.001,  0.00001, 0.1 };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.0,      0.0,    0.0,     1.00 };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.1,      1.0,    0.01,    0.01 };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 2.0,      0.000,  0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.1,      0.001,  0.00001, 0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.05,     0.000,  0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.01,     0.001,  0.0001,  0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultPosition                 { 0.050,    0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsPosition   { 0.001,    1.0,    0.001,   0.1 };

constexpr MotorPairController::ControlMode_t CONTROL_MODE = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;
const PIDF::PIDF_t& speedPID_Default = speedPID_DefaultParallel;
const PIDF::PIDF_t& speedPID_TelemetryScaleFactors = speedPID_TelemetryScaleFactorsParallel;

constexpr float maxMotorRPM                 {500.0};
constexpr float wheelDiameterMM             {45.0};
constexpr float wheelTrackMM                {75.0};
constexpr float pitchBalanceAngleDegrees    {-2.5};
constexpr float motorSwitchOffAngleDegrees  {70.0};


#elif defined(MOTORS_4_ENCODER_MOTOR)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0460,  0.000,  0.00130, 0.0 };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001,  0.001,  0.00001, 0.1 };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.0,     0.0,    0.0,     0.50 };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.1,     1.0,    0.01,    0.01 };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 0.0001,  0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.0001,  0.001,  0.00001, 0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.650,   0.000,  0.0000,  0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.001,   0.001,  0.0001,  0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultPosition                 { 0.650,   0.00,   0.0000,  0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsPosition   { 0.001,   0.01,   0.0001,  0.1 };

constexpr MotorPairController::ControlMode_t CONTROL_MODE = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;
const PIDF::PIDF_t& speedPID_Default = speedPID_DefaultParallel;
const PIDF::PIDF_t& speedPID_TelemetryScaleFactors = speedPID_TelemetryScaleFactorsParallel;

constexpr float maxMotorRPM                 {170.0};
constexpr float wheelDiameterMM             {68.0};
constexpr float wheelTrackMM                {180.0};
constexpr float pitchBalanceAngleDegrees    {-0.5};
constexpr float motorSwitchOffAngleDegrees  {65.0};
constexpr float encoderStepsPerRevolution   {750.0};


#elif defined(MOTORS_ROLLER_CAN) || defined(MOTORS_PWR_CAN) || defined(MOTORS_O_DRIVE_CAN) || defined(MOTORS_O_DRIVE_TWAI)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0400,  0.0,    0.00000, 0.0 };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001,  1.0,    0.00001, 0.1 };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.0,     0.0,    0.0,     1.00 };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.1,     1.0,    0.01,    0.01 };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 0.0001,  0.0,    0.0,     0.0};
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.0001,  1.0,    0.00001, 0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.001,   0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.001,   1.0,    0.0001,  0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultPosition                 { 0.001,   0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsPosition   { 0.001,   1.0,    0.0001,  0.1 };

constexpr MotorPairController::ControlMode_t CONTROL_MODE = MotorPairController::CONTROL_MODE_SERIAL_PIDS;
const PIDF::PIDF_t& speedPID_Default = speedPID_DefaultSerial;
const PIDF::PIDF_t& speedPID_TelemetryScaleFactors = speedPID_TelemetryScaleFactorsSerial;

constexpr float maxMotorRPM                 {100.0};
constexpr float wheelDiameterMM             {68.0};
constexpr float wheelTrackMM                {170.0};
constexpr float pitchBalanceAngleDegrees    {0.0};
constexpr float motorSwitchOffAngleDegrees  {70.0};
constexpr float encoderStepsPerRevolution   {1000.0};


#elif defined(MOTORS_BALA_C)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0300,  0.0,    0.00000, 0.0 };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001,  1.0,    0.00001, 0.1 };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.0,     0.0,    0.0,     1.00 };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.1,     1.0,    0.01,    0.01 };

// Bala C has no encoders, so set speedPID to zero
constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 0.00,    0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.01,    0.001,  0.00001, 0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.050,    0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.001,    1.0,    0.001,   0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultPosition                 { 0.050,    0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsPosition   { 0.001,    1.0,    0.001,   0.1 };

constexpr MotorPairController::ControlMode_t CONTROL_MODE = MotorPairController::CONTROL_MODE_SERIAL_PIDS;
const PIDF::PIDF_t& speedPID_Default = speedPID_DefaultPosition;
const PIDF::PIDF_t& speedPID_TelemetryScaleFactors = speedPID_TelemetryScaleFactorsPosition;

constexpr float maxMotorRPM                 {80.0};
constexpr float wheelDiameterMM             {66.5};
constexpr float wheelTrackMM                {70.0};
constexpr float pitchBalanceAngleDegrees    {-13.0};
constexpr float motorSwitchOffAngleDegrees  {70.0};


#elif defined(MOTORS_GO_PLUS_2) || defined(MOTORS_ATOMIC_MOTION_BASE)


constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0000,  0.0,    0.00000, 0.0 };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001,  1.0,    0.00001, 0.1 };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.0,     0.0,    0.0,     1.00 };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.1,     1.0,    0.01,    0.01 };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 0.0000,  0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.0001,  0.001,  0.00001, 0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.0000,  0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.0001,  0.001,  0.00001, 0.1 };

constexpr PIDF::PIDF_t speedPID_DefaultPosition                 { 0.0000,  0.0,    0.0,     0.0 };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsPosition   { 0.0001,  0.001,  0.00001, 0.1 };

constexpr MotorPairController::ControlMode_t CONTROL_MODE = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;
const PIDF::PIDF_t& speedPID_Default = speedPID_DefaultParallel;
const PIDF::PIDF_t& speedPID_TelemetryScaleFactors = speedPID_TelemetryScaleFactorsParallel;

constexpr float maxMotorRPM                 {100.0};
constexpr float wheelDiameterMM             {56.0};
constexpr float wheelTrackMM                {155.0};
constexpr float pitchBalanceAngleDegrees    {0.0};
constexpr float motorSwitchOffAngleDegrees  {70.0};


#endif

} // end namespace
