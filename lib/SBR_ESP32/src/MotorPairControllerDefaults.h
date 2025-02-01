#pragma once

#include <MotorPairController.h>
#include <PIDF.h>

// use anonymous namespace to make items local to this translation unit
namespace {// NOLINT(clang-diagnostic-implicit-int) false positive

constexpr PIDF::PIDF_t pitchPID_Default                         { 0.0300F,  0.0F,   0.00020F,   0.0F };
constexpr PIDF::PIDF_t pitchPID_TelemetryScaleFactors           { 0.0001F,  0.001F, 0.00001F,   0.1F };

constexpr PIDF::PIDF_t yawRatePID_Default                       { 0.00F,    0.00F,  0.0F,       1.00F };
constexpr PIDF::PIDF_t yawRatePID_TelemetryScaleFactors         { 0.01F,    0.01F,  0.01F,      0.01F };

constexpr PIDF::PIDF_t speedPID_DefaultSerial                   { 1.00F,    0.00F,  0.0F,       0.0F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsSerial     { 0.01F,    0.01F,  0.00001F,   0.1F };

constexpr PIDF::PIDF_t speedPID_DefaultParallel                 { 0.030F,   1.00F,  0.0F,       0.00F };
constexpr PIDF::PIDF_t speedPID_TelemetryScaleFactorsParallel   { 0.001F,   0.01F,  0.0001F,    0.01F };

constexpr PIDF::PIDF_t positionPID_Default                      { 1.00F,    0.00F,  0.0F,       0.0F };
constexpr PIDF::PIDF_t positionPID_TelemetryScaleFactors        { 0.10F,    0.01F,  0.001F,     0.1F };

constexpr MotorPairController::ControlMode_t controlModeDefault = MotorPairController::CONTROL_MODE_PARALLEL_PIDS;

constexpr float maxMotorRPM                 {200.0}; // this is an estimate of max RPM under load
constexpr float wheelDiameterMM             {45.0};
constexpr float wheelTrackMM                {75.0};
constexpr float pitchBalanceAngleDegrees    {-2.5};
constexpr float motorSwitchOffAngleDegrees  {70.0};

} // end namespace
