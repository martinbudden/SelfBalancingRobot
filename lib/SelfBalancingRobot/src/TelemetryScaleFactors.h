#pragma once

#include "MotorPairController.h"


class TelemetryScaleFactors {
public:
    explicit TelemetryScaleFactors(MotorPairController::ControlMode_t controlMode);
public:
    void setControlMode(MotorPairController::ControlMode_t controlMode);
    inline const PIDF::PIDF_t& getPIDTelemetryScaleFactor(MotorPairController::pid_index_t pidIndex) const { return _scaleFactors[pidIndex]; }
private:
    // Scale factors for telemetry and PID tuning. Not used by any of the update functions.
    std::array<PIDF::PIDF_t, MotorPairController::PID_COUNT> _scaleFactors;
};
