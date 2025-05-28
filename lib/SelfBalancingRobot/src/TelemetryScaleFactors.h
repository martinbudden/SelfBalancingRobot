#pragma once

#include "MotorPairController.h"


class TelemetryScaleFactors {
public:
    explicit TelemetryScaleFactors(MotorPairController::control_mode_e controlMode);
public:
    void setControlMode(MotorPairController::control_mode_e controlMode);
    inline const PIDF::PIDF_t& getTelemetryScaleFactor(MotorPairController::pid_index_e pidIndex) const { return _scaleFactors[pidIndex]; }
private:
    // Scale factors for telemetry and PID tuning. Not used by any of the update functions.
    std::array<PIDF::PIDF_t, MotorPairController::PID_COUNT> _scaleFactors {};
};
