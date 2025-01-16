#pragma once

#include "MotorPairController.h"


class TelemetryScaleFactors {
public:
    TelemetryScaleFactors();
public:
    void setControlMode(MotorPairController::ControlMode_t controlMode);
    inline const PIDF::PIDF_t& getPitchPIDTelemetryScaleFactors() const { return _pitchPIDTelemetryScaleFactors; }
    inline const PIDF::PIDF_t& getSpeedPIDTelemetryScaleFactors() const { return _speedPIDTelemetryScaleFactors; }
    inline const PIDF::PIDF_t& getYawRatePIDTelemetryScaleFactors() const { return _yawRatePIDTelemetryScaleFactors; }
private:
    // Scale factors for telemetry and PID tuning. Not used by any of the update functions.
    PIDF::PIDF_t _pitchPIDTelemetryScaleFactors;
    PIDF::PIDF_t _speedPIDTelemetryScaleFactors;
    PIDF::PIDF_t _yawRatePIDTelemetryScaleFactors;
};
