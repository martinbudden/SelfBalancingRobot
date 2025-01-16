#include "TelemetryScaleFactors.h"
#include "MotorPairControllerDefaults.h"


void TelemetryScaleFactors::setControlMode(MotorPairController::ControlMode_t controlMode)
{
    if (controlMode == MotorPairController::CONTROL_MODE_SERIAL_PIDS) {
        _speedPIDTelemetryScaleFactors = speedPID_TelemetryScaleFactorsSerial;
    } else if (controlMode == MotorPairController::CONTROL_MODE_PARALLEL_PIDS) {
        _speedPIDTelemetryScaleFactors = speedPID_TelemetryScaleFactorsParallel;
    } else {
        _speedPIDTelemetryScaleFactors = speedPID_TelemetryScaleFactorsPosition;
    }
}

/*!
Constructor. Sets member data.
*/
TelemetryScaleFactors::TelemetryScaleFactors() :
    // scale factors to bring PIDs into approximately the range [0, 100] for telemetry display and PID tuning
    _pitchPIDTelemetryScaleFactors(pitchPID_TelemetryScaleFactors),
    _speedPIDTelemetryScaleFactors(speedPID_TelemetryScaleFactors),
    _yawRatePIDTelemetryScaleFactors(yawRatePID_TelemetryScaleFactors)
{
}