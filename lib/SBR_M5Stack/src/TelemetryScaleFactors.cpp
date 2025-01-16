#include "TelemetryScaleFactors.h"
#include "MotorPairControllerDefaults.h"


void TelemetryScaleFactors::setControlMode(MotorPairController::ControlMode_t controlMode)
{
    if (controlMode == MotorPairController::CONTROL_MODE_SERIAL_PIDS) {
        _speedPIDTelemetryScaleFactors = speedPID_TelemetryScaleFactorsSerial;
    } else if (controlMode == MotorPairController::CONTROL_MODE_PARALLEL_PIDS) {
        _speedPIDTelemetryScaleFactors = speedPID_TelemetryScaleFactorsParallel;
    }
}

/*!
Constructor. Sets member data.
*/
TelemetryScaleFactors::TelemetryScaleFactors(MotorPairController::ControlMode_t controlMode) :
    _pitchPIDTelemetryScaleFactors(pitchPID_TelemetryScaleFactors),
    _positionPIDTelemetryScaleFactors(positionPID_TelemetryScaleFactors),
    _yawRatePIDTelemetryScaleFactors(yawRatePID_TelemetryScaleFactors)
{
    setControlMode(controlMode);
}