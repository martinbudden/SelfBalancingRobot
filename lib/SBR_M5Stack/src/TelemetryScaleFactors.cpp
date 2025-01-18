#include "TelemetryScaleFactors.h"
#include "MotorPairControllerDefaults.h"


void TelemetryScaleFactors::setControlMode(MotorPairController::ControlMode_t controlMode)
{
    _scaleFactors[MotorPairController::SPEED] = (controlMode == MotorPairController::CONTROL_MODE_SERIAL_PIDS)
        ? speedPID_TelemetryScaleFactorsSerial : speedPID_TelemetryScaleFactorsParallel;
}

/*!
Constructor. Sets member data.
*/
TelemetryScaleFactors::TelemetryScaleFactors(MotorPairController::ControlMode_t controlMode)
{
    setControlMode(controlMode);
    _scaleFactors[MotorPairController::PITCH_ANGLE] = pitchPID_TelemetryScaleFactors;
    _scaleFactors[MotorPairController::YAW_RATE] = yawRatePID_TelemetryScaleFactors;
    _scaleFactors[MotorPairController::POSITION] = positionPID_TelemetryScaleFactors;
}
