#include "TelemetryScaleFactors.h"
#include "MotorPairControllerDefaults.h"


void TelemetryScaleFactors::setControlMode(MotorPairController::control_mode_t controlMode)
{
    _scaleFactors[MotorPairController::SPEED_DPS] = (controlMode == MotorPairController::CONTROL_MODE_SERIAL_PIDS)
        ? speedPID_TelemetryScaleFactorsSerial : speedPID_TelemetryScaleFactorsParallel;
}

/*!
Constructor. Sets member data.
*/
TelemetryScaleFactors::TelemetryScaleFactors(MotorPairController::control_mode_t controlMode)
{
    setControlMode(controlMode);
    _scaleFactors[MotorPairController::PITCH_ANGLE_DEGREES] = pitchPID_TelemetryScaleFactors;
    _scaleFactors[MotorPairController::YAW_RATE_DPS] = yawRatePID_TelemetryScaleFactors;
    _scaleFactors[MotorPairController::POSITION_DEGREES] = positionPID_TelemetryScaleFactors;
}
