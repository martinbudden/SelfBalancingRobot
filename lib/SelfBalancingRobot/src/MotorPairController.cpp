#include "MotorPairController.h"

#include "MotorPairBase.h"
#include "MotorPairControllerTelemetry.h"
#include <AHRS.h>
#include <Filters.h>

#include <ReceiverBase.h>

#include <cmath>
#if defined(USE_FREERTOS)
#include <esp32-hal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
inline void YIELD_TASK() { taskYIELD(); }
#else
inline void YIELD_TASK() {}
#endif


const std::array<std::string, MotorPairController::PID_COUNT> PID_NAMES = {
    "PITCH_ANGLE",
    "SPEED",
    "POSITION",
    "YAW_RATE"
};

std::string MotorPairController::getPIDName(pid_index_t pidIndex) const
{
    return PID_NAMES[pidIndex];
}

std::string MotorPairController::getBalanceAngleName() const
{
    return "BALANCE_ANGLE";
}

void MotorPairController::motorsResetEncodersToZero()
{
    _motors.resetEncodersToZero();
}

/*!
Return a reference to the MPC telemetry data.

The telemetry object is shared between this task (the MPC task) and the MAIN_LOOP_TASK, however it is deliberately not protected by a mutex.
This is because:
1. Only this task writes to the telemetry object. The main task only reads it (for display and to send to the backchannel).
2. The only inconsistency that can occur is that the main task might use a partially updated telemetry object, however this is not a problem because
   all the member data are continuous, and so a partially updated object is still meaningful to display.
3. The overhead of a mutex is thus avoided.
*/
void MotorPairController::getTelemetryData(motor_pair_controller_telemetry_t& telemetry) const
{
   if (motorsIsOn()) {
        telemetry.pitchError = _PIDS[PITCH_ANGLE_DEGREES].getError();
        telemetry.speedError = _PIDS[SPEED_DPS].getError();
        telemetry.positionError = _PIDS[POSITION_DEGREES].getError();
   } else {
        telemetry.pitchError = { 0.0F, 0.0F, 0.0F };
        telemetry.speedError = { 0.0F, 0.0F, 0.0F };
        telemetry.positionError = { 0.0F, 0.0F, 0.0F };
   }
    telemetry.pitchAngleOutput = _outputs[PITCH_ANGLE_DEGREES];
    telemetry.speedOutput = _outputs[SPEED_DPS];
    telemetry.positionOutput = _outputs[POSITION_DEGREES];
    telemetry.yawRateOutput = _outputs[YAW_RATE_DPS];

    telemetry.powerLeft = _mixer.getPowerLeft();
    telemetry.powerRight = _mixer.getPowerRight();

    telemetry.encoderLeft = _encoderLeft;
    telemetry.encoderRight = _encoderRight;
    telemetry.encoderLeftDelta = static_cast<int16_t>(_encoderLeftDelta);
    telemetry.encoderRightDelta = static_cast<int16_t>(_encoderRightDelta);

    telemetry.speedLeftDPS = _speedLeftDPS;
    telemetry.speedRightDPS = _speedRightDPS;
    telemetry.speedDPS_Filtered = _speedDPS;
    // copy of motorMaxSpeedDPS, so telemetry viewer can scale motor speed
    telemetry.motorMaxSpeedDPS = _motorMaxSpeedDPS;
}

void MotorPairController::updateSetpointsAndMotorSpeedEstimates(float deltaT)
{
    // If new joystick values are available from the receiver, then map them to the range [-1.0, 1.0] and use them to update the setpoints.
    if (_newStickValuesAvailable) {
        _newStickValuesAvailable = false;
        _receiver.mapControls(_throttleStick, _rollStick, _pitchStick, _yawStick);

        _PIDS[SPEED_DPS].setSetpoint(_throttleStick);
        // Note the negative multiplier
        _PIDS[PITCH_ANGLE_DEGREES].setSetpoint(-_pitchStick * _pitchMaxAngleDegrees);
        // Note the negative multiplier, since pushing the yaw stick to the right results in a clockwise rotation, ie a negative yaw rate
        _PIDS[YAW_RATE_DPS].setSetpoint(-_yawStick * _yawStickMultiplier); // limit yaw rate to sensible range.
    }
#if defined(MOTORS_HAVE_ENCODERS)
    _motors.readEncoder();
    _encoderLeft = _motors.getLeftEncoder();
    _encoderRight = _motors.getRightEncoder();

    _encoderLeftDelta = _encoderLeft - _encoderLeftPrevious;
    _encoderLeftPrevious = _encoderLeft;
    _encoderRightDelta = _encoderRight - _encoderRightPrevious;
    _encoderRightPrevious = _encoderRight;

    if (_motors.canAccuratelyEstimateSpeed()) {
        _speedLeftDPS = _motors.getLeftSpeed();
        _speedRightDPS = _motors.getRightSpeed();
        _speedDPS = (_speedLeftDPS + _speedRightDPS) * 0.5F;
    } else {
        const float speedMultiplier = 360.0F / (_motorStepsPerRevolution * deltaT);
        _speedLeftDPS = static_cast<float>(_encoderLeftDelta) * speedMultiplier;
        _speedRightDPS = static_cast<float>(_encoderRightDelta) * speedMultiplier;

        // encoders have discrete output and so are subject to digital noise
        // At a speed of 60rpm or 1rps with a 420 steps per revolution we have 420 steps per second,
        // The MPC task runs at 100Hz or 200Hz, depending on build settings, so that is one or two steps per iteration
        // So an error of one step is at least a 50% error. So we average over 8 iterations (40ms, or 80ms) to reduce this digital noise.
        static FilterMovingAverage<8> speedMovingAverageFilter;
        float speedDPS = (_speedLeftDPS + _speedRightDPS) * 0.5F;
        speedDPS = speedMovingAverageFilter.update(speedDPS);
        // additionally apply IIR filter.
        _speedDPS = _speedFilter.update(speedDPS);
       
        //static float motorSpeed {0.0};
        //static constexpr float motorSpeedWeighting {0.8};
        //motorSpeed = motorSpeedWeighting * _motorSpeed + (1.0F - motorSpeedWeighting) * (_encoderLeftDelta + _encoderRightDelta);
        //motorSpeed = motorSpeedWeighting * motorSpeed + (1.0F - motorSpeedWeighting) * speedDPS;
        //_speedDPS = motorSpeed;
    }
#else
    // no encoders, so estimate speed from power output
    (void)deltaT; // so LINT doesn't report and unused parameter.
    _speedDPS = MotorPairBase::clip((_mixer.getPowerLeft() + _mixer.getPowerRight()) * 0.5F, -1.0F, 1.0F) * _motorMaxSpeedDPS;
#endif
}


void MotorPairController::updateOutputsUsingPIDs(float deltaT)
{
    [[maybe_unused]] bool orientationUpdated;
    const Quaternion orientation = _ahrs.getOrientationUsingLock(orientationUpdated);
    [[maybe_unused]] bool ahrsDataUpdated;
    const AHRS::data_t data = _ahrs.getAhrsDataUsingLock(ahrsDataUpdated);
    updateOutputsUsingPIDs(data.gyroRPS, data.acc, orientation, deltaT);
}

void MotorPairController::updateOutputsUsingPIDs(const xyz_t& gyroRPS, [[maybe_unused]] const xyz_t& acc, const Quaternion& orientation, float deltaT)
{
    // NOTE COORDINATE TRANSFORM: Madgwick filter uses Euler angles where roll is defined as rotation around the x-axis and pitch is rotation around the y-axis.
    // For the Self Balancing Robot, pitch is rotation around the x-axis and roll is rotation around the y-axis,
    // so ROLL and PITCH are REVERSED.
    _pitchAngleDegreesRaw = orientation.calculateRollDegrees();
    _mixer.setPitchAngleDegreesRaw(_pitchAngleDegreesRaw);
//#define CALCULATE_ROLL_AND_YAW
#if defined(CALCULATE_ROLL_AND_YAW)
    // Roll and yaw are not required for the MotorPairController calculations,
    // but if they are calculated they will be displayed by the screen and telemetry, which can be useful in debugging.
    _rollAngleDegreesRaw = orientation.calculatePitchDegrees();
    _yawAngleDegreesRaw = orientation.calculateYawDegrees();
#endif

    if (!motorsIsOn() || motorsIsDisabled()) { // [[unlikely]]
        _outputs[PITCH_ANGLE_DEGREES] = 0.0F;
        _PIDS[PITCH_ANGLE_DEGREES].resetIntegral();
        _outputs[SPEED_DPS] = 0.0F;
        _PIDS[SPEED_DPS].resetIntegral();
        _outputs[YAW_RATE_DPS] = 0.0F;
        _PIDS[YAW_RATE_DPS].resetIntegral();
#if !defined(AHRS_RECORD_UPDATE_TIMES)
        YIELD_TASK();
        return;
#endif
    }

    // calculate _outputs[SPEED_DPS] according to the control mode.
    _outputs[SPEED_DPS] = _PIDS[SPEED_DPS].update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT); // _speedDPS * _motorMaxSpeedDPS_reciprocal is in range [-1.0, 1.0]
    if (_controlMode == CONTROL_MODE_SERIAL_PIDS) {
        // feed the speed update back into the pitchAngle PID and set _outputs[SPEED_DPS] to zero
        _PIDS[PITCH_ANGLE_DEGREES].setSetpoint(-_outputs[SPEED_DPS]);
        _outputs[SPEED_DPS] = 0.0F;
    } else if (_controlMode == CONTROL_MODE_POSITION) {
        // NOTE: THIS IS EXPERIMENTAL AND NOT YET FULLY IMPLEMENTED
        updatePositionOutputs(deltaT);
    }

    // calculate _outputs[PITCH_ANGLE_DEGREES]
    const float pitchAngleDegrees = _pitchAngleDegreesRaw - _pitchBalanceAngleDegrees;
    float pitchAngleDegreesDelta = pitchAngleDegrees - _pitchAngleDegreesPrevious;
    _pitchAngleDegreesPrevious = pitchAngleDegrees;
    // Calculate the filtered value to use as input into the PID, so the D-term is calculated using the filtered value.
    // This is beneficial because the D-term is especially susceptible to noise.
    static FilterMovingAverage<4> pitchAngleDeltaFilter;
    pitchAngleDegreesDelta = pitchAngleDeltaFilter.update(pitchAngleDegreesDelta);
    _outputs[PITCH_ANGLE_DEGREES] = _PIDS[PITCH_ANGLE_DEGREES].updateDelta(pitchAngleDegrees, pitchAngleDegreesDelta, deltaT);

    // calculate _outputs[YAW_RATE_DPS]
    const float yawRate = -gyroRPS.z * Quaternion::radiansToDegrees;
    _outputs[YAW_RATE_DPS] = _PIDS[YAW_RATE_DPS].update(yawRate, deltaT);
}

void MotorPairController::updatePositionOutputs(float deltaT)
{
    // NOTE: THIS IS EXPERIMENTAL AND NOT YET FULLY IMPLEMENTED
#if defined(MOTORS_HAVE_ENCODERS)
    #if true
    _positionDegrees = static_cast<float>(_encoderLeft + _encoderRight) * 360.0F / (2.0F * _motorStepsPerRevolution);
    #else
    // experimental calculation of position using complementary filter of position and
    const float positionDegrees = static_cast<float>(_encoderLeft + _encoderRight) * 360.0F / (2.0F * _motorStepsPerRevolution);
    const float distanceDegrees = (positionDegrees - _positionDegrees);
    constexpr float alpha = 0.9;
    const float  speedEstimate = MotorPairBase::clip((_powerLeft + _powerRight) * 0.5F, -1.0F, 1.0F) * _motorMaxSpeedDPS;
    _positionDegrees += alpha*distanceDegrees + (1.0F - alpha)*speedEstimate*deltaT;
    #endif
#else
    _positionDegrees += _speedDPS * deltaT;
#endif
    // scale the throttleStick value to get desired speed
    const float desiredSpeedDPS = _throttleStick * _motorMaxSpeedDPS;
    _positionSetpointDegrees += desiredSpeedDPS * deltaT;
    _PIDS[POSITION_DEGREES].setSetpoint(_positionSetpointDegrees);
    const float updatePositionDegreesPrevious = _outputs[POSITION_DEGREES];
    _outputs[POSITION_DEGREES] = _PIDS[POSITION_DEGREES].update(_positionDegrees, deltaT);
    _outputs[SPEED_DPS] = (_outputs[POSITION_DEGREES] - updatePositionDegreesPrevious) / deltaT;
}

void MotorPairController::outputToMotors(float deltaT, uint32_t tickCount)
{
    const MotorMixer::output_t outputs = {
        .speed  = _outputs[SPEED_DPS],
        .roll   = 0.0F,
        .pitch  = _outputs[PITCH_ANGLE_DEGREES],
        .yaw    = _outputs[YAW_RATE_DPS]
    };
    _mixer.outputToMotors(outputs, deltaT, tickCount);
}

/*!
Task loop for the MotorPairController. Uses PID controllers to update the motor pair.

Setpoints are provided by the receiver(joystick), and inputs(process variables) come from the AHRS and the motor encoders.

There are three PIDs, a pitch PID, a speed PID, and a yawRate PID.
*/
void MotorPairController::loop(float deltaT, uint32_t tickCount)
{
    updateSetpointsAndMotorSpeedEstimates(deltaT);
    // If the AHRS is configured to run updateOutputsUsingPIDs, then we don't need to
    if (!_ahrs.configuredToUpdateOutputs()) {
        updateOutputsUsingPIDs(deltaT);
    }
    outputToMotors(deltaT, tickCount);
}

/*!
Task function for the MotorPairController. Sets up and runs the task loop() function.
*/
void MotorPairController::Task(const TaskParameters* taskParameters)
{
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    _tickIntervalTicks = pdMS_TO_TICKS(taskParameters->tickIntervalMilliSeconds);
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next tickIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, _tickIntervalTicks);

        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than _tickIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const uint32_t timeMicroSeconds = micros();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;

        if (_tickCountDelta > 0) { // guard against the case of the while loop executing twice on the same tick interval
            const float deltaT = pdTICKS_TO_MS(_tickCountDelta) * 0.001F;
            loop(deltaT, tickCount);
        }
    }
#endif
}

/*!
Wrapper function for MotorPairController::Task with the correct signature to be used in xTaskCreate.
*/
void MotorPairController::Task(void* arg)
{
    const MotorPairController::TaskParameters* taskParameters = static_cast<MotorPairController::TaskParameters*>(arg);

    MotorPairController* mpc = taskParameters->motorPairController;
    mpc->Task(taskParameters);
}
