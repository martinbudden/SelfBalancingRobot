#if defined(USE_MOTOR_PAIR_CONTROLLER)

#include "MotorPairController.h"

#include "AHRS_Base.h"
#include "MotorPairControllerTelemetry.h"
#include "ReceiverBase.h"
#include <Filters.h>

//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif

#include <cmath>
#if defined(USE_FREERTOS)
#include <esp32-hal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

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
        telemetry.pitchError = _pitchPID.getError();
        telemetry.speedError = _speedPID.getError();
   } else {
        telemetry.pitchError = { 0.0F, 0.0F, 0.0F };
        telemetry.speedError = { 0.0F, 0.0F, 0.0F };
   }
    telemetry.powerLeft = _powerLeft;
    telemetry.powerRight = _powerRight;
    telemetry.encoderLeft =_encoderLeft;
    telemetry.encoderRight =_encoderRight;
    telemetry.encoderLeftDelta =_encoderLeftDelta;
    telemetry.encoderRightDelta =_encoderRightDelta;
    telemetry.speedLeftDPS = _speedLeftDPS;
    telemetry.speedRightDPS = _speedRightDPS;
    telemetry.speedDPS_Filtered = _speedDPS;
    // copy of motorMaxSpeedDPS, so telemetry viewer can scale motor speed
    telemetry.motorMaxSpeedDPS = _motorMaxSpeedDPS;

}

void MotorPairController::updateMotors()
{
    static FilterMovingAverage<4> powerLeftFilter; // filter length of 4 means division is not used in calculating average.
    static FilterMovingAverage<4> powerRightFilter;

    if (motorsIsOn() && !_motorsDisabled) {
        _powerLeft  = _pitchUpdate + _speedUpdate - _yawRateUpdate;
        _powerRight = _pitchUpdate + _speedUpdate + _yawRateUpdate;

        // filter the power input into the motors so they run more smoothly.
        const float powerLeftFiltered = powerLeftFilter.update(_powerLeft);
        const float powerRightFiltered = powerRightFilter.update(_powerRight);
        _motors.setPower(powerLeftFiltered, powerRightFiltered);
    } else {
        // Motors switched off, so set everything to zero, ready for motors to be switched on again.
        _motors.setPower(0.0F, 0.0F);
        _powerLeft  = 0.0F;
        _powerRight = 0.0F;
        powerLeftFilter.reset();
        powerRightFilter.reset();
    }
#if defined(SERIAL_OUTPUT)
    static int loopCount {0};
    ++loopCount;
    if (loopCount == 10) {
        loopCount = 0;

    //Serial.printf(">pitchAngleDegrees:%6.2f, pitchRate:%8.4f\r\n", pitchAngleDegrees, pitchRate);

    //Serial.printf(">pitchPidErrorP:%8.2f, pidErrorI:%8.2f, pidErrorD:%8.2f, update:%8.2f\r\n",
    //    _pitchPID.getError().P, _pitchPID.getError().I, _pitchPID.getError().D, _telemetry.pitchUpdate);

    //Serial.printf(">pitchSetpoint:%7.2f, pitchAngleDegrees:%6.2f, pitchUpdate:%8.4f, speedSetpoint:%7.2f, speedUpdate:%7.3f, speedUpdateRaw:%7.3f, speedError:%7.3f, deltaT:%6.3f\r\n",
    //   _pitchPID.getSetpoint(), pitchAngleDegrees, _telemetry.pitchUpdate, _speedPID.getSetpoint(), _telemetry.speedUpdate, speedUpdate, _speedPID.getError().P/_speedPID.getP(), deltaT);

    //Serial.printf(">pitchSetpoint:%7.2f, pitchAngleDegrees:%6.2f, pitchUpdate:%8.4f, speedSetpoint:%7.2f, speedUpdate:%7.3f, deltaT:%6.3f\r\n",
    //   _pitchPID.getSetpoint(), pitchAngleDegrees, _telemetry.pitchUpdate, _speedPID.getSetpoint(), _telemetry.speedUpdate, deltaT);

    //Serial.printf(">speed:%8.2f, setpoint:%8.2f, pidErrorP:%8.2f, pidErrorI:%8.2f, update:%8.2f\r\n",
    //    _speedDPS / _telemetry.motorMaxSpeedDPS, _speedPID.getSetpoint(), _speedPID.getError().P, _speedPID.getError().I, _telemetry.speedUpdate);

    /*if (fabs(_yawRatePID.getSetpoint()) > 0.01f) {
        Serial.printf(">yawRateSetpoint:%7.2f, yawRate:%6.2f, yawRatePower:%6.2f, yawRateUpdate:%f\r\n",
            _yawRatePID.getSetpoint(), yawRate, yawRatePower, _telemetry.yawRateUpdate);
    }*/

    //Serial.printf(">pitchAngleDegrees:%6.2f, pitchUpdate:%6.3f, speedDPS:%5.0F, spdPE:%7.3f, speedUpdate:%8.5f\r\n",
    //    pitchAngleDegrees, _telemetry.pitchUpdate, _telemetry.speedDPS_Filtered, speedPowerEquivalent, _telemetry.speedUpdate);
    }
#endif
}

void MotorPairController::updateSetpointsAndMotorSpeedEstimates(float deltaT, uint32_t tickCount)
{
    // If new joystick values are available from the receiver, then map them to the range [-1.0, 1.0] and use them to update the setpoints.
    if (_newStickValuesAvailable) {
        _newStickValuesAvailable = false;
        _receiver.mapControls(_throttleStick, _rollStick, _pitchStick, _yawStick);

        _speedPID.setSetpoint(_throttleStick);
        _pitchPID.setSetpoint(_pitchStick * _pitchMaxAngleDegrees);
        // Note the negative multiplier, since pushing the yaw stick to the right results in a clockwise rotation, ie a negative yaw rate
        _yawRatePID.setSetpoint(-_yawStick * _yawStickMultiplier); // limit yaw rate to sensible range.
    }
#if defined(MOTORS_HAVE_ENCODERS)
    _motors.readEncoder();
    _encoderLeft = _motors.getLeftEncoder();
    _encoderRight = _motors.getRightEncoder();

    _encoderLeftDelta = static_cast<int16_t>(_encoderLeft - _encoderLeftPrevious);
    _encoderLeftPrevious = _encoderLeft;
    _encoderRightDelta = static_cast<int16_t>(_encoderRight - _encoderRightPrevious);
    _encoderRightPrevious = _encoderRight;

    if (_motors.canAccuratelyEstimateSpeed()) {
        _speedLeftDPS = _motors.getLeftSpeed();
        _speedRightDPS = _motors.getRightSpeed();
        _speedDPS = (_speedLeftDPS + _speedRightDPS) * 0.5F;
    } else {
        const float speedMultiplier = 360.0F / (_motorStepsPerRevolution * deltaT);
        _speedLeftDPS = static_cast<float>(_encoderLeftDelta) * speedMultiplier;
        _speedRightDPS = static_cast<float>(_encoderRightDelta) * speedMultiplier;

        // encoders are very noisy, so the calculated speed value needs to be filtered
        static FilterMovingAverage<4> speedFilter;
        const float speedDPS = (_speedLeftDPS + _speedRightDPS) * 0.5F;
        _speedDPS = speedFilter.update(speedDPS);
    }
#else
    // no encoders, so estimate speed from power output
    (void)deltaT; // so LINT doesn't report and unused parameter.
    _speedDPS = MotorPairBase::clip((_powerLeft + _powerRight) * 0.5F, -1.0F, 1.0F) * _motorMaxSpeedDPS;
#endif
    // Disable the motors if the pitchAngle exceeds the switchOffAngle.
    // Don't switch on again for at least 2 seconds after robot falls over (ie don't switch on if it falls over and bounces back up again).
    constexpr uint32_t robotDebounceIntervalMs = 2000;
    _motorsDisabled = (fabs(_pitchAngleDegreesRaw) >= _motorSwitchOffAngleDegrees) || ((tickCount - _motorSwitchOffTickCount) < robotDebounceIntervalMs);
    if (!motorsIsOn() || _motorsDisabled) { // [[unlikely]]
        if (_motorSwitchOffTickCount == 0) { // the motors haven't already been switched off
            // Record the current tickCount so we can stop the motors turning back on if the robot bounces when it falls over.
            _motorSwitchOffTickCount = tickCount;
        }
        // Motors switched off, so set everything to zero, ready for motors to be switched on again.
        _motors.setPower(0.0F, 0.0F);
        _powerLeft  = 0.0F;
        _powerRight = 0.0F;

        _pitchUpdate = 0.0F;
        _pitchPID.resetIntegral();

        _speedUpdate = 0.0F;
        _speedPID.resetIntegral();

        _yawRateUpdate = 0.0F;
        _yawRatePID.resetIntegral();
    } else {
        _motorSwitchOffTickCount = 0; // reset the bounce prevention tickcount
    }
}


inline void MotorPairController::updatePIDs(float deltaT)
{
    [[maybe_unused]] bool orientationUpdated;
    const Quaternion orientation = _ahrs.getOrientationUsingLock(orientationUpdated);
    updatePIDs(orientation, deltaT);
}

void MotorPairController::updatePIDs(const Quaternion& orientation, float deltaT)
{

    // NOTE COORDINATE TRANSFORM: Madgwick filter uses Euler angles where roll is defined as rotation around the x-axis and pitch is rotation around the y-axis.
    // For the Self Balancing Robot, pitch is rotation around the x-axis and roll is rotation around the y-axis,
    // so ROLL and PITCH are REVERSED.
    _pitchAngleDegreesRaw = orientation.calculateRollDegrees();
//#define CALCULATE_ROLL_AND_YAW
#if defined(CALCULATE_ROLL_AND_YAW)
    // Roll and yaw are not required for the MotorPairController calculations,
    // but if they are calculated they will be displayed by the screen and telemetry, which can be useful in debugging.
    _rollAngleDegreesRaw = orientation.calculatePitchDegrees();
    _yawAngleDegreesRaw = orientation.calculateYawDegrees();
#endif

    if (!motorsIsOn() || _motorsDisabled) { // [[unlikely]]
        YIELD_TASK();
        return;
    }

    // calculate _speedUpdate according to the control mode.
    if (_controlMode == CONTROL_MODE_SERIAL_PIDS) {
        const float speedUpdate = _speedPID.update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT); // _speedDPS * _motorMaxSpeedDPS_reciprocal is in range [-1.0, 1.0]
        // feed the speedUpdate back into the pitchPID and set _speedUpdate to zero
        _pitchPID.setSetpoint(speedUpdate);
        _speedUpdate = 0.0F;
    } else if (_controlMode == CONTROL_MODE_PARALLEL_PIDS) {
        #if 0
        // motor_speed filter
        static constexpr float motorSpeedWeighting {0.8};
        static float motorSpeedDPS {0.0};
        //motorSpeed = motorSpeedWeighting * _motorSpeed + (1.0F - motorSpeedWeighting) * (_encoderLeftDelta + _encoderRightDelta);
        motorSpeedDPS = motorSpeedWeighting * motorSpeed + (1.0F - motorSpeedWeighting) * _speedDPS;
        _speedUpdate = _speedPID.update(motorSpeedDPS / _motorMaxSpeedDPS, deltaT);
        #endif
        _speedUpdate = _speedPID.update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT); // _speedDPS * _motorMaxSpeedDPS_reciprocal is in range [-1.0, 1.0]
    } else if (_controlMode == CONTROL_MODE_POSITION) {
        // NOTE: THIS IS NOT YET FULLY IMPLEMENTED
#if defined(MOTORS_HAVE_ENCODERS)
        _positionDegrees = static_cast<float>(_encoderLeft + _encoderRight) * 360.0F / (2.0F * _motorStepsPerRevolution);
#else
        _positionDegrees += _speedDPS * deltaT;
#endif
        // get the speed setpoint set in setSetPoints()
        const float speedSetpointDPS = _throttleStick * _motorMaxSpeedDPS;
        _positionSetpointDegrees += speedSetpointDPS * deltaT;
        // repurpose the speed PID as a position PID, since it is not being used for speed regulation in this mode
        _speedPID.setSetpoint(_positionSetpointDegrees);
        _speedUpdate = _speedPID.update(_positionDegrees, deltaT);
    }

    // update the pitch PID
    const float pitchAngleDegrees = _pitchAngleDegreesRaw - _pitchBalanceAngleDegrees;
    float pitchAngleDegreesDelta = pitchAngleDegrees - _pitchAngleDegreesPrevious;
    _pitchAngleDegreesPrevious = pitchAngleDegrees;
    // Calculate the filtered value to use as input into the PID, so the D-term is calculated using the filtered value.
    // This is beneficial because the D-term is especially susceptible to noise.
    static FilterMovingAverage<4> pitchAngleDeltaFilter; // moving average of length 4 involves no division, only addition and multiplication
    pitchAngleDegreesDelta = pitchAngleDeltaFilter.update(pitchAngleDegreesDelta);
    _pitchUpdate = _pitchPID.updateDelta(pitchAngleDegrees, pitchAngleDegreesDelta, deltaT);

    _yawRateUpdate = _yawRatePID.update(0.0F, deltaT); // yawRate is entirely feedforward, ie only depends on setpoint
}

/*!
Task loop for the MotorPairController. Uses PID controllers to update the motor pair.

Setpoints are provided by the receiver(joystick), and inputs(process variables) come from the AHRS and the motor encoders.

There are three PIDs, a pitch PID, a speed PID, and a yawRate PID.
*/
inline void MotorPairController::loop(float deltaT, uint32_t tickCount)
{
    updateSetpointsAndMotorSpeedEstimates(deltaT, tickCount);
    if (_ahrs.getMotorController() == nullptr) {
        updatePIDs(deltaT);
        updateMotors();
    } else {
        // if the AHRS has a motor controller set, then it will call updatePIDs, so we don't need to call updatePIDs here.
        updateMotors();
    }
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
        const uint32_t timeMicroSecond = micros();
        _timeMicroSecondDelta = timeMicroSecond - _timeMicroSecondPrevious;
        _timeMicroSecondPrevious = timeMicroSecond;

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
#endif // USE_MOTOR_PAIR_CONTROLLER
