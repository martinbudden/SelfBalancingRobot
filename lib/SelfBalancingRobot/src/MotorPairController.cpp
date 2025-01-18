#include "MotorPairController.h"

#include "MotorPairBase.h"
#include "MotorPairControllerTelemetry.h"
#include <AHRS.h>
#include <Filters.h>

#if !defined(UNIT_TEST_BUILD)
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif
#endif

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
        telemetry.pitchError = _PIDS[PITCH_ANGLE].getError();
        telemetry.speedError = _PIDS[SPEED].getError();
        telemetry.positionError = _PIDS[POSITION].getError();
   } else {
        telemetry.pitchError = { 0.0F, 0.0F, 0.0F };
        telemetry.speedError = { 0.0F, 0.0F, 0.0F };
        telemetry.positionError = { 0.0F, 0.0F, 0.0F };
   }
    telemetry.pitchUpdate = _updates[PITCH_ANGLE];
    telemetry.speedUpdate = _updates[SPEED];
    telemetry.positionUpdate = _updates[POSITION];
    telemetry.yawRateUpdate = _updates[YAW_RATE];

    telemetry.powerLeft = _mixer.powerLeft;
    telemetry.powerRight = _mixer.powerRight;

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

void MotorPairController::updateMotors(uint32_t tickCount)
{
    static FilterMovingAverage<4> powerLeftFilter; // filter length of 4 means division is not used in calculating average.
    static FilterMovingAverage<4> powerRightFilter;

    // Disable the motors if the pitchAngle exceeds the switchOffAngle.
    // Don't switch on again for at least 2 seconds after robot falls over (ie don't switch on if it falls over and bounces back up again).
    constexpr uint32_t robotDebounceIntervalMs = 2000;
    _mixer.motorsDisabled = (fabs(_pitchAngleDegreesRaw) >= _mixer.motorSwitchOffAngleDegrees) || ((tickCount - _mixer.motorSwitchOffTickCount) < robotDebounceIntervalMs);

    if (motorsIsOn() && !_mixer.motorsDisabled) {
        _mixer.motorSwitchOffTickCount = 0; // reset the bounce prevention tickcount

        _mixer.powerLeft  = _updates[PITCH_ANGLE] + _updates[SPEED] - _updates[YAW_RATE];
        _mixer.powerRight = _updates[PITCH_ANGLE] + _updates[SPEED] + _updates[YAW_RATE];

        // filter the power input into the motors so they run more smoothly.
        const float powerLeftFiltered = powerLeftFilter.update(_mixer.powerLeft);
        const float powerRightFiltered = powerRightFilter.update(_mixer.powerRight);
#if defined(AHRS_RECORD_TIMES_CHECKS)
        const uint32_t timeMicroSeconds0 = micros();
#endif
        _motors.setPower(powerLeftFiltered, powerRightFiltered);
#if defined(AHRS_RECORD_TIMES_CHECKS)
        _mixer.outputPowerTimeMicroSeconds = micros() - timeMicroSeconds0;
#endif
    } else {
        if (_mixer.motorSwitchOffTickCount == 0) { // the motors haven't already been switched off
            // Record the current tickCount so we can stop the motors turning back on if the robot bounces when it falls over.
            _mixer.motorSwitchOffTickCount = tickCount;
        }
        // Motors switched off, so set everything to zero, ready for motors to be switched on again.
        _motors.setPower(0.0F, 0.0F);
        _mixer.powerLeft  = 0.0F;
        _mixer.powerRight = 0.0F;
        powerLeftFilter.reset();
        powerRightFilter.reset();
    }
#if defined(SERIAL_OUTPUT)
    static int loopCount {0};
    ++loopCount;
    if (loopCount == 1) {
        loopCount = 0;

    //Serial.printf(">pitchPidErrorP:%8.2f, pidErrorI:%8.2f, pidErrorD:%8.2f, update:%8.2f\r\n",
    //    _pitchPID.getError().P, _pitchPID.getError().I, _pitchPID.getError().D, _updates[PITCH_ANGLE]);

    //Serial.printf(">pitchSetpoint:%7.2f, pitchAngleDegrees:%6.2f, pitchUpdate:%8.4f, speedSetpoint:%7.2f, speedUpdate:%7.3f, speedError:%7.3f\r\n",
    //   _pitchPID.getSetpoint(), _pitchAngleDegreesRaw, _updates[PITCH_ANGLE], _PIDS[SPEED].getSetpoint(), _updates[SPEED], _PIDS[SPEED].getError().P/_PIDS[SPEED].getP());

    //Serial.printf(">pitchSetpoint:%7.2f, pitchAngleDegrees:%6.2f, pitchUpdate:%8.4f, speedSetpoint:%7.2f, speedUpdate:%7.3f\r\n",
    //   _pitchPID.getSetpoint(), _pitchAngleDegreesRaw, _updates[PITCH_ANGLE], _PIDS[SPEED].getSetpoint(), _updates[SPEED]);

    //Serial.printf(">pitchAngleDegrees:%6.2f, pitchUpdate:%6.3f, speedDPS:%5.0F, speedUpdate:%8.5f\r\n",
    //    _pitchAngleDegreesRaw, _updates[PITCH_ANGLE], _speedDPS, _updates[SPEED]);

    Serial.printf(">speed:%8.2f, setpoint:%8.2f, pidErrorP:%8.2f, update:%8.2f, eL:%d, eR:%d\r\n",
        _speedDPS, _PIDS[SPEED].getSetpoint(), _PIDS[SPEED].getError().P, _updates[SPEED], _encoderLeftDelta, _encoderRightDelta);
    }
#endif
}

void MotorPairController::updateSetpointsAndMotorSpeedEstimates(float deltaT)
{
    // If new joystick values are available from the receiver, then map them to the range [-1.0, 1.0] and use them to update the setpoints.
    if (_newStickValuesAvailable) {
        _newStickValuesAvailable = false;
        _receiver.mapControls(_throttleStick, _rollStick, _pitchStick, _yawStick);

        _PIDS[SPEED].setSetpoint(_throttleStick);
        // Note the negative multiplier
        _PIDS[PITCH_ANGLE].setSetpoint(-_pitchStick * _pitchMaxAngleDegrees);
        // Note the negative multiplier, since pushing the yaw stick to the right results in a clockwise rotation, ie a negative yaw rate
        _PIDS[YAW_RATE].setSetpoint(-_yawStick * _yawStickMultiplier); // limit yaw rate to sensible range.
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
    _speedDPS = MotorPairBase::clip((_mixer.powerLeft + _mixer.powerRight) * 0.5F, -1.0F, 1.0F) * _motorMaxSpeedDPS;
#endif
}


void MotorPairController::updatePIDs(float deltaT)
{
    [[maybe_unused]] bool orientationUpdated;
    const Quaternion orientation = _ahrs.getOrientationUsingLock(orientationUpdated);
    [[maybe_unused]] bool ahrsDataUpdated;
    const AHRS::data_t data = _ahrs.getAhrsDataUsingLock(ahrsDataUpdated);
    updatePIDs(data.gyroRPS, data.acc, orientation, deltaT);
}

void MotorPairController::updatePIDs(const xyz_t& gyroRPS, [[maybe_unused]] const xyz_t& acc, const Quaternion& orientation, float deltaT)
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

    if (!motorsIsOn() || _mixer.motorsDisabled) { // [[unlikely]]
        _updates[PITCH_ANGLE] = 0.0F;
        _PIDS[PITCH_ANGLE].resetIntegral();
        _updates[SPEED] = 0.0F;
        _PIDS[SPEED].resetIntegral();
        _updates[YAW_RATE] = 0.0F;
        _PIDS[YAW_RATE].resetIntegral();
#if !defined(AHRS_RECORD_UPDATE_TIMES)
        YIELD_TASK();
        return;
#endif
    }

    // calculate _updates[SPEED] according to the control mode.
    _updates[SPEED] = _PIDS[SPEED].update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT); // _speedDPS * _motorMaxSpeedDPS_reciprocal is in range [-1.0, 1.0]
    if (_controlMode == CONTROL_MODE_SERIAL_PIDS) {
        // feed the speed update back into the pitchAngle PID and set _updates[SPEED] to zero
        _PIDS[PITCH_ANGLE].setSetpoint(-_updates[SPEED]);
        _updates[SPEED] = 0.0F;
    } else if (_controlMode == CONTROL_MODE_POSITION) {
        // NOTE: THIS IS EXPERIMENTAL AND NOT YET FULLY IMPLEMENTED
        updatePosition(deltaT);
    }

    // calculate _updates[PITCH_ANGLE]
    const float pitchAngleDegrees = _pitchAngleDegreesRaw - _pitchBalanceAngleDegrees;
    float pitchAngleDegreesDelta = pitchAngleDegrees - _pitchAngleDegreesPrevious;
    _pitchAngleDegreesPrevious = pitchAngleDegrees;
    // Calculate the filtered value to use as input into the PID, so the D-term is calculated using the filtered value.
    // This is beneficial because the D-term is especially susceptible to noise.
    static FilterMovingAverage<4> pitchAngleDeltaFilter;
    pitchAngleDegreesDelta = pitchAngleDeltaFilter.update(pitchAngleDegreesDelta);
    _updates[PITCH_ANGLE] = _PIDS[PITCH_ANGLE].updateDelta(pitchAngleDegrees, pitchAngleDegreesDelta, deltaT);

    // calculate _updates[YAW_RATE]
    const float yawRate = -gyroRPS.z * Quaternion::radiansToDegrees;
    _updates[YAW_RATE] = _PIDS[YAW_RATE].update(yawRate, deltaT);
}

void MotorPairController::updatePosition(float deltaT)
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
    _PIDS[POSITION].setSetpoint(_positionSetpointDegrees);
    const float updatePositionPrevious = _updates[POSITION];
    _updates[POSITION] = _PIDS[POSITION].update(_positionDegrees, deltaT);
    _updates[SPEED] = (_updates[POSITION] - updatePositionPrevious) / deltaT;
}

/*!
Task loop for the MotorPairController. Uses PID controllers to update the motor pair.

Setpoints are provided by the receiver(joystick), and inputs(process variables) come from the AHRS and the motor encoders.

There are three PIDs, a pitch PID, a speed PID, and a yawRate PID.
*/
void MotorPairController::loop(float deltaT, uint32_t tickCount)
{
    updateSetpointsAndMotorSpeedEstimates(deltaT);
    // If the AHRS has a pointer to the MPC then it will run updatePIDs, otherwise the MPC should run updatePIDs here
    if (_ahrs.getMotorController() == nullptr) {
        updatePIDs(deltaT);
    }
    updateMotors(tickCount);
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
