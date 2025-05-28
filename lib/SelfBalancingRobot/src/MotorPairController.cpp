#include "MotorPairController.h"

#include "MotorPairBase.h"
#include "MotorPairControllerTelemetry.h"
#include <AHRS.h>
#include <Filters.h>
#include <ReceiverBase.h>

#include <cmath>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
inline void YIELD_TASK() { taskYIELD(); }
#else
inline void YIELD_TASK() {}
#endif

#if defined(USE_FREERTOS)

#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#include <esp_timer.h>
static uint32_t timeUs() { return static_cast<uint32_t>(esp_timer_get_time()); }
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <esp32-hal.h>
static uint32_t timeUs() { return micros(); }
#endif

#endif


static const std::array<std::string, MotorPairController::PID_COUNT> PID_NAMES = {
    "ROLL_ANGLE",
    "PITCH_ANGLE",
    "YAW_RATE",
    "SPEED",
    "POSITION"
};

const std::string& MotorPairController::getPID_Name(pid_index_e pidIndex) const
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
Return he MPC telemetry data.

The telemetry data is shared between this task (the MPC task) and the MAIN_LOOP_TASK, however it is deliberately not protected by a mutex.
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

void MotorPairController::motorsSwitchOff()
{
    _mixer.motorsSwitchOff();
}

void MotorPairController::motorsSwitchOn()
{
    // don't allow motors to be switched on if the sensor fusion has not initialized
    if (!_ahrs.sensorFusionFilterIsInitializing()) {
        _mixer.motorsSwitchOn();
    }
}

void MotorPairController::motorsToggleOnOff()
{
    if (motorsIsOn()) {
        motorsSwitchOff();
    } else {
        motorsSwitchOn();
    }
}

void MotorPairController::outputToMotors(float deltaT, uint32_t tickCount)
{
    const MotorMixerBase::output_t outputs =
    _failSafeOn ?
        MotorMixerBase::output_t {
            .speed  = 0.0F,
            .roll   = 0.0F,
            .pitch  = 0.0F,
            .yaw    = 0.0F
        }
    :
        MotorMixerBase::output_t {
            .speed  = _outputs[SPEED_DPS],
            .roll   = _outputs[ROLL_ANGLE_DEGREES],
            .pitch  = _outputs[PITCH_ANGLE_DEGREES],
            .yaw    = _outputs[YAW_RATE_DPS]
        };
    _mixer.outputToMotors(outputs, deltaT, tickCount);
}

/*!
Map the yaw stick non-linearly to give more control for small values of yaw.
*/
float MotorPairController::mapYawStick(float yawStick)
{
    // map the yaw stick to a quadratic curve to give more control for small values of yaw.
    // higher values of a increase the effect
    // a=0 gives a linear response, a=1 gives a parabolic (x^2) curve
    static constexpr float a { 0.2F };
    const float ret = (1.0F - a) * yawStick + (yawStick < 0.0F ? -a*yawStick*yawStick : a*yawStick*yawStick);
    return ret;
}

/*!
If new stick values are available then update the setpoint using the stick values, using the ENU coordinate convention.

NOTE: this function runs in the context of the VehicleController task, in particular the FPU usage is in that context, so this avoids the
need to save the ESP32 FPU registers on a context switch.
*/
void MotorPairController::updateSetpoints([[maybe_unused]] float deltaT, uint32_t tickCount)
{
    // failsafe handling
    if (_receiver.isNewPacketAvailable()) {
        _receiver.clearNewPacketAvailable();
        _receiverInUse = true;
        _failSafeOn = false;
        _failSafeTickCount = tickCount;
        // Get the new joystick values from the receiver and use them to update the setpoints.
        _receiver.getStickValues(_throttleStick, _rollStick, _pitchStick, _yawStick);
        _yawStick = mapYawStick(_yawStick);

        _PIDS[SPEED_DPS].setSetpoint(_throttleStick);

        // MotorPairController uses ENU coordinate convention

        // Pushing the ROLL stick to the right gives a positive value of rollStick and we want this to be left side up.
        // For ENU left side up is positive roll, so sign of setpoint is same sign as rollStick.
        // So sign of _rollStick is left unchanged.
        _PIDS[ROLL_ANGLE_DEGREES].setSetpoint(_rollStick * _rollMaxAngleDegrees);

        // Pushing the PITCH stick forward gives a positive value of pitchStick and we want this to be nose down.
        // For ENU nose down is positive pitch, so sign of setpoint is same sign as pitchStick.
        // So sign of _pitchStick is left unchanged.
        _PIDS[PITCH_ANGLE_DEGREES].setSetpoint(_pitchStick * _pitchMaxAngleDegrees);

        // Pushing the YAW stick to the right gives a positive value of yawStick and we want this to be nose right.
        // For ENU nose left is positive yaw, so sign of setpoint is same as sign of yawStick.
        // So sign of _yawStick is negated
        _yawStick = -_yawStick;
        _PIDS[YAW_RATE_DPS].setSetpoint(_yawStick * _yawStickMultiplier); // limit yaw rate to sensible range.

        const bool motorOnOff = _receiver.getSwitch(ReceiverBase::MOTOR_ON_OFF_SWITCH);
        if (motorOnOff) {
            _onOffSwitchPressed = true;
        } else {
            if (_onOffSwitchPressed) {
                // motorOnOff false and _onOffPressed true means the  on/off button is being released, so toggle the motor state
                motorsToggleOnOff();
                _onOffSwitchPressed = false;
            }
        }
    } else if ((tickCount - _failSafeTickCount > _failSafeTickCountThreshold) && _receiverInUse) {
        // _receiverInUse is initialized to false, so the motors won't turn off it the transmitter hasn't been turned on yet.
        // We've had 1500 ticks (1.5 seconds) without a packet, so we seem to have lost contact with the transmitter,
        // so switch off the motors to prevent the vehicle from doing a runaway.
        _failSafeOn = true;
        if ((tickCount - _failSafeTickCount > _failSafeTickCountSwitchOffThreshold)) {
            motorsSwitchOff();
            _receiverInUse = false; // set to false to allow us to switch the motors on again if we regain a signal
        }
    }
}

/*!
If new stick values are available then update the setpoint using the stick values, using the ENU coordinate convention.
*/
void MotorPairController::updateMotorSpeedEstimates(float deltaT)
{
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
        // For reference, at 420 steps per revolution, with a 100Hz (10ms) update rate, 1 step per update gives a speed of (360 * 1/420) * 100 = 85 dps
        // With a 200Hz (5ms) update rate that is 170 DPS.
        const float speedMultiplier = 360.0F / (_motorStepsPerRevolution * deltaT);
        _speedLeftDPS = static_cast<float>(_encoderLeftDelta) * speedMultiplier;
        _speedRightDPS = static_cast<float>(_encoderRightDelta) * speedMultiplier;

        // encoders have discrete output and so are subject to digital noise
        // At a speed of 60rpm or 1rps with a 420 steps per revolution we have 420 steps per second,
        // If the MPC task is running at 100Hz, that is four steps per iteration, so an error of 1 step is 25%
        // If the MPC task is running at 200Hz, that is two steps per iteration, so an error of 1 step is 50%
        // So we average over 4 iterations to reduce this digital noise.
        static FilterMovingAverage<4> speedMovingAverageFilter;
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
    (void)deltaT; // so LINT doesn't report an unused parameter.
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

/*!
Gimbal lock occurs when the X-axis of an IMU points straight up or straight down.
Gimbal lock effects are significant when the X-axis angle is above +/-85 degrees.
We want to be able to work with tilt angles in excess of this (eg when recovering from a fall, or when starting up from a lying down position),
so we cannot point the X-axis in the direction of motion.

So we point the X-axis to the left and the Y axis forward keeping the Z-axis pointing up.
This means pitch is about the Y-axis and roll about the X-axis (normally pitch is about X-axis and roll is about the Y-axis),
so we need to convert the values returned by calculatePitchDegrees() and calculateRollDegrees().
*/
void MotorPairController::updateOutputsUsingPIDs(const xyz_t& gyroRPS, [[maybe_unused]] const xyz_t& acc, const Quaternion& orientation, float deltaT)
{
    // AHRS orientation assumes (as is conventional) that pitch is around the X-axis, so convert.
    _pitchAngleDegreesRaw = -orientation.calculateRollDegrees();
    _mixer.setPitchAngleDegreesRaw(_pitchAngleDegreesRaw); // the mixer will switch off the motors if the pitch angle exceeds the maximum pitch angle
#define CALCULATE_ROLL
#define CALCULATE_YAW
#if defined(CALCULATE_ROLL)
    // Roll and yaw are not required for the MotorPairController calculations,
    // but if they are calculated they will be displayed by the screen and telemetry, which can be useful in debugging.
    // AHRS orientation assumes (as is conventional) that roll is around the Y-axis, so convert.
    _rollAngleDegreesRaw = orientation.calculatePitchDegrees();
#if defined(CALCULATE_YAW)
    _yawAngleDegreesRaw = orientation.calculateYawDegrees();
#endif
#endif

    if (!motorsIsOn() || motorsIsDisabled()) { // [[unlikely]]
        for (int ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
            _outputs[ii] = 0.0F;
            _PIDS[ii].resetIntegral();
        }
#if !defined(AHRS_RECORD_UPDATE_TIMES)
        YIELD_TASK();
        return;
#endif
    }

    // calculate _outputs[SPEED_DPS] according to the control mode.
    if (_controlMode == CONTROL_MODE_PARALLEL_PIDS) {
        _outputs[SPEED_DPS] = -_PIDS[SPEED_DPS].update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT); // _speedDPS * _motorMaxSpeedDPS_reciprocal is in range [-1.0, 1.0]
    } else if (_controlMode == CONTROL_MODE_SERIAL_PIDS) {
        const float speedOutput = _PIDS[SPEED_DPS].update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT);
        // feed the speed update back into the pitchAngle PID and set _outputs[SPEED_DPS] to zero
        _PIDS[PITCH_ANGLE_DEGREES].setSetpoint(speedOutput);
        _outputs[SPEED_DPS] = 0.0F;
    } else if (_controlMode == CONTROL_MODE_POSITION) {
        updatePositionOutputs(deltaT);
    }

    // calculate _outputs[PITCH_ANGLE_DEGREES]
    const float pitchAngleDegrees = _pitchAngleDegreesRaw - _pitchBalanceAngleDegrees;
    // Calculate the filtered value to use as input into the PID, so the DTerm is calculated using the filtered value.
    // This is beneficial because the DTerm is especially susceptible to noise.
    const float pitchAngleDegreesDelta = pitchAngleDegrees - _pitchAngleDTermFilter.update(_PIDS[PITCH_ANGLE_DEGREES].getPreviousMeasurement());
    _outputs[PITCH_ANGLE_DEGREES] = -_PIDS[PITCH_ANGLE_DEGREES].updateDelta(pitchAngleDegrees, pitchAngleDegreesDelta, deltaT);

    // calculate _outputs[YAW_RATE_DPS]
    const float yawRateDPS = -gyroRPS.z * Quaternion::radiansToDegrees;
    _outputs[YAW_RATE_DPS] = _PIDS[YAW_RATE_DPS].update(yawRateDPS, deltaT);
}

/*!
Calculate _outputs[SPEED_DPS] using a position PID controller.
*/
void MotorPairController::updatePositionOutputs(float deltaT)
{
    // NOTE: THIS IS EXPERIMENTAL AND NOT YET FULLY IMPLEMENTED
#if defined(MOTORS_HAVE_ENCODERS)
    _positionDegrees = static_cast<float>(_encoderLeft + _encoderRight) * 360.0F / (2.0F * _motorStepsPerRevolution);
#if false
    // experimental calculation of position using complementary filter of position from encoders and position estimated from integrating power output
    const float distanceDegrees = _positionDegrees - _positionDegreesPrevious;
    _positionDegreesPrevious = _positionDegrees;
    constexpr float alpha = 0.9F;
    const float speedEstimate = MotorPairBase::clip((_mixer.getPowerLeft() + _mixer.getPowerRight()) * 0.5F, -1.0F, 1.0F) * _motorMaxSpeedDPS;
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

/*!
Task loop for the MotorPairController. Uses PID controllers to update the motor pair.

Setpoints are provided by the receiver(joystick), and inputs(process variables) come from the AHRS and the motor encoders.

There are three PIDs, a pitch PID, a speed PID, and a yawRate PID.
*/
void MotorPairController::loop(float deltaT, uint32_t tickCount)
{
    updateSetpoints(deltaT, tickCount);
    updateMotorSpeedEstimates(deltaT);
    // If the AHRS is configured to run updateOutputsUsingPIDs, then we don't need to
    if (!_ahrs.configuredToUpdateOutputs()) {
        updateOutputsUsingPIDs(deltaT);
    }
    outputToMotors(deltaT, tickCount);
}

/*!
Task function for the MotorPairController. Sets up and runs the task loop() function.
*/
[[noreturn]] void MotorPairController::Task(const TaskParameters* taskParameters)
{
    _taskIntervalMicroSeconds = taskParameters->taskIntervalMicroSeconds;
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    const uint32_t taskIntervalTicks = pdMS_TO_TICKS(taskParameters->taskIntervalMicroSeconds / 1000);
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);

        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than taskIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const uint32_t timeMicroSeconds = timeUs();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;

        if (_tickCountDelta > 0) { // guard against the case of the while loop executing twice on the same tick interval
            const float deltaT = pdTICKS_TO_MS(_tickCountDelta) * 0.001F;
            loop(deltaT, tickCount);
        }
    }
#else
    (void)taskParameters;
    while (true) {}
#endif
}

/*!
Wrapper function for MotorPairController::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void MotorPairController::Task(void* arg)
{
    const MotorPairController::TaskParameters* taskParameters = static_cast<MotorPairController::TaskParameters*>(arg);

    MotorPairController* mpc = taskParameters->motorPairController;
    mpc->Task(taskParameters);
}
