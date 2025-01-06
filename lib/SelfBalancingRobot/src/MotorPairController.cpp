#if defined(USE_MOTOR_PAIR_CONTROLLER)

#include "MotorPairController.h"

#include "AHRS_Base.h"
#include <Filters.h>

//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif

#include <cmath>
#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

/*!
Use the joystick values provided by the receiver to set the setpoints for motor speed, pitch angle, and yaw rate.

Note all setpoints are scaled to be in the range [-1.0, 1.0]. This in particular means we can feed back the speedUpdate into the pitch setpoint.

NOTE: this function runs in the context of the MAIN_LOOP_TASK, so use of the FPU is avoided so that the ESP32 FPU registers don't have to be saved on a context switch.
The values are converted to floats in the range [-1.0, 1.0] in the MotorPairController::loop() function, which runs in the context of the MPC_TASK.
*/
void MotorControllerBase::setSetpoints(int32_t throttleStickQ4dot12, int32_t rollStickQ4dot12, int32_t pitchStickQ4dot12, int32_t yawStickQ4dot12) // NOLINT(bugprone-easily-swappable-parameters)
{
    _throttleStickQ4dot12 = throttleStickQ4dot12;
    _rollStickQ4dot12 = rollStickQ4dot12;
    _pitchStickQ4dot12 = pitchStickQ4dot12;
    _yawStickQ4dot12 = yawStickQ4dot12;
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
const motor_pair_controller_telemetry_t& MotorPairController::getTelemetryData() const
{
    return _telemetry;
}

/*!
Map the yaw stick non-linearly to give more control for small values of yaw.
*/
float MotorPairController::mapYawStick(float yawStick)
{
    // map the yaw stick to a quadratic curve to give more control for small values of yaw.
    // higher values of a increase the effect
    // a=0 gives a linear response, a=1 gives and x^2 curve
    static constexpr float a { 0.2 };
    const float ret = (1.0F - a) * yawStick + (yawStick < 0.0F ? -a*yawStick*yawStick : a*yawStick*yawStick);
    return ret;
}

/*!
Task loop for the MotorPairController. Uses PID controllers to update the motor pair.

Setpoints are provided by the receiver(joystick), and inputs(process variables) come from the AHRS and the motor encoders.

There are three PIDs, a pitch PID, a speed PID, and a yawRate PID.

The MPC also stores its state in a telemetry variable. This can be used for display on the screen, or sent via the backchannel.
*/
void MotorPairController::loop(float deltaT, uint32_t tickCount)
{
    static FilterMovingAverage<4> powerLeftFilter; // filter length of 4 means division is not used in calculating average.
    static FilterMovingAverage<4> powerRightFilter;

#if defined(MOTORS_HAVE_ENCODERS)
    _motors.readEncoder();

    _telemetry.encoderLeft = _motors.getLeftEncoder();
    _telemetry.encoderLeftDelta = static_cast<int16_t>(_telemetry.encoderLeft - _encoderLeftPrevious);
    _encoderLeftPrevious = _telemetry.encoderLeft;

    _telemetry.encoderRight = _motors.getRightEncoder();
    _telemetry.encoderRightDelta = static_cast<int16_t>(_telemetry.encoderRight - _encoderRightPrevious);
    _encoderRightPrevious = _telemetry.encoderRight;

    if (_motors.canAccuratelyEstimateSpeed()) {
        _telemetry.speedLeftDPS = _motors.getLeftSpeed();
        _telemetry.speedRightDPS = _motors.getRightSpeed();

        const float speedDPS = (_telemetry.speedLeftDPS + _telemetry.speedRightDPS)/2;
        _telemetry.speedDPS_Filtered = speedDPS;
    } else {
        const float speedMultiplier =  360.0F / (_motorStepsPerRevolution * deltaT);
        _telemetry.speedLeftDPS = static_cast<float>(_telemetry.encoderLeftDelta) * speedMultiplier;
        _telemetry.speedRightDPS = static_cast<float>(_telemetry.encoderRightDelta) * speedMultiplier;

        // the encoders are very noisy, so the calculated speed value needs to be filtered
        const float speedDPS = (_telemetry.speedLeftDPS + _telemetry.speedRightDPS)/2;
        static FilterMovingAverage<4> speedFilter;
        _telemetry.speedDPS_Filtered = speedFilter.update(speedDPS);
        //Serial.printf(">speedDPS:%7.3f, speedDPSFiltered:%7.3f\r\n", speedDPS/60.0F, _telemetry.speedDPS_Filtered/60.0F);
    }

    //Serial.printf(">elDelta:%d, erDelta:%d, elRotDelta:%f, elRotDelta2:%f, tickDelta:%d\r\n",
      //  _telemetry.encoderLeftDelta, _telemetry.encoderRightDelta, (1.0F * _telemetry.encoderLeftDelta) / (_motorStepsPerRevolution * deltaT), (1.0F * _telemetry.encoderLeftDelta) / (_motorStepsPerRevolution * deltaT), tickCountDelta);
    _speedDPS = _telemetry.speedDPS_Filtered;
#else
    // no encoders, so estimate speed from power output
    _speedDPS = MotorPairBase::clip((_telemetry.powerLeft + _telemetry.powerRight) / 2, -1.0, 1.0) * _motorMaxSpeedDPS;
#endif

    const Quaternion orientation = _ahrs.getOrientationUsingLock();
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
    const float pitchAngleDegrees = _pitchAngleDegreesRaw - _pitchBalanceAngleDegrees;
    const float pitchAngleDegreesDelta = pitchAngleDegrees - _pitchAngleDegreesPrevious;
    _pitchAngleDegreesPrevious = pitchAngleDegrees;
    static FilterMovingAverage<4> pitchAngleDegreesDeltaFilter;
    const float pitchAngleDegreesDeltaFiltered = pitchAngleDegreesDeltaFilter.update(pitchAngleDegreesDelta);

    constexpr uint32_t robotDebounceIntervalMs = 2000; // don't switch on again for at least 2 seconds after robot falls over.
    if (motorsIsOn() && (fabs(pitchAngleDegrees) < _motorSwitchOffAngleDegrees) && ((tickCount - _motorSwitchOffTickCount) > robotDebounceIntervalMs)) {
        _motorSwitchOffTickCount = 0;
        if (fabs(pitchAngleDegrees) > _pitchMaxAngleDegrees) {
            // we are way off balance, and the I-term is now a hindrance
            _pitchPID.resetIntegral();
        }

        float speedUpdate {0.0};
        if (_controlMode == CONTROL_MODE_SERIAL_PIDS) {
            _speedPID.setSetpoint(Q4dot12_to_float(_throttleStickQ4dot12));
            _telemetry.speedUpdate = _speedPID.update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT); // _speedDPS * _motorMaxSpeedDPS_reciprocal is in range [-1.0, 1.0]
            // feed the speedUpdate back into the pitchPID and set speedUpdate to zero
            _pitchPID.setSetpoint(_telemetry.speedUpdate);
            speedUpdate = 0.0F;
        } else if (_controlMode == CONTROL_MODE_PARALLEL_PIDS) {
            _speedPID.setSetpoint(Q4dot12_to_float(_throttleStickQ4dot12));
            _pitchPID.setSetpoint(-Q4dot12_to_float(_pitchStickQ4dot12) * _pitchMaxAngleDegrees);
            #if 0
            // motor_speed filter
            static constexpr float motorSpeedWeighting {0.8};
            static float motorSpeedDPS {0.0};
            //motorSpeed = motorSpeedWeighting * _motorSpeed + (1.0F - motorSpeedWeighting) * (_telemetry.encoderLeftDelta + _telemetry.encoderRightDelta);
            motorSpeedDPS = motorSpeedWeighting * motorSpeed + (1.0F - motorSpeedWeighting) * _speedDPS;
            _telemetry.speedUpdate = _speedPID.update(motorSpeedDPS / _telemetry.motorMaxSpeedDPS, deltaT);
            #endif
            _telemetry.speedUpdate = _speedPID.update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT); // _speedDPS * _motorMaxSpeedDPS_reciprocal is in range [-1.0, 1.0]
            speedUpdate = _telemetry.speedUpdate;
        } else if (_controlMode == CONTROL_MODE_POSITION) {
            _pitchPID.setSetpoint(-Q4dot12_to_float(_pitchStickQ4dot12) * _pitchMaxAngleDegrees);
#if defined(MOTORS_HAVE_ENCODERS)
            _positionDegrees = static_cast<float>(_telemetry.encoderLeft + _telemetry.encoderRight) * 360.0F / (2.0F * _motorStepsPerRevolution);
#else
            _positionDegrees += _speedDPS * deltaT;
#endif
            // get the speed setpoint set in setSetPoints()
            const float speedSetpointDPS = Q4dot12_to_float(_throttleStickQ4dot12) * _motorMaxSpeedDPS;
            _positionSetpointDegrees += speedSetpointDPS * deltaT;
            // repurpose the speed PID as a position PID, since it is not being used for speed regulation in this mode
            _speedPID.setSetpoint(_positionSetpointDegrees);
            _telemetry.speedUpdate = _speedPID.update(_positionDegrees, deltaT);
            speedUpdate = _telemetry.speedUpdate;
        }

        _telemetry.pitchUpdate = getPitchRateIsFiltered() ?
            _pitchPID.updateDelta(pitchAngleDegrees, pitchAngleDegreesDeltaFiltered, deltaT) :
            _pitchPID.update(pitchAngleDegrees, deltaT);

        // Note the negative multiplier, since pushing the yaw stick to the right results in a clockwise rotation, ie a negative yaw rate
        _yawRatePID.setSetpoint(-mapYawStick(Q4dot12_to_float(_yawStickQ4dot12)) * _yawStickMultiplier); // limit yaw rate to sensible range.
        _telemetry.yawRateUpdate = _yawRatePID.update(0.0, deltaT); // yawRate is entirely feedforward, ie only depends on setpoint

        _telemetry.powerLeft  = _telemetry.pitchUpdate + speedUpdate - _telemetry.yawRateUpdate;
        _telemetry.powerRight = _telemetry.pitchUpdate + speedUpdate + _telemetry.yawRateUpdate;

        // filter the power input into the motors so they run more smoothly.
        const float powerLeftFiltered = powerLeftFilter.update(_telemetry.powerLeft);
        const float powerRightFiltered = powerRightFilter.update(_telemetry.powerRight);
        _motors.setPower(powerLeftFiltered, powerRightFiltered);
        //_motors.setPower(_telemetry.powerLeft, _telemetry.powerRight);

        // update the telemetry error data, yawRateError not updated, since yawRate is controlled by feed forward
        _telemetry.pitchError = _pitchPID.getError();
        _telemetry.speedError = _speedPID.getError();

#if defined(SERIAL_OUTPUT)
        static int loopCount {0};
        ++loopCount;
        if (loopCount == 10) {
            loopCount = 0;

        //Serial.printf(">pitchAngleDegrees:%6.2f, pitchRate:%8.4f\r\n", pitchAngleDegrees, pitchRate);

        //Serial.printf(">pidErrorP:%8.2f, pidErrorI:%8.2f, pidErrorD:%8.2f, pitchAngleDegreesDelta:%8.2f, pitchAngleDegreesDeltaFiltered:%8.2f, pitchRate:%8.2f\r\n",
        //    _pitchPID.getErrorRaw().P, _pitchPID.getErrorRaw().I, _pitchPID.getErrorRaw().D, pitchAngleDegreesDelta / deltaT, pitchAngleDegreesDeltaFiltered / deltaT, -pitchRate);

        //Serial.printf(">pitchAngleDegreesDelta:%8.2f, pitchAngleDegreesDeltaFiltered:%8.2f, pitchRate:%8.2f, deltaT:%6.3f\r\n",
        //    pitchAngleDegreesDelta / deltaT, pitchAngleDegreesDeltaFiltered / deltaT, -pitchRate, deltaT);

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

    } else {

        // Record the current tickCount so we can stop the motors turning back on if the robot bounces when it falls over.
        if (_motorSwitchOffTickCount == 0) {
            _motorSwitchOffTickCount = tickCount;
        }
        // Motors switched off, so set everything to zero, ready for motors to be switched on again.
        _motors.setPower(0.0F, 0.0F);
        _telemetry.powerLeft = 0.0F;
        _telemetry.powerRight = 0.0F;
        powerLeftFilter.reset();
        powerRightFilter.reset();

        _telemetry.pitchUpdate = 0.0F;
        _telemetry.pitchError = { 0.0F, 0.0F, 0.0F };
        _pitchPID.resetIntegral();

        _telemetry.speedUpdate = 0.0F;
        _telemetry.speedError = { 0.0F, 0.0F, 0.0F };
        _speedPID.resetIntegral();

        _telemetry.yawRateUpdate = 0.0F;
        _yawRatePID.resetIntegral();
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
    _previousWakeTime = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next tickIntervalTicks
        vTaskDelayUntil(&_previousWakeTime, _tickIntervalTicks);

        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than _tickIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;

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
