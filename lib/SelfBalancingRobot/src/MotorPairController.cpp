#include "MotorPairController.h"
#include "MotorPairBase.h"
#include "RadioController.h"

#include <AHRS.h>
#include <Blackbox.h>
#include <TimeMicroSeconds.h>

#include <cmath>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
inline void YIELD_TASK() { taskYIELD(); }
#else
inline void YIELD_TASK() {}
#endif


static const std::array<std::string, MotorPairController::PID_COUNT> PID_NAMES = {
    "ROLL_ANGLE",
    "PITCH_ANGLE",
    "YAW_RATE",
    "SPEED_SERIAL",
    "SPEED_PARALLEL",
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
    _motorPair.resetEncodersToZero();
}

void MotorPairController::setPID_Constants(const pidf_uint8_array_t& pids)
{
    for (size_t ii = PID_BEGIN; ii < PID_COUNT; ++ii) {
        const auto pidIndex = static_cast<pid_index_e>(ii);
        setPID_P_MSP(pidIndex, pids[pidIndex].kp);
        setPID_I_MSP(pidIndex, pids[pidIndex].ki);
        setPID_D_MSP(pidIndex, pids[pidIndex].kd);
        setPID_F_MSP(pidIndex, pids[pidIndex].kf);
    }
}

uint32_t MotorPairController::getOutputPowerTimeMicroSeconds() const
{
    return _motorPairMixer.getOutputPowerTimeMicroSeconds();
}

VehicleControllerBase::PIDF_uint8_t MotorPairController::getPID_MSP(size_t index) const
{
    assert(index < PID_COUNT);

    const auto pidIndex = static_cast<pid_index_e>(index);
    const PIDF_uint8_t ret = {
        .kp = static_cast<uint8_t>(_PIDS[pidIndex].getP() / _scaleFactors[pidIndex].kp),
        .ki = static_cast<uint8_t>(_PIDS[pidIndex].getI() / _scaleFactors[pidIndex].ki),
        .kd = static_cast<uint8_t>(_PIDS[pidIndex].getD() / _scaleFactors[pidIndex].kd),
        .kf = static_cast<uint8_t>(_PIDS[pidIndex].getF() / _scaleFactors[pidIndex].kf),
    };
    return ret;
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
motor_pair_controller_telemetry_t MotorPairController::getTelemetryData() const
{
    motor_pair_controller_telemetry_t telemetry;

    telemetry.pitchError = _PIDS[PITCH_ANGLE_DEGREES].getError();
    telemetry.speedError = _PIDS[_controlMode == CONTROL_MODE_SERIAL_PIDS ? SPEED_SERIAL_DPS : SPEED_PARALLEL_DPS].getError();
    telemetry.positionError = _PIDS[POSITION_DEGREES].getError();
    telemetry.pitchAngleOutput = _outputs[OUTPUT_PITCH_ANGLE_DEGREES];
    telemetry.speedOutput = _outputs[OUTPUT_SPEED_DPS];
    telemetry.positionOutput = _outputs[OUTPUT_POSITION_DEGREES];
    telemetry.yawRateOutput = _outputs[OUTPUT_YAW_RATE_DPS];

    telemetry.powerLeft = _motorPairMixer.getPowerLeft();
    telemetry.powerRight = _motorPairMixer.getPowerRight();

    telemetry.encoderLeft = _encoderLeft;
    telemetry.encoderRight = _encoderRight;
    telemetry.encoderLeftDelta = static_cast<int16_t>(_encoderLeftDelta);
    telemetry.encoderRightDelta = static_cast<int16_t>(_encoderRightDelta);

    telemetry.speedLeftDPS = _speedLeftDPS;
    telemetry.speedRightDPS = _speedRightDPS;
    telemetry.speedDPS_Filtered = _speedDPS;
    // copy of motorMaxSpeedDPS, so telemetry viewer can scale motor speed
    telemetry.motorMaxSpeedDPS = _motorMaxSpeedDPS;

    return telemetry;
}

void MotorPairController::motorsSwitchOff()
{
    _motorPairMixer.motorsSwitchOff();
    for (auto pid : _PIDS) {
        pid.switchIntegrationOff();
    }
}

void MotorPairController::motorsSwitchOn()
{
    // reset the PID integrals when we switch the motors on, so they don't start with residual I-term windup
    resetIntegrals();
    // don't allow motors to be switched on if the sensor fusion has not initialized
    if (!_ahrs.sensorFusionFilterIsInitializing()) {
        _motorPairMixer.motorsSwitchOn();
        for (auto pid : _PIDS) {
            pid.switchIntegrationOn();
        }
        if (_blackbox) {
            _blackbox->start({.debugMode = 0, .motorCount = 2, .servoCount = 0});
        }
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
    const MotorMixerBase::commands_t commands =
    (_radioController.getFailsafePhase() != RadioController::FAILSAFE_IDLE || !motorsIsOn()) ?
        MotorMixerBase::commands_t {
            .speed  = 0.0F,
            .roll   = 0.0F,
            .pitch  = 0.0F,
            .yaw    = 0.0F
        }
    :
        MotorMixerBase::commands_t {
            .speed  = _outputs[OUTPUT_SPEED_DPS],
            .roll   = _outputs[ROLL_ANGLE_DEGREES],
            .pitch  = _outputs[PITCH_ANGLE_DEGREES],
            .yaw    = _outputs[YAW_RATE_DPS]
        };
    _mixerThrottle = commands.speed;
    _motorPairMixer.outputToMotors(commands, deltaT, tickCount);
}

/*!
Use the new joystick values from the receiver to update the PID setpoints
using the ENU (East-North-Up) coordinate convention.

NOTE: this function is called form `updateControlValues` in the ReceiverTask loop() function,
 as a result of receiving new values from the receiver.
How often it is called depends on the type of transmitter and receiver the user has,
but is typically at intervals of between 5 milliseconds and 40 milliseconds (ie 200Hz to 25Hz).
In particular it runs much less frequently than `updateOutputsUsingPIDs` which typically runs at 1000Hz to 8000Hz.
*/
void MotorPairController::updateSetpoints(const RadioControllerBase::controls_t& controls)
{
    _throttleStick = controls.throttleStick;
    _rollStick = controls.rollStick;
    _pitchStick = controls.pitchStick;
    _yawStick = controls.yawStick;

    // Set the SPEED PIDs from the THROTTLE stick
    _PIDS[SPEED_SERIAL_DPS].setSetpoint(_throttleStick);
    _PIDS[SPEED_PARALLEL_DPS].setSetpoint(_throttleStick);

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
}

/*!
If new stick values are available then update the setpoint using the stick values, using the ENU coordinate convention.
*/
void MotorPairController::updateMotorSpeedEstimates(float deltaT)
{
#if defined(MOTORS_HAVE_ENCODERS)
    _motorPair.readEncoder();
    _encoderLeft = _motorPair.getLeftEncoder();
    _encoderRight = _motorPair.getRightEncoder();

    _encoderLeftDelta = _encoderLeft - _encoderLeftPrevious;
    _encoderLeftPrevious = _encoderLeft;
    _encoderRightDelta = _encoderRight - _encoderRightPrevious;
    _encoderRightPrevious = _encoderRight;

    if (_motorPair.canAccuratelyEstimateSpeed()) {
        _speedLeftDPS = _motorPair.getLeftSpeed();
        _speedRightDPS = _motorPair.getRightSpeed();
        _speedDPS = (_speedLeftDPS + _speedRightDPS) * 0.5F;
    } else {
        // For reference, at 420 steps per revolution, with a 100Hz (10ms) update rate,
        // 1 step per update gives a speed of (360 * 1/420) * 100 = 85 DPS (degrees per second)
        // With a 200Hz (5ms) update rate that is 170 DPS.
        const float speedMultiplier = 360.0F / (_motorPairStepsPerRevolution * deltaT);
        _speedLeftDPS = static_cast<float>(_encoderLeftDelta) * speedMultiplier;
        _speedRightDPS = static_cast<float>(_encoderRightDelta) * speedMultiplier;
        float speedDPS = (_speedLeftDPS + _speedRightDPS) * 0.5F; // NOLINT(misc-const-correctness) false positive

        // Encoders have discrete output and so are subject to digital noise
        // At a speed of 60rpm or 1rps with a 420 steps per revolution we have 420 steps per second,
        // If the MPC task is running at 100Hz, that is four steps per iteration, so an error of 1 step is 25%
        // If the MPC task is running at 200Hz, that is two steps per iteration, so an error of 1 step is 50%
        // So we average over 4 iterations to reduce this digital noise.
        speedDPS = _speedMovingAverageFilter.filter(speedDPS);
        // Additionally apply IIR filter.
        _speedDPS = _speedFilter.filter(speedDPS);

        //static float motorSpeed {0.0};
        //static constexpr float motorSpeedWeighting {0.8};
        //motorSpeed = motorSpeedWeighting * _motorSpeed + (1.0F - motorSpeedWeighting) * (_encoderLeftDelta + _encoderRightDelta);
        //motorSpeed = motorSpeedWeighting * motorSpeed + (1.0F - motorSpeedWeighting) * speedDPS;
        //_speedDPS = motorSpeed;
    }
#else
    (void)deltaT;
    // no encoders, so estimate speed from power output
    _speedDPS =  _motorMaxSpeedDPS * MotorPairBase::clip((_motorPairMixer.getPowerLeft() + _motorPairMixer.getPowerRight()) * 0.5F, -1.0F, 1.0F);
#endif
}

/*!
Gimbal lock occurs when the X-axis of the IMU points straight up or straight down.
Gimbal lock effects are significant when the X-axis angle is above +/-85 degrees.
We want to be able to work with tilt angles in excess of this (eg when recovering from a fall, or when starting up from a lying down position),
so we cannot point the X-axis in the direction of motion.

So we point the X-axis to the left and the Y axis forward, keeping the Z-axis pointing up.
This means pitch is about the Y-axis and roll about the X-axis (normally pitch is about X-axis and roll is about the Y-axis),
so we need to convert the values returned by calculatePitchDegrees() and calculateRollDegrees().
*/
void MotorPairController::updateOutputsUsingPIDs(const xyz_t& gyroRPS, [[maybe_unused]] const xyz_t& acc, const Quaternion& orientation, float deltaT)
{
    // AHRS orientation assumes (as is conventional) that pitch is around the X-axis, so convert.
    _pitchAngleDegreesRaw = -orientation.calculateRollDegrees();
    _motorPairMixer.setPitchAngleDegreesRaw(_pitchAngleDegreesRaw); // the mixer will switch off the motors if the pitch angle exceeds the maximum pitch angle
#define CALCULATE_ROLL
#define CALCULATE_YAW
#if defined(CALCULATE_ROLL)
    // Roll and yaw are not required for the MotorPairController calculations.
    // They are calculated to be displayed on the screen and by telemetry, which can be useful in debugging.
    // AHRS orientation assumes (as is conventional) that roll is around the Y-axis, so convert.
    _rollAngleDegreesRaw = orientation.calculatePitchDegrees();
#if defined(CALCULATE_YAW)
    _yawAngleDegreesRaw = orientation.calculateYawDegrees();
#endif
#endif

    // calculate _outputs[OUTPUT_SPEED_DPS] and setpoints according to the control mode.
    if (_controlMode == CONTROL_MODE_PARALLEL_PIDS) {
        _outputs[OUTPUT_SPEED_DPS] = -_PIDS[SPEED_PARALLEL_DPS].update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT); // _speedDPS * _motorMaxSpeedDPS_reciprocal is in range [-1.0, 1.0]
    } else if (_controlMode == CONTROL_MODE_SERIAL_PIDS) {
        const float speedOutput = _PIDS[SPEED_SERIAL_DPS].update(_speedDPS * _motorMaxSpeedDPS_reciprocal, deltaT);
        // feed the speed update back into the pitchAngle PID and set _outputs[OUTPUT_SPEED_DPS] to zero
        _PIDS[PITCH_ANGLE_DEGREES].setSetpoint(speedOutput);
        _outputs[OUTPUT_SPEED_DPS] = 0.0F;
    } else if (_controlMode == CONTROL_MODE_POSITION) {
        updatePositionOutputs(deltaT);
    }

    // calculate _outputs[PITCH_ANGLE_DEGREES]
    const float pitchAngleDegrees = _pitchAngleDegreesRaw - _pitchBalanceAngleDegrees;
    const float pitchAngleDegreesDelta = pitchAngleDegrees - _PIDS[PITCH_ANGLE_DEGREES].getPreviousMeasurement();
    // Use the filtered value of pitchAngleDegreesDelta as input into the PID update.
    // This is beneficial because the DTerm is especially susceptible to noise (since it is the derivative of a noisy value).
    const float pitchAngleFilteredDTerm = _pitchAngleDTermFilter.filter(pitchAngleDegreesDelta);
    _outputs[PITCH_ANGLE_DEGREES] = -_PIDS[PITCH_ANGLE_DEGREES].updateDelta(pitchAngleDegrees, pitchAngleFilteredDTerm, deltaT);

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
    _positionDegrees = static_cast<float>(_encoderLeft + _encoderRight) * 360.0F / (2.0F * _motorPairStepsPerRevolution);
#if false
    // experimental calculation of position using complementary filter of position from encoders and position estimated from integrating power output
    const float distanceDegrees = _positionDegrees - _positionDegreesPrevious;
    _positionDegreesPrevious = _positionDegrees;
    constexpr float alpha = 0.9F;
    const float speedEstimate = MotorPairBase::clip((_motorPairMixer.getPowerLeft() + _motorPairMixer.getPowerRight()) * 0.5F, -1.0F, 1.0F) * _motorMaxSpeedDPS;
    _positionDegrees += alpha*distanceDegrees + (1.0F - alpha)*speedEstimate*deltaT;
#endif
#else
    _positionDegrees += _speedDPS * deltaT;
#endif
    // scale the throttleStick value to get desired speed
    const float desiredSpeedDPS = _throttleStick * _motorMaxSpeedDPS;
    _positionSetpointDegrees += desiredSpeedDPS * deltaT;
    _PIDS[POSITION_DEGREES].setSetpoint(_positionSetpointDegrees);
    const float updatePositionDegreesPrevious = _outputs[OUTPUT_POSITION_DEGREES];
    _outputs[OUTPUT_POSITION_DEGREES] = _PIDS[POSITION_DEGREES].update(_positionDegrees, deltaT);
    _outputs[OUTPUT_SPEED_DPS] = (_outputs[OUTPUT_POSITION_DEGREES] - updatePositionDegreesPrevious) / deltaT;
}

uint32_t MotorPairController::updateBlackbox(uint32_t timeMicroSeconds, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc)
{
    assert(_blackbox!=nullptr && "_blackbox not set");
    return _blackbox->update(timeMicroSeconds, &gyroRPS, &gyroRPS_unfiltered, &acc);
}

/*!
Task loop for the MotorPairController. Uses PID controllers to update the motor pair.

Setpoints are provided by the receiver(joystick), and inputs(process variables) come from the AHRS and the motor encoders.

There are three PIDs, a pitch PID, a speed PID, and a yawRate PID.
*/
void MotorPairController::loop(float deltaT, uint32_t tickCount)
{
    updateMotorSpeedEstimates(deltaT);
    // If the AHRS is configured to run updateOutputsUsingPIDs, then we don't need to
    if (!_ahrs.configuredToUpdateOutputs()) {
        const Quaternion orientation = _ahrs.getOrientationUsingLock();
        const AHRS::data_t data = _ahrs.getAhrsDataUsingLock();

        updateOutputsUsingPIDs(data.gyroRPS, data.acc, orientation, deltaT);
    }
    outputToMotors(deltaT, tickCount);
}
