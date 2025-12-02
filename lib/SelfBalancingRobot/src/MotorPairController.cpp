#include "AHRS_MessageQueue.h"
#include "Cockpit.h"
#include "MotorPairBase.h"
#include "MotorPairController.h"

#include <AHRS.h>
#include <Blackbox.h>
#include <TimeMicroseconds.h>


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

void MotorPairController::motorsResetAllEncoders()
{
    _motorMixer.resetAllEncoders();
}

void MotorPairController::setPID_Constants(const pidf_uint16_array_t& pids)
{
    for (size_t ii = PID_BEGIN; ii < PID_COUNT; ++ii) {
        const auto pidIndex = static_cast<pid_index_e>(ii);
        setPID_Constants(pidIndex, pids[pidIndex]);
    }
}

/*!
Set the P, I, D, F, and S values for the PID with index pidIndex.
*/
void MotorPairController::setPID_Constants(pid_index_e pidIndex, const PIDF_uint16_t& pid16)
{
    setPID_P_MSP(pidIndex, pid16.kp);
    setPID_I_MSP(pidIndex, pid16.ki);
    setPID_D_MSP(pidIndex, pid16.kd);
    setPID_S_MSP(pidIndex, pid16.ks);
    setPID_K_MSP(pidIndex, pid16.kk);
}

void MotorPairController::setPID_P_MSP(pid_index_e pidIndex, uint16_t kp)
{
    _PIDS[pidIndex].setP(kp * _scaleFactors.kp);
}

void MotorPairController::setPID_PD_MSP(pid_index_e pidIndex, uint16_t kp)
{
    const PIDF pid = getPID(pidIndex);
    const float ratio = pid.getD() / pid.getP();
    setPID_P_MSP(pidIndex, kp);
    setPID_D_MSP(pidIndex, static_cast<uint16_t>(kp*ratio));
}

void MotorPairController::setPID_I_MSP(pid_index_e pidIndex, uint16_t ki)
{
    _PIDS[pidIndex].setI(ki * _scaleFactors.ki);
}

void MotorPairController::setPID_D_MSP(pid_index_e pidIndex, uint16_t kd)
{
    _PIDS[pidIndex].setD(kd * _scaleFactors.kd);
}

void MotorPairController::setPID_S_MSP(pid_index_e pidIndex, uint16_t ks)
{
    _PIDS[pidIndex].setS(ks * _scaleFactors.ks);
}

void MotorPairController::setPID_K_MSP(pid_index_e pidIndex, uint16_t kk)
{
    _PIDS[pidIndex].setK(kk * _scaleFactors.kk);
}

VehicleControllerBase::PIDF_uint16_t MotorPairController::getPID_MSP(size_t index) const
{
    assert(index < PID_COUNT);

    const auto pidIndex = static_cast<pid_index_e>(index);
    const PIDF_uint16_t ret = {
        .kp = static_cast<uint16_t>(_PIDS[pidIndex].getP() / _scaleFactors.kp),
        .ki = static_cast<uint16_t>(_PIDS[pidIndex].getI() / _scaleFactors.ki),
        .kd = static_cast<uint16_t>(_PIDS[pidIndex].getD() / _scaleFactors.kd),
        .ks = static_cast<uint16_t>(_PIDS[pidIndex].getS() / _scaleFactors.ks),
        .kk = static_cast<uint16_t>(_PIDS[pidIndex].getK() / _scaleFactors.kk),
    };
    return ret;
}

uint32_t MotorPairController::getOutputPowerTimeMicroseconds() const
{
    return _motorMixer.getOutputPowerTimeMicroseconds();
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

    const PIDF::error_t pitchError = _PIDS[PITCH_ANGLE_DEGREES].getError();
    telemetry.pitchError = { pitchError.P, pitchError.I, pitchError.D, pitchError.S, pitchError.K };

    const PIDF::error_t speedError = _PIDS[_controlMode == CONTROL_MODE_SERIAL_PIDS ? SPEED_SERIAL_DPS : SPEED_PARALLEL_DPS].getError();
    telemetry.speedError = { speedError.P, speedError.I, speedError.D, speedError.S, speedError.K };

    const PIDF::error_t positionError = _PIDS[POSITION_DEGREES].getError();
    telemetry.positionError = { positionError.P, positionError.I, positionError.D, positionError.S, positionError.K };

    telemetry.pitchAngleOutput = _outputs[OUTPUT_PITCH_ANGLE_DEGREES];
    telemetry.speedOutput = _outputs[OUTPUT_SPEED_DPS];
    telemetry.positionOutput = _outputs[OUTPUT_POSITION_DEGREES];
    telemetry.yawRateOutput = _outputs[OUTPUT_YAW_RATE_DPS];

    telemetry.powerLeft = _motorMixer.getMotorOutput(MotorPairMixer::MOTOR_LEFT);
    telemetry.powerRight = _motorMixer.getMotorOutput(MotorPairMixer::MOTOR_RIGHT);

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

bool MotorPairController::motorsIsOn() const
{
    return _motorMixer.motorsIsOn();
}

void MotorPairController::motorsSwitchOff()
{
    _motorMixer.motorsSwitchOff();
    // Switch of PID integration when the motors are switched off, so we don't get integral windup.
    for (auto pid : _PIDS) {
        pid.switchIntegrationOff();
    }
}

void MotorPairController::motorsSwitchOn()
{
    // reset the PID integrals when we switch the motors on, so they don't start with residual I-term windup
    resetIntegrals();
    // don't allow motors to be switched on if the sensor fusion has not initialized
    if (!_sensorFusionFilterIsInitializing) {
        _motorMixer.motorsSwitchOn();
        // and switch PID integration back on
        for (auto pid : _PIDS) {
            pid.switchIntegrationOn();
        }
    }
}

void MotorPairController::setControlMode(control_mode_e controlMode)
{
    _controlMode = controlMode; resetIntegrals();
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
void MotorPairController::updateSetpoints(const controls_t& controls)
{
    setControlMode(controls.controlMode);

    _throttleStick = controls.throttleStick;

    // Set the SPEED PIDs from the THROTTLE stick
    _PIDS[SPEED_SERIAL_DPS].setSetpoint(controls.throttleStick);
    _PIDS[SPEED_PARALLEL_DPS].setSetpoint(controls.throttleStick);

    // MotorPairController uses ENU coordinate convention

    // Pushing the ROLL stick to the right gives a positive value of rollStick and we want this to be left side up.
    // For ENU left side up is positive roll, so sign of setpoint is same sign as rollStick.
    // So sign of _rollStick is left unchanged.
    _PIDS[ROLL_ANGLE_DEGREES].setSetpoint(controls.rollStickDegrees);

    // Pushing the PITCH stick forward gives a positive value of pitchStick and we want this to be nose down.
    // For ENU nose down is positive pitch, so sign of setpoint is same sign as pitchStick.
    // So sign of _pitchStick is left unchanged.
    _PIDS[PITCH_ANGLE_DEGREES].setSetpoint(controls.pitchStickDegrees);

    // Pushing the YAW stick to the right gives a positive value of yawStick and we want this to be nose right.
    // For ENU nose left is positive yaw, so sign of setpoint is same as sign of yawStick.
    // So sign of _yawStick is negated
    _PIDS[YAW_RATE_DPS].setSetpoint(-controls.yawStickDPS * _yawStickMultiplier); // limit yaw rate to sensible range.
}

/*!
If new stick values are available then update the setpoint using the stick values, using the ENU coordinate convention.
*/
void MotorPairController::updateMotorSpeedEstimates(float deltaT)
{
#if defined(MOTORS_HAVE_ENCODERS)
    _motorMixer.readEncoder(MotorPairMixer::MOTOR_LEFT);
    _motorMixer.readEncoder(MotorPairMixer::MOTOR_RIGHT);

    _encoderLeft = _motorMixer.getEncoder(MotorPairMixer::MOTOR_LEFT);
    _encoderLeftDelta = _encoderLeft - _encoderLeftPrevious;
    _encoderLeftPrevious = _encoderLeft;

    _encoderRight = _motorMixer.getEncoder(MotorPairMixer::MOTOR_RIGHT);
    _encoderRightDelta = _encoderRight - _encoderRightPrevious;
    _encoderRightPrevious = _encoderRight;

    if (_motorMixer.canReportSpeed(MotorPairMixer::MOTOR_LEFT)) {
        _speedLeftDPS = _motorMixer.getMotorSpeedDPS(MotorPairMixer::MOTOR_LEFT);
        _speedRightDPS = _motorMixer.getMotorSpeedDPS(MotorPairMixer::MOTOR_RIGHT);
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

        //static float motorSpeed {0.0F};
        //static constexpr float motorSpeedWeighting {0.8F};
        //motorSpeed = motorSpeedWeighting * _motorSpeed + (1.0F - motorSpeedWeighting) * (_encoderLeftDelta + _encoderRightDelta);
        //motorSpeed = motorSpeedWeighting * motorSpeed + (1.0F - motorSpeedWeighting) * speedDPS;
        //_speedDPS = motorSpeed;
    }
#else
    (void)deltaT;
    // no encoders, so estimate speed from power output
    _speedDPS =  _motorMaxSpeedDPS * MotorPairBase::clip((_motorMixer.getMotorOutput(MotorPairMixer::MOTOR_LEFT) + _motorMixer.getMotorOutput(MotorPairMixer::MOTOR_LEFT)) * 0.5F, -1.0F, 1.0F);
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
void MotorPairController::updateOutputsUsingPIDs(const AHRS::ahrs_data_t& ahrsDataNED)
{
#if defined(USE_BLACKBOX)
    _ahrsMessageQueue.SEND(imuDataNED);
#endif
    _ahrsMessageQueue.SEND_AHRS_DATA(ahrsDataNED);
    // AHRS orientation assumes (as is conventional) that pitch is around the X-axis, so convert.
    const float pitchAngleDegreesRaw = -ahrsDataNED.orientation.calculateRollDegrees();
    _motorMixer.setPitchAngleDegreesRaw(pitchAngleDegreesRaw); // the mixer will switch off the motors if the pitch angle exceeds the maximum pitch angle

    // calculate _outputs[OUTPUT_SPEED_DPS] and setpoints according to the control mode.
    // _speedDPS * _motorMaxSpeedDPS_reciprocal is in range [-1.0, 1.0]
    if (_controlMode == CONTROL_MODE_PARALLEL_PIDS) {
        _outputs[OUTPUT_SPEED_DPS] = -_PIDS[SPEED_PARALLEL_DPS].update(_speedDPS * _motorMaxSpeedDPS_reciprocal, ahrsDataNED.deltaT);
    } else if (_controlMode == CONTROL_MODE_SERIAL_PIDS) {
        const float speedOutput = _PIDS[SPEED_SERIAL_DPS].update(_speedDPS * _motorMaxSpeedDPS_reciprocal, ahrsDataNED.deltaT);
        // feed the speed update back into the pitchAngle PID and set _outputs[OUTPUT_SPEED_DPS] to zero
        _PIDS[PITCH_ANGLE_DEGREES].setSetpoint(speedOutput);
        _outputs[OUTPUT_SPEED_DPS] = 0.0F;
    } else if (_controlMode == CONTROL_MODE_POSITION) {
        updatePositionOutputs(ahrsDataNED.deltaT);
    }

    // calculate _outputs[PITCH_ANGLE_DEGREES]
    const float pitchAngleDegrees = pitchAngleDegreesRaw - _pitchBalanceAngleDegrees;
    const float pitchAngleDegreesDelta = pitchAngleDegrees - _PIDS[PITCH_ANGLE_DEGREES].getPreviousMeasurement();
    // Use the filtered value of pitchAngleDegreesDelta as input into the PID update.
    // This is beneficial because the DTerm is especially susceptible to noise (since it is the derivative of a noisy value).
    const float pitchAngleFilteredDTerm = _pitchAngleDTermFilter.filter(pitchAngleDegreesDelta);
    _outputs[PITCH_ANGLE_DEGREES] = -_PIDS[PITCH_ANGLE_DEGREES].updateDelta(pitchAngleDegrees, pitchAngleFilteredDTerm, ahrsDataNED.deltaT);

    // calculate _outputs[YAW_RATE_DPS]
    const float yawRateDPS = -ahrsDataNED.accGyroRPS.gyroRPS.z * Quaternion::radiansToDegrees;
    _outputs[YAW_RATE_DPS] = _PIDS[YAW_RATE_DPS].update(yawRateDPS, ahrsDataNED.deltaT);

    const VehicleControllerMessageQueue::queue_item_t queueItem {
        .throttle = _outputs[OUTPUT_SPEED_DPS],
        .roll   = _outputs[ROLL_ANGLE_DEGREES],
        .pitch  = _outputs[PITCH_ANGLE_DEGREES],
        .yaw = _outputs[YAW_RATE_DPS]
    };
    SIGNAL(queueItem);
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
    static constexpr float alpha = 0.9F;
    const float speedEstimate = MotorPairBase::clip((_motorMixer.getPowerLeft() + _motorMixer.getPowerRight()) * 0.5F, -1.0F, 1.0F) * _motorMaxSpeedDPS;
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

/*!
Task loop for the MotorPairController. Uses PID controllers to update the motor pair.

Setpoints are provided by the receiver(joystick), and inputs(process variables) come from the AHRS and the motor encoders.

There are three PIDs, a pitch PID, a speed PID, and a yawRate PID.
*/
/*!
Called from within the VehicleControllerTask when signalled that output data is available.
*/
void MotorPairController::outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem)
{
    ++_taskSignalledCount;
    if (_taskSignalledCount < _outputToMotorsDenominator) {
        return;
    }
    _taskSignalledCount = 0;

    updateMotorSpeedEstimates(deltaT);
    MotorPairMixer::commands_t commands = { // NOLINT(misc-const-correctness)
        .throttle = queueItem.throttle,
        .roll   = queueItem.roll,
        .pitch  = queueItem.pitch,
        .yaw    = queueItem.yaw
    };

    _motorMixer.outputToMotors(commands, deltaT, tickCount);
}
