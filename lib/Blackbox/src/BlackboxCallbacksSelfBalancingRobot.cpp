#include "BlackboxCallbacksSelfBalancingRobot.h"

#include <AHRS.h>
#include <Blackbox.h>
#include <MotorPairController.h>
#include <ReceiverBase.h>
#include <cmath>


bool BlackboxCallbacksSelfBalancingRobot::isArmed() const
{ 
    // ARMING_FLAG(ARMED)
    return _motorPairController.motorsIsOn();
}

bool BlackboxCallbacksSelfBalancingRobot::areMotorsRunning() const
{
    return _motorPairController.motorsIsOn();
}

bool BlackboxCallbacksSelfBalancingRobot::isBlackboxRcModeActive() const
{
    // IS_RC_MODE_ACTIVE(BOX_BLACKBOX)
    return true; 
};

bool BlackboxCallbacksSelfBalancingRobot::isBlackboxModeActivationConditionPresent() const
{ 
    //isModeActivationConditionPresent(BOX_BLACKBOX);
    return true;
}

uint32_t BlackboxCallbacksSelfBalancingRobot::getArmingBeepTimeMicroSeconds() const
{
    return 0;
}

uint32_t BlackboxCallbacksSelfBalancingRobot::rcModeActivationMask() const
{
    return 0;
}

void BlackboxCallbacksSelfBalancingRobot::loadSlowStateFromFlightController(blackboxSlowState_t& slowState)
{
    //memcpy(&slow->flightModeFlags, &_rcModeActivationMask, sizeof(slow->flightModeFlags)); //was flightModeFlags;
    slowState.flightModeFlags = 0;//!!_motorPairController.getFlightModeFlags();
    slowState.stateFlags = 0; // this is GPS state
    slowState.failsafePhase = _motorPairController.getFailsafePhase();
    //slowState.rxSignalReceived = _receiver.isRxReceivingSignal();
    slowState.rxSignalReceived = true;
    slowState.rxFlightChannelsValid = true;
}

void BlackboxCallbacksSelfBalancingRobot::loadMainStateFromFlightController(blackboxMainState_t& mainState)
{
    const AHRS::data_t ahrsData = _ahrs.getAhrsDataForInstrumentationUsingLock();

    loadMainStateFromFlightController(mainState, ahrsData.gyroRPS, ahrsData.gyroRPS_unfiltered, ahrsData.acc);
}

void BlackboxCallbacksSelfBalancingRobot::loadMainStateFromFlightController(blackboxMainState_t& mainState, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc)
{
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

    constexpr float radiansToDegrees {180.0 / M_PI};
    constexpr float gyroScale {radiansToDegrees * 10.0F};

    mainState.gyroADC[0] = lrintf(gyroRPS.x * gyroScale);
    mainState.gyroADC[1] = lrintf(gyroRPS.y * gyroScale);
    mainState.gyroADC[2] = lrintf(gyroRPS.z * gyroScale);
    mainState.gyroUnfiltered[0] = lrintf(gyroRPS_unfiltered.x * gyroScale);
    mainState.gyroUnfiltered[1] = lrintf(gyroRPS_unfiltered.y * gyroScale);
    mainState.gyroUnfiltered[2] = lrintf(gyroRPS_unfiltered.z * gyroScale);
    mainState.accADC[0] = lrintf(acc.x * 4096);
    mainState.accADC[1] = lrintf(acc.y * 4096);
    mainState.accADC[2] = lrintf(acc.z * 4096);

    for (int ii = 0; ii < blackboxMainState_t::XYZ_AXIS_COUNT; ++ii) {
        const auto pidIndex = static_cast<MotorPairController::pid_index_e>(ii);
        const PIDF& pid = _motorPairController.getPID(pidIndex);
        const PIDF::error_t pidError = pid.getError();
        mainState.axisPID_P[ii] = lrintf(pidError.P);
        mainState.axisPID_I[ii] = lrintf(pidError.I);
        mainState.axisPID_D[ii] = lrintf(pidError.D);
        mainState.axisPID_F[ii] = lrintf(pidError.F);
        mainState.setpoint[ii] = lrintf(pid.getSetpoint());
#if defined(USE_MAG)
        mainState.magADC[i] = lrintf(mag.magADC.v[i]);
#endif
    }

    const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM(); // returns controls in PWM range, [1000, 2000]
    mainState.rcCommand[0] = controls.throttleStick;
    mainState.rcCommand[1] = controls.rollStick - ReceiverBase::CHANNEL_MIDDLE;
    mainState.rcCommand[2] = controls.pitchStick - ReceiverBase::CHANNEL_MIDDLE;
    mainState.rcCommand[3] = controls.yawStick - ReceiverBase::CHANNEL_MIDDLE;

    // log the final throttle value used in the mixer
    mainState.setpoint[3] = lrintf(_motorPairController.getMixerThrottle() * 1000.0F);

    for (int ii = 0; ii < blackboxMainState_t::DEBUG_VALUE_COUNT; ++ii) {
        mainState.debug[ii] = static_cast<uint16_t>(_ahrs.getTimeChecksMicroSeconds(ii));
    }
    const motor_pair_controller_telemetry_t telemetry = _motorPairController.getTelemetryData();
    mainState.motor[0] = lrintf(telemetry.powerLeft);
    mainState.motor[1] = lrintf(telemetry.powerRight);

    constexpr float DPS_to_RPM = 60.0F / 360.0F;
    mainState.erpm[0] = lrintf(telemetry.speedLeftDPS * DPS_to_RPM);
    mainState.erpm[0] = lrintf(telemetry.speedRightDPS * DPS_to_RPM);

    mainState.vbatLatest = 0; //getBatteryVoltageLatest();
    mainState.amperageLatest = 0; //getAmperageLatest();

#if defined(USE_BARO)
    mainState.baroAlt = baro.altitude;
#endif

#if defined(USE_RANGEFINDER)
    // Store the raw sonar value without applying tilt correction
    mainState.surfaceRaw = rangefinderGetLatestAltitude();
#endif

    mainState.rssi = 0;//getRssi();

#if defined(USE_SERVOS)
    for (unsigned i = 0; i < ARRAYLEN(mainState.servo); i++) {
        mainState.servo[i] = servo[i];
    }
#endif
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}
