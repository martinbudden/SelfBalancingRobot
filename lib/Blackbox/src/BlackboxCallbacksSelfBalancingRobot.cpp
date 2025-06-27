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

void BlackboxCallbacksSelfBalancingRobot::loadSlowStateFromFlightController(blackboxSlowState_t& slow)
{
    //memcpy(&slow->flightModeFlags, &_rcModeActivationMask, sizeof(slow->flightModeFlags)); //was flightModeFlags;
    slow.flightModeFlags = 0;//!!_motorPairController.getFlightModeFlags();
    slow.stateFlags = 0; // this is GPS state
    slow.failsafePhase = _motorPairController.getFailsafePhase();
    //slow.rxSignalReceived = _receiver.isRxReceivingSignal();
    slow.rxSignalReceived = true;
    slow.rxFlightChannelsValid = true;
}

void BlackboxCallbacksSelfBalancingRobot::loadMainStateFromFlightController(blackboxMainState_t& blackboxCurrent, float blackboxHighResolutionScale)
{
    const AHRS::data_t ahrsData = _ahrs.getAhrsDataForInstrumentationUsingLock();

    loadMainStateFromFlightController(blackboxCurrent, blackboxHighResolutionScale, ahrsData.gyroRPS, ahrsData.gyroRPS_unfiltered, ahrsData.acc);
}

void BlackboxCallbacksSelfBalancingRobot::loadMainStateFromFlightController(blackboxMainState_t& blackboxCurrent, float blackboxHighResolutionScale, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc)
{
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

    constexpr float radiansToDegrees {180.0 / M_PI};
    blackboxCurrent.gyroADC[0] = lrintf(gyroRPS.x * radiansToDegrees * blackboxHighResolutionScale);
    blackboxCurrent.gyroADC[1] = lrintf(gyroRPS.y * radiansToDegrees * blackboxHighResolutionScale);
    blackboxCurrent.gyroADC[2] = lrintf(gyroRPS.z * radiansToDegrees * blackboxHighResolutionScale);
    blackboxCurrent.gyroUnfiltered[0] = lrintf(gyroRPS_unfiltered.x * radiansToDegrees * blackboxHighResolutionScale);
    blackboxCurrent.gyroUnfiltered[1] = lrintf(gyroRPS_unfiltered.y * radiansToDegrees * blackboxHighResolutionScale);
    blackboxCurrent.gyroUnfiltered  [2] = lrintf(gyroRPS_unfiltered.z * radiansToDegrees * blackboxHighResolutionScale);
    blackboxCurrent.accADC[0] = lrintf(acc.x);
    blackboxCurrent.accADC[1] = lrintf(acc.y);
    blackboxCurrent.accADC[2] = lrintf(acc.z);

    for (int ii = 0; ii < blackboxMainState_t::XYZ_AXIS_COUNT; ++ii) {
        const auto pidIndex = static_cast<MotorPairController::pid_index_e>(ii);
        const PIDF& pid = _motorPairController.getPID(pidIndex);
        const PIDF::error_t pidError = pid.getError();
        blackboxCurrent.axisPID_P[ii] = lrintf(pidError.P);
        blackboxCurrent.axisPID_I[ii] = lrintf(pidError.I);
        blackboxCurrent.axisPID_D[ii] = lrintf(pidError.D);
        blackboxCurrent.axisPID_F[ii] = lrintf(pidError.F);
        blackboxCurrent.setpoint[ii] = lrintf(pid.getSetpoint() * blackboxHighResolutionScale);
#if defined(USE_MAG)
        blackboxCurrent.magADC[i] = lrintf(mag.magADC.v[i]);
#endif
    }

    const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM(); // returns controls in PWM range, [1000, 2000]
    blackboxCurrent.rcCommand[0] = controls.throttleStick;
    blackboxCurrent.rcCommand[1] = controls.rollStick - ReceiverBase::CHANNEL_MIDDLE;
    blackboxCurrent.rcCommand[2] = controls.pitchStick - ReceiverBase::CHANNEL_MIDDLE;
    blackboxCurrent.rcCommand[3] = controls.yawStick - ReceiverBase::CHANNEL_MIDDLE;

    // log the final throttle value used in the mixer
    blackboxCurrent.setpoint[3] = lrintf(_motorPairController.getMixerThrottle() * 1000.0F);

    for (int ii = 0; ii < blackboxMainState_t::DEBUG16_VALUE_COUNT; ++ii) {
        blackboxCurrent.debug[ii] = static_cast<uint16_t>(_ahrs.getTimeChecksMicroSeconds(ii));
    }
    const motor_pair_controller_telemetry_t telemetry = _motorPairController.getTelemetryData();
    blackboxCurrent.motor[0] = lrintf(telemetry.powerLeft);
    blackboxCurrent.motor[1] = lrintf(telemetry.powerRight);

    constexpr float DPS_to_RPM = 60.0F / 360.0F;
    blackboxCurrent.erpm[0] = lrintf(telemetry.speedLeftDPS * DPS_to_RPM);
    blackboxCurrent.erpm[0] = lrintf(telemetry.speedRightDPS * DPS_to_RPM);

    blackboxCurrent.vbatLatest = 0; //getBatteryVoltageLatest();
    blackboxCurrent.amperageLatest = 0; //getAmperageLatest();

#if defined(USE_BARO)
    blackboxCurrent.baroAlt = baro.altitude;
#endif

#if defined(USE_RANGEFINDER)
    // Store the raw sonar value without applying tilt correction
    blackboxCurrent.surfaceRaw = rangefinderGetLatestAltitude();
#endif

    blackboxCurrent.rssi = 0;//getRssi();

#if defined(USE_SERVOS)
    for (unsigned i = 0; i < ARRAYLEN(blackboxCurrent.servo); i++) {
        blackboxCurrent.servo[i] = servo[i];
    }
#endif
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}
