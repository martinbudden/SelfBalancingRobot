#include "BlackboxCallbacksSelfBalancingRobot.h"

#include <AHRS.h>
#include <Blackbox.h>
#include <MotorPairController.h>
#include <RadioController.h>
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

void BlackboxCallbacksSelfBalancingRobot::loadSlowState(blackboxSlowState_t& slowState)
{
    //memcpy(&slow->flightModeFlags, &_rcModeActivationMask, sizeof(slow->flightModeFlags)); //was flightModeFlags;
    slowState.flightModeFlags = 0;//!!_motorPairController.getFlightModeFlags();
    slowState.stateFlags = 0; // this is GPS state
    slowState.failsafePhase = static_cast<uint8_t>(_radioController.getFailsafePhase());
    //slowState.rxSignalReceived = _receiver.isRxReceivingSignal();
    slowState.rxSignalReceived = (slowState.failsafePhase == RadioController::FAILSAFE_IDLE);
    slowState.rxFlightChannelsValid = (slowState.failsafePhase == RadioController::FAILSAFE_IDLE);
}

void BlackboxCallbacksSelfBalancingRobot::loadMainState(blackboxMainState_t& mainState, uint32_t currentTimeUs)
{

#if true
    mainState.time = currentTimeUs;
    const AHRS::data_t ahrsData = _ahrs.getAhrsDataForInstrumentationUsingLock();
    const xyz_t gyroRPS = ahrsData.gyroRPS;
    const xyz_t gyroRPS_unfiltered = ahrsData.gyroRPS_unfiltered;
    const xyz_t acc = ahrsData.acc;
#else
    (void)currentTimeUs;
    mainState.time = _queueItem.timeMicroSeconds;
    const xyz_t gyroRPS = _queueItem.gyroRPS;
    const xyz_t gyroRPS_unfiltered = _queueItem.gyroRPS_unfiltered;
    const xyz_t acc = _queueItem.acc;
#endif
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

    constexpr float radiansToDegrees {180.0F / static_cast<float>(M_PI)};
    constexpr float gyroScale {radiansToDegrees * 10.0F};

    mainState.gyroADC[0] = static_cast<int16_t>(std::lroundf(gyroRPS.x * gyroScale));
    mainState.gyroADC[1] = static_cast<int16_t>(std::lroundf(gyroRPS.y * gyroScale));
    mainState.gyroADC[2] = static_cast<int16_t>(std::lroundf(gyroRPS.z * gyroScale));
    mainState.gyroUnfiltered[0] = static_cast<int16_t>(std::lroundf(gyroRPS_unfiltered.x * gyroScale));
    mainState.gyroUnfiltered[1] = static_cast<int16_t>(std::lroundf(gyroRPS_unfiltered.y * gyroScale));
    mainState.gyroUnfiltered[2] = static_cast<int16_t>(std::lroundf(gyroRPS_unfiltered.z * gyroScale));
    // just truncate for gyro
    mainState.accADC[0] = static_cast<int16_t>(acc.x * 4096);
    mainState.accADC[1] = static_cast<int16_t>(acc.y * 4096);
    mainState.accADC[2] = static_cast<int16_t>(acc.z * 4096);


    for (int ii = 0; ii < blackboxMainState_t::XYZ_AXIS_COUNT; ++ii) {
        const auto pidIndex = static_cast<MotorPairController::pid_index_e>(ii);
        const PIDF& pid = _motorPairController.getPID(pidIndex);
        const PIDF::error_t pidError = pid.getError();
        mainState.axisPID_P[ii] = std::lroundf(pidError.P);
        mainState.axisPID_I[ii] = std::lroundf(pidError.I);
        mainState.axisPID_D[ii] = std::lroundf(pidError.D);
        mainState.axisPID_F[ii] = std::lroundf(pidError.F);
        mainState.setpoint[ii] = static_cast<int16_t>(std::lroundf(pid.getSetpoint()));
#if defined(USE_MAG)
        mainState.magADC[ii] = static_cast<int16_t>(mag.magADC.v[ii]);
#endif
    }

    // interval [1000,2000] for THROTTLE and [-500,+500] for ROLL/PITCH/YAW
    const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM(); // returns controls in range [1000, 2000]
    mainState.rcCommand[0] = static_cast<int16_t>(controls.rollStick - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[1] = static_cast<int16_t>(controls.pitchStick - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[2] = static_cast<int16_t>(controls.yawStick - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[3] = controls.throttleStick;

    // log the final throttle value used in the mixer
    mainState.setpoint[3] = static_cast<int16_t>(_motorPairController.getMixerThrottle() * 1000.0F);

    for (int ii = 0; ii < blackboxMainState_t::DEBUG_VALUE_COUNT; ++ii) {
        mainState.debug[ii] = static_cast<uint16_t>(_ahrs.getTimeChecksMicroSeconds(ii));
    }
    const motor_pair_controller_telemetry_t telemetry = _motorPairController.getTelemetryData();
    mainState.motor[0] = static_cast<int16_t>(std::lroundf(telemetry.powerLeft));
    mainState.motor[1] = static_cast<int16_t>(telemetry.powerRight);

    constexpr float DPS_to_RPM = 60.0F / 360.0F;
    mainState.erpm[0] = static_cast<int16_t>(std::lroundf(telemetry.speedLeftDPS * DPS_to_RPM));
    mainState.erpm[1] = static_cast<int16_t>(std::lroundf(telemetry.speedRightDPS * DPS_to_RPM));

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
