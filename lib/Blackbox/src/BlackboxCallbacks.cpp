#include "BlackboxCallbacks.h"
#include "AHRS_MessageQueue.h"

#include <AHRS.h>
#include <Blackbox.h>
#include <Cockpit.h>
#include <MotorPairController.h>
#include <ReceiverBase.h>
#include <cmath>


bool BlackboxCallbacks::isArmed() const
{
    // ARMING_FLAG(ARMED)
    return _motorPairController.motorsIsOn();
}

bool BlackboxCallbacks::areMotorsRunning() const
{
    return _motorPairController.motorsIsOn();
}

bool BlackboxCallbacks::isBlackboxRcModeActive() const
{
    // IS_RC_MODE_ACTIVE(BOX_BLACKBOX)
    return true;
};

bool BlackboxCallbacks::isBlackboxModeActivationConditionPresent() const
{
    //isModeActivationConditionPresent(BOX_BLACKBOX);
    return true;
}

uint32_t BlackboxCallbacks::getArmingBeepTimeMicroseconds() const
{
    return 0;
}

uint32_t BlackboxCallbacks::rcModeActivationMask() const
{
    return 0;
}

void BlackboxCallbacks::loadSlowState(blackboxSlowState_t& slowState)
{
    //memcpy(&slow->flightModeFlags, &_rcModeActivationMask, sizeof(slow->flightModeFlags)); //was flightModeFlags;
    slowState.flightModeFlags = 0;//!!_motorPairController.getFlightModeFlags();
    slowState.stateFlags = 0; // this is GPS state
    slowState.failsafePhase = static_cast<uint8_t>(_cockpit.getFailsafePhase());
    //slowState.rxSignalReceived = _receiver.isRxReceivingSignal();
    slowState.rxSignalReceived = (slowState.failsafePhase == Cockpit::FAILSAFE_IDLE);
    slowState.rxFlightChannelsValid = (slowState.failsafePhase == Cockpit::FAILSAFE_IDLE);
}

void BlackboxCallbacks::loadMainState(blackboxMainState_t& mainState, uint32_t currentTimeUs)
{
    (void)currentTimeUs;

    const AHRS::ahrs_data_t ahrsData = _messageQueue.getReceivedAHRS_Data();

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

    static constexpr float radiansToDegrees {180.0F / static_cast<float>(M_PI)};
    static constexpr float gyroScale {radiansToDegrees * 10.0F};

    mainState.gyroADC[0] = static_cast<int16_t>(std::lroundf(ahrsData.accGyroRPS.gyroRPS.x * gyroScale));
    mainState.gyroADC[1] = static_cast<int16_t>(std::lroundf(ahrsData.accGyroRPS.gyroRPS.y * gyroScale));
    mainState.gyroADC[2] = static_cast<int16_t>(std::lroundf(ahrsData.accGyroRPS.gyroRPS.z * gyroScale));
    mainState.gyroUnfiltered[0] = static_cast<int16_t>(std::lroundf(ahrsData.gyroRPS_unfiltered.x * gyroScale));
    mainState.gyroUnfiltered[1] = static_cast<int16_t>(std::lroundf(ahrsData.gyroRPS_unfiltered.y * gyroScale));
    mainState.gyroUnfiltered[2] = static_cast<int16_t>(std::lroundf(ahrsData.gyroRPS_unfiltered.z * gyroScale));
    // just truncate for acc
    mainState.accADC[0] = static_cast<int16_t>(ahrsData.accGyroRPS.acc.x * 4096);
    mainState.accADC[1] = static_cast<int16_t>(ahrsData.accGyroRPS.acc.y * 4096);
    mainState.accADC[2] = static_cast<int16_t>(ahrsData.accGyroRPS.acc.z * 4096);

    for (int ii = 0; ii < blackboxMainState_t::XYZ_AXIS_COUNT; ++ii) {
        const auto pidIndex = static_cast<MotorPairController::pid_index_e>(ii);
        const PIDF& pid = _motorPairController.getPID(pidIndex);
        const PIDF::error_t pidError = pid.getError();
        mainState.axisPID_P[ii] = static_cast<int32_t>(std::lroundf(pidError.P));
        mainState.axisPID_I[ii] = static_cast<int32_t>(std::lroundf(pidError.I));
        mainState.axisPID_D[ii] = static_cast<int32_t>(std::lroundf(pidError.D));
        mainState.axisPID_S[ii] = static_cast<int32_t>(std::lroundf(pidError.S));
        mainState.axisPID_K[ii] = static_cast<int32_t>(std::lroundf(pidError.K));
        mainState.setpoint[ii] = static_cast<int16_t>(std::lroundf(pid.getSetpoint()));
#if defined(USE_MAG)
        mainState.magADC[ii] = static_cast<int16_t>(mag.magADC.v[ii]);
#endif
    }

    // interval [1000,2000] for THROTTLE and [-500,+500] for ROLL/PITCH/YAW
    const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM(); // returns controls in range [1000, 2000]
    mainState.rcCommand[0] = static_cast<int16_t>(controls.roll - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[1] = static_cast<int16_t>(controls.pitch - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[2] = static_cast<int16_t>(controls.yaw - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rcCommand[3] = static_cast<int16_t>(controls.throttle);

    // log the final throttle value used in the mixer
    mainState.setpoint[3] = static_cast<int16_t>(_motorPairController.getMixerThrottleCommand() * 1000.0F);

    for (int ii = 0; ii < blackboxMainState_t::DEBUG_VALUE_COUNT; ++ii) {
        mainState.debug[ii] = static_cast<uint16_t>(_ahrs.getTimeChecksMicroseconds(ii));
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
