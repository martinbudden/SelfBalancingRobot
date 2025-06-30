#include "BlackboxSelfBalancingRobot.h"

#include <BlackboxCallbacksBase.h>


// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#ifndef BLACKBOX_PRINT_HEADER_LINE
#define BLACKBOX_PRINT_HEADER_LINE(name, format, ...) case __COUNTER__: \
                                                headerPrintfHeaderLine(name, format, __VA_ARGS__); \
                                                break;
#define BLACKBOX_PRINT_HEADER_LINE_CUSTOM(...) case __COUNTER__: \
                                                    {__VA_ARGS__}; \
                                               break;
#endif
// NOLINTEND(cppcoreguidelines-macro-usage)


/*!
Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0.
Returns true iff transmission is complete, otherwise call again later to continue transmission.
*/
Blackbox::write_e BlackboxSelfBalancingRobot::writeSystemInformation()
{
    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    if (!headerReserveBufferSpace()) {
        return WRITE_NOT_COMPLETE;
    }

// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    switch (_xmitState.headerIndex) {
    BLACKBOX_PRINT_HEADER_LINE("Firmware type", "%s",                   "Cleanflight");
    BLACKBOX_PRINT_HEADER_LINE("Firmware revision", "%s %s (%s) %s",    "BetaFlight", "3.3.1", "611bc70f8", "REVOLT");
    BLACKBOX_PRINT_HEADER_LINE("Firmware date", "%s %s",                "Mar 21 2018", "03:14:05");
    //BLACKBOX_PRINT_HEADER_LINE("DeviceUID", "%08x%08x%08x",             U_ID_0, U_ID_1, U_ID_2);
#ifdef USE_BOARD_INFO
    BLACKBOX_PRINT_HEADER_LINE("Board information", "%s %s",            getManufacturerId(), getBoardName());
#endif
    BLACKBOX_PRINT_HEADER_LINE("Log start datetime", "%s",              "0000-01-01T00:00:00.000");
    BLACKBOX_PRINT_HEADER_LINE("Craft name", "%s",                      "TestCraft");
    BLACKBOX_PRINT_HEADER_LINE("I interval", "%d",                      blackboxIInterval);
    BLACKBOX_PRINT_HEADER_LINE("P interval", "%d",                      blackboxPInterval);
    BLACKBOX_PRINT_HEADER_LINE("P ratio", "%d",                         (uint16_t)(blackboxIInterval / blackboxPInterval));
    BLACKBOX_PRINT_HEADER_LINE("minthrottle", "%d",                     1070);
    BLACKBOX_PRINT_HEADER_LINE("maxthrottle", "%d",                     2000);
    BLACKBOX_PRINT_HEADER_LINE("gyro_scale", "%s",                      "0x3f800000");
    //BLACKBOX_PRINT_HEADER_LINE("gyro_scale","0x%x",                     BlackboxEncoder::castFloatBytesToInt(1.0F));
    BLACKBOX_PRINT_HEADER_LINE("motorOutput", "%d,%d",                  158,2047);
    //BLACKBOX_PRINT_HEADER_LINE("motor_kv", "%d",                        motorConfig()->kv);
#if defined(USE_ACC)
    BLACKBOX_PRINT_HEADER_LINE("acc_1G", "%u",                          acc.dev.acc_1G);
#endif

    BLACKBOX_PRINT_HEADER_LINE("vbat_scale", "%u",                      110);

    BLACKBOX_PRINT_HEADER_LINE("vbatcellvoltage", "%u,%u,%u",           33,35,43);
    BLACKBOX_PRINT_HEADER_LINE("vbatref", "%u",                         112);

    BLACKBOX_PRINT_HEADER_LINE("currentSensor", "%d,%d",                0, 235);
    BLACKBOX_PRINT_HEADER_LINE("looptime", "%d",                        _motorPairController.getTaskIntervalMicroSeconds());
    BLACKBOX_PRINT_HEADER_LINE("gyro_sync_denom", "%d",                 1);
    BLACKBOX_PRINT_HEADER_LINE("pid_process_denom", "%d",               1);

    BLACKBOX_PRINT_HEADER_LINE("thr_mid", "%d",                         0);
    BLACKBOX_PRINT_HEADER_LINE("thr_expo", "%d",                        0);
    //BLACKBOX_PRINT_HEADER_LINE("mixer_type", "%s",             lookupTableMixerType[mixerConfig()->mixer_type]);

    BLACKBOX_PRINT_HEADER_LINE("yawPID", "%d,%d,%d",                    _motorPairController.getPID_MSP(MotorPairController::YAW_RATE_DPS).kp,
                                                                        _motorPairController.getPID_MSP(MotorPairController::YAW_RATE_DPS).ki,
                                                                        _motorPairController.getPID_MSP(MotorPairController::YAW_RATE_DPS).kd);
    BLACKBOX_PRINT_HEADER_LINE("rollAnglePID", "%d,%d,%d",              _motorPairController.getPID_MSP(MotorPairController::ROLL_ANGLE_DEGREES).kp,
                                                                        _motorPairController.getPID_MSP(MotorPairController::ROLL_ANGLE_DEGREES).ki,
                                                                        _motorPairController.getPID_MSP(MotorPairController::ROLL_ANGLE_DEGREES).kd);
    BLACKBOX_PRINT_HEADER_LINE("pitchAnglePID", "%d,%d,%d",             _motorPairController.getPID_MSP(MotorPairController::PITCH_ANGLE_DEGREES).kp,
                                                                        _motorPairController.getPID_MSP(MotorPairController::PITCH_ANGLE_DEGREES).ki,
                                                                        _motorPairController.getPID_MSP(MotorPairController::PITCH_ANGLE_DEGREES).kd);
    BLACKBOX_PRINT_HEADER_LINE("ff_weight", "%d,%d,%d",                 _motorPairController.getPID_MSP(MotorPairController::ROLL_ANGLE_DEGREES).kf,
                                                                        _motorPairController.getPID_MSP(MotorPairController::PITCH_ANGLE_DEGREES).kf,
                                                                        _motorPairController.getPID_MSP(MotorPairController::YAW_RATE_DPS).kf);


    BLACKBOX_PRINT_HEADER_LINE("gyro_notch_hz", "%d,%d",                0,0);
    BLACKBOX_PRINT_HEADER_LINE("gyro_notch_cutoff", "%d,%d",            300,100);
    BLACKBOX_PRINT_HEADER_LINE("acc_lpf_hz", "%d",                      1000);
    BLACKBOX_PRINT_HEADER_LINE("acc_lpf_hz", "%d",                      1);
    BLACKBOX_PRINT_HEADER_LINE("gyro_cal_on_first_arm", "%d",           0);
    BLACKBOX_PRINT_HEADER_LINE("serialrx_provider", "%d",               3);
    BLACKBOX_PRINT_HEADER_LINE("use_unsynced_pwm", "%d",                0);
    BLACKBOX_PRINT_HEADER_LINE("motor_pwm_protocol", "%d",              6);
    BLACKBOX_PRINT_HEADER_LINE("motor_pwm_rate", "%d",                  480);
    BLACKBOX_PRINT_HEADER_LINE("motor_idle", "%d",                      550);
    BLACKBOX_PRINT_HEADER_LINE("debug_mode", "%d",                      0);
    BLACKBOX_PRINT_HEADER_LINE("features", "%d",                        541130760); //0x2041'0008
    default:
        return WRITE_COMPLETE;
    }
// NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg)

    ++_xmitState.headerIndex;
    return WRITE_NOT_COMPLETE;
}
