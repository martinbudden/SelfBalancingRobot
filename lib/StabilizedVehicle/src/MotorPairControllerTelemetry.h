#pragma once

#include <PIDF.h>
#include <cstdint>

struct motor_pair_controller_telemetry_t {
    int32_t encoderLeft {0}; //!< value read from left motor encoder, raw
    int32_t encoderRight {0}; //!< value read from right motor encoder, raw
    int16_t encoderLeftDelta {0}; //!< difference between current left motor encoder value and previous value, raw
    int16_t encoderRightDelta {0}; //!< difference between current right motor encoder value and previous value, raw

    float motorMaxSpeedDPS {0};
    float speedLeftDPS {0}; //!< rotation speed of left motor, degrees per second
    float speedRightDPS {0}; //!< rotation speed of right motor, degrees per second
    float speedDPS_Filtered {0}; //!< speed calculated as the average of speedLeftDPS and speedRightDPS, then filtered

    float powerLeft {0}; //!< power value sent to left motor
    float powerRight {0}; //!< power value sent to right motor

    float pitchUpdate {0}; //!< pitch update value calculated by PID
    float speedUpdate {0}; //!< speed update value calculated by PID
    float positionUpdate {0}; //!< speed update value calculated by PID
    float yawRateUpdate {0}; //!< yawRate update value calculated by PID

    PIDF::error_t pitchError {0, 0, 0}; //!< P, I, and D errors calculated in pitch PID update
    PIDF::error_t speedError {0, 0, 0}; //!< P, I, and D errors calculated in speed PID update
    PIDF::error_t positionError {0, 0, 0}; //!< P, I, and D errors calculated in yawRate PID update
};
