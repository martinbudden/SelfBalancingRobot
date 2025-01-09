#pragma once

#include "MotorPairBase.h"

#include <CAN_config.h>
#include <ESP32CAN.h>


class MotorsPwrCAN final : public MotorPairBase {
public:
    MotorsPwrCAN();
public:
    virtual void readEncoder() override;
    virtual void setPower(float leftPower, float rightPower) override;
private:
    enum { ENCODER_STEPS_PER_REVOLUTION = 1024 };
private:
    CAN_frame_t _rxFrame {};
    CAN_frame_t _txFrame {};
};
