#pragma once

#include "I2C.h"
#include "MotorPairBase.h"
#include <stddef.h>

class Motors4EncoderMotor final : public MotorPairBase {
public:
    Motors4EncoderMotor(uint8_t SDA_pin, uint8_t SCL_pin, float encoderStepsPerRevolution);
public:
    virtual void readEncoder() override;
    virtual void setPower(float leftPower, float rightPower) override;
    float getCurrent() const;
    float getVoltage() const;
private:
    enum { MIN_POWER = -127, MAX_POWER = 127 };

    static constexpr uint8_t I2C_ADDRESS = 0x24;

    enum : uint8_t { MOTOR_0 = 0, MOTOR_1 = 1, MOTOR_2 = 2, MOTOR_3 = 3 };

    enum : uint8_t { REGISTER_PWM_DUTY   = 0x20 };
    enum : uint8_t { REGISTER_ENCODER    = 0x30 };
    enum : uint8_t { REGISTER_SPEED      = 0x40 };
    enum : uint8_t { REGISTER_CONFIGURE  = 0x50 };
    enum : uint8_t { REGISTER_CURRENT    = 0x90 };
    enum : uint8_t { REGISTER_ADC_8_BIT  = 0xA0 };
    enum : uint8_t { REGISTER_ADC_12_BIT = 0xB0 };

    enum : uint8_t { NORMAL_MODE = 0x00, POSITION_MODE = 0x01, SPEED_MODE = 0x02 };

    // map left and right motors to MOTOR_0 and MOTOR_1
    enum { MOTOR_LEFT = MOTOR_0, MOTOR_RIGHT = MOTOR_1 };
private:
    I2C _I2C;
};
