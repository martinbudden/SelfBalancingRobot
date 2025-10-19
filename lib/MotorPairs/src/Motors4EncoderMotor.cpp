#include "Motors4EncoderMotor.h"
#include <array>
#include <cmath>
#include <cstring>


// see https://github.com/m5stack/M5Module-4EncoderMotor/blob/main/src/M5Module4EncoderMotor.cpp
// and https://github.com/m5stack/M5Core2/blob/ede1d33798e6bfa1117a7a346176ed9d24e54178/examples/Module/4EncoderMotor/4EncoderMotor.ino


Motors4EncoderMotor::Motors4EncoderMotor(uint8_t SDA_pin, uint8_t SCL_pin, float encoderStepsPerRevolution) :
    MotorPairBase(encoderStepsPerRevolution, CANNOT_ACCURATELY_ESTIMATE_SPEED),
    _I2C(I2C_ADDRESS, BUS_I2C::i2c_pins_t{.sda=SDA_pin, .scl=SCL_pin, .irq=BUS_I2C::IRQ_NOT_SET})
{
    // cppcheck-suppress badBitmaskCheck
    _I2C.writeRegister(REGISTER_CONFIGURE | (MOTOR_LEFT << 4), NORMAL_MODE);

    _I2C.writeRegister(REGISTER_CONFIGURE | (MOTOR_RIGHT << 4), NORMAL_MODE);
}

void Motors4EncoderMotor::readEncoder()
{
    std::array<uint8_t, 4> data;

    i2cSemaphoreTake();

    // cppcheck-suppress badBitmaskCheck
    _I2C.readRegister(REGISTER_ENCODER | (MOTOR_LEFT << 2), &data[0], 4);
    _leftEncoder =   ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
    _I2C.readRegister(REGISTER_ENCODER | (MOTOR_RIGHT << 2), &data[0], 4);
    _rightEncoder = -((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);

    // this speed value is very coarse and not useful for PID control
    // cppcheck-suppress badBitmaskCheck
    _leftSpeed = _I2C.readRegister(REGISTER_SPEED | MOTOR_LEFT);
    _rightSpeed = _I2C.readRegister(REGISTER_SPEED | MOTOR_RIGHT);

    i2cSemaphoreGive();
}

void Motors4EncoderMotor::setPower(float leftPower, float rightPower)
{
    const float leftClipped = clip(leftPower, -1.0F, 1.0F);
    const float rightClipped = clip(rightPower, -1.0F, 1.0F);
    // set signs so positive power moves motor in a forward direction
    const auto leftOutput =  static_cast<int8_t>( std::roundf(leftClipped * MAX_POWER)); // NOLINT(hicpp-use-auto,modernize-use-auto)
    const auto rightOutput = static_cast<int8_t>(-std::roundf(rightClipped * MAX_POWER));

    i2cSemaphoreTake();
    // cppcheck-suppress badBitmaskCheck
    _I2C.writeRegister(REGISTER_PWM_DUTY | MOTOR_LEFT, leftOutput);
    _I2C.writeRegister(REGISTER_PWM_DUTY | MOTOR_RIGHT, rightOutput);
    i2cSemaphoreGive();
}

float Motors4EncoderMotor::getCurrent() const
{
    std::array<uint8_t, 4> data;

    i2cSemaphoreTake();
    _I2C.readRegister(REGISTER_CURRENT, &data[0], sizeof(data));
    i2cSemaphoreGive();

    float current;
    memcpy(&current, &data[0], sizeof(data));

    return current;
}

float Motors4EncoderMotor::getVoltage() const
{
    i2cSemaphoreTake();
    const uint8_t v = _I2C.readRegister(REGISTER_ADC_8_BIT);
    i2cSemaphoreGive();

    const float voltage = static_cast<float>(v) / 255.0F * 3.3F / 0.16F;

    return voltage;
}
