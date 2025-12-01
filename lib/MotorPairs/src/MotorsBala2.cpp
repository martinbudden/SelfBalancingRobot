#include "MotorsBala2.h"
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#elif defined(M5_STACK)
#include <M5Stack.h>
#endif
#include <array>
#include <cmath>


MotorsBala2::MotorsBala2(uint8_t SDA_pin, uint8_t SCL_pin) :
    MotorPairBase(ENCODER_STEPS_PER_REVOLUTION, CANNOT_ACCURATELY_ESTIMATE_SPEED)
{
    (void)SDA_pin;
    (void)SCL_pin;
}

void MotorsBala2::readEncoder()
{
    std::array<uint8_t, 8> data {};

    i2cSemaphoreTake();
#if defined(M5_UNIFIED)
    M5.Ex_I2C.readRegister(I2C_ADDRESS, REGISTER_ENCODER, &data[0], sizeof(data), I2C_FREQUENCY);
#elif defined(M5_STACK)
    M5.I2C.readBytes(I2C_ADDRESS, REGISTER_ENCODER, 8, &data[0]);
#endif
    i2cSemaphoreGive();

    _leftEncoder = (data[0] << 24U) | (data[1] << 16U) | (data[2] << 8U) | data[3];
    _rightEncoder = (data[4] << 24U) | (data[5] << 16U) | (data[6] << 8U) | data[7];
}

void MotorsBala2::setPower(float leftPower, float rightPower)
{
    const float leftClipped = clip(leftPower, -1.0F, 1.0F);
    const float rightClipped = clip(rightPower, -1.0F, 1.0F);
    // set signs so positive power moves motor in a forward direction
    const auto left = static_cast<int16_t>(-roundf(leftClipped * MAX_POWER));
    const auto right = static_cast<int16_t>(-roundf(rightClipped * MAX_POWER));

    // NOLINTBEGIN(hicpp-signed-bitwise)
#if defined(M5_STACK)
    uint8_t data[4] = {
#else
    const std::array<uint8_t, 4> data = {
#endif
        static_cast<uint8_t>(left >> 8),
        static_cast<uint8_t>(left & 0xFF),
        static_cast<uint8_t>(right >> 8),
        static_cast<uint8_t>(right & 0xFF)
    };
    // NOLINTEND(hicpp-signed-bitwise)

    i2cSemaphoreTake();
#if defined(M5_UNIFIED)
    M5.Ex_I2C.writeRegister(I2C_ADDRESS, REGISTER_SPEED, &data[0], sizeof(data), I2C_FREQUENCY);
#elif defined(M5_STACK)
    M5.I2C.writeBytes(I2C_ADDRESS, REGISTER_SPEED, &data[0], 4);
#else
    (void)data;
#endif
    i2cSemaphoreGive();
}
