#pragma once

#include <cstddef>
#include <cstdint>

class TwoWire;


class BUS_I2C {
public:
    BUS_I2C(uint8_t I2C_address, uint8_t SDA_pin, uint8_t SCL_pin);
public:
    uint8_t readRegister(uint8_t reg) const;
    bool readRegister(uint8_t reg, uint8_t* data, size_t length) const;
    bool readBytes(uint8_t* data, size_t length) const;
    bool readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const;
    uint8_t writeRegister(uint8_t reg, uint8_t data);
    uint8_t writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    uint8_t writeBytes(const uint8_t* data, size_t length);
private:
#if defined(USE_I2C_ARDUINO)
    TwoWire& _wire;
#endif
    uint8_t _I2C_address;
    uint8_t _SDA_pin;
    uint8_t _SCL_pin;
    uint8_t _filler {0};
};
