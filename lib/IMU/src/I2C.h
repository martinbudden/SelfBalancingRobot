#pragma once

#include <cstddef>
#include <cstdint>

class I2C {
public:
    I2C(uint8_t I2C_address, uint8_t SDA_pin, uint8_t SCL_pin);
public:
    uint8_t readByte(uint8_t reg) const;
    bool readBytes(uint8_t reg, uint8_t* data, size_t length) const;
    bool readBytes(uint8_t* data, size_t length) const;
    uint8_t writeByte(uint8_t reg, uint8_t data);
    uint8_t writeBytes(uint8_t reg, const uint8_t* data, size_t length);
    uint8_t writeBytes(const uint8_t* data, size_t length);
private:
    uint8_t _I2C_address;
    uint8_t _SDA_pin;
    uint8_t _SCL_pin;
    uint8_t _filler {0};
};
