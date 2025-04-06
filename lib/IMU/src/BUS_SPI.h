#pragma once

#include <cstddef>
#include <cstdint>

class SPIClass;


class BUS_SPI {
public:
    explicit BUS_SPI(uint8_t CS_pin);
public:
    uint8_t readRegister(uint8_t reg) const;
    bool readRegister(uint8_t reg, uint8_t* data, size_t length) const;
    bool readBytes(uint8_t* data, size_t length) const;
    bool readBytesWithTimeout(uint8_t* data, size_t length, uint8_t timeoutMs) const;
    uint8_t writeRegister(uint8_t reg, uint8_t data);
    uint8_t writeRegister(uint8_t reg, const uint8_t* data, size_t length);
    uint8_t writeBytes(const uint8_t* data, size_t length);
private:
#if defined(USE_SPI_ARDUINO)
    SPIClass& _spi;
#endif
    uint8_t _CS_pin;
};
