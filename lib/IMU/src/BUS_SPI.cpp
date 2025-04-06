#if defined(USE_SPI)

#include "BUS_SPI.h"
#if defined(USE_SPI_ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#endif

static constexpr uint8_t READ_BIT = 0x80;
static constexpr uint32_t SPEED_20_MHz = 20000000;


BUS_SPI::BUS_SPI(uint8_t CS_pin) :
#if defined(USE_SPI_ARDUINO)
    _spi(SPI),
#endif
    _CS_pin(CS_pin)
{
#if defined(USE_SPI_ARDUINO)
    _spi.begin();
#else
#endif
}

uint8_t BUS_SPI::readRegister(uint8_t reg) const
{
#if defined(USE_SPI_ARDUINO)
    _spi.beginTransaction(SPISettings(SPEED_20_MHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    _spi.transfer(reg | READ_BIT);
    const uint8_t ret = _spi.transfer(0);
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return ret;
#else
    (void)reg;
    return 0;
#endif
}

bool BUS_SPI::readRegister(uint8_t reg, uint8_t* data, size_t length) const
{
#if defined(USE_SPI_ARDUINO)
    _spi.beginTransaction(SPISettings(SPEED_20_MHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    _spi.transfer(reg | READ_BIT);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return true;
#else
    (void)reg;
    (void)data;
    (void)length;
    return false;
#endif
}

bool BUS_SPI::readBytes(uint8_t* data, size_t length) const
{
#if defined(USE_SPI_ARDUINO)
    _spi.beginTransaction(SPISettings(SPEED_20_MHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return true;
#else
    (void)data;
    (void)length;
    return false;
#endif
}

bool BUS_SPI::readBytesWithTimeout(uint8_t* data, size_t length, uint8_t timeoutMs) const
{
#if defined(USE_SPI_ARDUINO)
    (void)timeoutMs;
    _spi.beginTransaction(SPISettings(SPEED_20_MHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return true;
#else
    (void)data;
    (void)length;
    (void)timeoutMs;
#endif
    return false;
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, uint8_t data)
{
#if defined(USE_SPI_ARDUINO)
    _spi.beginTransaction(SPISettings(SPEED_20_MHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    _spi.transfer(reg);
    _spi.transfer(data);
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return 0;
#else
    (void)reg;
    (void)data;
    return 0;
#endif
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(USE_SPI_ARDUINO)
    _spi.beginTransaction(SPISettings(SPEED_20_MHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    _spi.transfer(reg);
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return 0;
#else
    (void)reg;
    (void)data;
    (void)length;
    return 0;
#endif
}

uint8_t BUS_SPI::writeBytes(const uint8_t* data, size_t length)
{
#if defined(USE_SPI_ARDUINO)
    _spi.beginTransaction(SPISettings(SPEED_20_MHz, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return 0;
#else
    (void)data;
    (void)length;
    return 0;
#endif
}
#endif // USE_SPI
