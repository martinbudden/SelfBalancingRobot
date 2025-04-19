#include "BUS_SPI.h"

#if defined(FRAMEWORK_ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#elif defined(FRAMEWORK_PICO)
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include <array>
#endif

static constexpr uint8_t READ_BIT = 0x80;

#if defined(FRAMEWORK_PICO)
static inline void cs_select(uint8_t CS_pin) {
    asm volatile("nop \n nop \n nop");
    //gpio_put(uint8_t CS_pin, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(uint8_t CS_pin) {
    asm volatile("nop \n nop \n nop");
    //gpio_put(uint8_t CS_pin, 1);
    asm volatile("nop \n nop \n nop");
}
#endif

BUS_SPI::BUS_SPI(uint32_t frequency, uint8_t CS_pin) :
#if defined(FRAMEWORK_ARDUINO)
    BUS_SPI(frequency, CS_pin, 0, 0, 0)
#elif defined(FRAMEWORK_PICO)
    BUS_SPI(frequency, CS_pin, PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN)
#else
    BUS_SPI(frequency, CS_pin, 0, 0, 0)
#endif
{
}

BUS_SPI::BUS_SPI(uint32_t frequency, uint8_t CS_pin, uint8_t SCK_pin, uint8_t CIPO_pin, uint8_t COPI_pin) :
#if defined(FRAMEWORK_ARDUINO)
    _spi(SPI),
#elif defined(FRAMEWORK_PICO)
    _spi(spi_default),
#endif
    _frequency(frequency),
    _CS_pin(CS_pin),
    _SCK_pin(SCK_pin),
    _CIPO_pin(CIPO_pin),
    _COPI_pin(COPI_pin)
{
#if defined(FRAMEWORK_ARDUINO)
    _spi.begin();
    pinMode(_CS_pin, OUTPUT);
    digitalWrite(_CS_pin, HIGH);
#elif defined(FRAMEWORK_PICO)
    spi_init(_SPI, _frequency);
    gpio_set_function(_CIPO_pin, GPIO_FUNC_SPI);
    gpio_set_function(_SCK_pin, GPIO_FUNC_SPI);
    gpio_set_function(_COPI_pin, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(_CIPO_pin, _COPI_pin, _SCK_pin, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(_CS_pin);
    gpio_set_dir(_CS_pin, GPIO_OUT);
    gpio_put(_CS_pin, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(_CS_pin, "SPI CS"));
#else
#endif
}

uint8_t BUS_SPI::readRegister(uint8_t reg) const
{
#if defined(FRAMEWORK_ARDUINO)
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    _spi.transfer(reg | READ_BIT);
    const uint8_t ret = _spi.transfer(0);
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return ret;
#elif defined(FRAMEWORK_PICO)
    cs_select(_CS_pin);
    reg |= READ_BIT;
    spi_write_blocking(_SPI, &reg, 1);
    sleep_ms(10);
    uint8_t ret;
    spi_read_blocking(_SPI, 0, &ret, 1);
    cs_deselect(_CS_pin);
    return ret;
#else
    (void)reg;
    return 0;
#endif
}

uint8_t BUS_SPI::readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const
{
    (void)timeoutMs;
    return readRegister(reg);
}

bool BUS_SPI::readRegister(uint8_t reg, uint8_t* data, size_t length) const
{
#if defined(FRAMEWORK_ARDUINO)
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    _spi.transfer(reg | READ_BIT);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return true;
#elif defined(FRAMEWORK_PICO)
    cs_select(_CS_pin);
    reg |= READ_BIT;
    spi_write_blocking(_SPI, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(_SPI, 0, data, length);
    cs_deselect(_CS_pin);
#else
    (void)reg;
    *data = 0;
    (void)length;
    return false;
#endif
}

bool BUS_SPI::readBytes(uint8_t* data, size_t length) const
{
#if defined(FRAMEWORK_ARDUINO)
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return true;
#elif defined(FRAMEWORK_PICO)
    cs_select(_CS_pin);
    spi_read_blocking(_SPI, 0, data, length);
    cs_deselect(_CS_pin);
#else
    *data = 0;
    (void)length;
    return false;
#endif
}

bool BUS_SPI::readBytesWithTimeout(uint8_t* data, size_t length, uint8_t timeoutMs) const
{
    (void)timeoutMs;
    return readBytes(data, length);
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, uint8_t data)
{
#if defined(FRAMEWORK_ARDUINO)
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    _spi.transfer(reg);
    _spi.transfer(data);
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return 0;
#elif defined(FRAMEWORK_PICO)
    std::array<uint8_t, 2> buf = { reg & 0x7FU, data }; // remove read bit as this is a write
    cs_select(_CS_pin);
    spi_write_blocking(_SPI, &buf[0], 2);
    cs_deselect(_CS_pin);
    sleep_ms(10);
    return 0;
#else
    (void)reg;
    (void)data;
    return 0;
#endif
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(FRAMEWORK_ARDUINO)
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    _spi.transfer(reg);
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return 0;
#elif defined(FRAMEWORK_PICO)
    cs_select(_CS_pin);
    reg &= 0x7FU;
    spi_write_blocking(_SPI, &reg, 1);
    spi_write_blocking(_SPI, data, length);
    cs_deselect(_CS_pin);
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
#if defined(FRAMEWORK_ARDUINO)
    _spi.beginTransaction(SPISettings(_frequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_CS_pin, LOW);
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    digitalWrite(_CS_pin, HIGH);
    _spi.endTransaction();
    return 0;
#elif defined(FRAMEWORK_PICO)
    cs_select(_CS_pin);
    spi_write_blocking(_SPI, data, length);
    cs_deselect(_CS_pin);
    return 0;
#else
    (void)data;
    (void)length;
    return 0;
#endif
}
