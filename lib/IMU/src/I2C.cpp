#if defined(USE_I2C)

#include "I2C.h"
#if defined(I2C_ARDUINO_WIRE)
#include <Wire.h>
#endif


I2C::I2C(uint8_t I2C_address, uint8_t SDA_pin, uint8_t SCL_pin) :
#if defined(I2C_ARDUINO_WIRE)
    _wire(Wire),
#endif
    _I2C_address(I2C_address),
    _SDA_pin(SDA_pin),
    _SCL_pin(SCL_pin)
{
#if defined(I2C_ARDUINO_WIRE)
#if defined(USE_I2C_BEGIN_2_PARAMETERS)
    _wire.begin(SDA_pin, SCL_pin);
#else
    _wire.begin();
#endif
#endif
}

uint8_t I2C::readRegister(uint8_t reg) const
{
#if defined(I2C_ARDUINO_WIRE)
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.endTransmission();

    if (_wire.requestFrom(_I2C_address, 1U)) {
        return static_cast<uint8_t>(_wire.read());
    }
#else
    (void)reg;
#endif
    return 0;
}

bool I2C::readRegister(uint8_t reg, uint8_t* data, size_t length) const
{
#if defined(I2C_ARDUINO_WIRE)
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.endTransmission();

    if (_wire.requestFrom(_I2C_address, static_cast<uint8_t>(length))) {
        uint8_t pos = 0; // NOLINT(misc-const-correctness) false positive
        for (size_t ii = 0; ii < length; ++ii) {
            data[pos++] = static_cast<uint8_t>(_wire.read()); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
        return true;
    }
#else
    (void)reg;
    (void)data;
    (void)length;
#endif
    return false;
}

bool I2C::readBytes(uint8_t* data, size_t length) const
{
#if defined(I2C_ARDUINO_WIRE)
    if (_wire.requestFrom(_I2C_address, static_cast<uint8_t>(length))) {
        uint8_t pos = 0; // NOLINT(misc-const-correctness) false positive
        for (size_t ii = 0; ii < length; ++ii) {
            data[pos++] = static_cast<uint8_t>(_wire.read()); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
        return true;
    }
#else
    (void)data;
    (void)length;
#endif
    return false;
}

uint8_t I2C::writeRegister(uint8_t reg, uint8_t data)
{
#if defined(I2C_ARDUINO_WIRE)
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.write(data);
    return _wire.endTransmission();
#else
    (void)reg;
    (void)data;
    return 0;
#endif
}

uint8_t I2C::writeRegister(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(I2C_ARDUINO_WIRE)
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.write(data, length);
    return _wire.endTransmission();
#else
    (void)reg;
    (void)data;
    (void)length;
    return 0;
#endif
}

uint8_t I2C::writeBytes(const uint8_t* data, size_t length)
{
#if defined(I2C_ARDUINO_WIRE)
    _wire.beginTransmission(_I2C_address);
    _wire.write(data, length);
    return _wire.endTransmission();
#else
    (void)data;
    (void)length;
    return 0;
#endif
}

#endif // USE_I2C
