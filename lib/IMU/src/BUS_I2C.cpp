#if defined(USE_I2C)

#include <BUS_I2C.h>
#if defined(USE_I2C_ARDUINO)
#include <Wire.h>
#endif


BUS_I2C::BUS_I2C(uint8_t I2C_address, uint8_t SDA_pin, uint8_t SCL_pin) :
#if defined(USE_I2C_ARDUINO)
    _wire(Wire),
#endif
    _I2C_address(I2C_address),
    _SDA_pin(SDA_pin),
    _SCL_pin(SCL_pin)
{
#if defined(USE_I2C_ARDUINO)
#if defined(USE_I2C_BEGIN_2_PARAMETERS)
    _wire.begin(SDA_pin, SCL_pin);
#else
    _wire.begin();
#endif
#endif
}

uint8_t BUS_I2C::readRegister(uint8_t reg) const
{
#if defined(USE_I2C_ARDUINO)
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

uint8_t BUS_I2C::readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const
{
#if defined(USE_I2C_ARDUINO)
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.endTransmission(false);

    _wire.requestFrom(_I2C_address, 1U);
    for (int ii = 0; ii < timeoutMs; ++ii) {
        if (_wire.available() > 0) {
            return static_cast<uint8_t>(_wire.read());
        }
        delay(1);
    }
#else
    (void)reg;
#endif
    return 0;
}

bool BUS_I2C::readRegister(uint8_t reg, uint8_t* data, size_t length) const
{
#if defined(USE_I2C_ARDUINO)
    _wire.beginTransmission(_I2C_address);
    _wire.write(reg);
    _wire.endTransmission();

    return readBytes(data, length);
#else
    (void)reg;
    (void)data;
    (void)length;
#endif
    return false;
}

bool BUS_I2C::readBytes(uint8_t* data, size_t length) const
{
#if defined(USE_I2C_ARDUINO)
    if (_wire.requestFrom(_I2C_address, static_cast<uint8_t>(length))) {
        for (size_t ii = 0; ii < length; ++ii) {
            data[ii] = static_cast<uint8_t>(_wire.read()); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
        return true;
    }
#else
    (void)data;
    (void)length;
#endif
    return false;
}

bool BUS_I2C::readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const
{
#if defined(USE_I2C_ARDUINO)
    _wire.requestFrom(_I2C_address, static_cast<uint8_t>(length));
    for (int ii = 0; ii < timeoutMs; ++ii) {
        if (_wire.available() > 0) {
            for (size_t jj = 0; jj < length; ++jj) {
                data[jj] = static_cast<uint8_t>(_wire.read()); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            }
            return true;
        }
        delay(1);
    }
    return false;
#else
    (void)data;
    (void)length;
    (void)timeoutMs;
#endif
    return false;
}

uint8_t BUS_I2C::writeRegister(uint8_t reg, uint8_t data)
{
#if defined(USE_I2C_ARDUINO)
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

uint8_t BUS_I2C::writeRegister(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(USE_I2C_ARDUINO)
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

uint8_t BUS_I2C::writeBytes(const uint8_t* data, size_t length)
{
#if defined(USE_I2C_ARDUINO)
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
