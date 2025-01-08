#if defined(USE_I2C)

#include "I2C.h"
#if !defined(UNIT_TEST_BUILD)
#include <Wire.h>
#endif

I2C::I2C(uint8_t I2C_address, uint8_t SDA_pin, uint8_t SCL_pin) :
    _I2C_address(I2C_address),
    _SDA_pin(SDA_pin),
    _SCL_pin(SCL_pin)
{
#if !defined(UNIT_TEST_BUILD)
#if defined(USE_I2C_BEGIN_2_PARAMETERS)
    Wire.begin(SDA_pin, SCL_pin);
#else
    Wire.begin();
#endif
#endif
}

uint8_t I2C::readByte(uint8_t reg) const
{
#if !defined(UNIT_TEST_BUILD)
    Wire.beginTransmission(_I2C_address);
    Wire.write(reg);
    Wire.endTransmission();

    if (Wire.requestFrom(_I2C_address, static_cast<uint8_t>(1))) {
        return Wire.read();
    }
#endif
    return 0;
}

bool I2C::readBytes(uint8_t reg, uint8_t* data, size_t length) const
{
#if !defined(UNIT_TEST_BUILD)
    Wire.beginTransmission(_I2C_address);
    Wire.write(reg);
    Wire.endTransmission();

    if (Wire.requestFrom(_I2C_address, length)) {
        uint8_t pos = 0; // NOLINT(misc-const-correctness) false positive
        for (size_t ii = 0; ii < length; ++ii) {
            data[pos++] = Wire.read();
        }
        return true;
    }
#endif
    return false;
}

uint8_t I2C::writeByte(uint8_t reg, uint8_t data)
{
#if !defined(UNIT_TEST_BUILD)
    Wire.beginTransmission(_I2C_address);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission();
#else
    return 0;
#endif
}

uint8_t I2C::writeBytes(uint8_t reg, const uint8_t* data, size_t length)
{
#if !defined(UNIT_TEST_BUILD)
    Wire.beginTransmission(_I2C_address);
    Wire.write(reg);
    Wire.write(data, length);
    return Wire.endTransmission();
#else
    return 0;
#endif
}

#endif // USE_I2C
