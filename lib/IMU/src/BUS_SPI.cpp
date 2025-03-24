#if defined(USE_SPI)

#include "BUS_SPI.h"


BUS_SPI::BUS_SPI()
{
#if defined(USE_SPI_ARDUINO)
#else
#endif
}

uint8_t BUS_SPI::readRegister(uint8_t reg) const
{
#if defined(USE_SPI_ARDUINO)
#else
    (void)reg;
#endif
    return 0;
}

bool BUS_SPI::readRegister(uint8_t reg, uint8_t* data, size_t length) const
{
#if defined(USE_SPI_ARDUINO)
#else
    (void)reg;
    (void)data;
    (void)length;
#endif
    return false;
}

bool BUS_SPI::readBytes(uint8_t* data, size_t length) const
{
#if defined(USE_SPI_ARDUINO)
#else
    (void)data;
    (void)length;
#endif
    return false;
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, uint8_t data)
{
#if defined(USE_SPI_ARDUINO)
#else
    (void)reg;
    (void)data;
    return 0;
#endif
}

uint8_t BUS_SPI::writeRegister(uint8_t reg, const uint8_t* data, size_t length)
{
#if defined(USE_SPI_ARDUINO)
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
#else
    (void)data;
    (void)length;
    return 0;
#endif
}

#endif // USE_SPI
