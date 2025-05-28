#pragma once

#if defined(FRAMEWORK_RPI_PICO)

#include <pico/time.h>
static uint32_t timeUs() { return time_us_32(); }

#elif defined(FRAMEWORK_ESPIDF)

#include <esp_timer.h>
static uint32_t timeUs() { return static_cast<uint32_t>(esp_timer_get_time()); }

#elif defined(FRAMEWORK_TEST)

static uint32_t timeUs() { return 0; }

#else // defaults to FRAMEWORK_ARDUINO

#if defined(USE_ARDUINO_ESP32)
#include <esp32-hal.h>
#endif
#include <Arduino.h>
static uint32_t timeUs() { return micros(); }

#endif // FRAMEWORK
