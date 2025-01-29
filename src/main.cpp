#if defined(BUILD_ENV_ESP32)
#include "MainTask_ESP32.h"
#elif defined(BUILD_ENV_M5STACK)
#include "MainTask_M5Stack.h"
#endif

#include <Arduino.h>

namespace { // use anonymous namespace to make items local to this translation unit
MainTask* mainTask;
} // end namespace


void setup()// cppcheck-suppress unusedFunction
{
    static MainTask mainTaskStatic;
    mainTask = &mainTaskStatic;
    mainTask->setup();
}


void loop()// cppcheck-suppress unusedFunction
{
    mainTask->loop();
}
