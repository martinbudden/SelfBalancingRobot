#include "MainTask.h"


#if defined(FRAMEWORK_ARDUINO)

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

#elif defined(FRAMEWORK_ESPIDF)

extern "C" void app_main()
{
    static MainTask mainTask;
    mainTask.setup();
    mainTask.loop();
}

#elif defined(FRAMEWORK_RPI_PICO)

int main()
{
    static MainTask mainTask;
    mainTask.setup();
    mainTask.loop();
}

#endif