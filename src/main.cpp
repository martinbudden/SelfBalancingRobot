#include "MainTask.h"

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
