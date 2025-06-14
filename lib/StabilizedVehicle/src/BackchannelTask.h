#pragma once

#include "TaskBase.h"

class BackchannelBase;

class BackchannelTask : public TaskBase {
public:
    BackchannelTask(uint32_t taskIntervalMicroSeconds, BackchannelBase& backchannel) :
        TaskBase(taskIntervalMicroSeconds),
        _backchannel(backchannel) {}
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    BackchannelBase& _backchannel;
};
