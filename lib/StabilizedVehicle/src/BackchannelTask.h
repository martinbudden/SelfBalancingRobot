#pragma once

#include "TaskBase.h"

class BackchannelBase;

class BackchannelTask : public TaskBase {
public:
    BackchannelTask(uint32_t taskIntervalMicroSeconds, BackchannelBase& backchannel) : TaskBase(taskIntervalMicroSeconds), _backchannel(backchannel) {}
private:
    // class is not copyable or moveable
    BackchannelTask(const BackchannelTask&) = delete;
    BackchannelTask& operator=(const BackchannelTask&) = delete;
    BackchannelTask(BackchannelTask&&) = delete;
    BackchannelTask& operator=(BackchannelTask&&) = delete;
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    BackchannelBase& _backchannel;
};
