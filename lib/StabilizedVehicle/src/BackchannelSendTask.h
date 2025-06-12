#pragma once

#include "TaskBase.h"

class BackchannelBase;

class BackchannelSendTask : public TaskBase {
public:
    BackchannelSendTask(uint32_t taskIntervalMicroSeconds, BackchannelBase& backchannel) : TaskBase(taskIntervalMicroSeconds), _backchannel(backchannel) {}
private:
    // class is not copyable or moveable
    BackchannelSendTask(const BackchannelSendTask&) = delete;
    BackchannelSendTask& operator=(const BackchannelSendTask&) = delete;
    BackchannelSendTask(BackchannelSendTask&&) = delete;
    BackchannelSendTask& operator=(BackchannelSendTask&&) = delete;
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    BackchannelBase& _backchannel;
};
