#pragma once

#include "TaskBase.h"

class BackchannelBase;

class BackchannelReceiveTask : public TaskBase {
public:
    explicit BackchannelReceiveTask(BackchannelBase& backchannel) : TaskBase(0), _backchannel(backchannel) {}
private:
    // class is not copyable or moveable
    BackchannelReceiveTask(const BackchannelReceiveTask&) = delete;
    BackchannelReceiveTask& operator=(const BackchannelReceiveTask&) = delete;
    BackchannelReceiveTask(BackchannelReceiveTask&&) = delete;
    BackchannelReceiveTask& operator=(BackchannelReceiveTask&&) = delete;
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    BackchannelBase& _backchannel;
};
