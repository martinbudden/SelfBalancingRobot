#pragma once

#include "TaskBase.h"

class BackchannelBase;

class BackchannelSendTask : public TaskBase {
public:
    explicit BackchannelSendTask(BackchannelBase& backchannel);
private:
    // class is not copyable or moveable
    BackchannelSendTask(const BackchannelSendTask&) = delete;
    BackchannelSendTask& operator=(const BackchannelSendTask&) = delete;
    BackchannelSendTask(BackchannelSendTask&&) = delete;
    BackchannelSendTask& operator=(BackchannelSendTask&&) = delete;
public:
    const BackchannelBase& getBackchannel() const { return _backchannel; };
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    BackchannelBase& _backchannel;
};
