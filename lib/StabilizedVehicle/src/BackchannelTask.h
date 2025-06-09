#pragma once

#include "TaskBase.h"

class BackchannelBase;

class BackchannelTask : public TaskBase {
public:
    BackchannelTask(BackchannelBase& backchannel);
private:
    // class is not copyable or moveable
    BackchannelTask(const BackchannelTask&) = delete;
    BackchannelTask& operator=(const BackchannelTask&) = delete;
    BackchannelTask(BackchannelTask&&) = delete;
    BackchannelTask& operator=(BackchannelTask&&) = delete;
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
