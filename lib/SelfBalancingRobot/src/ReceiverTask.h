#pragma once

#include "TaskBase.h"

class ReceiverBase;
class ReceiverWatcher;

class ReceiverTask : public TaskBase {
public:
public:
    ReceiverTask(uint32_t taskIntervalMicroSeconds, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher);
private:
    // class is not copyable or moveable
    ReceiverTask(const ReceiverTask&) = delete;
    ReceiverTask& operator=(const ReceiverTask&) = delete;
    ReceiverTask(ReceiverTask&&) = delete;
    ReceiverTask& operator=(ReceiverTask&&) = delete;
public:
    const ReceiverBase& getReceiver() const { return _receiver; };
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    ReceiverBase& _receiver;
    ReceiverWatcher* _receiverWatcher;
};
