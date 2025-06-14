#pragma once

#include "TaskBase.h"

class ReceiverBase;
class ReceiverWatcher;

class ReceiverTask : public TaskBase {
public:
    ReceiverTask(uint32_t taskIntervalMicroSeconds, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    ReceiverBase& _receiver;
    ReceiverWatcher* _receiverWatcher;
};
