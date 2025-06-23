#pragma once

#include <TaskBase.h>

class ReceiverBase;
class ReceiverWatcher;

class ReceiverTask : public TaskBase {
public:
    ReceiverTask(uint32_t taskIntervalMicroSeconds, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher);
public:
    static ReceiverTask* createTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    static ReceiverTask* createTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    static ReceiverTask* createTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID);
    static ReceiverTask* createTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    ReceiverBase& _receiver;
    ReceiverWatcher* _receiverWatcher;
};
