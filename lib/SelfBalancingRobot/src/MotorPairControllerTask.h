#pragma once

#include "TaskBase.h"

class MotorPairController;

class MotorPairControllerTask : public TaskBase {
public:
    MotorPairControllerTask(uint32_t taskIntervalMicroSeconds, MotorPairController& mpc) :
        TaskBase(taskIntervalMicroSeconds),
        _mpc(mpc) {}

    MotorPairController& getMotorPairController() const { return _mpc; }
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
    MotorPairController& _mpc;
};