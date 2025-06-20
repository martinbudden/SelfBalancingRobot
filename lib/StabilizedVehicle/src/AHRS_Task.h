#pragma once

#include "TaskBase.h"

class AHRS;

class AHRS_Task : public TaskBase {
public:
    AHRS_Task(uint32_t taskIntervalMicroSeconds, AHRS& ahrs) :
        TaskBase(taskIntervalMicroSeconds),
        _ahrs(ahrs) {}

public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    AHRS& _ahrs;
};
