#pragma once

#include "TaskBase.h"

class VehicleControllerBase;

class VehicleControllerTask : public TaskBase {
public:
    VehicleControllerTask(uint32_t taskIntervalMicroSeconds, VehicleControllerBase& vehicleController) :
        TaskBase(taskIntervalMicroSeconds),
        _vehicleController(vehicleController) {}

public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
    VehicleControllerBase& _vehicleController;
};