#pragma once

#include <TaskBase.h>

class VehicleControllerBase;

class VehicleControllerTask : public TaskBase {
public:
    VehicleControllerTask(uint32_t taskIntervalMicroSeconds, VehicleControllerBase& vehicleController) :
        TaskBase(taskIntervalMicroSeconds),
        _vehicleController(vehicleController) {}
public:
    static VehicleControllerTask* createTask(task_info_t& taskInfo, VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    static VehicleControllerTask* createTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    VehicleControllerBase& _vehicleController;
};
