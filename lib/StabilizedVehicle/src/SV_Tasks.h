#pragma once

#include "AHRS_Task.h"
#include "BackchannelReceiveTask.h"
#include "BackchannelSendTask.h"
#include "ReceiverTask.h"
#include "VehicleControllerTask.h"


namespace SV_Tasks {
    struct task_info_t {
        void* taskHandle;
        const char* name;
        uint32_t stackDepth;
        uint8_t* stackBuffer;
        uint8_t priority;
        uint8_t coreID;
    };
    void reportMainTask();
    AHRS_Task* createAHRS_Task(task_info_t& taskInfo, AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    AHRS_Task* createAHRS_Task(AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    VehicleControllerTask* createVehicleControllerTask(task_info_t& taskInfo, VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    VehicleControllerTask* createVehicleControllerTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    ReceiverTask* createReceiverTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    ReceiverTask* createReceiverTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    ReceiverTask* createReceiverTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID);
    ReceiverTask* createReceiverTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID);

    BackchannelReceiveTask* createBackchannelReceiveTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    BackchannelReceiveTask* createBackchannelReceiveTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    BackchannelSendTask* createBackchannelSendTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    BackchannelSendTask* createBackchannelSendTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
} // end namespace