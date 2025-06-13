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
    AHRS_Task* setupAHRS_Task(task_info_t& taskInfo, AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    AHRS_Task* setupAHRS_Task(AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    VehicleControllerTask* setupVehicleControllerTask(task_info_t& taskInfo, VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    VehicleControllerTask* setupVehicleControllerTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    ReceiverTask* setupReceiverTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    ReceiverTask* setupReceiverTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    ReceiverTask* setupReceiverTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID);
    ReceiverTask* setupReceiverTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID);

    BackchannelReceiveTask* setupBackchannelReceiveTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    BackchannelReceiveTask* setupBackchannelReceiveTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    BackchannelSendTask* setupBackchannelSendTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    BackchannelSendTask* setupBackchannelSendTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
} // end namespace