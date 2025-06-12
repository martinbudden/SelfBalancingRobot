#pragma once

#include "AHRS_Task.h"
#include "BackchannelReceiveTask.h"
#include "BackchannelSendTask.h"
#include "ReceiverTask.h"
#include "VehicleControllerTask.h"


namespace SV_Tasks {
    void reportMainTask();
    AHRS_Task* setupAHRS_Task(AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    VehicleControllerTask* setupVehicleControllerTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    ReceiverTask* setupReceiverTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    ReceiverTask* setupReceiverTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID);

    BackchannelReceiveTask* setupBackchannelReceiveTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID);
    BackchannelSendTask* setupBackchannelSendTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
} // end namespace