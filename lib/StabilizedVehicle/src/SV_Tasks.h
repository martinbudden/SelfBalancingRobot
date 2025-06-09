#pragma once

#include "AHRS_Task.h"
#include "BackchannelTask.h"
#include "ReceiverTask.h"
#include "VehicleControllerTask.h"


class Tasks {
public:
    static void reportMainTask();
    static AHRS_Task* setupTask(AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    static VehicleControllerTask* setupTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    static ReceiverTask* setupTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    static ReceiverTask* setupTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID) {
        return setupTask(receiver, receiverWatcher, priority, coreID, 0);
    }
    static BackchannelTask* setupTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
};