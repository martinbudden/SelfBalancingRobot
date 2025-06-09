#pragma once

#include "AHRS_Task.h"
#include "BackchannelTask.h"
#include "ReceiverTask.h"
#include "VehicleControllerTask.h"


namespace SV_Tasks {
    void reportMainTask();
    AHRS_Task* setupTask(AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    VehicleControllerTask* setupTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    ReceiverTask* setupTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);

    BackchannelTask* setupTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
} // end namespace