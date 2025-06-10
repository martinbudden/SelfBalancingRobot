#pragma once

#include <BackchannelStabilizedVehicle.h>
#include <BackchannelTransceiverESPNOW.h>


class BackchannelStabilizedVehicleESPNOW : public BackchannelStabilizedVehicle {
public:
    BackchannelStabilizedVehicleESPNOW(
        ESPNOW_Transceiver& espnowTransceiver,
        const uint8_t* backchannelMacAddress,
        VehicleControllerTask& vehicleControllerTask,
        VehicleControllerBase& vehicleController,
        AHRS_Task& ahrsTask,
        AHRS& ahrs,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences
    );
protected:
    BackchannelTransceiverESPNOW _backchannelTransceiver;
    // If using MSP, then the MSP packets are packed into _transmitDataBuffer by MSP::processOutCommand,
    // so _transmitDataBuffer must be large enough to hold the larges MSP packet.
    // If the packet length exceeds ESP_NOW_MAX_DATA_LEN, then it is not sent,
    // but we don't know its length until we have unpacked it.
    uint8_t _transmitDataBuffer[512] {};
    uint8_t _receivedDataBuffer[256] {}; // must be >= ESP_NOW_MAX_DATA_LEN
};
