#include "BackchannelStabilizedVehicleESPNOW.h"


BackchannelStabilizedVehicleESPNOW::BackchannelStabilizedVehicleESPNOW(
        ESPNOW_Transceiver& espnowTransceiver,
        const uint8_t* backchannelMacAddress,
        VehicleControllerTask& vehicleControllerTask,
        VehicleControllerBase& vehicleController,
        AHRS_Task& ahrsTask,
        AHRS& ahrs,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences
    ) :
    BackchannelStabilizedVehicle(
        vehicleControllerTask,
        vehicleController,
        ahrsTask,
        ahrs,
        mainTask,
        receiver,
        preferences
    ),
    _backchannelTransceiver(espnowTransceiver, backchannelMacAddress, _receivedDataBuffer, sizeof(_receivedDataBuffer))
{
#if defined(USE_ESPNOW)
    static_assert(sizeof(_transmitDataBuffer) >= ESP_NOW_MAX_DATA_LEN && "transmit buffer too small");
    static_assert(sizeof(_receivedDataBuffer) >= ESP_NOW_MAX_DATA_LEN && "receive buffer too small");
#endif
    _backchannelTransceiverPtr = &_backchannelTransceiver;
    _transmitDataBufferPtr = &_transmitDataBuffer[0]; 
    _transmitDataBufferSize = sizeof(_transmitDataBuffer);
    _receivedDataBufferPtr = &_receivedDataBuffer[0];
    _receivedDataBufferSize = sizeof(_receivedDataBuffer);

    _backchannelID = idFromMacAddress(backchannelMacAddress);
    _telemetryID = idFromMacAddress(_backchannelTransceiver.getMacAddress());
}
