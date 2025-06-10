#include "BackchannelESPNOW.h"


BackchannelESPNOW::BackchannelESPNOW(
        const uint8_t* backChannelMacAddress,
        VehicleControllerTask& vehicleControllerTask,
        MotorPairController& motorPairController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        TelemetryScaleFactors& telemetryScaleFactors,
        SV_Preferences& preferences
    ) :
    BackchannelSBR(
        backChannelMacAddress,
        _transceiverESPNOW,
        vehicleControllerTask,
        motorPairController,
        ahrsTask,
        mainTask,
        receiver,
        telemetryScaleFactors,
        preferences,
        &_transmitDataBuffer[0],
        sizeof(_transmitDataBuffer),
        &_receivedDataBuffer[0],
        sizeof(_receivedDataBuffer)
    ),
    _transceiverESPNOW(_receivedDataBuffer, sizeof(_receivedDataBuffer), backChannelMacAddress)
{
#if defined(USE_ESPNOW)
    static_assert(sizeof(_transmitDataBuffer) >= ESP_NOW_MAX_DATA_LEN);
    static_assert(sizeof(_receivedDataBuffer) >= ESP_NOW_MAX_DATA_LEN);
#endif

    setTelemetryID(_backchannelTransceiver.getMacAddress());
}
