#include "BackchannelESPNOW.h"


BackchannelESPNOW::BackchannelESPNOW(
        ESPNOW_Transceiver& espnowTransceiver,
        const uint8_t* backChannelMacAddress,
        VehicleControllerTask& vehicleControllerTask,
        MotorPairController& motorPairController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        TelemetryScaleFactors& telemetryScaleFactors
    ) :
    BackchannelSBR(
        vehicleControllerTask,
        motorPairController,
        ahrsTask,
        mainTask,
        receiver,
        preferences,
        telemetryScaleFactors,
        &_transmitDataBuffer[0],
        sizeof(_transmitDataBuffer),
        &_receivedDataBuffer[0],
        sizeof(_receivedDataBuffer)
    ),
    _backchannelTransceiver(espnowTransceiver, backChannelMacAddress, _receivedDataBuffer, sizeof(_receivedDataBuffer))
{
#if defined(USE_ESPNOW)
    static_assert(sizeof(_transmitDataBuffer) >= ESP_NOW_MAX_DATA_LEN);
    static_assert(sizeof(_receivedDataBuffer) >= ESP_NOW_MAX_DATA_LEN);
#endif
    _backchannelTransceiverPtr = &_backchannelTransceiver;
    // use the last 4 bytes of backchannelMacAddress as the backchannelID
    const uint8_t* pB = backChannelMacAddress;
    _backchannelID = (*(pB + 2U) << 24U) | (*(pB + 3U) << 16U) | (*(pB + 4U) << 8U) | *(pB + 5U); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
    // use the last 4 bytes of myMacAddress as the telemetryID
    const uint8_t* pM = _backchannelTransceiver.getMacAddress();
    _telemetryID = (*(pM + 2U) << 24U) | (*(pM + 3U) << 16U) | (*(pM + 4U) << 8U) | *(pM + 5U); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)

}
