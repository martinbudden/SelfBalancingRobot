#include "BackchannelESPNOW.h"


BackchannelESPNOW::BackchannelESPNOW(
        ESPNOW_Transceiver& espnowTransceiver,
        const uint8_t* backchannelMacAddress,
        const uint8_t* myMacAddress,
        MotorPairController& motorPairController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        const TaskBase* mainTask
    ) :
    BackchannelSBR(
        base_init_t { 
            .backchannelTransceiverPtr = &_backchannelTransceiver,
            .transmitDataBufferPtr = &_transmitDataBuffer[0],
            .transmitDataBufferSize = sizeof(_transmitDataBuffer),
            .receivedDataBufferPtr = &_receivedDataBuffer[0],
            .receivedDataBufferSize = sizeof(_receivedDataBuffer)
        },
        idFromMacAddress(backchannelMacAddress), // backchannelID
        idFromMacAddress(myMacAddress), // telemetryID
        motorPairController,
        ahrs,
        receiver,
        preferences,
        mainTask
    ),
    _backchannelTransceiver(espnowTransceiver, backchannelMacAddress, &_receivedDataBuffer[0], sizeof(_receivedDataBuffer))
{
#if defined(USE_ESPNOW)
    static_assert(sizeof(_transmitDataBuffer) >= ESP_NOW_MAX_DATA_LEN && "transmit buffer too small");
    static_assert(sizeof(_receivedDataBuffer) >= ESP_NOW_MAX_DATA_LEN && "receive buffer too small");
#endif
}
