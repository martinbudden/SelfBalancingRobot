#include "BackchannelTransceiverESPNOW.h"


#if defined(USE_ESPNOW)

void BackchannelTransceiverESPNOW::WAIT_FOR_DATA_RECEIVED()
{
    _transceiver.WAIT_FOR_SECONDARY_DATA_RECEIVED();
}

int BackchannelTransceiverESPNOW::sendData(const uint8_t* data, size_t len) const
{
    return _transceiver.sendDataSecondary(data, len);
}

const uint8_t* BackchannelTransceiverESPNOW::getMacAddress() const
{
    return _transceiver.myMacAddress();
}

size_t BackchannelTransceiverESPNOW::getReceivedDataLength() const
{
    return _received_data.len;
}

void BackchannelTransceiverESPNOW::setReceivedDataLengthToZero()
{
    _received_data.len = 0;
}

BackchannelTransceiverESPNOW::BackchannelTransceiverESPNOW(
        uint8_t* receivedDataBuffer,
        size_t receivedDataBufferSize,
        const uint8_t* macAddress
    ) :
    _transceiver(macAddress),
    _received_data(receivedDataBuffer, receivedDataBufferSize)
{
    assert(receivedDataBufferSize >= ESP_NOW_MAX_DATA_LEN);

    _peer_data.receivedDataPtr = &_received_data;
    // add the backchannel as a secondary peer so data may be received from the backchannel
    _transceiver.addSecondaryPeer(_received_data, macAddress); // this stores the value of the MAC address
}

uint32_t BackchannelTransceiverESPNOW::getTickCountDeltaAndReset()
{
    return _transceiver.getTickCountDeltaAndReset();
}

#endif // USE_ESPNOW
