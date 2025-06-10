#pragma once

#include "BackchannelBase.h"
#include <ESPNOW_Transceiver.h>


class BackchannelTransceiverESPNOW : public BackchannelTransceiverBase {
public:
    BackchannelTransceiverESPNOW(
        uint8_t* receivedDataBuffer,
        size_t receivedDataBufferSize,
        const uint8_t* macAddress
    );
public:
    virtual int sendData(const uint8_t* data, size_t len) const override;
    virtual void WAIT_FOR_DATA_RECEIVED() override;
    virtual const uint8_t* getMacAddress() const override;
    virtual size_t getReceivedDataLength() const override;
    virtual void setReceivedDataLengthToZero() override;
    virtual uint32_t getTickCountDeltaAndReset() override;
private:
#if defined(USE_ESPNOW)
    ESPNOW_Transceiver _transceiver;
    ESPNOW_Transceiver::received_data_t _received_data;
    ESPNOW_Transceiver::peer_data_t _peer_data {};
#endif
};
