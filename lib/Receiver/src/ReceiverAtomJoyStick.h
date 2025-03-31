#pragma once

#include "ReceiverBase.h"
#include <AtomJoyStickReceiver.h>


class ReceiverAtomJoyStick : public ReceiverBase {
public:
    explicit ReceiverAtomJoyStick(const uint8_t* macAddress);
private:
    // Receiver is not copyable or moveable
    ReceiverAtomJoyStick(const ReceiverAtomJoyStick&) = delete;
    ReceiverAtomJoyStick& operator=(const ReceiverAtomJoyStick&) = delete;
    ReceiverAtomJoyStick(ReceiverAtomJoyStick&&) = delete;
    ReceiverAtomJoyStick& operator=(ReceiverAtomJoyStick&&) = delete;
public:
    enum { MODE_SWITCH = 1, ALT_MODE_SWITCH = 2 };
public:
    esp_err_t setup(uint8_t channel);

    virtual bool update(uint32_t tickCountDelta) override;
    virtual void getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const override;
    virtual EUI_48_t getMyEUI() const override;
    virtual EUI_48_t getPrimaryPeerEUI() const override;
    virtual void broadcastMyEUI() const override;
    uint32_t getAuxiliaryChannel(size_t index) const override;

    ESPNOW_Transceiver& getESPNOW_Transceiver() { return _atomJoyStickReceiver.getTransceiver(); }
private:
    AtomJoyStickReceiver _atomJoyStickReceiver;
    uint32_t _packetCount {0};
    uint32_t _receivedPacketCount {0};
    int32_t _droppedPacketCount {0};
    int32_t _droppedPacketCountPrevious {0};
    int _flipPressed {false}; //!< for debouncing the flip button
};
