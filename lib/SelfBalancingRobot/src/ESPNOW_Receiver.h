#pragma once

#include "ReceiverBase.h"
#include <AtomJoyStickReceiver.h>

class MotorControllerBase;


class Receiver : public ReceiverBase {
public:
    Receiver(MotorControllerBase& motorController, const uint8_t* macAddress);
private:
    // Receiver is not copyable or moveable
    Receiver(const Receiver&) = delete;
    Receiver& operator=(const Receiver&) = delete;
    Receiver(Receiver&&) = delete;
    Receiver& operator=(Receiver&&) = delete;
public:
    esp_err_t setup(int channel);
    virtual bool update(uint32_t tickCountDelta) override;
    virtual controls_t getControls() const override;
    virtual uint32_t getFlags() const override;
    void broadcastMyMacAddressForBinding() const { _atomJoyStickReceiver.broadcastMyMacAddressForBinding(); }
    ESPNOW_Transceiver& getESPNOW_Transceiver() { return _atomJoyStickReceiver.getTransceiver(); }
    const uint8_t* getMyMacAddress() const { return _atomJoyStickReceiver.myMacAddress(); }
    const uint8_t* getPrimaryPeerMacAddress() const { return _atomJoyStickReceiver.getPrimaryPeerMacAddress(); }
    inline uint8_t getMode() const { return _atomJoyStickReceiver.getMode(); }
    inline uint8_t getAltMode() const { return _atomJoyStickReceiver.getAltMode(); }
    inline uint8_t getFlipButton() const { return _atomJoyStickReceiver.getFlipButton(); }

private:
    AtomJoyStickReceiver _atomJoyStickReceiver;
    MotorControllerBase& _motorController;
    uint32_t _packetCount {0};
    uint32_t _receivedPacketCount {0};
    int32_t _droppedPacketCount {0};
    int32_t _droppedPacketCountPrevious {0};
    int _flipPressed {false}; //!< for debouncing the flip button
};
