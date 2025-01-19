#pragma once

#include "ReceiverBase.h"
#include <AtomJoyStickReceiver.h>

class MotorPairController;


class Receiver : public ReceiverBase {
public:
    explicit Receiver(const uint8_t* macAddress);
private:
    // Receiver is not copyable or moveable
    Receiver(const Receiver&) = delete;
    Receiver& operator=(const Receiver&) = delete;
    Receiver(Receiver&&) = delete;
    Receiver& operator=(Receiver&&) = delete;
public:
    esp_err_t setup(int channel);
    void setMotorController(MotorPairController* motorController) { _motorController = motorController; } //!< Sets the motorController, which must be set before update() is called.

    virtual bool update(uint32_t tickCountDelta) override;
    virtual void mapControls(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const override;
    virtual EUI_48_t getMyEUI() const override;
    virtual EUI_48_t getPrimaryPeerEUI() const override;

    void broadcastMyMacAddressForBinding() const { _atomJoyStickReceiver.broadcastMyMacAddressForBinding(); }
    ESPNOW_Transceiver& getESPNOW_Transceiver() { return _atomJoyStickReceiver.getTransceiver(); }

    static float mapYawStick(float yawStick);
private:
    AtomJoyStickReceiver _atomJoyStickReceiver;
    MotorPairController* _motorController {nullptr};
    uint32_t _packetCount {0};
    uint32_t _receivedPacketCount {0};
    int32_t _droppedPacketCount {0};
    int32_t _droppedPacketCountPrevious {0};
    int _flipPressed {false}; //!< for debouncing the flip button
};
