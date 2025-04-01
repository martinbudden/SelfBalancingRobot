#pragma once

#include "ESPNOW_Transceiver.h"
#include "ReceiverBase.h"


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
    enum { MODE_STABLE = 0, MODE_SPORT = 1 };
    enum { ALT_MODE_AUTO = 4, ALT_MODE_MANUAL = 5};
public:
    esp_err_t setup(uint8_t channel);

    virtual bool update(uint32_t tickCountDelta) override;
    virtual void getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const override;
    virtual EUI_48_t getMyEUI() const override;
    virtual EUI_48_t getPrimaryPeerEUI() const override;
    virtual void broadcastMyEUI() const override;
    uint32_t getAuxiliaryChannel(size_t index) const override;

    ESPNOW_Transceiver& getESPNOW_Transceiver() { return _transceiver; }
    static int32_t ubyte4float_to_Q4dot12(const uint8_t f[4]);
private:
    // from AtomJoyStickReceiver
    inline bool isPacketEmpty() const { return _received_data.len == 0 ? true : false;  }
    inline void setPacketEmpty() { _received_data.len = 0; }
    enum { DEFAULT_BROADCAST_COUNT = 20, DEFAULT_BROADCAST_DELAY_MS = 50 };
    esp_err_t broadcastMyMacAddressForBinding(int broadcastCount=DEFAULT_BROADCAST_COUNT, uint32_t broadcastDelayMs=DEFAULT_BROADCAST_DELAY_MS) const;
    enum checkPacket_t { CHECK_PACKET, DONT_CHECK_PACKET };
    bool unpackPacket(checkPacket_t checkPacket);
    void resetSticks();
    void setDeadZones(int32_t deadZone);
    void setCurrentReadingsToBias();
    int32_t normalizedStick(int stickIndex) const;
private:
    struct stick_t {
        int32_t rawQ4dot12 {0};
        int32_t biasQ4dot12 {0};
        int32_t deadZoneQ4dot12 {16}; // last 4 bits of number
    };
private:
    ESPNOW_Transceiver _transceiver;
    ESPNOW_Transceiver::received_data_t _received_data;
    uint32_t _packetCount {0};
    uint32_t _receivedPacketCount {0};
    int32_t _droppedPacketCount {0};
    int32_t _droppedPacketCountPrevious {0};
    enum { THROTTLE = 0, ROLL = 1, PITCH = 2, YAW = 3, STICK_COUNT = 4 };
    std::array<stick_t, STICK_COUNT> _sticks {};
    enum { PACKET_SIZE = 25 };
    uint8_t _packet[PACKET_SIZE] {};
    uint8_t _biasIsSet {false}; //NOTE: if `bool` type is used here then `getMode()` sometimes returns incorrect value
    uint8_t _mode {0};
    uint8_t _altMode {0};
    uint8_t _armButton {0};
    uint8_t _flipButton {0};
    uint8_t _proactiveFlag {0};
};
