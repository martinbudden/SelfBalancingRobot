#if defined(USE_ESPNOW)

#include "ReceiverAtomJoyStick.h"

#if !defined(UNIT_TEST_BUILD)
#include <HardwareSerial.h>
#endif


ReceiverAtomJoyStick::ReceiverAtomJoyStick(const uint8_t* macAddress) :
    _transceiver(macAddress),
    _received_data(&_packet[0], sizeof(_packet))
    {}

/*!
Setup the receiver. Initialize the transceiver.
*/
esp_err_t ReceiverAtomJoyStick::setup(uint8_t channel)
{
    const esp_err_t err = _transceiver.init(_received_data, channel, nullptr);
    return err;
}

/*!
If a packet was received from the atomJoyStickReceiver then unpack it and inform the receiver target that new stick values are available.

Returns true if a packet has been received.
*/
bool ReceiverAtomJoyStick::update(uint32_t tickCountDelta)
{
    if (isPacketEmpty()) {
        return false;
    }

    _packetReceived = true;

    // record tickoutDelta for instrumentation
    _tickCountDelta = tickCountDelta;

    // track dropped packets
    _receivedPacketCount = _transceiver.getReceivedPacketCount();
    ++_packetCount;
    _droppedPacketCount = static_cast<int32_t>(_receivedPacketCount - _packetCount);
    _droppedPacketCountDelta = _droppedPacketCount - _droppedPacketCountPrevious;
    _droppedPacketCountPrevious = _droppedPacketCount;

    if (unpackPacket(CHECK_PACKET)) {
        if (_packetCount == 5) {
            // set the JoyStick bias so that the current readings are zero.
            setCurrentReadingsToBias();
        }

        // Save the stick values.
        _controls.throttleStickQ4dot12 = normalizedStick(THROTTLE);
        _controls.rollStickQ4dot12 = normalizedStick(ROLL);
        _controls.pitchStickQ4dot12 = normalizedStick(PITCH);
        _controls.yawStickQ4dot12 = normalizedStick(YAW);

        // Save the button values.
        setSwitch(MOTOR_ON_OFF_SWITCH, _flipButton);
        setSwitch(MODE_SWITCH, _mode);
        setSwitch(ALT_MODE_SWITCH, _altMode == 4 ? 0 : 1); // _altMode has a value of 4 or 5

        _newPacketAvailable = true;
        return true;
    }
    Serial.printf("BadPacket\r\n");
    // we've had a packet even though it is a bad one, so we haven't lost contact with the receiver
    return true;
}

/*!
Maps the joystick values from Q4dot12 format in the range [-2048, 2047] to floats in the range [-1, 1].

NOTE: this function runs in the context of the MotorController task, in particular the FPU usage is in that context, so this avoids the
need to save the ESP32 FPU registers on a context switch.
*/
void ReceiverAtomJoyStick::getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const
{
    throttleStick = Q4dot12_to_float(_controls.throttleStickQ4dot12);
    rollStick = Q4dot12_to_float(_controls.rollStickQ4dot12);
    pitchStick = Q4dot12_to_float(_controls.pitchStickQ4dot12);
    yawStick = Q4dot12_to_float(_controls.yawStickQ4dot12);
}

ReceiverBase::EUI_48_t ReceiverAtomJoyStick::getMyEUI() const
{
    EUI_48_t ret {};
    memcpy(&ret, _transceiver.myMacAddress(), sizeof(EUI_48_t));
    return ret;
}

ReceiverBase::EUI_48_t ReceiverAtomJoyStick::getPrimaryPeerEUI() const
{
    EUI_48_t ret {};
    memcpy(&ret, _transceiver.getPrimaryPeerMacAddress(), sizeof(EUI_48_t));
    return ret;
}

void ReceiverAtomJoyStick::broadcastMyEUI() const
{
    broadcastMyMacAddressForBinding();
}

uint32_t ReceiverAtomJoyStick::getAuxiliaryChannel(size_t index) const
{
    (void)index;
    return 0;
}

esp_err_t ReceiverAtomJoyStick::broadcastMyMacAddressForBinding(int broadcastCount, uint32_t broadcastDelayMs) const
{
    // peer command as used by the StampFlyController, see: https://github.com/m5stack/Atom-JoyStick/blob/main/examples/StampFlyController/src/main.cpp#L117
    static const std::array<uint8_t, 4> peerCommand { 0xaa, 0x55, 0x16, 0x88 };
    std::array<uint8_t, 16> data;
    static_assert(sizeof(data) > sizeof(peerCommand) + ESP_NOW_ETH_ALEN + 2);

    data[0] = _transceiver.getBroadcastChannel();
    memcpy(&data[1], _transceiver.myMacAddress(), ESP_NOW_ETH_ALEN);
    memcpy(&data[1 + ESP_NOW_ETH_ALEN], &peerCommand[0], sizeof(peerCommand));

    for (int ii = 0; ii < broadcastCount; ++ii) {
        if (esp_err_t err = _transceiver.broadcastData(&data[0], sizeof(data)) != ESP_OK) {
            Serial.printf("broadcastMyMacAddressForBinding failed: %X\r\n", err);
            return err;
        }
        delay(broadcastDelayMs); // delay() function has units of milliseconds
    }
    return ESP_OK;
}

/*!
Convert the 4 bytes of a floating point number to a fixed point integer in Q4dot12 format, ie in range [-2048,2047].
*/
int32_t ReceiverAtomJoyStick::ubyte4float_to_Q4dot12(const uint8_t f[4]) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    union bi_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        std::array<uint8_t, 4> b;
        uint32_t i;
    };
    const bi_t n = { .b = { f[0], f[1], f[2], f[3] } }; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    const uint8_t sign     = static_cast<uint8_t>((n.i >> 31U) & 0x1U); // 0x1000 0000 // NOLINT(cppcoreguidelines-pro-type-union-access,hicpp-use-auto,modernize-use-auto)
    const uint8_t exponent = static_cast<uint8_t>((n.i >> 23U) & 0xFFU); // 0x7F80 0000 // NOLINT(cppcoreguidelines-pro-type-union-access,hicpp-use-auto,modernize-use-auto)
    if (exponent == 0) {
        return 0;
    }

    const uint32_t mantissa = (n.i & 0x7FFFFFU) | 0x800000U; // 0x007F FFFF, OR in implicit bit NOLINT(cppcoreguidelines-pro-type-union-access)

    const int32_t i = static_cast<int32_t>(mantissa >> ((22U-11U) - (exponent - 0x80U))); // -Wshift-count-overflow NOLINT(hicpp-use-auto,modernize-use-auto)
    return sign ? -i : i;
}

/*!
Check the packet if `checkPacket` set. If the packet is valid then unpack it into the member data and set the packet to empty.

Returns true if a valid packet received, false otherwise.
*/
bool ReceiverAtomJoyStick::unpackPacket(checkPacket_t checkPacket)
{
    // see https://github.com/M5Fly-kanazawa/AtomJoy2024June/blob/main/src/main.cpp#L560 for packet format
    if (isPacketEmpty()) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t ii = 0; ii < PACKET_SIZE - 1; ++ii) {
        checksum += _packet[ii]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    if (checkPacket == CHECK_PACKET) {
        if (checksum != _packet[PACKET_SIZE - 1]) {
            //Serial.printf("checksum:%d, packet[24]:%d, packet[0]:%d, len:%d\r\n", checksum, _packet[24], _packet[0], receivedDataLen());
            setPacketEmpty();
            return false;
        }
        const uint8_t* macAddress = _transceiver.myMacAddress();
        if (_packet[0] != macAddress[3] || _packet[1] != macAddress[4] || _packet[2] != macAddress[5]) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            //Serial.printf("packet: %02X:%02X:%02X\r\n", _packet[0], _packet[1], _packet[2]);
            //Serial.printf("my:     %02X:%02X:%02X\r\n", macAddress[3], macAddress[4], macAddress[5]);
            setPacketEmpty();
            return false;
        }
    } else {
#if !defined(UNIT_TEST_BUILD)
        Serial.printf("packet: %02X:%02X:%02X\r\n", _packet[0], _packet[1], _packet[2]);
#endif
        //Serial.printf("peer:   %02X:%02X:%02X\r\n", _primaryPeerInfo.peer_addr[3], _primaryPeerInfo.peer_addr[4], _primaryPeerInfo.peer_addr[5]);
        //Serial.printf("my:     %02X:%02X:%02X\r\n", macAddress[3], macAddress[4], macAddress[5]);
    }

    _sticks[YAW].rawQ4dot12 = ubyte4float_to_Q4dot12(&_packet[3]); // cppcheck-suppress invalidPointerCast
    _sticks[THROTTLE].rawQ4dot12 = ubyte4float_to_Q4dot12(&_packet[7]); // cppcheck-suppress invalidPointerCast
    _sticks[ROLL].rawQ4dot12 = ubyte4float_to_Q4dot12(&_packet[11]); // cppcheck-suppress invalidPointerCast
    _sticks[PITCH].rawQ4dot12 = -ubyte4float_to_Q4dot12(&_packet[15]); // cppcheck-suppress invalidPointerCast

    _armButton = _packet[19];
    _flipButton = _packet[20];
    _mode = _packet[21];  // _mode: stable or sport
    _altMode = _packet[22];
    _proactiveFlag = _packet[23];

    setPacketEmpty();
    return true;
}

void ReceiverAtomJoyStick::setCurrentReadingsToBias()
{
    _biasIsSet = static_cast<int>(true);
    for (auto& stick : _sticks) {
        stick.biasQ4dot12 = stick.rawQ4dot12;
    }
}

int32_t ReceiverAtomJoyStick::normalizedStick(int stickIndex) const
{
    const stick_t stick = _sticks[stickIndex];
    if (!_biasIsSet) {
        return stick.rawQ4dot12;
    }

    const int32_t ret = stick.rawQ4dot12 - stick.biasQ4dot12;
    if (ret < -stick.deadZoneQ4dot12) {
        return -(-stick.deadZoneQ4dot12 - ret); // (stick.bias - stick.deadZone/2 - min);
    }
    if (ret > stick.deadZoneQ4dot12) {
        return (ret - stick.deadZoneQ4dot12); // (max - stick.bias - stick.deadZone/2);
    }
    return 0.0F;
}

void ReceiverAtomJoyStick::resetSticks()
{
    _biasIsSet = static_cast<int>(false);
    for (auto& stick : _sticks) {
        stick.biasQ4dot12 = 0;
        stick.deadZoneQ4dot12 = 0;
    }
}

void ReceiverAtomJoyStick::setDeadZones(int32_t deadZoneQ4dot12)
{
    for (auto& stick : _sticks) {
        stick.deadZoneQ4dot12 = deadZoneQ4dot12;
    }
}
#endif // USE_ESPNOW
