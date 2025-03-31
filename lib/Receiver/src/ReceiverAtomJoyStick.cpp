#if defined(USE_ESPNOW)

#include "ReceiverAtomJoyStick.h"

#include <HardwareSerial.h>


ReceiverAtomJoyStick::ReceiverAtomJoyStick(const uint8_t* macAddress) :
    _atomJoyStickReceiver(macAddress)
    {}

/*!
Setup the receiver. Initialize the Atom JoyStick receiver.
*/
esp_err_t ReceiverAtomJoyStick::setup(uint8_t channel)
{
#if defined(ATOM_JOYSTICK_MAC_ADDRESS)
    static constexpr uint8_t atomJoyStickMacAddress[ESP_NOW_ETH_ALEN] ATOM_JOYSTICK_MAC_ADDRESS;
    const esp_err_t err = _atomJoyStickReceiver.init(channel, atomJoyStickMacAddress);
#else
    const esp_err_t err = _atomJoyStickReceiver.init(channel, nullptr);
#endif

    return err;
}

/*!
If a packet was received from the atomJoyStickReceiver then unpack it and inform the receiver target that new stick values are available.

Returns true if a packet has been received.
*/
bool ReceiverAtomJoyStick::update(uint32_t tickCountDelta)
{
    if (_atomJoyStickReceiver.isPacketEmpty()) {
        return false;
    }

    _packetReceived = true;

    // record tickoutDelta for instrumentation
    _tickCountDelta = tickCountDelta;

    // track dropped packets
    _receivedPacketCount = _atomJoyStickReceiver.getTransceiver().getReceivedPacketCount();
    ++_packetCount;
    _droppedPacketCount = static_cast<int32_t>(_receivedPacketCount - _packetCount);
    _droppedPacketCountDelta = _droppedPacketCount - _droppedPacketCountPrevious;
    _droppedPacketCountPrevious = _droppedPacketCount;

    if (_atomJoyStickReceiver.unpackPacket()) {
        if (_packetCount == 5) {
            // set the JoyStick bias so that the current readings are zero.
            _atomJoyStickReceiver.setCurrentReadingsToBias();
        }

        // use the flip button to turn the motors on or off
        setSwitch(MOTOR_ON_OFF_SWITCH, _atomJoyStickReceiver.getFlipButton() ? 1 : 0);

        // Save the stick values.
        _controls.throttleStickQ4dot12 = _atomJoyStickReceiver.getThrottleQ4dot12();
        _controls.rollStickQ4dot12 = _atomJoyStickReceiver.getRollQ4dot12();
        _controls.pitchStickQ4dot12 = _atomJoyStickReceiver.getPitchQ4dot12();
        _controls.yawStickQ4dot12 = _atomJoyStickReceiver.getYawQ4dot12();

        // Save the button values.
        setSwitch(MOTOR_ON_OFF_SWITCH, _atomJoyStickReceiver.getFlipButton());
        setSwitch(MODE_SWITCH, _atomJoyStickReceiver.getMode());
        const uint8_t altMode = _atomJoyStickReceiver.getAltMode(); // this returns a value of 4 or 5
        setSwitch(ALT_MODE_SWITCH, altMode == 4 ? 0 : 1);

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
    memcpy(&ret, _atomJoyStickReceiver.myMacAddress(), sizeof(EUI_48_t));
    return ret;
}

ReceiverBase::EUI_48_t ReceiverAtomJoyStick::getPrimaryPeerEUI() const
{
    EUI_48_t ret {};
    memcpy(&ret, _atomJoyStickReceiver.getPrimaryPeerMacAddress(), sizeof(EUI_48_t));
    return ret;
}

void ReceiverAtomJoyStick::broadcastMyEUI() const
{
    _atomJoyStickReceiver.broadcastMyMacAddressForBinding();
}

uint32_t ReceiverAtomJoyStick::getAuxiliaryChannel(size_t index) const
{
    (void)index;
    return 0;
}
#endif // USE_ESPNOW
