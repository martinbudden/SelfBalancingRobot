#if defined(USE_ESPNOW)

#include "ESPNOW_Receiver.h"

#include "MotorControllerBase.h"

#include <HardwareSerial.h>


Receiver::Receiver(const uint8_t* macAddress) :
    _atomJoyStickReceiver(macAddress)
    {}

/*!
Setup the receiver. Initialize the Atom JoyStick receiver.
*/
esp_err_t Receiver::setup(int channel)
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
If a packet was received from the atomJoyStickReceiver then unpack it and send the stick values to the MotorController.

Returns true if a packet has been received.
*/
bool Receiver::update(uint32_t tickCountDelta)
{
    if (_atomJoyStickReceiver.isPacketEmpty()) {
        return false;
    }

    // record tickoutDelta for instrumentation
    _tickCountDelta = tickCountDelta;

    // track dropped packets
    _receivedPacketCount = _atomJoyStickReceiver.getTransceiver().getReceivedPacketCount();
    ++_packetCount;
    _droppedPacketCount = static_cast<int32_t>(_receivedPacketCount - _packetCount);
    _droppedPacketCountDelta = _droppedPacketCount - _droppedPacketCountPrevious;
    _droppedPacketCountPrevious = _droppedPacketCount;

    if (_atomJoyStickReceiver.unpackPacket()) {
        assert(_motorController != nullptr);
        if (_packetCount == 5) {
            // set the JoyStick bias so that the current readings are zero.
            _atomJoyStickReceiver.setCurrentReadingsToBias();
        }
        // use the flip button to turn the motors on or off
        if (_atomJoyStickReceiver.getFlipButton()) {
            _flipPressed = true;
        } else {
            if (_flipPressed) {
                // flipButton being released, so toggle the motor state
                _motorController->motorsToggleOnOff();
                _flipPressed = static_cast<int>(false);
            }
        }

        // Save the stick values.
        _controls.throttleStickQ4dot12 = _atomJoyStickReceiver.getThrottleQ4dot12();
        _controls.rollStickQ4dot12 = _atomJoyStickReceiver.getRollQ4dot12();
        _controls.pitchStickQ4dot12 = _atomJoyStickReceiver.getPitchQ4dot12();
        _controls.yawStickQ4dot12 = _atomJoyStickReceiver.getYawQ4dot12();

        // Save the button values.
        setSwitch(0, _atomJoyStickReceiver.getMode());
        const uint8_t altMode = _atomJoyStickReceiver.getAltMode(); // this returns a value of 4 or 5
        setSwitch(1, altMode == 4 ? 0 : 1);
        setSwitch(2, _atomJoyStickReceiver.getFlipButton());

        // Inform the motor controller that new stick values are available.
        _motorController->newStickValuesReceived();
        return true;
    }
    Serial.printf("BadPacket\r\n");
    // we've had a packet even though it is a bad one, so we haven't lost contact with the receiver
    return true;
}

/*!
Map the yaw stick non-linearly to give more control for small values of yaw.

Runs in the context of the MotorController.
*/
float Receiver::mapYawStick(float yawStick)
{
    // map the yaw stick to a quadratic curve to give more control for small values of yaw.
    // higher values of a increase the effect
    // a=0 gives a linear response, a=1 gives a parabolic (x^2) curve
    static constexpr float a { 0.2 };
    const float ret = (1.0F - a) * yawStick + (yawStick < 0.0F ? -a*yawStick*yawStick : a*yawStick*yawStick);
    return ret;
}

/*!
Maps the joystick values from Q4dot12 format in the range [-2048, 2047] to floats in the range [-1, 1].

NOTE: this function runs in the context of the MotorController task, in particular the FPU usage is in that context, so this avoids the
need to save the ESP32 FPU registers on a context switch.
*/
void Receiver::mapControls(float&  throttleStick, float&  rollStick, float&  pitchStick, float&  yawStick) const
{
    throttleStick = Q4dot12_to_float(_controls.throttleStickQ4dot12);
    rollStick = Q4dot12_to_float(_controls.rollStickQ4dot12);
    pitchStick = Q4dot12_to_float(_controls.pitchStickQ4dot12);
    yawStick = mapYawStick(Q4dot12_to_float(_controls.yawStickQ4dot12));
}


ReceiverBase::EUI_48_t Receiver::getMyEUI() const
{
    EUI_48_t ret {};
    memcpy(&ret, _atomJoyStickReceiver.myMacAddress(), sizeof(EUI_48_t));
    return ret;
}

ReceiverBase::EUI_48_t Receiver::getPrimaryPeerEUI() const
{
    EUI_48_t ret {};
    memcpy(&ret, _atomJoyStickReceiver.getPrimaryPeerMacAddress(), sizeof(EUI_48_t));
    return ret;
}
#endif // USE_ESPNOW
