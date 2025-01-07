#if defined(USE_ESPNOW)

#include "ESPNOW_Receiver.h"

#include "MotorControllerBase.h"
#include "MotorPairBase.h"

#include <HardwareSerial.h>


Receiver::Receiver(MotorControllerBase& motorController, const uint8_t* macAddress) :
    _atomJoyStickReceiver(macAddress),
    _motorController(motorController)
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
If a packet was received from the atomJoyStickReceiver then unpack it and send the stick values to the motorPairController.

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
                _motorController.motorsToggleOnOff();
                _flipPressed = static_cast<int>(false);
            }
        }

        // use the stick values to set the MPC setpoints
        const int32_t throttleStickQ4dot12 = _atomJoyStickReceiver.getThrottleQ4dot12();
        const int32_t rollStickQ4dot12 = _atomJoyStickReceiver.getRollQ4dot12();
        const int32_t pitchStickQ4dot12 = _atomJoyStickReceiver.getPitchQ4dot12();
        const int32_t yawStickQ4dot12 = _atomJoyStickReceiver.getYawQ4dot12();
        _motorController.setSetpoints(throttleStickQ4dot12, rollStickQ4dot12, pitchStickQ4dot12, yawStickQ4dot12);
        return true;
    }
    Serial.printf("Receiver::update Bad packet\r\n");
    return false;
}

ReceiverBase::controls_t Receiver::getControls() const
{
    return controls_t {
        .throttleStickQ4dot12 =_atomJoyStickReceiver.getThrottleQ4dot12(),
        .rollStickQ4dot12 = _atomJoyStickReceiver.getRollQ4dot12(),
        .pitchStickQ4dot12 = _atomJoyStickReceiver.getPitchQ4dot12(),
        .yawStickQ4dot12 = _atomJoyStickReceiver.getYawQ4dot12()
    };
}

uint32_t Receiver::getFlags() const
{
    return 0;
}

#endif // USE_ESPNOW
