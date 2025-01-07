#if defined(USE_ESPNOW)

#include "AHRS_Base.h"
#include "CommandPacket.h"
#include "ESPNOW_Backchannel.h"
#include "ESPNOW_Receiver.h"
#include "MotorPairBase.h"
#include "MotorPairController.h"
#if defined(USE_ESP32_PREFERENCES)
#include "SBR_Preferences.h"
#endif
#include "SBR_Telemetry.h"
#include "TaskBase.h"
#include <HardwareSerial.h>

static_assert(sizeof(TD_TickIntervals) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_PID) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_MPC) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_AHRS) <= ESP_NOW_MAX_DATA_LEN);


Backchannel::Backchannel(ESPNOW_Transceiver& transceiver, const uint8_t* backchannelMacAddress, // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init) false positive
        MotorPairController& motorPairController,
        const AHRS_Base& ahrs,
        const TaskBase& mainTask, 
        const ReceiverBase& receiver,
        SBR_Preferences* preferences) :
    _transceiver(transceiver),
    _received_data(_receivedDataBuffer, sizeof(_receivedDataBuffer)),
    _motorPairController(motorPairController),
    _ahrs(ahrs),
    _mainTask(mainTask),
    _receiver(receiver),
    _preferences(preferences)
{
    _peer_data.receivedDataPtr = &_received_data;
    // add the backchannel as a secondary peer so data may be received from the backchannel
    addToTransceiverAsSecondaryPeer(backchannelMacAddress);
    // use the last 4 bytes of backchannelMacAddress as the backchannelID
    const uint8_t* pB = backchannelMacAddress;
    _backchannelID = (*(pB + 2) << 24) | (*(pB + 3) << 16) | (*(pB + 4) << 8)  | *(pB + 5); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    // use the last 4 bytes of myMacAddress as the telemetryID
    const uint8_t* pM = _transceiver.myMacAddress();
    _telemetryID = (*(pM + 2) << 24) | (*(pM + 3) << 16) | (*(pM + 4) << 8)  | *(pM + 5); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

void Backchannel::packetControl(const CommandPacketControl& packet) {
    //Serial.printf("Control packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);
    switch (packet.value) {
    case CommandPacketControl::MOTORS_SWITCH_OFF:
        _motorPairController.motorsSwitchOff();
        break;
    case CommandPacketControl::MOTORS_SWITCH_ON:
        _motorPairController.motorsSwitchOn();
        break;
    case CommandPacketControl::ENCODERS_RESET:
        _motorPairController.motorsResetEncodersToZero();
        break;
    case CommandPacketControl::MPC_CONTROL_MODE_SERIAL_PIDS:
        _motorPairController.setControlMode(MotorPairController::CONTROL_MODE_SERIAL_PIDS);
        break;
    case CommandPacketControl::MPC_CONTROL_MODE_PARALLEL_PIDS:
        _motorPairController.setControlMode(MotorPairController::CONTROL_MODE_PARALLEL_PIDS);
        break;
    }
}

void Backchannel::packetRequestData(const CommandPacketRequestData& packet) {
    //Serial.printf("TransmitRequest packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);
    static_assert(sizeof(TD_TickIntervals) < sizeof(_transmitDataBuffer));
    static_assert(sizeof(TD_PID) < sizeof(_transmitDataBuffer));
    static_assert(sizeof(TD_AHRS) < sizeof(_transmitDataBuffer));
    static_assert(sizeof(TD_MPC) < sizeof(_transmitDataBuffer));

    int len {0}; // NOLINT(misc-const-correctness) false positive
    switch (packet.value) {
    case CommandPacketRequestData::REQUEST_STOP_SENDING_DATA:
        _sendType = SEND_NO_DATA;
        break;
    case CommandPacketRequestData::REQUEST_TICK_INTERVAL_DATA:
        _sendType = SEND_TICK_INTERVAL_DATA;
        len = packTelemetryData_TickIntervals(_transmitDataBuffer, _telemetryID,
                _ahrs.getTickCountDelta(),
                _ahrs.getFifoCount(),
                _motorPairController.getTickCountDelta(),
                _mainTask.getTickCountDelta(),
                _transceiver.getTickCountDeltaAndReset(),
                _receiver.getDroppedPacketCountDelta());
        sendData(_transmitDataBuffer, len);
        break;
    case CommandPacketRequestData::REQUEST_PID_DATA:
        _sendType = SEND_PID_DATA;
        len = packTelemetryData_PID(_transmitDataBuffer, _telemetryID, _motorPairController);
        sendData(_transmitDataBuffer, len);
        break;
    case CommandPacketRequestData::REQUEST_AHRS_DATA:
        _sendType = SEND_AHRS_DATA;
        len = packTelemetryData_AHRS(_transmitDataBuffer, _telemetryID, _ahrs, _motorPairController);
        sendData(_transmitDataBuffer, len);
        break;
    case CommandPacketRequestData::REQUEST_MPC_DATA:
        _sendType = SEND_MPC_DATA;
        len = packTelemetryData_MPC(_transmitDataBuffer, _telemetryID, _motorPairController);
        sendData(_transmitDataBuffer, len);
        break;
    case CommandPacketRequestData::REQUEST_RECEIVER_DATA:
        _sendType = SEND_RECEIVER_DATA;
        len = packTelemetryData_Receiver(_transmitDataBuffer, _telemetryID, _receiver);
        sendData(_transmitDataBuffer, len);
        break;
    }
}

void Backchannel::packetSetPID(const CommandPacketSetPID& packet) {
    //Serial.printf("SetPID packet type:%d, len:%d, pidType:%d setType:%d value:%f\r\n", packet.type, packet.len, packet.pidType, packet.setType, packet.value);
    bool transmit = false;
    PIDF* pid =
        packet.pidType == CommandPacketSetPID::PID_PITCH ? _motorPairController.getPitchPID() :
        packet.pidType == CommandPacketSetPID::PID_SPEED ? _motorPairController.getSpeedPID() :
        packet.pidType == CommandPacketSetPID::PID_YAW_RATE ? _motorPairController.getYawRatePID() :
        nullptr;

    switch (packet.pidType) {
    case CommandPacketSetPID::PID_PITCH:
    case CommandPacketSetPID::PID_SPEED:
    case CommandPacketSetPID::PID_YAW_RATE:
        switch (packet.setType) {
        case CommandPacketSetPID::SET_SETPOINT:
            pid->setSetpoint(packet.value);
            transmit = true;
            break;
        case CommandPacketSetPID::RESET_PID:
            // Not currently implemented.
            break;
        case CommandPacketSetPID::SET_PITCH_BALANCE_ANGLE:
            // Set the balance angel, the value of packet.pidType is ignored.
            _motorPairController.setPitchBalanceAngleDegrees(packet.value);
            transmit = true;
            break;
        case CommandPacketSetPID::SET_P:
            pid->setP(packet.value);
            transmit = true;
            break;
        case CommandPacketSetPID::SET_I:
            pid->setI(packet.value);
            transmit = true;
            break;
        case CommandPacketSetPID::SET_D:
            pid->setD(packet.value);
            transmit = true;
            break;
        case CommandPacketSetPID::SET_F:
            pid->setF(packet.value);
            transmit = true;
            break;
#if defined(USE_ESP32_PREFERENCES)
        case CommandPacketSetPID::SAVE_PITCH_BALANCE_ANGLE:
            // Save the balance angel, the value of packet.pidType is ignored.
            _preferences->putPitchBalanceAngleDegrees(_motorPairController.getPitchBalanceAngleDegrees());
            break;
        case CommandPacketSetPID::SAVE_P:
        case CommandPacketSetPID::SAVE_I:
        case CommandPacketSetPID::SAVE_D:
        case CommandPacketSetPID::SAVE_F:
            //Serial.printf("Saved PID packetType:%d pidType:%d  setType:%d\r\n", packet.type, packet.pidType, packet.setType);
            // Currently we don't save individual PID constants: if any save request is received we save all the PID constants.
            if (packet.pidType == CommandPacketSetPID::PID_PITCH) {
                _preferences->putPitchPID(_motorPairController.getPitchPIDConstants());
            } else if (packet.pidType == CommandPacketSetPID::PID_SPEED) {
                _preferences->putSpeedPID(_motorPairController.getSpeedPIDConstants());
            } else if (packet.pidType == CommandPacketSetPID::PID_YAW_RATE) {
                _preferences->putYawRatePID(_motorPairController.getYawRatePIDConstants());
            }
            break;
#endif
        default:
            //Serial.printf("Backchannel::update invalid pidType:%d\r\n", packet.pidType);
            break;
        }
    default:
        break;
    }
    if (transmit) {
        // send back the new data for display
        const int len = packTelemetryData_PID(_transmitDataBuffer, _telemetryID, _motorPairController);
        sendData(_transmitDataBuffer, len);
    }
}

/*!
If no data was received then send telemetry data if it has been requested and return false.

If data was received then interpret it as a packet and return true.
Three types of packets may be received:

1. A command packet, for example a command to switch off the motors.
2. A request to transmit telemetry. In this case format the telemetry data and send it.
3. A request to set a PID value. In this case set the PID value and then send back the value of all the PIDs.
*/
bool Backchannel::update()
{
    if (_received_data.len == 0) {
        // esp_now_send runs at a high priority, so shorter packets mean less blocking of the other tasks.
        // packet lengths are 10(TICK_INTERVAL), 44(AHRS), and 88(MPC)
        if (_sendType == SEND_NO_DATA) {
            return false;
        }
        if (_sendType == SEND_TICK_INTERVAL_DATA) {
            const int len = packTelemetryData_TickIntervals(_transmitDataBuffer, _telemetryID,
                _ahrs.getTickCountDelta(),
                _ahrs.getFifoCount(),
                _motorPairController.getTickCountDelta(),
                _mainTask.getTickCountDelta(),
                _transceiver.getTickCountDeltaAndReset(),
                _receiver.getDroppedPacketCountDelta());
            //Serial.printf("tiLen:%d\r\n", len);
            sendData(_transmitDataBuffer, len);
        } else if (_sendType == SEND_AHRS_DATA) {
            const int len = packTelemetryData_AHRS(_transmitDataBuffer, _telemetryID, _ahrs, _motorPairController);
            //Serial.printf("ahrsLen:%d\r\n", len);
            sendData(_transmitDataBuffer, len);
        } else if (_sendType == SEND_MPC_DATA) {
            const int len = packTelemetryData_MPC(_transmitDataBuffer, _telemetryID, _motorPairController);
            //Serial.printf("mpcLen:%d\r\n", len);
            sendData(_transmitDataBuffer, len);
        } else if (_sendType == SEND_RECEIVER_DATA) {
            const int len = packTelemetryData_Receiver(_transmitDataBuffer, _telemetryID, _receiver);
            //Serial.printf("receiverLen:%d\r\n", len);
            sendData(_transmitDataBuffer, len);
        } else if (_sendType == RESET_SCREEN_AND_SEND_NO_DATA) {
            const int len = packTelemetryData_Minimal(_transmitDataBuffer, _telemetryID);
            sendData(_transmitDataBuffer, len);
            _sendType = SEND_NO_DATA;
        }
        return false;
    }

    const auto controlPacket = reinterpret_cast<const CommandPacketControl*>(_receivedDataBuffer);
    if (controlPacket->id == _backchannelID) {
        const int type = controlPacket->type;
        //Serial.printf("Backchannel::update id:%x, type:%d, len:%d value:%d\r\n", packetControl->id, packetControl->type, packetControl->len, packetControl->value);
        if (type == CommandPacketControl::TYPE) {
            const CommandPacketControl* packet = reinterpret_cast<const CommandPacketControl*>(_receivedDataBuffer);
            packetControl(*packet);
        } else if (type == CommandPacketRequestData::TYPE) {
            const CommandPacketRequestData* packet = reinterpret_cast<const CommandPacketRequestData*>(_receivedDataBuffer);
            packetRequestData(*packet);
        } else if (type == CommandPacketSetPID::TYPE) {
            const CommandPacketSetPID* packet = reinterpret_cast<const CommandPacketSetPID*>(_receivedDataBuffer);
            packetSetPID(*packet);
        }
        _received_data.len = 0;
    }
    return true;
}

int Backchannel::receivedDataType() const
{
    return reinterpret_cast<const CommandPacketControl*>(_receivedDataBuffer)->type;
}

#endif // USE_ESPNOW