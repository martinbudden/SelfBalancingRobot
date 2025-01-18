#if defined(USE_ESPNOW)

#include "ESPNOW_Backchannel.h"
#include "ESPNOW_Receiver.h"
#include "MotorPairController.h"
#include "SBR_Telemetry.h"
#include "SBR_TelemetryData.h"

#include <AHRS.h>
#include <CommandPacket.h>
#include <HardwareSerial.h>
#if defined(USE_ESP32_PREFERENCES)
#include <SV_Preferences.h>
#endif
#include <SV_Telemetry.h>
#include <TaskBase.h>

static_assert(sizeof(TD_TICK_INTERVALS) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_SBR_PIDS) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_MPC) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_AHRS) <= ESP_NOW_MAX_DATA_LEN);


Backchannel::Backchannel(ESPNOW_Transceiver& transceiver, const uint8_t* backchannelMacAddress, // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init) false positive
        MotorPairController& motorPairController,
        const AHRS& ahrs,
        const TaskBase& mainTask, 
        const ReceiverBase& receiver,
        TelemetryScaleFactors& telemetryScaleFactors,
        SV_Preferences* preferences) :
    _transceiver(transceiver),
    _received_data(_receivedDataBuffer, sizeof(_receivedDataBuffer)),
    _motorPairController(motorPairController),
    _ahrs(ahrs),
    _mainTask(mainTask),
    _receiver(receiver),
    _telemetryScaleFactors(telemetryScaleFactors),
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
    case CommandPacketControl::RESET:
        _motorPairController.motorsResetEncodersToZero();
        break;
    case CommandPacketControl::CONTROL_MODE_0:
        _motorPairController.setControlMode(MotorPairController::CONTROL_MODE_SERIAL_PIDS);
        _telemetryScaleFactors.setControlMode(MotorPairController::CONTROL_MODE_SERIAL_PIDS);
        break;
    case CommandPacketControl::CONTROL_MODE_1:
        _motorPairController.setControlMode(MotorPairController::CONTROL_MODE_PARALLEL_PIDS);
        _telemetryScaleFactors.setControlMode(MotorPairController::CONTROL_MODE_PARALLEL_PIDS);
        break;
    }
}

void Backchannel::packetRequestData(const CommandPacketRequestData& packet) {
    //Serial.printf("TransmitRequest packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);
    static_assert(sizeof(TD_TICK_INTERVALS) < sizeof(_transmitDataBuffer));
    static_assert(sizeof(TD_SBR_PIDS) < sizeof(_transmitDataBuffer));
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
                _ahrs,
                _motorPairController,
                _motorPairController.getOutputPowerTimeMicroSeconds(),
                _mainTask.getTickCountDelta(),
                _transceiver.getTickCountDeltaAndReset(),
                _receiver.getDroppedPacketCountDelta());
        sendData(_transmitDataBuffer, len);
        break;
    case CommandPacketRequestData::REQUEST_PID_DATA:
        _sendType = SEND_PID_DATA;
        len = packTelemetryData_PID(_transmitDataBuffer, _telemetryID, _motorPairController, _telemetryScaleFactors);
        sendData(_transmitDataBuffer, len);
        break;
    case CommandPacketRequestData::REQUEST_AHRS_DATA:
        _sendType = SEND_AHRS_DATA;
        len = packTelemetryData_AHRS(_transmitDataBuffer, _telemetryID, _ahrs, _motorPairController);
        sendData(_transmitDataBuffer, len);
        break;
    case CommandPacketRequestData::REQUEST_RECEIVER_DATA:
        _sendType = SEND_RECEIVER_DATA;
        len = packTelemetryData_Receiver(_transmitDataBuffer, _telemetryID, _receiver);
        sendData(_transmitDataBuffer, len);
        break;
    case CommandPacketRequestData::REQUEST_MOTOR_CONTROLLER_DATA:
        _sendType = SEND_MPC_DATA;
        len = packTelemetryData_MPC(_transmitDataBuffer, _telemetryID, _motorPairController);
        sendData(_transmitDataBuffer, len);
        break;
    }
}

void Backchannel::packetSetPID(const CommandPacketSetPID& packet) {
    //Serial.printf("SetPID packet type:%d, len:%d, pidType:%d setType:%d value:%f\r\n", packet.type, packet.len, packet.pidType, packet.setType, packet.value);
    bool transmit = false;
    const MotorPairController::pid_index_t pidIndex = static_cast<MotorPairController::pid_index_t>(packet.pidIndex); // NOLINT(hicpp-use-auto,modernize-use-auto)

    if (pidIndex >= MotorPairController::PID_COUNT) {
        //Serial.printf("Backchannel::packetSetPID invalid pidType:%d\r\n", packet.pidType);
        return;
    }

    switch (packet.setType) {
    case CommandPacketSetPID::SET_P:
        _motorPairController.setPID_P(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_I:
        _motorPairController.setPID_I(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_D:
        _motorPairController.setPID_D(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_F:
        _motorPairController.setPID_F(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_SETPOINT:
        _motorPairController.setPIDSetpoint(pidIndex, packet.value);
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
#if defined(USE_ESP32_PREFERENCES)
    case CommandPacketSetPID::SAVE_PITCH_BALANCE_ANGLE:
        // Save the balance angel, the value of packet.pidType is ignored.
        _preferences->putFloat(_motorPairController.getBalanceAngleName(), _motorPairController.getPitchBalanceAngleDegrees());
        break;
    case CommandPacketSetPID::SAVE_P:
    case CommandPacketSetPID::SAVE_I:
    case CommandPacketSetPID::SAVE_D:
    case CommandPacketSetPID::SAVE_F:
        //Serial.printf("Saved PID packetType:%d pidType:%d  setType:%d\r\n", packet.type, packet.pidType, packet.setType);
        // Currently we don't save individual PID constants: if any save request is received we save all the PID constants.
        _preferences->putPID(_motorPairController.getPIDName(pidIndex), _motorPairController.getPIDConstants(pidIndex));
        break;
#endif
    default:
        //Serial.printf("Backchannel::packetSetPID invalid setType:%d\r\n", packet.pidType);
        break;
    }

    if (transmit) {
        // send back the new data for display
        const int len = packTelemetryData_PID(_transmitDataBuffer, _telemetryID, _motorPairController, _telemetryScaleFactors);
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
                _ahrs,
                _motorPairController,
                _motorPairController.getOutputPowerTimeMicroSeconds(),
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
            const CommandPacketControl& packet = *reinterpret_cast<const CommandPacketControl*>(_receivedDataBuffer);
            packetControl(packet);
        } else if (type == CommandPacketRequestData::TYPE) {
            const CommandPacketRequestData& packet = *reinterpret_cast<const CommandPacketRequestData*>(_receivedDataBuffer);
            packetRequestData(packet);
        } else if (type == CommandPacketSetPID::TYPE) {
            const CommandPacketSetPID& packet = *reinterpret_cast<const CommandPacketSetPID*>(_receivedDataBuffer);
            packetSetPID(packet);
        }
        _received_data.len = 0;
    }
    return true;
}

#endif // USE_ESPNOW