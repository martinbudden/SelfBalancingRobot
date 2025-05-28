#if defined(USE_ESPNOW)

#include "ESPNOW_Backchannel.h"
#include "SBR_Telemetry.h"
#include "SBR_TelemetryData.h"
#include "TelemetryScaleFactors.h"

#include <AHRS.h>
#include <HardwareSerial.h>
#include <ReceiverTelemetry.h>
#include <ReceiverTelemetryData.h>
#include <SV_Preferences.h>
#include <SV_Telemetry.h>
#include <SV_TelemetryData.h>

static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_AHRS) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_RECEIVER) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_SBR_PIDS) <= ESP_NOW_MAX_DATA_LEN);
static_assert(sizeof(TD_MPC) <= ESP_NOW_MAX_DATA_LEN);


Backchannel::Backchannel(ESPNOW_Transceiver& transceiver, const uint8_t* backchannelMacAddress, // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init) false positive
        MotorPairController& motorPairController,
        AHRS& ahrs,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        TelemetryScaleFactors& telemetryScaleFactors,
        SV_Preferences& preferences) :
    _transceiver(transceiver),
    _received_data(_receivedDataBuffer, sizeof(_receivedDataBuffer)),
    _motorPairController(motorPairController),
    _ahrs(ahrs),
    _mainTask(mainTask),
    _receiver(receiver),
    _telemetryScaleFactors(telemetryScaleFactors),
    _preferences(preferences)
{
    // NOTE: esp_now_send runs at a high priority, so shorter packets mean less blocking of the other tasks.
    static_assert(sizeof(TD_TASK_INTERVALS) < sizeof(_transmitDataBuffer)); // 12
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) < sizeof(_transmitDataBuffer)); // 28
    static_assert(sizeof(TD_AHRS) < sizeof(_transmitDataBuffer)); // 60
    static_assert(sizeof(TD_RECEIVER) < sizeof(_transmitDataBuffer)); // 40
    static_assert(sizeof(TD_SBR_PIDS) < sizeof(_transmitDataBuffer)); // 192
    static_assert(sizeof(TD_MPC) < sizeof(_transmitDataBuffer)); // 100

    _peer_data.receivedDataPtr = &_received_data;
    // add the backchannel as a secondary peer so data may be received from the backchannel
    addToTransceiverAsSecondaryPeer(backchannelMacAddress);
    // use the last 4 bytes of backchannelMacAddress as the backchannelID
    const uint8_t* pB = backchannelMacAddress;
    _backchannelID = (*(pB + 2U) << 24U) | (*(pB + 3U) << 16U) | (*(pB + 4U) << 8U) | *(pB + 5U); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)

    // use the last 4 bytes of myMacAddress as the telemetryID
    const uint8_t* pM = _transceiver.myMacAddress();
    _telemetryID = (*(pM + 2U) << 24U) | (*(pM + 3U) << 16U) | (*(pM + 4U) << 8U) | *(pM + 5U); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
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
    default:
        // do nothing
        break;
    } // end switch
}

void Backchannel::packetSetPID(const CommandPacketSetPID& packet) {
    //Serial.printf("SetPID packet type:%d, len:%d, pidIndex:%d setType:%d value:%f\r\n", packet.type, packet.len, packet.pidIndex, packet.setType, packet.value);
    const MotorPairController::pid_index_e pidIndex = static_cast<MotorPairController::pid_index_e>(packet.pidIndex); // NOLINT(hicpp-use-auto,modernize-use-auto)

    if (pidIndex >= MotorPairController::PID_COUNT) {
        //Serial.printf("Backchannel::packetSetPID invalid pidIndex:%d\r\n", packet.pidIndex);
        return;
    }

    bool transmit = false;
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
        _motorPairController.setPID_Setpoint(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_PITCH_BALANCE_ANGLE:
        // Set the balance angle, the value of packet.pidIndex is ignored.
        _motorPairController.setPitchBalanceAngleDegrees(packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SAVE_P: // NOLINT(bugprone-branch-clone) false positive
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_I:
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_D:
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_F:
        //Serial.printf("Saved PID packetType:%d pidIndex:%d setType:%d\r\n", packet.type, packet.pidIndex, packet.setType);
        // Currently we don't save individual PID constants: if any save request is received we save all the PID constants.
        _preferences.putPID(_motorPairController.getPID_Name(pidIndex), _motorPairController.getPID_Constants(pidIndex));
        break;
    case CommandPacketSetPID::RESET_PID:
        _preferences.putPID(_motorPairController.getPID_Name(pidIndex), PIDF::PIDF_t { SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET });
        break;
    case CommandPacketSetPID::SAVE_PITCH_BALANCE_ANGLE:
        // Save the balance angel, the value of packet.pidIndex is ignored.
        _preferences.putFloat(_motorPairController.getBalanceAngleName(), _motorPairController.getPitchBalanceAngleDegrees());
        break;
    default:
        //Serial.printf("Backchannel::packetSetPID invalid setType:%d\r\n", packet.pidIndex);
        break;
    }

    if (transmit) {
        // send back the new data for display
        const size_t len = packTelemetryData_PID(_transmitDataBuffer, _telemetryID, _sequenceNumber, _motorPairController, _telemetryScaleFactors);
        sendData(_transmitDataBuffer, len);
    }
}

void Backchannel::packetSetOffset(const CommandPacketSetOffset& packet) {
    IMU_Base::xyz_int32_t gyroOffset = _ahrs.getGyroOffsetMapped();
    IMU_Base::xyz_int32_t accOffset = _ahrs.getAccOffsetMapped();

    bool transmit = false;

    switch (packet.setType) {
    case CommandPacketSetOffset::SET_GYRO_OFFSET_X:
        gyroOffset.x = packet.value;
        _ahrs.setGyroOffsetMapped(gyroOffset);
        transmit = true;
        break;
    case CommandPacketSetOffset::SET_GYRO_OFFSET_Y:
        gyroOffset.y = packet.value;
        _ahrs.setGyroOffsetMapped(gyroOffset);
        transmit = true;
        break;
    case CommandPacketSetOffset::SET_GYRO_OFFSET_Z:
        gyroOffset.z = packet.value;
        _ahrs.setGyroOffsetMapped(gyroOffset);
        transmit = true;
        break;
    case CommandPacketSetOffset::SET_ACC_OFFSET_X:
        accOffset.x = packet.value;
        _ahrs.setAccOffsetMapped(accOffset);
        transmit = true;
        break;
    case CommandPacketSetOffset::SET_ACC_OFFSET_Y:
        accOffset.y = packet.value;
        _ahrs.setAccOffsetMapped(accOffset);
        transmit = true;
        break;
    case CommandPacketSetOffset::SET_ACC_OFFSET_Z:
        accOffset.z = packet.value;
        _ahrs.setAccOffsetMapped(accOffset);
        transmit = true;
        break;
    case CommandPacketSetOffset::SAVE_GYRO_OFFSET: // NOLINT(bugprone-branch-clone) false positive
        gyroOffset = _ahrs.getGyroOffset();
        _preferences.putGyroOffset(gyroOffset.x, gyroOffset.y, gyroOffset.z);
        break;
    case CommandPacketSetOffset::SAVE_ACC_OFFSET: // NOLINT(bugprone-branch-clone) false positive
        accOffset = _ahrs.getAccOffset();
        _preferences.putAccOffset(accOffset.x, accOffset.y, accOffset.z);
        break;
    default:
        Serial.printf("Backchannel::packetSetOffset invalid itemIndex:%d\r\n", packet.setType);
        break;
    }

    if (transmit) {
        // send back the new data for display
        const size_t len = packTelemetryData_AHRS(_transmitDataBuffer, _telemetryID, _sequenceNumber, _ahrs, _motorPairController);
        sendData(_transmitDataBuffer, len);
    }
}

void Backchannel::packetRequestData(const CommandPacketRequestData& packet) {
    //Serial.printf("TransmitRequest packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);

    _requestType = packet.value;
    sendTelemetryPacket();
}

bool Backchannel::sendTelemetryPacket()
{
    switch (_requestType) {
    case CommandPacketRequestData::NO_REQUEST: {
        return false;
    }
    case CommandPacketRequestData::REQUEST_STOP_SENDING_DATA: {
        // send a minimal packet so the client can reset its screen
        const size_t len = packTelemetryData_Minimal(_transmitDataBuffer, _telemetryID, _sequenceNumber);
        sendData(_transmitDataBuffer, len);
        // set _requestType to NO_REQUEST so no further data sent
        _requestType = CommandPacketRequestData::NO_REQUEST;
        break;
    }
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_DATA: {
        const size_t len = packTelemetryData_TaskIntervals(_transmitDataBuffer, _telemetryID, _sequenceNumber,
            _ahrs,
            _motorPairController,
            _mainTask.getTickCountDelta(),
            _receiver.getDroppedPacketCountDelta());
        //Serial.printf("tiLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_EXTENDED_DATA: {
        const size_t len = packTelemetryData_TaskIntervalsExtended(_transmitDataBuffer, _telemetryID, _sequenceNumber,
            _ahrs,
            _motorPairController,
            _motorPairController.getOutputPowerTimeMicroSeconds(),
            _mainTask.getTickCountDelta(),
            _transceiver.getTickCountDeltaAndReset(),
            _receiver.getDroppedPacketCountDelta());
        //Serial.printf("tiLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_AHRS_DATA: {
        const size_t len = packTelemetryData_AHRS(_transmitDataBuffer, _telemetryID, _sequenceNumber, _ahrs, _motorPairController);
        //Serial.printf("ahrsLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_RECEIVER_DATA: {
        const size_t len = packTelemetryData_Receiver(_transmitDataBuffer, _telemetryID, _sequenceNumber, _receiver);
        //Serial.printf("receiverLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_PID_DATA: {
        const size_t len = packTelemetryData_PID(_transmitDataBuffer, _telemetryID, _sequenceNumber, _motorPairController, _telemetryScaleFactors);
        //Serial.printf("pidLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        _requestType = CommandPacketRequestData::NO_REQUEST; // reset _sendType to NO_REQUEST, since SEND_PID_DATA is a one shot, as response to keypress
        break;
    }
    case CommandPacketRequestData::REQUEST_VEHICLE_CONTROLLER_DATA: {
        const size_t len = packTelemetryData_MPC(_transmitDataBuffer, _telemetryID, _sequenceNumber, _motorPairController);
        //Serial.printf("mpcLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        break;
    }
    default:
        return false;
    } // end switch
    return true;
}

/*!
If data was received then interpret it as a packet and return true.
Four types of packets may be received:

1. A command packet, for example a command to switch off the motors.
2. A request to transmit telemetry. In this case format the telemetry data and send it.
3. A request to set a PID value. In this case set the PID value and then send back a TD_SBR_PIDS packet for display.
4. A request to set an IMU offset value. In this case set the offset value and send back an TD_AHRS packet for display.


If no data was received then send a telemetry data packet if there is an outstanding telemetry request.
*/
bool Backchannel::update()
{
    if (_received_data.len != 0) {
        // We have a packet, so process it
        _received_data.len = 0; // Set _received_data.len to indicate we have processed this packet

        const auto controlPacket = reinterpret_cast<const CommandPacketControl*>(_receivedDataBuffer);
        if (controlPacket->id == _backchannelID) {
            // it's our packet, so process it

            //Serial.printf("Backchannel::update id:%x, type:%d, len:%d value:%d\r\n", packetControl->id, packetControl->type, packetControl->len, packetControl->value);
            switch (controlPacket->type) {
            case CommandPacketControl::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetControl(*reinterpret_cast<const CommandPacketControl*>(_receivedDataBuffer));
                break;
            case CommandPacketRequestData::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetRequestData(*reinterpret_cast<const CommandPacketRequestData*>(_receivedDataBuffer));
                break;
            case CommandPacketSetPID::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetPID(*reinterpret_cast<const CommandPacketSetPID*>(_receivedDataBuffer));
                break;
            case CommandPacketSetOffset::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetOffset(*reinterpret_cast<const CommandPacketSetOffset*>(_receivedDataBuffer));
                break;
            default:
                // do nothing
                break;
            } // end switch

            return true;
        }
    }

    // We haven't received a packet, so take the opportunity to send a telemetry packet if we have one to send
    return sendTelemetryPacket();
}

#endif // USE_ESPNOW
