#include "BackchannelSV.h"

#include <AHRS.h>
#include <AHRS_Task.h>
#include <HardwareSerial.h>
#include <ReceiverTelemetry.h>
#include <ReceiverTelemetryData.h>
#include <SV_Preferences.h>
#include <SV_Telemetry.h>
#include <SV_TelemetryData.h>
#include <VehicleControllerTask.h>



BackchannelSV::BackchannelSV(
        VehicleControllerTask& vehicleControllerTask,
        VehicleControllerBase& vehicleController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences
    ) :
    BackchannelBase(ahrsTask.getAHRS(), preferences),
    _vehicleControllerTask(vehicleControllerTask),
    _vehicleController(vehicleController),
    _ahrsTask(ahrsTask),
    _mainTask(mainTask),
    _receiver(receiver)
{
}

void BackchannelSV::packetSetOffset(const CommandPacketSetOffset& packet) {
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
#if defined(USE_ESPNOW)
        Serial.printf("Backchannel::packetSetOffset invalid itemIndex:%d\r\n", packet.setType);
#endif
        break;
    }

    if (transmit) {
        // send back the new data for display
        const size_t len = packTelemetryData_AHRS(_transmitDataBuffer, _telemetryID, _sequenceNumber, _ahrs, _vehicleController);
        sendData(_transmitDataBuffer, len);
    }
}

void BackchannelSV::packetRequestData(const CommandPacketRequestData& packet) {
    //Serial.printf("TransmitRequest packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);

    _requestType = packet.requestType;
    sendTelemetryPacket(packet.valueType);
}

bool BackchannelSV::sendTelemetryPacket(uint8_t subCommand)
{
    (void)subCommand;

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
            _ahrsTask,
            _vehicleControllerTask,
            _mainTask.getTickCountDelta(),
            _receiver.getTickCountDelta());
        //Serial.printf("tiLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_RECEIVER_DATA: {
        const size_t len = packTelemetryData_Receiver(_transmitDataBuffer, _telemetryID, _sequenceNumber, _receiver);
        //Serial.printf("receiverLen:%d\r\n", len);
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
*/
bool BackchannelSV::update(size_t receivedDataLength, uint8_t* receivedDataBuffer)
{
    if (receivedDataLength != 0) {
        // We have a packet, so process it

        const auto controlPacket = reinterpret_cast<const CommandPacketControl*>(receivedDataBuffer);
        if (controlPacket->id == _backchannelID) {
            // it's our packet, so process it

            //Serial.printf("Backchannel::update id:%x, type:%d, len:%d value:%d\r\n", packetControl->id, packetControl->type, packetControl->len, packetControl->value);
            switch (controlPacket->type) {
            case CommandPacketControl::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetControl(*reinterpret_cast<const CommandPacketControl*>(receivedDataBuffer));
                return true;
                break;
            case CommandPacketRequestData::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetRequestData(*reinterpret_cast<const CommandPacketRequestData*>(receivedDataBuffer));
                return true;
                break;
            case CommandPacketSetPID::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetPID(*reinterpret_cast<const CommandPacketSetPID*>(receivedDataBuffer));
                return true;
                break;
            case CommandPacketSetOffset::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetOffset(*reinterpret_cast<const CommandPacketSetOffset*>(receivedDataBuffer));
                return true;
                break;
            default:
                // do nothing
                break;
            } // end switch
        }
    }

    return false;
}
