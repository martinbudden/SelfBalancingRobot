#include "BackchannelStabilizedVehicle.h"

#include <AHRS.h>
#include <AHRS_Task.h>
#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif
#include <ReceiverTelemetry.h>
#include <ReceiverTelemetryData.h>
#include <SV_Preferences.h>
#include <SV_Telemetry.h>
#include <SV_TelemetryData.h>
#include <VehicleControllerTask.h>


BackchannelStabilizedVehicle::BackchannelStabilizedVehicle(
        VehicleControllerTask& vehicleControllerTask,
        VehicleControllerBase& vehicleController,
        AHRS_Task& ahrsTask,
        AHRS& ahrs,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences
    ) :
    _vehicleControllerTask(vehicleControllerTask),
    _vehicleController(vehicleController),
    _ahrsTask(ahrsTask),
    _ahrs(ahrs),
    _mainTask(mainTask),
    _receiver(receiver),
    _preferences(preferences)
{
#if !defined(ESP_NOW_MAX_DATA_LEN)
#define ESP_NOW_MAX_DATA_LEN (250)
#endif
    // NOTE: esp_now_send runs at a high priority, so shorter packets mean less blocking of the other tasks.
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= ESP_NOW_MAX_DATA_LEN); // 12
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= ESP_NOW_MAX_DATA_LEN); // 28
    static_assert(sizeof(TD_AHRS) <= ESP_NOW_MAX_DATA_LEN); // 60
    //static_assert(sizeof(TD_RECEIVER) <= ESP_NOW_MAX_DATA_LEN); // 40
}

uint32_t BackchannelStabilizedVehicle::idFromMacAddress(const uint8_t* macAddress)
{
    // use the last 4 bytes of th MacAddress as ID
    const uint8_t* pM = macAddress;
    const uint32_t ret =  (*(pM + 2U) << 24U) | (*(pM + 3U) << 16U) | (*(pM + 4U) << 8U) | *(pM + 5U); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)

    return ret;
}

bool BackchannelStabilizedVehicle::packetSetOffset(const CommandPacketSetOffset& packet)
{
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
        return true;
        break;
    case CommandPacketSetOffset::SAVE_ACC_OFFSET: // NOLINT(bugprone-branch-clone) false positive
        accOffset = _ahrs.getAccOffset();
        _preferences.putAccOffset(accOffset.x, accOffset.y, accOffset.z);
        return true;
        break;
    default:
#if defined(USE_ESPNOW)
        Serial.printf("Backchannel::packetSetOffset invalid itemIndex:%d\r\n", packet.setType);
#endif
        break;
    }

    if (transmit) {
        // send back the new data for display
        const size_t len = packTelemetryData_AHRS(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _ahrs, _vehicleController);
        sendData(_transmitDataBufferPtr, len);
        return true;
    }
    return false;
}

bool BackchannelStabilizedVehicle::packetControl(const CommandPacketControl& packet)
{
    (void)packet;
    return false;
}

bool BackchannelStabilizedVehicle::packetSetPID(const CommandPacketSetPID& packet)
{
    (void)packet;
    return false;
}

bool BackchannelStabilizedVehicle::packetRequestData(const CommandPacketRequestData& packet)
{
    //Serial.printf("TransmitRequest packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);

    _requestType = packet.requestType;
    sendTelemetryPacket(packet.valueType);
    return true;
}

bool BackchannelStabilizedVehicle::sendTelemetryPacket(uint8_t subCommand)
{
    (void)subCommand;

    switch (_requestType) {
    case CommandPacketRequestData::NO_REQUEST: {
        return false;
    }
    case CommandPacketRequestData::REQUEST_STOP_SENDING_DATA: {
        // send a minimal packet so the client can reset its screen
        const size_t len = packTelemetryData_Minimal(_transmitDataBufferPtr, _telemetryID, _sequenceNumber);
        sendData(_transmitDataBufferPtr, len);
        // set _requestType to NO_REQUEST so no further data sent
        _requestType = CommandPacketRequestData::NO_REQUEST;
        break;
    }
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_DATA: {
        const size_t len = packTelemetryData_TaskIntervals(_transmitDataBufferPtr, _telemetryID, _sequenceNumber,
            _ahrsTask,
            _vehicleControllerTask,
            0, //_mainTask.getTickCountDelta(),
            _receiver.getTickCountDelta());
        //Serial.printf("tiLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_AHRS_DATA: {
        const size_t len = packTelemetryData_AHRS(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _ahrs, _vehicleController);
        //Serial.printf("ahrsLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_RECEIVER_DATA: {
        const size_t len = packTelemetryData_Receiver(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _receiver);
        //Serial.printf("receiverLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
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
bool BackchannelStabilizedVehicle::update()
{
    //Serial.printf("update\r\n");
    const size_t receivedDataLength = _backchannelTransceiverPtr->getReceivedDataLength();
    if (receivedDataLength != 0) {
        //Serial.printf("rdLen:%d\r\n", receivedDataLength);
        _backchannelTransceiverPtr->setReceivedDataLengthToZero();
        // We have a packet, so process it

        const auto* const controlPacket = reinterpret_cast<const CommandPacketControl*>(_receivedDataBufferPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        if (controlPacket->id == _backchannelID) {
            // it's our packet, so process it

            //Serial.printf("Backchannel::update id:%x, type:%d, len:%d value:%d\r\n", controlPacket->id, controlPacket->type, controlPacket->len, controlPacket->value);
            switch (controlPacket->type) {
            case CommandPacketControl::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetControl(*reinterpret_cast<const CommandPacketControl*>(_receivedDataBufferPtr)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
                return true;
                break;
            case CommandPacketRequestData::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetRequestData(*reinterpret_cast<const CommandPacketRequestData*>(_receivedDataBufferPtr)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
                return true;
                break;
            case CommandPacketSetPID::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetPID(*reinterpret_cast<const CommandPacketSetPID*>(_receivedDataBufferPtr)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
                return true;
                break;
            case CommandPacketSetOffset::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetOffset(*reinterpret_cast<const CommandPacketSetOffset*>(_receivedDataBufferPtr)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
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
