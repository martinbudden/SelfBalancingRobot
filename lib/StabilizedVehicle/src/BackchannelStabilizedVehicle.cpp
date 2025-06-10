#include "BackchannelStabilizedVehicle.h"

#include <AHRS.h>
#include <AHRS_Task.h>
#include <HardwareSerial.h>
#include <ReceiverTelemetry.h>
#include <ReceiverTelemetryData.h>
#include <SV_Preferences.h>
#include <SV_Telemetry.h>
#include <SV_TelemetryData.h>
#include <VehicleControllerTask.h>


BackchannelStabilizedVehicle::BackchannelStabilizedVehicle(
        const uint8_t* backChannelMacAddress,
        BackchannelTransceiverBase& transceiver,
        VehicleControllerTask& vehicleControllerTask,
        VehicleControllerBase& vehicleController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        uint8_t* transmitDataBufferPtr,
        size_t transmitDataBufferSize,
        uint8_t* receivedDataBufferPtr,
        size_t receivedDataBufferSize
    ) :
    BackchannelBase(transceiver, ahrsTask.getAHRS(), preferences),
    _vehicleControllerTask(vehicleControllerTask),
    _vehicleController(vehicleController),
    _ahrsTask(ahrsTask),
    _mainTask(mainTask),
    _receiver(receiver),
    _transmitDataBufferPtr(transmitDataBufferPtr),
    _transmitDataBufferSize(transmitDataBufferSize),
    _receivedDataBufferPtr(receivedDataBufferPtr),
    _receivedDataBufferSize(receivedDataBufferSize)
{
    assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= transmitDataBufferSize); // 12
    assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= transmitDataBufferSize); // 28
    assert(sizeof(TD_AHRS) <= transmitDataBufferSize); // 60
    assert(sizeof(TD_RECEIVER) <= transmitDataBufferSize); // 40
    assert(sizeof(TD_MPC) <= transmitDataBufferSize); // 100
    assert(sizeof(TD_SBR_PIDS) <= transmitDataBufferSize); // 192

    // use the last 4 bytes of backchannelMacAddress as the backchannelID
    const uint8_t* pB = backChannelMacAddress;
    _backchannelID = (*(pB + 2U) << 24U) | (*(pB + 3U) << 16U) | (*(pB + 4U) << 8U) | *(pB + 5U); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
}

void BackchannelStabilizedVehicle::setTelemetryID(const uint8_t* macAddress)
{
    // use the last 4 bytes of myMacAddress as the telemetryID
    const uint8_t* pM = macAddress;
    _telemetryID = (*(pM + 2U) << 24U) | (*(pM + 3U) << 16U) | (*(pM + 4U) << 8U) | *(pM + 5U); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
}

void BackchannelStabilizedVehicle::packetSetOffset(const CommandPacketSetOffset& packet) {
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
        const size_t len = packTelemetryData_AHRS(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _ahrs, _vehicleController);
        sendData(_transmitDataBufferPtr, len);
    }
}

void BackchannelStabilizedVehicle::packetRequestData(const CommandPacketRequestData& packet) {
    //Serial.printf("TransmitRequest packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);

    _requestType = packet.requestType;
    sendTelemetryPacket(packet.valueType);
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
            _mainTask.getTickCountDelta(),
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
    const size_t receivedDataLength = _transceiver.getReceivedDataLength();
    if (receivedDataLength != 0) {
        _transceiver.setReceivedDataLengthToZero();
        // We have a packet, so process it

        const auto controlPacket = reinterpret_cast<const CommandPacketControl*>(_receivedDataBufferPtr);
        if (controlPacket->id == _backchannelID) {
            // it's our packet, so process it

            //Serial.printf("Backchannel::update id:%x, type:%d, len:%d value:%d\r\n", packetControl->id, packetControl->type, packetControl->len, packetControl->value);
            switch (controlPacket->type) {
            case CommandPacketControl::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetControl(*reinterpret_cast<const CommandPacketControl*>(_receivedDataBufferPtr));
                return true;
                break;
            case CommandPacketRequestData::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetRequestData(*reinterpret_cast<const CommandPacketRequestData*>(_receivedDataBufferPtr));
                return true;
                break;
            case CommandPacketSetPID::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetPID(*reinterpret_cast<const CommandPacketSetPID*>(_receivedDataBufferPtr));
                return true;
                break;
            case CommandPacketSetOffset::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetOffset(*reinterpret_cast<const CommandPacketSetOffset*>(_receivedDataBufferPtr));
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
