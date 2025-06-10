#pragma once

#include <BackchannelBase.h>
#include <CommandPacket.h>
#include <ESPNOW_Transceiver.h>

class AHRS_Task;
class ReceiverBase;
class TaskBase;
class VehicleControllerBase;
class VehicleControllerTask;

class BackchannelSV : public BackchannelBase {
public:
    BackchannelSV(
        VehicleControllerTask& vehicleControllerTask,
        VehicleControllerBase& vehicleController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences
    );
public:
    virtual bool sendTelemetryPacket(uint8_t subCommand) override;
protected:
    bool update(size_t receivedDataLength, uint8_t* receivedDataBuffer);
    virtual int sendData(const uint8_t* data, size_t len) const = 0;
    virtual void packetRequestData(const CommandPacketRequestData& packet);
    virtual void packetSetOffset(const CommandPacketSetOffset& packet);
    virtual void packetControl(const CommandPacketControl& packet) = 0;
    virtual void packetSetPID(const CommandPacketSetPID& packet) = 0;
protected:
    const VehicleControllerTask& _vehicleControllerTask;
    VehicleControllerBase& _vehicleController;
    const AHRS_Task& _ahrsTask;
    const TaskBase& _mainTask;
    const ReceiverBase& _receiver;
    uint32_t _telemetryID {0};
    uint32_t _backchannelID {0};
    uint32_t _requestType { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint32_t _sequenceNumber {0};
    uint8_t _transmitDataBuffer[256] {}; // ESP_NOW_MAX_DATA_LEN = 250
};