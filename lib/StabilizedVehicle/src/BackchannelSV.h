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
    BackchannelSV(ESPNOW_Transceiver& transceiver,
        const uint8_t* macAddress,
        VehicleControllerTask& vehicleControllerTask,
        VehicleControllerBase& vehicleController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences
    );
public:
    virtual void WAIT_FOR_DATA_RECEIVED() override;
    virtual bool update() override;
    virtual bool sendTelemetryPacket(uint8_t valueType) override;
protected:
    inline esp_err_t addToTransceiverAsSecondaryPeer(const uint8_t* macAddress = nullptr) { return _transceiver.addSecondaryPeer(_received_data, macAddress); }
    esp_err_t sendData(const uint8_t* data, size_t len) const;
    virtual void packetRequestData(const CommandPacketRequestData& packet);
    virtual void packetSetOffset(const CommandPacketSetOffset& packet);
    virtual void packetControl(const CommandPacketControl& packet) = 0;
    virtual void packetSetPID(const CommandPacketSetPID& packet) = 0;
protected:
    ESPNOW_Transceiver& _transceiver;
    ESPNOW_Transceiver::received_data_t _received_data;
    ESPNOW_Transceiver::peer_data_t _peer_data {};
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
#if defined(USE_ESPNOW)
    uint8_t _receivedDataBuffer[ESP_NOW_MAX_DATA_LEN] {};
    uint8_t _transmitDataBuffer[ESP_NOW_MAX_DATA_LEN] {};
#endif
};