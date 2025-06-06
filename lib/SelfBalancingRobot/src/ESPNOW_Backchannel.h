#pragma once

#include <CommandPacket.h>
#include <ESPNOW_Transceiver.h>

class AHRS;
class AHRS_Task;
class MotorPairController;
class MotorPairControllerTask;
class ReceiverBase;
class SV_Preferences;
class TaskBase;
class TelemetryScaleFactors;


class Backchannel {
public:
    Backchannel(ESPNOW_Transceiver& transceiver,
        const uint8_t* macAddress,
        MotorPairControllerTask& motorPairControllerTask,
        AHRS_Task& ahrsTask, 
        const TaskBase& mainTask, 
        const ReceiverBase& receiver, 
        TelemetryScaleFactors& telemetryScaleFactors, 
        SV_Preferences& preferences
    );
private:
    // Backchannel is not copyable or moveable
    Backchannel(const Backchannel&) = delete;
    Backchannel& operator=(const Backchannel&) = delete;
    Backchannel(Backchannel&&) = delete;
    Backchannel& operator=(Backchannel&&) = delete;
public:
    bool update();
private:
    inline esp_err_t addToTransceiverAsSecondaryPeer(const uint8_t* macAddress = nullptr) { return _transceiver.addSecondaryPeer(_received_data, macAddress); }
    inline esp_err_t sendData(const uint8_t* data, size_t len) const {return _transceiver.sendDataSecondary(data, len);}
    void packetControl(const CommandPacketControl& packet);
    void packetRequestData(const CommandPacketRequestData& packet);
    void packetSetPID(const CommandPacketSetPID& packet);
    void packetSetOffset(const CommandPacketSetOffset& packet);
    bool sendTelemetryPacket();
private:
    ESPNOW_Transceiver& _transceiver;
private:
    ESPNOW_Transceiver::received_data_t _received_data;
    ESPNOW_Transceiver::peer_data_t _peer_data {};
private:
    const MotorPairControllerTask& _motorPairControllerTask;
    MotorPairController& _motorPairController;
    const AHRS_Task& _ahrsTask;
    AHRS& _ahrs;
    const TaskBase& _mainTask;
    const ReceiverBase& _receiver;
    TelemetryScaleFactors& _telemetryScaleFactors;
    SV_Preferences& _preferences;
    uint32_t _telemetryID {0};
    uint32_t _backchannelID {0};
    uint32_t _sequenceNumber {0};
    uint32_t _requestType { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint8_t _receivedDataBuffer[ESP_NOW_MAX_DATA_LEN] {};
    uint8_t _transmitDataBuffer[ESP_NOW_MAX_DATA_LEN] {};
};
