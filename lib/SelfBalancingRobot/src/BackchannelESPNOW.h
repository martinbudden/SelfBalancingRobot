#pragma once

#include <BackchannelBase.h>
#include <CommandPacket.h>
#include <ESPNOW_Transceiver.h>

class AHRS_Task;
class MotorPairController;
class ReceiverBase;
class TaskBase;
class TelemetryScaleFactors;
class VehicleControllerTask;


class Backchannel : public BackchannelBase {
public:
    Backchannel(ESPNOW_Transceiver& transceiver,
        const uint8_t* macAddress,
        VehicleControllerTask& vehicleControllerTask,
        MotorPairController& motorPairController,
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
    virtual void WAIT_FOR_DATA_RECEIVED() override;
    virtual bool update() override;
    virtual bool sendTelemetryPacket() override;
private:
    inline esp_err_t addToTransceiverAsSecondaryPeer(const uint8_t* macAddress = nullptr) { return _transceiver.addSecondaryPeer(_received_data, macAddress); }
    esp_err_t sendData(const uint8_t* data, size_t len) const;
    void packetRequestData(const CommandPacketRequestData& packet);
    void packetControl(const CommandPacketControl& packet);
    void packetSetPID(const CommandPacketSetPID& packet);
    void packetSetOffset(const CommandPacketSetOffset& packet);
private:
    ESPNOW_Transceiver& _transceiver;
private:
    ESPNOW_Transceiver::received_data_t _received_data;
    ESPNOW_Transceiver::peer_data_t _peer_data {};
private:
    const VehicleControllerTask& _vehicleControllerTask;
    MotorPairController& _motorPairController;
    const AHRS_Task& _ahrsTask;
    const TaskBase& _mainTask;
    const ReceiverBase& _receiver;
    TelemetryScaleFactors& _telemetryScaleFactors;
    uint32_t _telemetryID {0};
    uint32_t _backchannelID {0};
    uint32_t _requestType { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint32_t _sequenceNumber {0};
#if defined(USE_ESPNOW)
    uint8_t _receivedDataBuffer[ESP_NOW_MAX_DATA_LEN] {};
    uint8_t _transmitDataBuffer[ESP_NOW_MAX_DATA_LEN] {};
#endif
};
