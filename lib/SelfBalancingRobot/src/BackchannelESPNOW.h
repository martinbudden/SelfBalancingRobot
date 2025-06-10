#pragma once

#include <BackchannelSV.h>
#include <ESPNOW_Transceiver.h>


class MotorPairController;
class TelemetryScaleFactors;

class BackchannelTransceiverESPNOW : public BackchannelTransceiverBase {
public:
    BackchannelTransceiverESPNOW(
        uint8_t* receivedDataBuffer,
        size_t receivedDataBufferSize,
        const uint8_t* macAddress
    );
public:
    virtual int sendData(const uint8_t* data, size_t len) const override;
    virtual void WAIT_FOR_DATA_RECEIVED() override;
    virtual const uint8_t* getMacAddress() const override;
    virtual size_t getReceivedDataLength() const override;
    virtual void setReceivedDataLengthToZero() override;
    virtual uint32_t getTickCountDeltaAndReset() override;
private:
#if defined(USE_ESPNOW)
    ESPNOW_Transceiver _transceiver;
    ESPNOW_Transceiver::received_data_t _received_data;
    ESPNOW_Transceiver::peer_data_t _peer_data {};
#endif
};

class Backchannel : public BackchannelSV {
public:
    Backchannel(
        const uint8_t* backChannelMacAddress,
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
    virtual bool sendTelemetryPacket(uint8_t subCommand) override;
protected:
    virtual void packetControl(const CommandPacketControl& packet) override;
    virtual void packetSetPID(const CommandPacketSetPID& packet) override;
protected:
    BackchannelTransceiverESPNOW _transceiver;
    MotorPairController& _motorPairController;
    TelemetryScaleFactors& _telemetryScaleFactors;
    uint8_t _transmitDataBuffer[256] {}; // > ESP_NOW_MAX_DATA_LEN
    uint8_t _receivedDataBuffer[256] {}; // > ESP_NOW_MAX_DATA_LEN
};
