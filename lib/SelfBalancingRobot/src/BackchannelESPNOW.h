#pragma once

#include <BackchannelSV.h>


class MotorPairController;
class TelemetryScaleFactors;


class Backchannel : public BackchannelSV {
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
    virtual bool sendTelemetryPacket(uint8_t subCommand) override;
protected:
    virtual void packetControl(const CommandPacketControl& packet) override;
    virtual void packetSetPID(const CommandPacketSetPID& packet) override;
    virtual int sendData(const uint8_t* data, size_t len) const override;
protected:
    MotorPairController& _motorPairController;
    TelemetryScaleFactors& _telemetryScaleFactors;
#if defined(USE_ESPNOW)
    ESPNOW_Transceiver& _transceiver;
    ESPNOW_Transceiver::received_data_t _received_data;
    ESPNOW_Transceiver::peer_data_t _peer_data {};
#endif
    uint8_t _receivedDataBuffer[256] {}; // > ESP_NOW_MAX_DATA_LEN
};
