#pragma once

#include <BackchannelStabilizedVehicle.h>
#include <BackchannelTransceiverESPNOW.h>

class MotorPairController;
class TelemetryScaleFactors;


class Backchannel : public BackchannelStabilizedVehicle {
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
