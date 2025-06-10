#pragma once

#include <BackchannelStabilizedVehicle.h>
#include <BackchannelTransceiverESPNOW.h>

class MotorPairController;
class TelemetryScaleFactors;

/*!
Backchannel for Self Balancing Robot.
*/
class BackchannelSBR : public BackchannelStabilizedVehicle {
public:
    BackchannelSBR(
        const uint8_t* backChannelMacAddress,
        BackchannelTransceiverBase& transceiver,
        VehicleControllerTask& vehicleControllerTask,
        MotorPairController& motorPairController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        TelemetryScaleFactors& telemetryScaleFactors,
        SV_Preferences& preferences,
        uint8_t* transmitDataBufferPtr,
        size_t transmitDataBufferSize,
        uint8_t* receivedDataBufferPtr,
        size_t receivedDataBufferSize
    );
public:
    virtual bool sendTelemetryPacket(uint8_t subCommand) override;
protected:
    virtual void packetControl(const CommandPacketControl& packet) override;
    virtual void packetSetPID(const CommandPacketSetPID& packet) override;
protected:
    MotorPairController& _motorPairController;
    TelemetryScaleFactors& _telemetryScaleFactors;
};

class Backchannel : public BackchannelSBR {
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
protected:
    BackchannelTransceiverESPNOW _transceiver;
    // If using MSP, then the MSP packets are packed into _transmitDataBuffer by MSP::processOutCommand,
    // so _transmitDataBuffer must be large enough to hold the larges MSP packet.
    // If the packet length exceeds ESP_NOW_MAX_DATA_LEN, then it is not sent,
    // but we don't know its length until we have unpacked it.
    uint8_t _transmitDataBuffer[512] {};
    uint8_t _receivedDataBuffer[256] {}; // must be >= ESP_NOW_MAX_DATA_LEN
};
