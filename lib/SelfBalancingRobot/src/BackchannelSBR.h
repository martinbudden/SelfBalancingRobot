#pragma once

#include <BackchannelStabilizedVehicle.h>

class MotorPairController;
class TelemetryScaleFactors;

/*!
Backchannel for Self Balancing Robot.
*/
class BackchannelSBR : public BackchannelStabilizedVehicle {
public:
    BackchannelSBR(
        VehicleControllerTask& vehicleControllerTask,
        MotorPairController& motorPairController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        TelemetryScaleFactors& telemetryScaleFactors,
        uint8_t* transmitDataBufferPtr,
        size_t transmitDataBufferSize,
        uint8_t* receivedDataBufferPtr,
        size_t receivedDataBufferSize
    );
public:
    virtual bool sendTelemetryPacket(uint8_t subCommand) override;
protected:
    virtual bool packetControl(const CommandPacketControl& packet) override;
    virtual bool packetSetPID(const CommandPacketSetPID& packet) override;
protected:
    MotorPairController& _motorPairController;
    TelemetryScaleFactors& _telemetryScaleFactors;
};
