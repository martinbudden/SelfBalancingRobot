#pragma once

#include <BackchannelStabilizedVehicle.h>

class MotorPairController;

/*!
Backchannel for Self Balancing Robot.
*/
class BackchannelSBR : public BackchannelStabilizedVehicle {
public:
    BackchannelSBR(
        MotorPairController& motorPairController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        const TaskBase* mainTask
    );
public:
    virtual bool sendTelemetryPacket(uint8_t subCommand) override;
protected:
    virtual bool packetControl(const CommandPacketControl& packet) override;
    virtual bool packetSetPID(const CommandPacketSetPID& packet) override;
protected:
    MotorPairController& _motorPairController;
};
