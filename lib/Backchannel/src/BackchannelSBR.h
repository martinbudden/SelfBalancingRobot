#pragma once

#include <BackchannelStabilizedVehicle.h>

class MotorPairController;
class SV_Preferences;

/*!
Backchannel for Self Balancing Robot.
*/
class BackchannelSBR : public BackchannelStabilizedVehicle {
public:
    BackchannelSBR(
        BackchannelTransceiverBase& backchannelTransceiver,
        const uint8_t* backchannelMacAddress,
        const uint8_t* myMacAddress,
        MotorPairController& motorPairController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        const TaskBase* mainTask,
        SV_Preferences& preferences
    );
public:
    virtual bool sendPacket(uint8_t subCommand) override;
protected:
    virtual bool packetSetOffset(const CommandPacketSetOffset& packet) override;
    virtual bool packetControl(const CommandPacketControl& packet) override;
    virtual bool packetSetPID(const CommandPacketSetPID& packet) override;
protected:
    MotorPairController& _motorPairController;
    SV_Preferences& _preferences;
};
