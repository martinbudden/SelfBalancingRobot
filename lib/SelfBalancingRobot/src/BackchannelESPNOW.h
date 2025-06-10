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
    virtual bool sendTelemetryPacket(uint8_t valueType) override;
private:
    virtual void packetControl(const CommandPacketControl& packet) override;
    virtual void packetSetPID(const CommandPacketSetPID& packet) override;
private:
    MotorPairController& _motorPairController;
    TelemetryScaleFactors& _telemetryScaleFactors;
};
