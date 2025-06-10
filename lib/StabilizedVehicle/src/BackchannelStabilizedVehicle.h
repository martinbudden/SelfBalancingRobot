#pragma once

#include <BackchannelBase.h>
#include <CommandPacket.h>

class AHRS_Task;
class ReceiverBase;
class TaskBase;
class VehicleControllerBase;
class VehicleControllerTask;

/*!
Backchannel that sends and receives packets that contain data for a stabilized vehicle.
*/
class BackchannelStabilizedVehicle : public BackchannelBase {
public:
    BackchannelStabilizedVehicle(
        VehicleControllerTask& vehicleControllerTask,
        VehicleControllerBase& vehicleController,
        AHRS_Task& ahrsTask,
        AHRS& ahrs,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences
    );
public:
    virtual bool sendTelemetryPacket(uint8_t subCommand) override;
    static uint32_t idFromMacAddress(const uint8_t* macAddress);
protected:
    virtual bool update() override;
    virtual bool packetRequestData(const CommandPacketRequestData& packet);
    virtual bool packetSetOffset(const CommandPacketSetOffset& packet);
    virtual bool packetControl(const CommandPacketControl& packet);
    virtual bool packetSetPID(const CommandPacketSetPID& packet);
protected:
    const VehicleControllerTask& _vehicleControllerTask;
    VehicleControllerBase& _vehicleController;
    const AHRS_Task& _ahrsTask;
    AHRS& _ahrs;
    const TaskBase& _mainTask;
    const ReceiverBase& _receiver;
    SV_Preferences& _preferences;
    uint32_t _telemetryID {0};
    uint32_t _backchannelID {0};
    uint32_t _requestType { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint32_t _sequenceNumber {0};
};
