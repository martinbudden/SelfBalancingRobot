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
        const uint8_t* backChannelMacAddress,
        BackchannelTransceiverBase& transceiver,
        VehicleControllerTask& vehicleControllerTask,
        VehicleControllerBase& vehicleController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        uint8_t* transmitDataBufferPtr,
        size_t transmitDataBufferSize,
        uint8_t* receivedDataBufferPtr,
        size_t receivedDataBufferSize
    );
public:
    virtual bool sendTelemetryPacket(uint8_t subCommand) override;
protected:
    void setTelemetryID(const uint8_t* macAddress);
    virtual bool update() override;
    virtual void packetRequestData(const CommandPacketRequestData& packet);
    virtual void packetSetOffset(const CommandPacketSetOffset& packet);
    virtual void packetControl(const CommandPacketControl& packet) = 0;
    virtual void packetSetPID(const CommandPacketSetPID& packet) = 0;
protected:
    const VehicleControllerTask& _vehicleControllerTask;
    VehicleControllerBase& _vehicleController;
    const AHRS_Task& _ahrsTask;
    const TaskBase& _mainTask;
    const ReceiverBase& _receiver;
    uint32_t _telemetryID {0};
    uint32_t _backchannelID {0};
    uint32_t _requestType { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint32_t _sequenceNumber {0};
    uint8_t* _transmitDataBufferPtr;
    size_t _transmitDataBufferSize;
    uint8_t* _receivedDataBufferPtr;
    size_t _receivedDataBufferSize;
};
