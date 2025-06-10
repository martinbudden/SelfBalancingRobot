#pragma once

#include <BackchannelSBR.h>
#include <BackchannelTransceiverESPNOW.h>


class BackchannelESPNOW : public BackchannelSBR {
public:
    BackchannelESPNOW(
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
    // BackchannelESPNOW is not copyable or moveable
    BackchannelESPNOW(const BackchannelESPNOW&) = delete;
    BackchannelESPNOW& operator=(const BackchannelESPNOW&) = delete;
    BackchannelESPNOW(BackchannelESPNOW&&) = delete;
    BackchannelESPNOW& operator=(BackchannelESPNOW&&) = delete;
protected:
    BackchannelTransceiverESPNOW _transceiverESPNOW;
    // If using MSP, then the MSP packets are packed into _transmitDataBuffer by MSP::processOutCommand,
    // so _transmitDataBuffer must be large enough to hold the larges MSP packet.
    // If the packet length exceeds ESP_NOW_MAX_DATA_LEN, then it is not sent,
    // but we don't know its length until we have unpacked it.
    uint8_t _transmitDataBuffer[512] {};
    uint8_t _receivedDataBuffer[256] {}; // must be >= ESP_NOW_MAX_DATA_LEN
};
