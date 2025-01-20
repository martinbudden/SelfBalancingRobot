#pragma once

#include <ESPNOW_Transceiver.h>

class CommandPacketControl;
class CommandPacketRequestData;
class CommandPacketSetPID;

class AHRS;
class MotorPairController;
class TaskBase;
class TelemetryScaleFactors;
class ReceiverBase;
class SV_Preferences;


class Backchannel {
public:
    Backchannel(ESPNOW_Transceiver& transceiver, const uint8_t* macAddress, MotorPairController& flightController, const AHRS& ahrs, const TaskBase& mainTask, const ReceiverBase& receiver, TelemetryScaleFactors& telemetryScaleFactors, SV_Preferences* preferences);
    Backchannel(ESPNOW_Transceiver& transceiver, const uint8_t* macAddress, MotorPairController& flightController, const AHRS& ahrs, const TaskBase& mainTask, const ReceiverBase& receiver, TelemetryScaleFactors& telemetryScaleFactors) :
        Backchannel(transceiver, macAddress, flightController, ahrs, mainTask, receiver, telemetryScaleFactors, nullptr) {}
private:
    // Backchannel is not copyable or moveable
    Backchannel(const Backchannel&) = delete;
    Backchannel& operator=(const Backchannel&) = delete;
    Backchannel(Backchannel&&) = delete;
    Backchannel& operator=(Backchannel&&) = delete;
public:
    bool update();
private:
    inline esp_err_t addToTransceiverAsSecondaryPeer(const uint8_t* macAddress = nullptr) { return _transceiver.addSecondaryPeer(_received_data, macAddress); }
    inline esp_err_t sendData(uint8_t* data, uint16_t len) const {return _transceiver.sendDataSecondary(data, len);}
    void packetControl(const CommandPacketControl& packet);
    void packetRequestData(const CommandPacketRequestData& packet);
    void packetSetPID(const CommandPacketSetPID& packet);
private:
    ESPNOW_Transceiver& _transceiver;
private:
    uint8_t _receivedDataBuffer[ESP_NOW_MAX_DATA_LEN] {};
    ESPNOW_Transceiver::received_data_t _received_data;
    ESPNOW_Transceiver::peer_data_t _peer_data {};
    uint8_t _transmitDataBuffer[ESP_NOW_MAX_DATA_LEN] {};
private:
    MotorPairController& _motorPairController;
    const AHRS& _ahrs;
    const TaskBase& _mainTask;
    const ReceiverBase& _receiver;
    TelemetryScaleFactors& _telemetryScaleFactors;
    SV_Preferences* _preferences {nullptr};
    uint32_t _telemetryID {0};
    uint32_t _backchannelID {0};
    enum SendType { SEND_NO_DATA=0, RESET_SCREEN_AND_SEND_NO_DATA=1, SEND_TICK_INTERVAL_DATA=2, SEND_AHRS_DATA=3, SEND_RECEIVER_DATA=4, SEND_PID_DATA=5, SEND_MPC_DATA=6 };
    SendType _sendType { RESET_SCREEN_AND_SEND_NO_DATA };
};
