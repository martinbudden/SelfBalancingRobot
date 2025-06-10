#include "BackchannelESPNOW.h"

#include "SBR_Telemetry.h"

#include <ReceiverBase.h>
#include <SV_Preferences.h>
#include <SV_Telemetry.h>


Backchannel::Backchannel(
        const uint8_t* backChannelMacAddress,
        VehicleControllerTask& vehicleControllerTask,
        MotorPairController& motorPairController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        TelemetryScaleFactors& telemetryScaleFactors,
        SV_Preferences& preferences
    ) :
    BackchannelStabilizedVehicle(
        backChannelMacAddress,
        _transceiver,
        vehicleControllerTask,
        motorPairController,
        ahrsTask,
        mainTask,
        receiver,
        preferences,
        &_transmitDataBuffer[0],
        sizeof(_transmitDataBuffer),
        &_receivedDataBuffer[0],
        sizeof(_receivedDataBuffer)
    ),
    _transceiver(_receivedDataBuffer, sizeof(_receivedDataBuffer), backChannelMacAddress),
    _motorPairController(motorPairController),
    _telemetryScaleFactors(telemetryScaleFactors)
{
#if defined(USE_ESPNOW)
    // NOTE: esp_now_send runs at a high priority, so shorter packets mean less blocking of the other tasks.
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= ESP_NOW_MAX_DATA_LEN); // 12
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= ESP_NOW_MAX_DATA_LEN); // 28
    static_assert(sizeof(TD_AHRS) <= ESP_NOW_MAX_DATA_LEN); // 60
    //static_assert(sizeof(TD_RECEIVER) <= ESP_NOW_MAX_DATA_LEN); // 40
    static_assert(sizeof(TD_MPC) <= ESP_NOW_MAX_DATA_LEN); // 100
    static_assert(sizeof(TD_SBR_PIDS) <= ESP_NOW_MAX_DATA_LEN); // 192
    static_assert(sizeof(_transmitDataBuffer) >= ESP_NOW_MAX_DATA_LEN);
    static_assert(sizeof(_receivedDataBuffer) >= ESP_NOW_MAX_DATA_LEN);
#endif
    setTelemetryID(_transceiver.getMacAddress());
}

void Backchannel::packetControl(const CommandPacketControl& packet) {
    //Serial.printf("Control packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);
    switch (packet.value) {
    case CommandPacketControl::MOTORS_SWITCH_OFF:
        _motorPairController.motorsSwitchOff();
        break;
    case CommandPacketControl::MOTORS_SWITCH_ON:
        _motorPairController.motorsSwitchOn();
        break;
    case CommandPacketControl::RESET:
        _motorPairController.motorsResetEncodersToZero();
        break;
    case CommandPacketControl::CONTROL_MODE_0:
        _motorPairController.setControlMode(MotorPairController::CONTROL_MODE_SERIAL_PIDS);
        _telemetryScaleFactors.setControlMode(MotorPairController::CONTROL_MODE_SERIAL_PIDS);
        break;
    case CommandPacketControl::CONTROL_MODE_1:
        _motorPairController.setControlMode(MotorPairController::CONTROL_MODE_PARALLEL_PIDS);
        _telemetryScaleFactors.setControlMode(MotorPairController::CONTROL_MODE_PARALLEL_PIDS);
        break;
    default:
        // do nothing
        break;
    } // end switch
}

void Backchannel::packetSetPID(const CommandPacketSetPID& packet) {
    //Serial.printf("SetPID packet type:%d, len:%d, pidIndex:%d setType:%d value:%f\r\n", packet.type, packet.len, packet.pidIndex, packet.setType, packet.value);
    const MotorPairController::pid_index_e pidIndex = static_cast<MotorPairController::pid_index_e>(packet.pidIndex); // NOLINT(hicpp-use-auto,modernize-use-auto)

    if (pidIndex >= MotorPairController::PID_COUNT) {
        //Serial.printf("Backchannel::packetSetPID invalid pidIndex:%d\r\n", packet.pidIndex);
        return;
    }

    bool transmit = false;
    switch (packet.setType) {
    case CommandPacketSetPID::SET_P:
        _motorPairController.setPID_P(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_I:
        _motorPairController.setPID_I(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_D:
        _motorPairController.setPID_D(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_F:
        _motorPairController.setPID_F(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_SETPOINT:
        _motorPairController.setPID_Setpoint(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_PITCH_BALANCE_ANGLE:
        // Set the balance angle, the value of packet.pidIndex is ignored.
        _motorPairController.setPitchBalanceAngleDegrees(packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SAVE_P: // NOLINT(bugprone-branch-clone) false positive
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_I:
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_D:
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_F:
        //Serial.printf("Saved PID packetType:%d pidIndex:%d setType:%d\r\n", packet.type, packet.pidIndex, packet.setType);
        // Currently we don't save individual PID constants: if any save request is received we save all the PID constants.
        _preferences.putPID(_motorPairController.getPID_Name(pidIndex), _motorPairController.getPID_Constants(pidIndex));
        break;
    case CommandPacketSetPID::RESET_PID:
        _preferences.putPID(_motorPairController.getPID_Name(pidIndex), PIDF::PIDF_t { SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET });
        break;
    case CommandPacketSetPID::SAVE_PITCH_BALANCE_ANGLE:
        // Save the balance angel, the value of packet.pidIndex is ignored.
        _preferences.putFloat(_motorPairController.getBalanceAngleName(), _motorPairController.getPitchBalanceAngleDegrees());
        break;
    default:
        //Serial.printf("Backchannel::packetSetPID invalid setType:%d\r\n", packet.pidIndex);
        break;
    }

    if (transmit) {
        // send back the new data for display
        const size_t len = packTelemetryData_PID(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _motorPairController, _telemetryScaleFactors);
        sendData(_transmitDataBufferPtr, len);
    }
}

bool Backchannel::sendTelemetryPacket(uint8_t subCommand)
{
    if (BackchannelStabilizedVehicle::sendTelemetryPacket(subCommand)) {
        // if the base class has sent the packet then we have nothing to do
        return true;
    }

    switch (_requestType) {
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_EXTENDED_DATA: {
        const size_t len = packTelemetryData_TaskIntervalsExtended(_transmitDataBufferPtr, _telemetryID, _sequenceNumber,
            _ahrsTask,
            _vehicleControllerTask,
            _motorPairController.getOutputPowerTimeMicroSeconds(),
            _mainTask.getTickCountDelta(),
            _transceiver.getTickCountDeltaAndReset(),
            _receiver.getTickCountDelta());
        //Serial.printf("tiLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_PID_DATA: {
        const size_t len = packTelemetryData_PID(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _motorPairController, _telemetryScaleFactors);
        //Serial.printf("pidLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        _requestType = CommandPacketRequestData::NO_REQUEST; // reset _sendType to NO_REQUEST, since SEND_PID_DATA is a one shot, as response to keypress
        break;
    }
    case CommandPacketRequestData::REQUEST_VEHICLE_CONTROLLER_DATA: {
        const size_t len = packTelemetryData_MPC(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _vehicleControllerTask, _motorPairController);
        //Serial.printf("mpcLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_MSP_DATA: {
        (void)subCommand;
        //const size_t len = packTelemetryData_MSP(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _msp, subCommand);
        //if (len <= ESP_NOW_MAX_DATA_LEN) {
        //    sendData(_transmitDataBufferPtr, len);
        //}
        break;
    }
    default:
        return false;
    } // end switch
    return true;
}
