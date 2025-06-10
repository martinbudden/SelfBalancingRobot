#if defined(USE_ESPNOW)

#include "BackchannelESPNOW.h"
#include "SBR_Telemetry.h"
#include "TelemetryScaleFactors.h"
#include <MotorPairController.h>
#include <ReceiverBase.h>
#include <SV_Preferences.h>
#include <SV_Telemetry.h>
#include <SV_TelemetryData.h>
#include <VehicleControllerTask.h>


Backchannel::Backchannel(
        ESPNOW_Transceiver& transceiver,
        const uint8_t* backchannelMacAddress,
        VehicleControllerTask& vehicleControllerTask,
        MotorPairController& motorPairController,
        AHRS_Task& ahrsTask,
        const TaskBase& mainTask,
        const ReceiverBase& receiver,
        TelemetryScaleFactors& telemetryScaleFactors,
        SV_Preferences& preferences
    ) :
    BackchannelSV(
        transceiver,
        backchannelMacAddress,
        vehicleControllerTask,
        motorPairController,
        ahrsTask,
        mainTask,
        receiver,
        preferences
    ),
    _motorPairController(motorPairController),
    _telemetryScaleFactors(telemetryScaleFactors)
{
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
        const size_t len = packTelemetryData_PID(_transmitDataBuffer, _telemetryID, _sequenceNumber, _motorPairController, _telemetryScaleFactors);
        sendData(_transmitDataBuffer, len);
    }
}

bool Backchannel::sendTelemetryPacket(uint8_t valueType)
{
    if (BackchannelSV::sendTelemetryPacket(valueType)) {
        // if the base class has sent the packet then we have nothing to do
        return true;
    }

    switch (_requestType) {
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_EXTENDED_DATA: {
        const size_t len = packTelemetryData_TaskIntervalsExtended(_transmitDataBuffer, _telemetryID, _sequenceNumber,
            _ahrsTask,
            _vehicleControllerTask,
            _motorPairController.getOutputPowerTimeMicroSeconds(),
            _mainTask.getTickCountDelta(),
            _transceiver.getTickCountDeltaAndReset(),
            _receiver.getTickCountDelta());
        //Serial.printf("tiLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_AHRS_DATA: {
        const size_t len = packTelemetryData_AHRS(_transmitDataBuffer, _telemetryID, _sequenceNumber, _ahrs, _motorPairController);
        //Serial.printf("ahrsLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_PID_DATA: {
        const size_t len = packTelemetryData_PID(_transmitDataBuffer, _telemetryID, _sequenceNumber, _motorPairController, _telemetryScaleFactors);
        //Serial.printf("pidLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        _requestType = CommandPacketRequestData::NO_REQUEST; // reset _sendType to NO_REQUEST, since SEND_PID_DATA is a one shot, as response to keypress
        break;
    }
    case CommandPacketRequestData::REQUEST_VEHICLE_CONTROLLER_DATA: {
        const size_t len = packTelemetryData_MPC(_transmitDataBuffer, _telemetryID, _sequenceNumber, _vehicleControllerTask, _motorPairController);
        //Serial.printf("mpcLen:%d\r\n", len);
        sendData(_transmitDataBuffer, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_MSP_DATA: {
        (void)valueType;
        //const size_t len = packTelemetryData_MSP(_transmitDataBuffer, _telemetryID, _sequenceNumber, _msp, valueType);
        //if (len <= ESP_NOW_MAX_DATA_LEN) {
        //    sendData(_transmitDataBuffer, len);
        //}
        break;
    }
    default:
        return false;
    } // end switch
    return true;
}

#endif // USE_ESPNOW
