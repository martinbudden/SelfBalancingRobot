#include "BackchannelSBR.h"

#include "SBR_Telemetry.h"

#include <ReceiverBase.h>
#include <SV_Preferences.h>
#include <SV_Telemetry.h>
#include <SV_TelemetryData.h>


BackchannelSBR::BackchannelSBR(
        const base_init_t& baseInit,
        uint32_t backchannelID,
        uint32_t telemetryID,
        MotorPairController& motorPairController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        const TaskBase* mainTask
    ) :
    BackchannelStabilizedVehicle(
        baseInit,
        backchannelID,
        telemetryID,
        motorPairController,
        ahrs,
        receiver,
        preferences,
        mainTask
    ),
    _motorPairController(motorPairController)
{
#if !defined(ESP_NOW_MAX_DATA_LEN)
#define ESP_NOW_MAX_DATA_LEN (250)
#endif
    static_assert(sizeof(TD_MPC) <= ESP_NOW_MAX_DATA_LEN); // 100
    static_assert(sizeof(TD_SBR_PIDS) <= ESP_NOW_MAX_DATA_LEN); // 192
}

bool BackchannelSBR::packetControl(const CommandPacketControl& packet)
{
    //Serial.printf("Control packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);
    switch (packet.control) {
    case CommandPacketControl::MOTORS_SWITCH_OFF:
        _motorPairController.motorsSwitchOff();
        return true;
        break;
    case CommandPacketControl::MOTORS_SWITCH_ON:
        _motorPairController.motorsSwitchOn();
        return true;
        break;
    case CommandPacketControl::RESET:
        _motorPairController.motorsResetEncodersToZero();
        return true;
        break;
    case CommandPacketControl::SET_MODE:
        _motorPairController.setControlMode(static_cast<MotorPairController::control_mode_e>(packet.value));
        return true;
        break;
    case CommandPacketControl::SET_PID_PROFILE:
        return false; // not implemented
    case CommandPacketControl::SET_RATES_PROFILE:
        return false; // not implemented
        break;
    default:
        // do nothing
        break;
    } // end switch
    return false;
}

bool BackchannelSBR::packetSetPID(const CommandPacketSetPID& packet)
{
    //Serial.printf("SetPID packet type:%d, len:%d, pidIndex:%d setType:%d value:%3d f0:%f\r\n", packet.type, packet.len, packet.pidIndex, packet.setType, packet.value, packet.f0);

    const auto pidIndex = static_cast<MotorPairController::pid_index_e>(packet.pidIndex);
    if (pidIndex >= MotorPairController::PID_COUNT) {
        //Serial.printf("Backchannel::packetSetPID invalid pidIndex:%d\r\n", packet.pidIndex);
        return false;
    }

    bool transmit = false;
    switch (packet.setType) {
    case CommandPacketSetPID::SET_P:
        _motorPairController.setPID_P_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_I:
        _motorPairController.setPID_I_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_D:
        _motorPairController.setPID_D_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_F:
        _motorPairController.setPID_F_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_SETPOINT:
        _motorPairController.setPID_Setpoint(pidIndex, packet.f0);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_PITCH_BALANCE_ANGLE:
        // Set the balance angle, the value of packet.pidIndex is ignored.
        _motorPairController.setPitchBalanceAngleDegrees(packet.f0);
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
        return true;
        break;
    case CommandPacketSetPID::RESET_PID:
        _preferences.putPID(_motorPairController.getPID_Name(pidIndex), PIDF::PIDF_t { SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET });
        return true;
        break;
    case CommandPacketSetPID::SAVE_PITCH_BALANCE_ANGLE:
        // Save the balance angel, the value of packet.pidIndex is ignored.
        _preferences.putFloat(_motorPairController.getBalanceAngleName(), _motorPairController.getPitchBalanceAngleDegrees());
        return true;
        break;
    default:
        //Serial.printf("Backchannel::packetSetPID invalid setType:%d\r\n", packet.pidIndex);
        break;
    }

    if (transmit) {
        // send back the new data for display
        const size_t len = packTelemetryData_PID(
            _transmitDataBufferPtr,
            _telemetryID,
            _sequenceNumber,
            _motorPairController, 
            _motorPairController.getControlMode(),
            _motorPairController.getPitchBalanceAngleDegrees(),
            0.0F
        );
        sendData(_transmitDataBufferPtr, len);
        return true;
    }
    return false;
}

bool BackchannelSBR::sendPacket(uint8_t subCommand)
{
    if (BackchannelStabilizedVehicle::sendPacket(subCommand)) {
        // if the base class has sent the packet then we have nothing to do
        return true;
    }

    switch (_requestType) {
    case CommandPacketRequestData::REQUEST_PID_DATA: {
        const size_t len = packTelemetryData_PID(
            _transmitDataBufferPtr,
            _telemetryID,
            _sequenceNumber,
            _motorPairController, 
            _motorPairController.getControlMode(),
            _motorPairController.getPitchBalanceAngleDegrees(),
            0.0F
        );
        //Serial.printf("pidLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        _requestType = CommandPacketRequestData::NO_REQUEST; // reset _requestType to NO_REQUEST, since REQUEST_PID_DATA is a one shot, as response to keypress
        break;
    }
    case CommandPacketRequestData::REQUEST_VEHICLE_CONTROLLER_DATA: {
        const size_t len = packTelemetryData_MPC(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _motorPairController);
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
