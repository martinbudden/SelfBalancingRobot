# pragma once

#include <cstdint>

#pragma pack(push, 1)
/*!
Command packet type 0 is reserved for future use.
*/
struct CommandPacketReserved {
    enum { TYPE = 0 };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketReserved)
    uint16_t value;
};

struct CommandPacketControl {
    enum { TYPE = 1 };
    enum { NO_ACTION = 0, MOTORS_SWITCH_OFF = 1, MOTORS_SWITCH_ON = 2, ENCODERS_RESET = 3, MPC_CONTROL_MODE_SERIAL_PIDS = 4, MPC_CONTROL_MODE_PARALLEL_PIDS = 5, MPC_CONTROL_MODE_POSITION_PID = 6 };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketControl)
    uint16_t value;
};

struct CommandPacketRequestData {
    enum { TYPE = 2 };
    enum { NO_REQUEST = 0, REQUEST_STOP_SENDING_DATA = 1, REQUEST_TICK_INTERVAL_DATA = 2, REQUEST_PID_DATA = 3, REQUEST_AHRS_DATA = 4, REQUEST_MPC_DATA = 5, REQUEST_RECEIVER_DATA = 6 };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketRequestData)
    uint16_t value;
};

struct CommandPacketSetPID {
    enum { TYPE = 3 };
    enum { PID_PITCH = 0, PID_SPEED = 1, PID_YAW_RATE = 2, PITCH_BALANCE_ANGLE = 3 };
    enum { NO_ACTION = 0, SET_SETPOINT = 1,
           SET_PITCH_BALANCE_ANGLE = 2,
           SAVE_PITCH_BALANCE_ANGLE = 3,
           SET_P = 4, SET_I = 5, SET_D = 6, SET_F = 7,
           SAVE_P = 8, SAVE_I = 9, SAVE_D = 10, SAVE_F = 11,
           RESET_PID = 12
        };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketSetPID)
    uint8_t pidType;
    uint8_t setType;
    float value;
};

struct CommandPacketSetFilter {
    enum { TYPE = 5 };
    enum { GYRO_ALL_LPF, GYRO_X_LPF, GYRO_Y_LP, GYRO_Z_LPF, ACC_ALL_LPF, ACC_X_LPF, ACC_Y_LPF, ACC_Z_LPF };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketSetPID)
    uint16_t frequency;
};
#pragma pack(pop)

