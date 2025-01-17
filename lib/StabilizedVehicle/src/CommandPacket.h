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
    enum { NO_ACTION = 0, MOTORS_SWITCH_OFF = 1, MOTORS_SWITCH_ON = 2, RESET = 3,
        CONTROL_MODE_0 = 20,
        CONTROL_MODE_1 = 21,
        CONTROL_MODE_2 = 22,
        CONTROL_MODE_3 = 23,
        CONTROL_MODE_4 = 24,
        CONTROL_MODE_5 = 25,
        CONTROL_MODE_6 = 26,
        CONTROL_MODE_7 = 27,
        CONTROL_MODE_8 = 28,
        CONTROL_MODE_9 = 29
    };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketControl)
    uint16_t value;
};

struct CommandPacketRequestData {
    enum { TYPE = 2 };
    enum { NO_REQUEST = 0,
        REQUEST_STOP_SENDING_DATA = 1,
        REQUEST_TICK_INTERVAL_DATA = 2,
        REQUEST_AHRS_DATA = 3,
        REQUEST_PID_DATA = 4,
        REQUEST_RECEIVER_DATA = 5,
        REQUEST_MOTOR_CONTROLLER_DATA = 6
    };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketRequestData)
    uint16_t value;
};

struct CommandPacketSetPID {
    enum { TYPE = 3 };
    enum { NO_ACTION = 0,
           SET_P = 1,  SET_I = 2,  SET_D = 3,  SET_F = 4,
           SAVE_P = 5, SAVE_I = 6, SAVE_D = 7, SAVE_F = 8,
           RESET_PID = 9,
           SET_SETPOINT = 10,
           SET_PITCH_BALANCE_ANGLE = 11,
           SAVE_PITCH_BALANCE_ANGLE = 12
        };
    enum { MPC_PITCH_ANGLE=0, MPC_SPEED=1, MPC_YAW_RATE=2, MPC_POSITION=3 };
    enum { FC_ROLL_RATE=0, FC_PITCH_RATE=1, FC_YAW_RATE=2, FC_ROLL_ANGLE=3, FC_PITCH_ANGLE=4 };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketSetPID)
    uint8_t pidIndex;
    uint8_t setType;
    float value;
};

struct CommandPacketSetFilter {
    enum { TYPE = 5 };
    enum { GYRO_ALL_LPF, GYRO_X_LPF, GYRO_Y_LPF, GYRO_Z_LPF, ACC_ALL_LPF, ACC_X_LPF, ACC_Y_LPF, ACC_Z_LPF };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketSetFilter)
    uint8_t filler0;
    uint8_t filler1;
    float value0;
    float value1;
    float value2;
    float value3;
};
#pragma pack(pop)

