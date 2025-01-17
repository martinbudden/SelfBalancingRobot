# pragma once

/*!
Packet definitions of telemetry data specific to Self Balancing Robots.
*/


#include "MotorPairControllerTelemetry.h"
#include <array>
#include <cstdint>
#include <xyz_type.h>

#pragma pack(push, 1)
/*!
Packet for the the transmission of PID constants, setpoints, and the balance angle, for self-balancing robots, to enable remote tuning.
*/
struct TD_SBR_PIDS {
    enum { TYPE = 50 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_SBR_PIDS)}; //!< length of whole packet, ie sizeof(TD_SBR_PIDS)

    uint8_t filler0 {0};
    uint8_t filler1 {0};
    enum { PITCH_ANGLE=0, SPEED=1, YAW_RATE=2, POSITION=3, PID_COUNT=4, PID_BEGIN=0 };
    struct SPID_t {
        float setpoint;
        PIDF::PIDF_t pid;
        PIDF::PIDF_t scale; //!< factor to scale value to range ~ [0, 100], for consistent display
    };
    struct Data {
        std::array<SPID_t, PID_COUNT> spids;
        float pitchBalanceAngleDegrees;
    };
    Data data;
};

/*!
Packet for the transmission of MotorPairController telemetry data.
*/
struct TD_MPC {
    enum { TYPE = 51 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_MPC)}; //!< length of whole packet, ie sizeof(TD_MPC)

    uint8_t tickInterval {0}; //!< tick interval of the MPC task
    enum : uint8_t { MOTORS_ON_FLAG = 0x04, CONTROL_MODE_MASK = 0x03 };
    uint8_t flags {0};
    motor_pair_controller_telemetry_t data;
};
#pragma pack(pop)

