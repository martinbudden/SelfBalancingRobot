# pragma once

#include "MotorPairControllerTelemetry.h"
#include "ReceiverBase.h"
#include <array>
#include <cstdint>
#include <xyz_type.h>

#pragma pack(push, 1)
/*!
Telemetry data type 0 is reserved for future use.
*/
struct TD_RESERVED {
    enum { TYPE = 0 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_RESERVED)}; //!< length of whole packet, ie sizeof(TD_Reserved)
    uint8_t filler0 {0};
    uint8_t filler1 {0};
};

/*!
Minimal sized packet. May be useful in future.
*/
struct TD_MINIMAL {
    enum { TYPE = 1 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_MINIMAL)}; //!< length of whole packet, ie sizeof(TD_MINIMAL)

    uint8_t data0 {0};
    uint8_t data1 {0};
};

/*!
Packet for the the transmission of AHRS, MPC, and MAIN tick intervals and timings;
*/
struct TD_TICK_INTERVALS {
    enum { TYPE = 2 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_TICK_INTERVALS)}; //!< length of whole packet, ie sizeof(TD_TICK_INTERVALS)

    uint8_t ahrsTaskTickIntervalTicks {0}; //!< tick interval of the AHRS_TASK
    uint8_t ahrsTaskFifoCount {0}; //!< tick interval of the AHRS_TASK
    uint16_t ahrsTaskTickIntervalMicroSeconds {0}; //!< execution interval of AHRS_TASK in microseconds
    static constexpr int TIME_CHECKS_COUNT = 4;
    std::array<uint16_t, TIME_CHECKS_COUNT> ahrsTimeChecksMicroSeconds {};

    uint16_t mpcOutputPowerTimeMicroSeconds {0}; //!< time taken to set the motor pair power
    uint16_t mpcTaskTickIntervalMicroSeconds {0}; //!< tick interval of the MPC_TASK
    uint8_t mpcTaskTickIntervalTicks {0}; //!< tick interval of the MPC_TASK
    uint8_t mainTaskTickInterval {0}; //!< tick interval of the MAIN_LOOP_TASK
    uint8_t transceiverTickCountDelta; //<<! tick interval of the ESP_NOW transceiver
    uint8_t receiverDroppedPacketCount {0}; //!< the number of packets dropped by the receiver
};

/*!
Packet for the transmission of Receiver telemetry data.
*/
struct TD_RECEIVER {
    enum { TYPE = 3 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_RECEIVER)}; //!< length of whole packet, ie sizeof(TD_RECEIVER)

    uint8_t tickInterval {0}; //!< tick number of ticks since last receiver update
    uint8_t droppedPacketCount {0}; //!< the number of packets dropped by the receiver
    struct Data {
        ReceiverBase::controls_t controls;
        std::array<uint8_t, 4> aux; //!< 4 8-bit auxiliary channels
        uint32_t switches;
    };
    Data data;
};

/*!
Packet for the transmission of AHRS telemetry data.
*/
struct TD_AHRS {
    enum { TYPE = 4 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_AHRS)}; //!< length of whole packet, ie sizeof(TD_AHRS)

    uint8_t tickInterval {0}; //!< tick interval of the AHRS task
    enum : uint8_t { FILTER_INITIALIZING_FLAG = 0x01 };
    uint8_t flags {0};
    struct Data {
        float pitch; //!< estimated pitch value calculated by Madgwick Orientation Filter
        float roll; //!< estimated roll value calculated by Madgwick Orientation Filter
        float yaw; //!< estimated yaw value calculated by Madgwick Orientation Filter
        xyz_t gyroRadians; //!< gyro outputs from IMU
        xyz_t acc; //!< acceleration outputs from IMU
    };
    Data data;
};

/*!
Packet for the the transmission of PID constants, setpoints, and the balance angle, for self-balancing robots, to enable remote tuning.
*/
struct TD_SBR_PIDS {
    enum { TYPE = 5 };
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
    enum { TYPE = 6 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_MPC)}; //!< length of whole packet, ie sizeof(TD_MPC)

    uint8_t tickInterval {0}; //!< tick interval of the MPC task
    enum : uint8_t { MOTORS_ON_FLAG = 0x04, CONTROL_MODE_MASK = 0x03 };
    uint8_t flags {0};
    motor_pair_controller_telemetry_t data;
};

#pragma pack(pop)

