# pragma once

/*!
Packet definitions of telemetry data useful to any Stabilized Vehicle.
*/
#include <array>
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

    uint16_t mcTaskTickIntervalMicroSeconds {0}; //!< tick interval of the MC_TASK in microseconds
    uint16_t mcOutputPowerTimeMicroSeconds {0}; //!< time taken to set the motor power
    uint8_t mcTaskTickIntervalTicks {0}; //!< tick interval of the MC_TASK
    uint8_t mainTaskTickInterval {0}; //!< tick interval of the MAIN_LOOP_TASK
    uint8_t transceiverTickCountDelta; //<<! tick interval of the ESP_NOW transceiver
    uint8_t receiverDroppedPacketCount {0}; //!< the number of packets dropped by the receiver
};

/*!
Packet for the transmission of AHRS telemetry data.
*/
struct TD_AHRS {
    enum { TYPE = 3 };
    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_AHRS)}; //!< length of whole packet, ie sizeof(TD_AHRS)

    uint8_t tickInterval {0}; //!< tick interval of the AHRS task
    enum : uint8_t { FILTER_INITIALIZING_FLAG = 0x01 };
    uint8_t flags {0};
    struct xyz_int16_t {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct Data {
        float pitch; //!< estimated pitch value calculated by Madgwick Orientation Filter
        float roll; //!< estimated roll value calculated by Madgwick Orientation Filter
        float yaw; //!< estimated yaw value calculated by Madgwick Orientation Filter
        xyz_t gyroRPS; //!< gyro outputs from IMU
        xyz_t acc; //!< acceleration outputs from IMU
        xyz_int16_t gyroOffset;
        xyz_int16_t accOffset;
    };
    Data data;
};
#pragma pack(pop)
