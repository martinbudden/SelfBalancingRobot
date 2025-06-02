# pragma once

/*!
Packet definitions of telemetry data useful to any Stabilized Vehicle.
*/
#include <PIDF.h>
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
    uint8_t subType {0};
    uint8_t sequenceNumber {0};
};

/*!
Minimal sized packet. May be useful in future.
*/
struct TD_MINIMAL {
    enum { TYPE = 1 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_MINIMAL)}; //!< length of whole packet, ie sizeof(TD_MINIMAL)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};
};

/*!
Packet for the the transmission of AHRS, Vehicle Controller, and MAIN tick intervals and timings;
*/
struct TD_TASK_INTERVALS {
    enum { TYPE = 2 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_TASK_INTERVALS)}; //!< length of whole packet, ie sizeof(TD_TASK_INTERVALS)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    uint8_t mainTaskIntervalTicks {0}; //!< tick interval of the MAIN_LOOP_TASK
    uint8_t ahrsTaskIntervalTicks {0}; //!< tick interval of the AHRS_TASK
    uint8_t vcTaskIntervalTicks {0}; //!< tick interval of the Vehicle Controller task
    uint8_t transceiverTickCountDelta; //<<! tick interval of the ESP_NOW transceiver
};

/*!
Packet for the the transmission of AHRS, Vehicle Controller, and MAIN tick intervals and timings;
*/
struct TD_TASK_INTERVALS_EXTENDED {
    enum { TYPE = 3 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_TASK_INTERVALS_EXTENDED)}; //!< length of whole packet, ie sizeof(TD_TASK_INTERVALS)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    uint8_t mainTaskIntervalTicks {0}; //!< tick interval of the MAIN_LOOP_TASK
    uint8_t ahrsTaskIntervalTicks {0}; //!< tick interval of the AHRS_TASK
    uint8_t vcTaskIntervalTicks {0}; //!< tick interval of the Vehicle Controller task
    uint8_t transceiverTickCountDelta {0}; //<<! tick interval of the ESP_NOW transceiver

    static constexpr int TIME_CHECKS_COUNT = 4;
    std::array<uint16_t, TIME_CHECKS_COUNT> ahrsTimeChecksMicroSeconds {};

    uint16_t ahrsTaskIntervalMicroSeconds {0}; //!< execution interval of AHRS_TASK in microseconds
    uint16_t vcTaskIntervalMicroSeconds {0}; //!< execution interval of the Vehicle Controller task in microseconds
    uint16_t vcOutputPowerTimeMicroSeconds {0}; //!< time taken to set the Vehicle output power

    uint16_t receiverDroppedPacketCount {0}; //!< the number of packets dropped by the receiver
};

/*!
Packet for the transmission of AHRS telemetry data.
*/
struct TD_AHRS {
    enum { TYPE = 4 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_AHRS)}; //!< length of whole packet, ie sizeof(TD_AHRS)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    uint8_t taskIntervalTicks {0}; //!< interval of the AHRS task, in ticks
    enum : uint8_t { FILTER_INITIALIZING_FLAG = 0x01 };
    uint8_t flags {0};
    uint16_t fifoCount {0};

    struct xyz_int16_t {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct data_t {
        float pitch; //!< estimated pitch value calculated by Sensor Fusion Filter
        float roll; //!< estimated roll value calculated by Sensor Fusion Filter
        float yaw; //!< estimated yaw value calculated by Sensor Fusion Filter
        xyz_t gyroRPS; //!< gyro outputs from IMU
        xyz_t acc; //!< acceleration outputs from IMU
        xyz_int16_t gyroOffset;
        xyz_int16_t accOffset;
    };
    data_t data;
};

/*!
TYPE RANGE of 30-40 reserved for MultiWii Serial Protocol (MSP)

MSP V1 packet is of the form:

3 bytes header: two start bytes $M followed by message direction (< or >) or the error message indicator (!).
< - from the flight controller (FC →),
> - to the flight controller (→ FC).
! - Error Message.

one byte payload length
one byte message type
payload
checksum - XOR of the size, type, and payload bytes.

The checksum of a request (ie a message with no payload) equals the type.
*/

struct TD_MSP {
    enum { TYPE = 36 }; // 'M'
    uint32_t id {0};

    enum { MAX_DATA_LEN = 246 }; // ESP_NOW_MAX_DATA_LEN - sizeof(id)
    struct msp_t {
        uint8_t headerDollar;
        uint8_t headerM;
        uint8_t headerDirection;
        uint8_t payloadLength;
        uint8_t messageType;
        std::array<uint8_t, MAX_DATA_LEN - 5> payload;
    };
    union u {
        msp_t msp;
        std::array<uint8_t, MAX_DATA_LEN> buffer;
    };
    u data;
};

/*!
TYPE RANGE of 40-59 reserved for multi-rotors
*/

/*!
Packet for the transmission of FlightController telemetry data for a Quadcopter.
*/
struct TD_FC_QUADCOPTER {
    enum { TYPE = 40 };
    enum {
        ROLL_RATE_DPS = 0,
        PITCH_RATE_DPS = 1,
        YAW_RATE_DPS = 2,
        ROLL_ANGLE_DEGREES = 3,
        PITCH_ANGLE_DEGREES = 4,
        PID_COUNT = 5,
        PID_BEGIN = 0
    };
    enum { MOTOR_COUNT = 4 };

    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_FC_QUADCOPTER)}; //!< length of whole packet, ie sizeof(TD_FC_QUADCOPTER)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    uint16_t taskIntervalTicks {0}; //!< tick interval of the FC task
    enum : uint16_t { MOTORS_ON_FLAG = 0x8000, CONTROL_MODE_MASK = 0x00FF };
    uint16_t flags {0};

    struct power_rpm_t {
        float power;
        float rpm;
    };
    struct data_t {
        std::array<power_rpm_t, MOTOR_COUNT> motors;
    };
    data_t data;
};

/*!
TYPE RANGE of 60-69 reserved for multi-rotors
*/

/*!
Packet for the the transmission of PID constants, setpoints, and the balance angle, for self-balancing robots, to enable remote tuning.
*/
struct TD_SBR_PIDS {
    enum { TYPE = 60 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_SBR_PIDS)}; //!< length of whole packet, ie sizeof(TD_SBR_PIDS)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    enum { ROLL_ANGLE=0, PITCH_ANGLE=1, YAW_RATE=2, SPEED=3, POSITION=4, PID_COUNT=5, PID_BEGIN=0 };
    struct SPID_t {
        float setpoint;
        PIDF::PIDF_t pid;
        PIDF::PIDF_t scale; //!< factor to scale value to range ~ [0, 100], for consistent display
    };
    struct data_t {
        std::array<SPID_t, PID_COUNT> spids;
        float pitchBalanceAngleDegrees;
    };
    data_t data;
};

struct motor_pair_controller_telemetry_t {
    int32_t encoderLeft {0}; //!< value read from left motor encoder, raw
    int32_t encoderRight {0}; //!< value read from right motor encoder, raw
    int16_t encoderLeftDelta {0}; //!< difference between current left motor encoder value and previous value, raw
    int16_t encoderRightDelta {0}; //!< difference between current right motor encoder value and previous value, raw

    float motorMaxSpeedDPS {0};
    float speedLeftDPS {0}; //!< rotation speed of left motor, degrees per second
    float speedRightDPS {0}; //!< rotation speed of right motor, degrees per second
    float speedDPS_Filtered {0}; //!< speed calculated as the average of speedLeftDPS and speedRightDPS, then filtered

    float powerLeft {0}; //!< power value sent to left motor
    float powerRight {0}; //!< power value sent to right motor

    float pitchAngleOutput {0}; //!< pitch output value calculated by PID
    float speedOutput {0}; //!< speed output value calculated by PID
    float positionOutput {0}; //!< position output value calculated by PID
    float yawRateOutput {0}; //!< yawRate output value calculated by PID

    PIDF::error_t pitchError {0, 0, 0}; //!< P, I, and D errors calculated in pitch PID update
    PIDF::error_t speedError {0, 0, 0}; //!< P, I, and D errors calculated in speed PID update
    PIDF::error_t positionError {0, 0, 0}; //!< P, I, and D errors calculated in yawRate PID update
};

/*!
Packet for the transmission of MotorPairController telemetry data.
*/
struct TD_MPC {
    enum { TYPE = 61 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_MPC)}; //!< length of whole packet, ie sizeof(TD_MPC)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    uint8_t taskIntervalTicks {0}; //!< interval of the MPC task, in ticks
    enum : uint8_t { MOTORS_ON_FLAG = 0x04, CONTROL_MODE_MASK = 0x03 };
    uint8_t flags {0};
    uint8_t filler1 {0};
    uint8_t filler2 {0};
    motor_pair_controller_telemetry_t data;
};

#pragma pack(pop)
