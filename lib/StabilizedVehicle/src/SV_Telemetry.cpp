#include "SV_Telemetry.h"
#include "MotorControllerBase.h"

/*!
Packs the TD_Minimal packet with zeros. Returns the length of the packet.
*/
size_t packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id)
{
    TD_MINIMAL* td = reinterpret_cast<TD_MINIMAL*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_MINIMAL::TYPE;
    td->len = sizeof(TD_MINIMAL);

    td->data0 = 0;
    td->data1 = 0;

    return td->len;
}

/*!
Packs the tick interval telemetry data into a TD_TICK_INTERVALS packet. Returns the length of the packet.
*/
size_t packTelemetryData_TickIntervals(uint8_t* telemetryDataPtr, uint32_t id,
        const AHRS& ahrs,
        const MotorControllerBase& motorController,
        uint32_t mcOutputPowerTimeMicroSeconds,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount)
{
    TD_TICK_INTERVALS* td = reinterpret_cast<TD_TICK_INTERVALS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TICK_INTERVALS::TYPE;
    td->len = sizeof(TD_TICK_INTERVALS);

    td->ahrsTaskTickIntervalTicks = static_cast<uint8_t>(ahrs.getTickCountDelta());
    td->ahrsTaskTickIntervalMicroSeconds = static_cast<uint16_t>(ahrs.getTimeMicroSecondDelta());

    static_assert(TD_TICK_INTERVALS::TIME_CHECKS_COUNT == AHRS::TIME_CHECKS_COUNT);
    td->ahrsTaskFifoCount = static_cast<uint8_t>(ahrs.getFifoCount());
    for (size_t ii = 0; ii < TD_TICK_INTERVALS::TIME_CHECKS_COUNT; ++ii) {
        td->ahrsTimeChecksMicroSeconds[ii] = static_cast<uint16_t>(ahrs.getTimeChecksMicroSeconds(ii));
    }

    td->mcTaskTickIntervalMicroSeconds = static_cast<uint16_t>(motorController.getTimeMicroSecondDelta());
    td->mcOutputPowerTimeMicroSeconds = static_cast<uint16_t>(mcOutputPowerTimeMicroSeconds);
    td->mcTaskTickIntervalTicks = static_cast<uint8_t>(motorController.getTickCountDelta());

    td->mainTaskTickInterval = static_cast<uint8_t>(mainTaskTickCountDelta);
    td->transceiverTickCountDelta = static_cast<uint8_t>(transceiverTickCountDelta);
    td->receiverDroppedPacketCount = static_cast<uint8_t>(receiverDroppedPacketCount);

    return td->len;
}

/*!
Packs the AHRS telemetry data into a TD_AHRS packet. Returns the length of the packet.
*/
size_t packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, const AHRS& ahrs, const MotorControllerBase& motorController)
{
    TD_AHRS* td = reinterpret_cast<TD_AHRS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_AHRS::TYPE;
    td->len = sizeof(TD_AHRS);

    const AHRS::data_t ahrsData = ahrs.getAhrsDataForInstrumentationUsingLock();
    td->data = {
        .pitch = motorController.getPitchAngleDegreesRaw(),
        .roll = motorController.getRollAngleDegreesRaw(),
        .yaw = motorController.getYawAngleDegreesRaw(),
        .gyroRPS = ahrsData.gyroRPS,
        .acc = ahrsData.acc
    };

    td->tickInterval = static_cast<uint8_t>(ahrsData.tickCountDelta);

    td->flags = ahrs.sensorFusionFilterIsInitializing() ? TD_AHRS::FILTER_INITIALIZING_FLAG : 0x00;

    return td->len;
}

/*!
Packs the Receiver telemetry data into a TD_RECEIVER packet. Returns the length of the packet.
*/
size_t packTelemetryData_Receiver(uint8_t* telemetryDataPtr, uint32_t id, const ReceiverBase& receiver)
{
    TD_RECEIVER* td = reinterpret_cast<TD_RECEIVER*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_RECEIVER::TYPE;
    td->len = sizeof(TD_RECEIVER);
    td->tickInterval = static_cast<uint8_t>(receiver.getTickCountDelta());
    td->droppedPacketCount = static_cast<uint8_t>(receiver.getDroppedPacketCountDelta());

    td->data.controls = receiver.getControls(),
    td->data.switches = receiver.getSwitches(),
    td->data.aux[0] = receiver.getAux(0);
    td->data.aux[1] = receiver.getAux(1);
    td->data.aux[2] = receiver.getAux(2);
    td->data.aux[3] = receiver.getAux(3);

    return td->len;
};
