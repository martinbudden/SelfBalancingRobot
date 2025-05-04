#include "AHRS.h"
#include "SV_Telemetry.h"
#include "SV_TelemetryData.h"
#include "VehicleControllerBase.h"


/*!
Packs the TD_Minimal packet with zeros. Returns the length of the packet.
*/
size_t packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id)
{
    TD_MINIMAL* td = reinterpret_cast<TD_MINIMAL*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;

    td->type = TD_MINIMAL::TYPE;
    td->len = sizeof(TD_MINIMAL);
    td->subType = 0;
    td->data0 = 0;

    return td->len;
}

/*!
Packs the tick interval telemetry data into a TD_TICK_INTERVALS packet. Returns the length of the packet.
*/
size_t packTelemetryData_TickIntervals(uint8_t* telemetryDataPtr, uint32_t id,
        const AHRS& ahrs,
        const VehicleControllerBase& vehicleController,
        uint32_t vcOutputPowerTimeMicroSeconds,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount)
{
    TD_TICK_INTERVALS* td = reinterpret_cast<TD_TICK_INTERVALS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TICK_INTERVALS::TYPE;
    td->len = sizeof(TD_TICK_INTERVALS);
    td->subType = 0;

    td->ahrsTaskTickIntervalTicks = static_cast<uint8_t>(ahrs.getTickCountDelta());
    td->ahrsTaskTickIntervalMicroSeconds = static_cast<uint16_t>(ahrs.getTimeMicroSecondDelta());

    static_assert(TD_TICK_INTERVALS::TIME_CHECKS_COUNT == AHRS::TIME_CHECKS_COUNT);
    for (size_t ii = 0; ii < TD_TICK_INTERVALS::TIME_CHECKS_COUNT; ++ii) {
        td->ahrsTimeChecksMicroSeconds[ii] = static_cast<uint16_t>(ahrs.getTimeChecksMicroSeconds(ii));
    }

    td->vcTaskTickIntervalMicroSeconds = static_cast<uint16_t>(vehicleController.getTimeMicroSecondDelta());
    td->vcOutputPowerTimeMicroSeconds = static_cast<uint16_t>(vcOutputPowerTimeMicroSeconds);
    td->vcTaskTickIntervalTicks = static_cast<uint8_t>(vehicleController.getTickCountDelta());

    td->mainTaskTickInterval = static_cast<uint8_t>(mainTaskTickCountDelta);
    td->transceiverTickCountDelta = static_cast<uint8_t>(transceiverTickCountDelta);
    td->receiverDroppedPacketCount = static_cast<uint8_t>(receiverDroppedPacketCount);

    return td->len;
}

/*!
Packs the AHRS telemetry data into a TD_AHRS packet. Returns the length of the packet.
*/
size_t packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, const AHRS& ahrs, const VehicleControllerBase& vehicleController)
{
    TD_AHRS* td = reinterpret_cast<TD_AHRS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_AHRS::TYPE;
    td->len = sizeof(TD_AHRS);
    td->subType = 0;

    const AHRS::data_t ahrsData = ahrs.getAhrsDataForInstrumentationUsingLock();
    const IMU_Base::xyz_int32_t gyroOffset = ahrs.getGyroOffsetMapped();
    const IMU_Base::xyz_int32_t accOffset = ahrs.getAccOffsetMapped();
    td->data = {
        .pitch = vehicleController.getPitchAngleDegreesRaw(),
        .roll = vehicleController.getRollAngleDegreesRaw(),
        .yaw = vehicleController.getYawAngleDegreesRaw(),
        .gyroRPS = ahrsData.gyroRPS,
        .acc = ahrsData.acc,
        .gyroOffset = {
            .x = static_cast<int16_t>(gyroOffset.x),
            .y = static_cast<int16_t>(gyroOffset.y),
            .z = static_cast<int16_t>(gyroOffset.z)
        },
        .accOffset = {
            .x = static_cast<int16_t>(accOffset.x),
            .y = static_cast<int16_t>(accOffset.y),
            .z = static_cast<int16_t>(accOffset.z)
        }
    };

    td->tickInterval = static_cast<uint8_t>(ahrsData.tickCountDelta);

    td->flags = ahrs.sensorFusionFilterIsInitializing() ? TD_AHRS::FILTER_INITIALIZING_FLAG : 0x00;
    td->fifoCount = static_cast<uint16_t>(ahrs.getFifoCount());

    return td->len;
}
