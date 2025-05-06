#include "AHRS.h"
#include "SV_Telemetry.h"
#include "SV_TelemetryData.h"
#include "VehicleControllerBase.h"


/*!
Packs the TD_Minimal packet with zeros. Returns the length of the packet.
*/
size_t packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber)
{
    TD_MINIMAL* td = reinterpret_cast<TD_MINIMAL*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;

    td->type = TD_MINIMAL::TYPE;
    td->len = sizeof(TD_MINIMAL);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    return td->len;
}

/*!
Packs the tick interval telemetry data into a TD_TASK_INTERVALS packet. Returns the length of the packet.
*/
size_t packTelemetryData_TaskIntervals(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
        const AHRS& ahrs,
        const VehicleControllerBase& vehicleController,
        uint32_t vcOutputPowerTimeMicroSeconds,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount)
{
    TD_TASK_INTERVALS* td = reinterpret_cast<TD_TASK_INTERVALS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TASK_INTERVALS::TYPE;
    td->len = sizeof(TD_TASK_INTERVALS);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    td->mainTaskIntervalTicks = static_cast<uint8_t>(mainTaskTickCountDelta);
    td->ahrsTaskIntervalTicks = static_cast<uint8_t>(ahrs.getTickCountDelta());
    td->vcTaskIntervalTicks = vehicleController.getTickCountDelta();
    td->transceiverTickCountDelta = static_cast<uint8_t>(transceiverTickCountDelta);

    td->ahrsTaskIntervalMicroSeconds = static_cast<uint16_t>(ahrs.getTimeMicroSecondDelta());

    static_assert(TD_TASK_INTERVALS::TIME_CHECKS_COUNT == AHRS::TIME_CHECKS_COUNT);
    for (size_t ii = 0; ii < TD_TASK_INTERVALS::TIME_CHECKS_COUNT; ++ii) {
        td->ahrsTimeChecksMicroSeconds[ii] = ahrs.getTimeChecksMicroSeconds(ii);
    }

    td->vcTaskIntervalMicroSeconds = vehicleController.getTimeMicroSecondDelta();
    td->vcOutputPowerTimeMicroSeconds = vcOutputPowerTimeMicroSeconds;

    td->receiverDroppedPacketCount = static_cast<uint8_t>(receiverDroppedPacketCount);

    return td->len;
}

/*!
Packs the AHRS telemetry data into a TD_AHRS packet. Returns the length of the packet.
*/
size_t packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const AHRS& ahrs, const VehicleControllerBase& vehicleController)
{
    TD_AHRS* td = reinterpret_cast<TD_AHRS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_AHRS::TYPE;
    td->len = sizeof(TD_AHRS);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

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

    td->taskIntervalTicks = static_cast<uint8_t>(ahrsData.tickCountDelta);

    td->flags = ahrs.sensorFusionFilterIsInitializing() ? TD_AHRS::FILTER_INITIALIZING_FLAG : 0x00;
    td->fifoCount = static_cast<uint16_t>(ahrs.getFifoCount());

    return td->len;
}
