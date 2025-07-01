#include "AHRS.h"
#include "AHRS_Task.h"
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
Packs the tick interval telemetry data into a TD_TASK_INTERVALS_EXTENDED packet. Returns the length of the packet.
*/
size_t packTelemetryData_TaskIntervals(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
        const TaskBase& ahrsTask,
        const TaskBase& vehicleControllerTask,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta)
{
    TD_TASK_INTERVALS* td = reinterpret_cast<TD_TASK_INTERVALS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TASK_INTERVALS::TYPE;
    td->len = sizeof(TD_TASK_INTERVALS);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    td->mainTaskIntervalTicks = static_cast<uint8_t>(mainTaskTickCountDelta);
    td->ahrsTaskIntervalTicks = static_cast<uint8_t>(ahrsTask.getTickCountDelta());
    td->vcTaskIntervalTicks = static_cast<uint8_t>(vehicleControllerTask.getTickCountDelta());
    td->transceiverTickCountDelta = static_cast<uint8_t>(transceiverTickCountDelta);

    return td->len;
}

/*!
Packs the tick interval telemetry data into a TD_TASK_INTERVALS_EXTENDED packet. Returns the length of the packet.
*/
size_t packTelemetryData_TaskIntervalsExtended(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
        const AHRS& ahrs,
        const VehicleControllerBase& vehicleController,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount)
{
    TD_TASK_INTERVALS_EXTENDED* td = reinterpret_cast<TD_TASK_INTERVALS_EXTENDED*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TASK_INTERVALS_EXTENDED::TYPE;
    td->len = sizeof(TD_TASK_INTERVALS_EXTENDED);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    const TaskBase* vehicleControllerTask = vehicleController.getTask();
    const TaskBase* ahrsTask = vehicleController.getTask();

    td->mainTaskIntervalTicks = static_cast<uint8_t>(mainTaskTickCountDelta);
    td->ahrsTaskIntervalTicks = static_cast<uint8_t>(ahrsTask->getTickCountDelta());
    td->vcTaskIntervalTicks = static_cast<uint8_t>(vehicleControllerTask->getTickCountDelta());
    td->transceiverTickCountDelta = static_cast<uint8_t>(transceiverTickCountDelta);

    td->ahrsTaskIntervalMicroSeconds = static_cast<uint16_t>(ahrsTask->getTimeMicroSecondDelta());

    static_assert(TD_TASK_INTERVALS_EXTENDED::TIME_CHECKS_COUNT == AHRS::TIME_CHECKS_COUNT);
    for (size_t ii = 0; ii < TD_TASK_INTERVALS_EXTENDED::TIME_CHECKS_COUNT; ++ii) {
        td->ahrsTimeChecksMicroSeconds[ii] = static_cast<uint16_t>(ahrs.getTimeChecksMicroSeconds(ii));
    }

    td->vcTaskIntervalMicroSeconds = static_cast<uint16_t>(vehicleControllerTask->getTimeMicroSecondDelta());
    td->vcOutputPowerTimeMicroSeconds = static_cast<uint16_t>(vehicleController.getOutputPowerTimeMicroSeconds());

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

    const uint32_t flags = ahrs.getFlags();
    td->flags = (flags & AHRS::SENSOR_FUSION_REQUIRES_INITIALIZATION) ? TD_AHRS::SENSOR_FUSION_REQUIRES_INITIALIZATION : 0;
    if (flags & AHRS::IMU_AUTO_CALIBRATES) {
        td->flags |= TD_AHRS::IMU_AUTO_CALIBRATES;
    }

    return td->len;
}

/*!
Packs the VehicleController PID telemetry data into a TD_PIDS packet. Returns the length of the packet.
*/
size_t packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const VehicleControllerBase& vehicleController, uint8_t controlMode, float f0, float f1)
{
    //static_assert(static_cast<int>(TD_SBR_PIDS::PID_COUNT) == static_cast<int>(MotorPairController::PID_COUNT));
    TD_PIDS* td = reinterpret_cast<TD_PIDS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_PIDS::TYPE;
    td->len = sizeof(TD_PIDS);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    td->data.pidCount = static_cast<uint8_t>(vehicleController.getPID_Count());
    td->data.pidProfile = 0;
    td->data.vehicleType = static_cast<uint8_t>(vehicleController.getType());
    td->data.controlMode = controlMode;

    td->data.f0 = f0; // general purpose value f0 used for pitchBalanceAngleDegrees in self balancing robots
    td->data.f1 = f1;

    const size_t pidCount = vehicleController.getPID_Count();
    for (size_t ii = 0; ii < pidCount; ++ii) {
        const auto pid = vehicleController.getPID_MSP(ii);
        td->data.pids[ii].kp = pid.kp;
        td->data.pids[ii].ki = pid.ki;
        td->data.pids[ii].kd = pid.kd;
        td->data.pids[ii].kf = pid.kf;
        //if (ii == MotorPairController::PITCH_ANGLE_DEGREES) {
        //    Serial.printf("KP: %d, %f, sc:%f\r\n", td->data.pids[ii].kp, motorPairController.getPID_Constants(pidIndex).kp, motorPairController.getScaleFactors()[pidIndex].kp);
        //}
    }

    return td->len;
}
