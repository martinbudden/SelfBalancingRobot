#include "SBR_Telemetry.h"

#include "AHRS_Base.h"
#include "MotorPairBase.h"

#include <cstring>

/*!
Packs the TD_Minimal packet with zeros. Returns the length of the packet.
*/
int packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id)
{
    TD_Minimal* td = reinterpret_cast<TD_Minimal*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_Minimal::TYPE;
    td->len = sizeof(TD_Minimal);

    td->data0 = 0;
    td->data1 = 0;

    return td->len;
}

/*!
Packs the tick interval telemetry data into a TD_TickIntervals packet. Returns the length of the packet.
*/
int packTelemetryData_TickIntervals(uint8_t* telemetryDataPtr, uint32_t id,
        const AHRS_Base& ahrs,
        const MotorControllerBase& motorController,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount)
{
    TD_TickIntervals* td = reinterpret_cast<TD_TickIntervals*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TickIntervals::TYPE;
    td->len = sizeof(TD_TickIntervals);

    td->ahrsTaskTickIntervalTicks = ahrs.getTickCountDelta();
    td->ahrsTaskFifoCount = ahrs.getFifoCount();
    td->ahrsTaskTickIntervalMicroSeconds = ahrs.getTimeMicroSecondDelta();
    td->ahrsUpdateTimeIMU_ReadMicroSeconds = ahrs.getUpdateTimeIMU_ReadMicroSeconds();
    td->ahrsUpdateTimeFiltersMicroSeconds = ahrs.getUpdateTimeFiltersMicroSeconds();
    td->ahrsUpdateTimeSensorFusionMicroSeconds = ahrs.getUpdateTimeSensorFusionMicroSeconds();
    td->ahrsUpdateTimePID_MicroSeconds = ahrs.getUpdateTimePID_MicroSeconds();

    td->mpcTaskTickIntervalTicks = motorController.getTickCountDelta();
    td->mpcOutputPowerTimeMicroSeconds = motorController.getOutputPowerTimeMicroSeconds();
    td->mpcTaskTickIntervalMicroSeconds = motorController.getTimeMicroSecondDelta();

    td->mainTaskTickInterval = mainTaskTickCountDelta;
    td->transceiverTickCountDelta = transceiverTickCountDelta;
    td->receiverDroppedPacketCount = receiverDroppedPacketCount;

    return td->len;
}

/*!
Packs the MotorPairController PID telemetry data into a TD_PID packet. Returns the length of the packet.
*/
int packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController)
{
    TD_PID* td = reinterpret_cast<TD_PID*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_PID::TYPE;
    td->len = sizeof(TD_PID);

    td->data.pitch.setpoint = motorPairController.getPitchPIDSetpoint();
    td->data.pitch.pid = motorPairController.getPitchPIDConstants();
    td->data.pitch.scale = motorPairController.getPitchPIDTelemetryScaleFactors();

    td->data.speed.setpoint = motorPairController.getSpeedPIDSetpoint();
    td->data.speed.pid = motorPairController.getSpeedPIDConstants();
    td->data.speed.scale = motorPairController.getSpeedPIDTelemetryScaleFactors();

    td->data.yawRate.setpoint = motorPairController.getYawRatePIDSetpoint();
    td->data.yawRate.pid = motorPairController.getYawRatePIDConstants();
    td->data.yawRate.scale = motorPairController.getYawRatePIDTelemetryScaleFactors();

    td->data.pitchBalanceAngleDegrees = motorPairController.getPitchBalanceAngleDegrees();

    return td->len;
}

/*!
Packs the AHRS telemetry data into a TD_AHRS packet. Returns the length of the packet.
*/
int packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, const AHRS_Base& ahrs, const MotorControllerBase& motorController)
{
    TD_AHRS* td = reinterpret_cast<TD_AHRS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_AHRS::TYPE;
    td->len = sizeof(TD_AHRS);

    const AHRS_Base::data_t ahrsData = ahrs.getAhrsDataForInstrumentationUsingLock();
    td->data = {
        .pitch = motorController.getPitchAngleDegreesRaw(),
        .roll = motorController.getRollAngleDegreesRaw(),
        .yaw = motorController.getYawAngleDegreesRaw(),
        .gyroRadians = ahrsData.gyroRadians,
        .acc = ahrsData.acc
    };

    td->tickInterval = ahrsData.tickCountDelta;

    td->flags = ahrs.sensorFusionFilterIsInitializing() ? TD_AHRS::FILTER_INITIALIZING_FLAG : 0x00;

    return td->len;
}

/*!
Packs the MotorPairController telemetry data into a TD_MPC packet. Returns the length of the packet.
*/
int packTelemetryData_MPC(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController)
{
    TD_MPC* td = reinterpret_cast<TD_MPC*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_MPC::TYPE;
    td->len = sizeof(TD_MPC);
    td->tickInterval = motorPairController.getTickCountDelta();

    td->flags = motorPairController.motorsIsOn() ? TD_MPC::MOTORS_ON_FLAG : 0x00;
    td->flags |= (TD_MPC::CONTROL_MODE_MASK & motorPairController.getControlMode());

    motor_pair_controller_telemetry_t telemetryData;
    motorPairController.getTelemetryData(telemetryData);
    memcpy(&td->data, &telemetryData, sizeof(motor_pair_controller_telemetry_t));

    return td->len;
};

/*!
Packs the Receiver telemetry data into a TD_Receiver packet. Returns the length of the packet.
*/
int packTelemetryData_Receiver(uint8_t* telemetryDataPtr, uint32_t id, const ReceiverBase& receiver)
{
    TD_Receiver* td = reinterpret_cast<TD_Receiver*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_Receiver::TYPE;
    td->len = sizeof(TD_Receiver);
    td->tickInterval = receiver.getTickCountDelta();
    td->droppedPacketCount = receiver.getDroppedPacketCountDelta();

    td->data = {
        .controls = receiver.getControls(),
        .flags = receiver.getFlags()
    };

    return td->len;
};

