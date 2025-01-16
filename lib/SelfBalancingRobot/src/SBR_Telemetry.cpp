#include "SBR_Telemetry.h"

#include "AHRS.h"
#include "MotorPairBase.h"

#include <cstring>

/*!
Packs the TD_Minimal packet with zeros. Returns the length of the packet.
*/
int packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id)
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
int packTelemetryData_TickIntervals(uint8_t* telemetryDataPtr, uint32_t id,
        const AHRS& ahrs,
        const MotorControllerBase& motorController,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount)
{
    TD_TICK_INTERVALS* td = reinterpret_cast<TD_TICK_INTERVALS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TICK_INTERVALS::TYPE;
    td->len = sizeof(TD_TICK_INTERVALS);

    td->ahrsTaskTickIntervalTicks = ahrs.getTickCountDelta();
    td->ahrsTaskTickIntervalMicroSeconds = ahrs.getTimeMicroSecondDelta();

    static_assert(TD_TICK_INTERVALS::TIME_CHECKS_COUNT == AHRS::TIME_CHECKS_COUNT);
    td->ahrsTaskFifoCount = ahrs.getFifoCount();
    for (int ii = 0; ii < TD_TICK_INTERVALS::TIME_CHECKS_COUNT; ++ii) {
        td->ahrsTimeChecksMicroSeconds[ii] = ahrs.getTimeChecksMicroSeconds(ii);
    }
 
    td->mpcTaskTickIntervalTicks = motorController.getTickCountDelta();
    td->mpcOutputPowerTimeMicroSeconds = motorController.getOutputPowerTimeMicroSeconds();
    td->mpcTaskTickIntervalMicroSeconds = motorController.getTimeMicroSecondDelta();

    td->mainTaskTickInterval = mainTaskTickCountDelta;
    td->transceiverTickCountDelta = transceiverTickCountDelta;
    td->receiverDroppedPacketCount = receiverDroppedPacketCount;

    return td->len;
}

/*!
Packs the MotorPairController PID telemetry data into a TD_SBR_PIDS packet. Returns the length of the packet.
*/
int packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController, const TelemetryScaleFactors& scaleFactors)
{
    TD_SBR_PIDS* td = reinterpret_cast<TD_SBR_PIDS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_SBR_PIDS::TYPE;
    td->len = sizeof(TD_SBR_PIDS);

    td->data.pitch.setpoint = motorPairController.getPitchPIDSetpoint();
    td->data.pitch.pid = motorPairController.getPitchPIDConstants();
    td->data.pitch.scale = scaleFactors.getPitchPIDTelemetryScaleFactors();

    td->data.speed.setpoint = motorPairController.getSpeedPIDSetpoint();
    td->data.speed.pid = motorPairController.getSpeedPIDConstants();
    td->data.speed.scale = scaleFactors.getSpeedPIDTelemetryScaleFactors();

    td->data.position.setpoint = motorPairController.getPositionPIDSetpoint();
    td->data.position.pid = motorPairController.getPositionPIDConstants();
    td->data.position.scale = scaleFactors.getPositionPIDTelemetryScaleFactors();

    td->data.yawRate.setpoint = motorPairController.getYawRatePIDSetpoint();
    td->data.yawRate.pid = motorPairController.getYawRatePIDConstants();
    td->data.yawRate.scale = scaleFactors.getYawRatePIDTelemetryScaleFactors();

    td->data.pitchBalanceAngleDegrees = motorPairController.getPitchBalanceAngleDegrees();

    return td->len;
}

/*!
Packs the AHRS telemetry data into a TD_AHRS packet. Returns the length of the packet.
*/
int packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, const AHRS& ahrs, const MotorControllerBase& motorController)
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
Packs the Receiver telemetry data into a TD_RECEIVER packet. Returns the length of the packet.
*/
int packTelemetryData_Receiver(uint8_t* telemetryDataPtr, uint32_t id, const ReceiverBase& receiver)
{
    TD_RECEIVER* td = reinterpret_cast<TD_RECEIVER*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_RECEIVER::TYPE;
    td->len = sizeof(TD_RECEIVER);
    td->tickInterval = receiver.getTickCountDelta();
    td->droppedPacketCount = receiver.getDroppedPacketCountDelta();

    td->data.controls = receiver.getControls(),
    td->data.switches = receiver.getSwitches(),
    td->data.aux[0] = receiver.getAux(0);
    td->data.aux[1] = receiver.getAux(1);
    td->data.aux[2] = receiver.getAux(2);
    td->data.aux[3] = receiver.getAux(3);

    return td->len;
};
