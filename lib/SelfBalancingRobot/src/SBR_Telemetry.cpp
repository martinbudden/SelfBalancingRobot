#include "MotorPairController.h"
#include "SBR_Telemetry.h"
#include "SBR_TelemetryData.h"
#include "TelemetryScaleFactors.h"

#include <cstring>


/*!
Packs the MotorPairController PID telemetry data into a TD_SBR_PIDS packet. Returns the length of the packet.
*/
size_t packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController, const TelemetryScaleFactors& scaleFactors)
{
    static_assert(static_cast<int>(TD_SBR_PIDS::PID_COUNT) == static_cast<int>(MotorPairController::PID_COUNT));
    TD_SBR_PIDS* td = reinterpret_cast<TD_SBR_PIDS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_SBR_PIDS::TYPE;
    td->len = sizeof(TD_SBR_PIDS);

    for (int ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        td->data.spids[ii].setpoint = motorPairController.getPID_Setpoint(static_cast<MotorPairController::pid_index_t>(ii));
        td->data.spids[ii].pid = motorPairController.getPID_Constants(static_cast<MotorPairController::pid_index_t>(ii));
        td->data.spids[ii].scale = scaleFactors.getTelemetryScaleFactor(static_cast<MotorPairController::pid_index_t>(ii));
    }

    td->data.pitchBalanceAngleDegrees = motorPairController.getPitchBalanceAngleDegrees();

    return td->len;
}

/*!
Packs the MotorPairController telemetry data into a TD_MPC packet. Returns the length of the packet.
*/
size_t packTelemetryData_MPC(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController)
{
    TD_MPC* td = reinterpret_cast<TD_MPC*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_MPC::TYPE;
    td->len = sizeof(TD_MPC);
    td->tickInterval = static_cast<uint8_t>(motorPairController.getTickCountDelta());

    td->flags = motorPairController.motorsIsOn() ? TD_MPC::MOTORS_ON_FLAG : 0x00;
    td->flags |= static_cast<uint8_t>(TD_MPC::CONTROL_MODE_MASK & motorPairController.getControlMode());

    motor_pair_controller_telemetry_t telemetryData;
    motorPairController.getTelemetryData(telemetryData);
    memcpy(&td->data, &telemetryData, sizeof(motor_pair_controller_telemetry_t));

    return td->len;
};

