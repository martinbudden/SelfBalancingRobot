#include "SBR_Telemetry.h"

#include <cstring>


/*!
Packs the MotorPairController PID telemetry data into a TD_PIDS packet. Returns the length of the packet.
*/
size_t packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const MotorPairController& motorPairController)
{
    //static_assert(static_cast<int>(TD_SBR_PIDS::PID_COUNT) == static_cast<int>(MotorPairController::PID_COUNT));
    TD_PIDS* td = reinterpret_cast<TD_PIDS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_PIDS::TYPE;
    td->len = sizeof(TD_PIDS);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    td->data.pidCount = MotorPairController::PID_COUNT;
    td->data.pidProfile = 0;
    td->data.vehicleType = TD_PIDS::SELF_BALANCING_ROBOT;
    td->data.controlMode = motorPairController.getControlMode();

    td->data.f0 = motorPairController.getPitchBalanceAngleDegrees(); // use general purpose value f0 for pitchBalanceAngleDegrees

    for (int ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        const auto pidIndex = static_cast<MotorPairController::pid_index_e>(ii);

        td->data.spids[ii].setpoint = motorPairController.getPID_Setpoint(pidIndex);
        td->data.spids[ii].pid.kp = motorPairController.getPID_P_MSP(pidIndex);
        td->data.spids[ii].pid.ki = motorPairController.getPID_I_MSP(pidIndex);
        td->data.spids[ii].pid.kd = motorPairController.getPID_D_MSP(pidIndex);
        td->data.spids[ii].pid.kf = motorPairController.getPID_F_MSP(pidIndex);
    }

    return td->len;
}

/*!
Packs the MotorPairController telemetry data into a TD_MPC packet. Returns the length of the packet.
*/
size_t packTelemetryData_MPC(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const MotorPairController& motorPairController)
{
    TD_MPC* td = reinterpret_cast<TD_MPC*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_MPC::TYPE;
    td->len = sizeof(TD_MPC);
    const TaskBase* motorPairControllerTask = motorPairController.getTask();
    td->taskIntervalTicks = static_cast<uint8_t>(motorPairControllerTask->getTickCountDelta());
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    td->motors = motorPairController.motorsIsOn();
    td->controlMode = static_cast<uint8_t>(motorPairController.getControlMode());

    motor_pair_controller_telemetry_t telemetryData;
    motorPairController.getTelemetryData(telemetryData, motorPairController.getControlMode());
    memcpy(&td->data, &telemetryData, sizeof(motor_pair_controller_telemetry_t));

    return td->len;
};
