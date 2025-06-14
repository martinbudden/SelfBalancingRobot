#include "SBR_Telemetry.h"
#include <SV_TelemetryData.h>

#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif

#include <cstring>

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
