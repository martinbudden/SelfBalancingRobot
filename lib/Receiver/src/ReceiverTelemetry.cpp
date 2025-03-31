#include "ReceiverTelemetry.h"
#include "ReceiverTelemetryData.h"


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
    td->data.aux[0] = receiver.getAuxiliaryChannel(0);
    td->data.aux[1] = receiver.getAuxiliaryChannel(1);
    td->data.aux[2] = receiver.getAuxiliaryChannel(2);
    td->data.aux[3] = receiver.getAuxiliaryChannel(3);

    return td->len;
};
