#include "Main.h"

#include <BackchannelSBR.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <BackchannelTransceiverESPNOW.h>
#endif
#include <ReceiverAtomJoyStick.h>


#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
BackchannelBase& Main::createBackchannel(MotorPairController& motorPairController, AHRS& ahrs, ReceiverBase& receiver, const TaskBase* dashboardTask, NonVolatileStorage& nvs)
{
    // statically allocate an MSP object
    // static MSP_ProtoFlight mspProtoFlightBackchannel(features, ahrs, flightController, cockpit, receiver);
    // Statically allocate the backchannel.
    static constexpr uint8_t backchannelMacAddress[ESP_NOW_ETH_ALEN] BACKCHANNEL_MAC_ADDRESS;
    auto& receiverAtomJoyStick = static_cast<ReceiverAtomJoyStick&>(receiver); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    static BackchannelTransceiverESPNOW backchannelTransceiverESPNOW(receiverAtomJoyStick.getESPNOW_Transceiver(), &backchannelMacAddress[0]);

    // Statically allocate the backchannel.
    static BackchannelSBR backchannel(
        backchannelTransceiverESPNOW,
        &backchannelMacAddress[0],
        //&myMacAddress[0],
        &receiver.getMyEUI().octets[0],
        motorPairController,
        ahrs,
        receiver,
        dashboardTask,
        nvs
    );

    return backchannel;
}
#endif
