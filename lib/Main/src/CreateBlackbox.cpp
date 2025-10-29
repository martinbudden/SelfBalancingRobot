#include "Main.h"

#include <AHRS.h>

#include <BlackboxCallbacks.h>
#include <BlackboxMessageQueue.h>
#include <BlackboxSelfBalancingRobot.h>
#include <BlackboxSerialDeviceSDCard.h>
#include <TimeMicroseconds.h>


#if defined(USE_BLACKBOX)
/*!
Statically allocate the Blackbox and associated objects.
*/
Blackbox& Main::createBlackBox(AHRS& ahrs, MotorPairController& motorPairController, RadioController& radioController)
{
    static BlackboxCallbacks            blackboxCallbacks(motorPairController.getBlackboxMessageQueue(), ahrs, motorPairController, radioController); // NOLINT(misc-const-correctness) false positive
    static BlackboxSerialDeviceSDCard   blackboxSerialDevice(BlackboxSerialDeviceSDCard::SDCARD_SPI_PINS); // NOLINT(misc-const-correctness) false positive

    static BlackboxSelfBalancingRobot   blackbox(blackboxCallbacks, motorPairController.getBlackboxMessageQueue(), blackboxSerialDevice, motorPairController);
    motorPairController.setBlackbox(blackbox);
    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        //.device = Blackbox::DEVICE_NONE,
        .mode = Blackbox::MODE_NORMAL, // logging starts on arming, file is saved when disarmed
        //.mode = Blackbox::MODE_ALWAYS_ON
    });

    return blackbox;
}
#endif
