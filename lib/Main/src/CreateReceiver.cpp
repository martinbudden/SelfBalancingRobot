#include "Main.h"

#include <ReceiverAtomJoyStick.h>
#include <ReceiverNull.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <WiFi.h>
#endif


ReceiverBase& Main::createReceiver()
{
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
    // Set WiFi to station mode
    WiFi.mode(WIFI_STA);
    // Disconnect from Access Point if it was previously connected
    WiFi.disconnect();
    // get my MAC address
    uint8_t myMacAddress[ESP_NOW_ETH_ALEN];
    WiFi.macAddress(&myMacAddress[0]);

    // Statically allocate and setup the receiver.
#if !defined(RECEIVER_CHANNEL)
    static constexpr uint8_t RECEIVER_CHANNEL {3};
#endif
    static ReceiverAtomJoyStick receiver(&myMacAddress[0], RECEIVER_CHANNEL);
    const esp_err_t espErr = receiver.init();
    //delay(400); // delay to allow serial port to initialize before first print
    Serial.print("\r\n\r\n**** ESP-NOW Ready:");
    Serial.println(espErr);
    Serial.println();
    assert(espErr == ESP_OK && "Unable to setup receiver.");
#else
    static ReceiverNull receiver;
#endif // LIBRARY_RECEIVER_USE_ESPNOW

    return receiver;
}
