#if defined(MOTORS_O_DRIVE_CAN)

#include "Motors_ODriveCAN.h"

#include <CANController.h>
#include <HardwareSerial.h>
#include <ODriveCAN.h> //https://github.com/odriverobotics/ODriveArduino/blob/master/src/ODriveCAN.h


#if defined (ARDUINO_ARCH_ESP32)

// cppcheck-suppress missingOverride
#include <ESP32SJA1000.h>
static bool sendMsg(CANControllerClass& can_intf, uint32_t id, uint8_t length, const uint8_t* data);
static void pumpEvents(CANControllerClass& can_intf);

// ESP32SJA1000Class is a subclass of CANControllerClass
// CREATE_CAN_INTF_WRAPPER creates a wrapper for the class with sendMsg and pumpEvents functions
// cppcheck-suppress cstyleCast
CREATE_CAN_INTF_WRAPPER(ESP32SJA1000Class) // NOLINT(cppcoreguidelines-pro-type-cstyle-cast)
static ESP32SJA1000Class& can_intf = CAN;

#endif


// Instantiate ODrive objects
static ODriveCAN oDrive0(wrap_can_intf(can_intf), Motors_ODriveCAN::O_DRIVE_0_NODE_ID); // NOLINT(cert-err58-cpp,fuchsia-statically-constructed-objects)
static ODriveCAN oDrive1(wrap_can_intf(can_intf), Motors_ODriveCAN::O_DRIVE_1_NODE_ID); // NOLINT(cert-err58-cpp,fuchsia-statically-constructed-objects)


static bool sendMsg(CANControllerClass& can_intf, uint32_t id, uint8_t length, const uint8_t* data)
{
    if (id & 0x80000000) {
        can_intf.beginExtendedPacket(static_cast<int32_t>(id) & 0x1fffffff, length, !data);
    } else {
        can_intf.beginPacket(static_cast<int>(id), length, !data);
    }
    if (data) {
        for (int i = 0; i < length; ++i) {
            can_intf.write(data[i]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        }
    }
    return static_cast<bool>(can_intf.endPacket());
}

static void pumpEvents([[maybe_unused]] CANControllerClass& can_intf)
{
    delay(10); // not sure why this resulted in less dropped messages, could have been a twisted coincidence
}

static void receiveCallback(int packet_size)
{
    enum { MAX_SIZE = 8 };
    std::array<uint8_t, MAX_SIZE> buffer;

    if (packet_size > MAX_SIZE) {
        return; // not supported
    }

    CAN.readBytes(&buffer[0], packet_size);

    oDrive0.onReceive(CAN.packetId(), packet_size, &buffer[0]);
    oDrive1.onReceive(CAN.packetId(), packet_size, &buffer[0]);
}


bool Motors_ODriveCAN::setup()
{
    // Register callbacks for the heartbeat and encoder feedback messages
    oDrive0.onFeedback(::onFeedback, &_oDrv0_user_data);
    oDrive0.onStatus(::onHeartbeat, &_oDrv0_user_data);

    oDrive1.onFeedback(::onFeedback, &_oDrv1_user_data);
    oDrive1.onStatus(::onHeartbeat, &_oDrv1_user_data);

    // Configure and initialize the CAN bus interface.
    //enum { CAN_RX_PIN = 16, CAN_TX_PIN = 17 }; // default is GPIO_NUM_4, GPIO_NUM_5
    CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
    CAN.onReceive(receiveCallback);
    if (CAN.begin(CAN_BAUDRATE) == 0) {
        Serial.println("CAN failed to initialize: reset required");
        while (true) {} // spin indefinitely
    }

    Serial.println("Waiting for ODrive0...");
    while (!_oDrv0_user_data.received_heartbeat) {
        pumpEvents(can_intf);
        delay(100);
    }
    Serial.println("Found ODrive0");

    Serial.println("Waiting for ODrive1...");
    while (!_oDrv1_user_data.received_heartbeat) {
        pumpEvents(can_intf);
        delay(100);
    }
    Serial.println("Found ODrive1");

    // Request bus voltage and current (1 second timeout)
    Serial.println("Attempting to read bus voltage and current");
    Get_Bus_Voltage_Current_msg_t vbus; // NOLINT(misc-const-correctness) false positive
    if (!oDrive0.request(vbus, 1)) {
        Serial.println("ODrive0 vbus request failed!");
        while (true) {} // spin indefinitely
    }
    Serial.printf("ODrive0 DC voltage [V]: %f DC current [A]: %f\r\n", vbus.Bus_Voltage, vbus.Bus_Current);

    if (!oDrive1.request(vbus, 1)) {
        Serial.println("ODrive1 vbus request failed!");
        while (true) {} // spin indefinitely
    }
    Serial.printf("ODrive1 DC voltage [V]: %f DC current [A]: %f\r\n", vbus.Bus_Voltage, vbus.Bus_Current);


    Serial.println("Enabling closed loop control...");
    while (_oDrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        oDrive0.clearErrors();
        delay(1);
        oDrive0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        // Pump events for 150ms. This delay is needed for two reasons;
        // 1. If there is an error condition, such as missing DC power, the ODrive might
        //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
        //    on the first heartbeat response, so we want to receive at least two
        //    heartbeats (100ms default interval).
        // 2. If the bus is congested, the setState command won't get through
        //    immediately but can be delayed.
        for (auto ii = 0; ii < 15; ++ii) {
            delay(10);
            pumpEvents(can_intf);
        }
    }
    Serial.println("ODrive0 running!");

    while (_oDrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        oDrive1.clearErrors();
        delay(1);
        oDrive1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        for (auto ii = 0; ii < 15; ++ii) {
            delay(10);
            pumpEvents(can_intf);
        }
    }
    Serial.println("ODrive1 running!");

    return true;
}

void Motors_ODriveCAN::setPower(float leftPower, float rightPower)
{
    oDrive0.setTorque(leftPower);
    oDrive1.setTorque(rightPower);
}

#endif
