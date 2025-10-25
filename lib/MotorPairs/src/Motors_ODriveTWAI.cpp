#if defined(MOTORS_O_DRIVE_TWAI)

#include "Motors_ODriveTWAI.h"

#include <HardwareSerial.h>

#include <ODriveCAN.h> //https://github.com/odriverobotics/ODriveArduino/blob/master/src/ODriveCAN.h

#include <driver/gpio.h>
// https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32/api-reference/peripherals/twai.html
#include <driver/twai.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <task.h>
#endif
#endif

#include <array>

/*!
Empty ODrive_Controller class, required as a placeholder for the ODrive CAN interface.
*/
class ODrive_Controller {
};

static bool sendMsg(ODrive_Controller& canInterface, uint32_t id, uint8_t length, const uint8_t* data);
static void pumpEvents(ODrive_Controller& canInterface);

// statically allocate the CAN class
ODrive_Controller CAN;
// CREATE_CAN_INTF_WRAPPER creates a wrapper for the class with sendMsg and pumpEvents functions
// cppcheck-suppress cstyleCast
CREATE_CAN_INTF_WRAPPER(ODrive_Controller) // NOLINT(cppcoreguidelines-pro-type-cstyle-cast)
static ODrive_Controller& canInterface = CAN;


// Instantiate ODrive objects
static ODriveCAN oDrive0(wrap_can_intf(canInterface), Motors_ODrive::O_DRIVE_0_NODE_ID); // NOLINT(cert-err58-cpp,fuchsia-statically-constructed-objects)
static ODriveCAN oDrive1(wrap_can_intf(canInterface), Motors_ODrive::O_DRIVE_1_NODE_ID); // NOLINT(cert-err58-cpp,fuchsia-statically-constructed-objects)

static bool sendMsg([[maybe_unused]] ODrive_Controller& canInterface, uint32_t id, uint8_t length, const uint8_t* data)
{
    twai_message_t message;

    if (id & 0x80000000) {
        message.identifier = id & 0x1fffffffU;
        message.extd = 1;
    } else {
        message.identifier = id;
        message.extd = 0;
    }
    if (length > TWAI_FRAME_MAX_DLC) {
        length = TWAI_FRAME_MAX_DLC;
    }
    message.data_length_code = length;
    for (auto ii = 0; ii < length; ++ii) {
        message.data[ii] = data[ii]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

    // Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        Serial.printf("Message queued for transmission\n");
    } else {
        Serial.printf("Failed to queue message for transmission\n");
        return false;
    }
    return true;
    /*if (id & 0x80000000) {
        canInterface.beginExtendedPacket(id & 0x1fffffff, length, !data);
    } else {
        canInterface.beginPacket(id, length, !data);
    }
    if (data) {
        for (int i = 0; i < length; ++i) {
            canInterface.write(data[i]);
        }
    }
    return canInterface.endPacket();*/
}

static void pumpEvents([[maybe_unused]] ODrive_Controller& canInterface)
{
    delay(10); // ODrive library comment states: not sure why this resulted in less dropped messages, could have been a twisted coincidence
}

// See https://github.com/m5stack/M5Unit-Roller/blob/main/src/unit_rollercan.cpp#L1383
static void onReceive(const twai_message_t& message)
{
    enum { MAX_SIZE = 8 };
    std::array<uint8_t, MAX_SIZE> buffer;

    const uint32_t packetId = message.identifier;
    const uint32_t packetSize = message.data_length_code;
    if (packetSize > MAX_SIZE) {
        return; // not supported
    }
    memcpy(&buffer[0], message.data, packetSize);

    oDrive0.onReceive(packetId, packetSize, &buffer[0]);
    oDrive1.onReceive(packetId, packetSize, &buffer[0]);
}


// see https://github.com/espressif/esp-idf/tree/v5.3.1/examples/peripherals/twai/twai_network
static void twai_receive_task(void* arg)
{
    (void)arg;

    twai_message_t message;
    while (true) {
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK) {
            onReceive(message);
        }
    }
}

/*!
Setup TWAI: define the TWAI configuration, install the TWAI driver, and start the TWAI driver.
*/
bool Motors_ODriveTWAI::setupTWAI()
{
    //Initialize the TWAI configuration structures using macro initializers
    static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    // Default ODrive baudrate is 250000
    static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    //Install the TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.printf("TWAI driver installed\n");
    } else {
        Serial.printf("Failed to install TWAI driver\n");
        return false;
    }

    //Start the TWAI driver
    if (twai_start() == ESP_OK) {
        Serial.printf("TWAI driver started\n");
    } else {
        Serial.printf("Failed to start TWAI driver\n");
        return false;
    }
    return true;
}

/*!
Setup ODrive
1. register the message callbacks.
2. start the TWAI task.
3. wait for the heartbeat from the 2 ODrives.
4. check the ODrives' bus voltage and current.
5. put the ODrives into closed loop control mode.
*/
bool Motors_ODriveTWAI::setup()
{
    setupTWAI();

    // Register callbacks for the heartbeat and encoder feedback messages
    oDrive0.onFeedback(::onFeedback, &_oDrv0_user_data);
    oDrive0.onStatus(::onHeartbeat, &_oDrv0_user_data);

    oDrive1.onFeedback(::onFeedback, &_oDrv1_user_data);
    oDrive1.onStatus(::onHeartbeat, &_oDrv1_user_data);

    enum { TASK_PRIORITY = 8 };
    enum { TASK_STACK_DEPTH = 4096 };
    static constexpr void* taskParameters = nullptr;
    static StaticTask_t taskBuffer;
    static StackType_t stack[TASK_STACK_DEPTH];
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(twai_receive_task, "TWAI_receive", TASK_STACK_DEPTH, taskParameters, TASK_PRIORITY, stack, &taskBuffer, tskNO_AFFINITY);
    assert(taskHandle != nullptr && "Unable to create TWAI receive task.");

    Serial.println("Waiting for ODrive0...");
    while (!_oDrv0_user_data.received_heartbeat) {
        pumpEvents(canInterface);
        delay(100);
    }
    Serial.println("Found ODrive0");

    Serial.println("Waiting for ODrive1...");
    while (!_oDrv1_user_data.received_heartbeat) {
        pumpEvents(canInterface);
        delay(100);
    }
    Serial.println("Found ODrive1");

    // Request bus voltage and current (1 second timeout)
    Serial.println("Attempting to read ODrive bus voltage and current");
    Get_Bus_Voltage_Current_msg_t vbus; // NOLINT(misc-const-correctness) false positive
    if (!oDrive0.request(vbus, 1)) {
        Serial.println("ODrive0 vbus request failed!");
        while (true) {} // spin indefinitely
    }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
        Serial.printf("ODrive0 DC voltage [V]: %f DC current [A]: %f\r\n", vbus.Bus_Voltage, vbus.Bus_Current);

    if (!oDrive1.request(vbus, 1)) {
        Serial.println("ODrive1 vbus request failed!");
        while (true) {} // spin indefinitely
    }
    Serial.printf("ODrive1 DC voltage [V]: %f DC current [A]: %f\r\n", vbus.Bus_Voltage, vbus.Bus_Current);
#pragma GCC diagnostic pop

    Serial.println("Enabling ODrive closed loop control...");
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
            pumpEvents(canInterface);
        }
    }
    Serial.println("ODrive0 running!");

    while (_oDrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        oDrive1.clearErrors();
        delay(1);
        oDrive1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        for (auto ii = 0; ii < 15; ++ii) {
            delay(10);
            pumpEvents(canInterface);
        }
    }
    Serial.println("ODrive1 running!");

    return true;
}

void Motors_ODriveTWAI::setPower(float leftPower, float rightPower)
{
    oDrive0.setTorque(leftPower);
    oDrive1.setTorque(rightPower);
}

#endif
