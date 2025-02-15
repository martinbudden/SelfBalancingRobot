#if defined(MOTORS_O_DRIVE)

#include "Motors_ODrive.h"

#include <ODriveCAN.h> //https://github.com/odriverobotics/ODriveArduino/blob/master/src/ODriveCAN.h

#include <cmath>

/*
See
https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html

https://github.com/sandeepmistry/arduino-CAN

https://github.com/m5stack/M5Stack/blob/master/examples/Unit/CAN/CAN.ino
https://github.com/arduino/ArduinoCore-API
https://github.com/arduino/ArduinoCore-API/blob/4a02bfc0a924e1fec34c3bb82ffd5dfba7643a0c/api/HardwareCAN.h

https://copperhilltech.com/blog/esp32-processor-adding-a-can-bus-transceiver/

see also https://github.com/handmade0octopus/ESP32-TWAI-CAN/blob/master/src/ESP32-TWAI-CAN.cpp for minimal TWAI library.
*/

/*
M5Stack CAN Modules:

**Module COMMU** has an MCP2515 CAN controller, connected via SPI, using pins: CS-GPIO12, INT-GPIO15, SCK-GPIO18, MISO-GPIO19 MOSI-GPIO23.
The MCP2515 CAN controller is subsequently connected to a TJA1051T/3 CAN transceiver.

**PwrCAN Module 13.2** connects directly to a CA-IS3050G isolated CAN transceiver. The GPIO pins for this connection are selectable via DIP switches.
DIP switches available are:

| Product | CAN_TX/RS485_TX      | CAN_RX/RS485_RX       |
| ------- | -------------------- | --------------------- |
| Basic   | G17 / G15 / G12 / G0 | G16 / G13 / G34 / G35 |
| Core2   | G14 / G2 / G27 / G0  | G13 / G19 / G34 / G35 |
| CoreS3  | G17 / G13 / G6 / G0  | G18 / G7 / G14 / G10  |

**Unit-Mini CAN** has a TJA1051T/3 CAN transceiver which is connected to a grove pin connector. Using PORT C, the pins utilized are:

| Product | RX  | TX  |
| ------- | --- | --- |
| Basic   | G16 | G17 |
| Core2   | G13 | G14 |
| CoreS3  | G18 | G17 |

**CAN Unit** has a CA-IS3050G isolated CAN transceiver which is connected to a grove pin connector. Pins are as for the **Unit-Mini CAN**.

**Atomic CAN Base** connects directly to a CA-IS3050G isolated CAN transceiver. The GPIO pins used are:

| Product     | RX  | TX  |
| ----------- | --- | --- |
| AtomS3      | G5  | G6  |
| Atom Lite   | G22 | G19 |
| Atom Matrix | G22 | G19 |

*/

namespace { // use anonymous namespace to make items local to this translation unit
/*!
A pointer to the Motors_ODrive object is required for the callback functions.
*/
Motors_ODrive* motors_ODrive;
} // end namespace

Motors_ODrive::Motors_ODrive(float stepsPerRevolution) :
    MotorPairBase(stepsPerRevolution, CAN_ACCURATELY_ESTIMATE_SPEED)
{

    // set motors_ODrive for use by callback functions.
    motors_ODrive = this;
}

void Motors_ODrive::readEncoder()
{
    _feedback0 = _oDrv0_user_data.last_feedback;
    _oDrv0_user_data.received_feedback = false;

    _feedback1 = _oDrv1_user_data.last_feedback;
    _oDrv1_user_data.received_feedback = false;

    _leftEncoder = round(_feedback0.Pos_Estimate * _stepsPerRevolution); // Pos_Estimate is revolution count
    _rightEncoder = round(_feedback1.Pos_Estimate * _stepsPerRevolution);

    _leftSpeed = _feedback0.Vel_Estimate; // revolutions/s
    _rightSpeed = _feedback1.Vel_Estimate;
}

/*!
Wrapper function for Motors_ODrive::onHeartbeat with the correct signature to be registered as a callback by ODriveCAN::onStatus.

Called every time a Heartbeat message arrives from the ODrive

By default this happens every 100ms, see:
https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages
*/
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data)
{
    motors_ODrive->onHeartbeat(msg, static_cast<Motors_ODrive::ODriveUserData*>(user_data));
}

void Motors_ODrive::onHeartbeat(const Heartbeat_msg_t& msg, ODriveUserData* oDriveUserData)
{
    oDriveUserData->last_heartbeat = msg;
    oDriveUserData->received_heartbeat = true;
}

/*!
Wrapper function for Motors_ODrive::onFeedback with the correct signature to be registered as a callback by ODriveCAN::onFeedback.

Called every time a feedback message arrives from the ODrive.

By default this happens every 10ms, see:
https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages
*/
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data)
{
    motors_ODrive->onFeedback(msg, static_cast<Motors_ODrive::ODriveUserData*>(user_data));
}

void Motors_ODrive::onFeedback(const Get_Encoder_Estimates_msg_t& msg, ODriveUserData* oDriveUserData)
{
    oDriveUserData->last_feedback = msg;
    oDriveUserData->received_feedback = true;
}

#endif
