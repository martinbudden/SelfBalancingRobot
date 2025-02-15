#if defined(MOTORS_PWR_CAN)

#include "MotorsPwrCAN.h"
#include <HardwareSerial.h>

// See https://github.com/m5stack/M5Stack/blob/master/examples/Unit/CAN/CAN.ino
// and https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html


CAN_device_t CAN_cfg; // NOLINT(fuchsia-statically-constructed-objects) this is required for the ESP32CAN library

namespace { // use anonymous namespace to make items local to this translation unit
StaticQueue_t xStaticQueue;
enum { RX_QUEUE_CAPACITY = 10 };
uint8_t ucQueueStorageArea[RX_QUEUE_CAPACITY * sizeof(CAN_frame_t)]; // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
} // end namespace

constexpr float encoderStepsPerRevolution {100.0};


MotorsPwrCAN::MotorsPwrCAN() :
    MotorPairBase(encoderStepsPerRevolution, MotorPairBase::CAN_ACCURATELY_ESTIMATE_SPEED)
{

    CAN_cfg.speed     = CAN_SPEED_125KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_26;
    CAN_cfg.rx_pin_id = GPIO_NUM_36;

    //CAN_cfg.rx_queue  = xQueueCreate(RX_QUEUE_CAPACITY, sizeof(CAN_frame_t));
    CAN_cfg.rx_queue  = xQueueCreateStatic(RX_QUEUE_CAPACITY, sizeof(CAN_frame_t), ucQueueStorageArea, &xStaticQueue );

    ESP32Can.CANInit();
}

void MotorsPwrCAN::readEncoder()
{
    if (xQueueReceive(CAN_cfg.rx_queue, &_rxFrame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
        if (_rxFrame.FIR.B.FF == CAN_frame_std) { // NOLINT(bugprone-branch-clone) false positive
            Serial.println("New standard frame");
        } else {
            Serial.println("New extended frame");
        }

        if (_rxFrame.FIR.B.RTR == CAN_RTR) { // NOLINT(bugprone-branch-clone) false positive
            Serial.printf("RTR from 0x%08X, DLC %d\r\n", _rxFrame.MsgID, _rxFrame.FIR.B.DLC);
        } else {
            Serial.printf("from 0x%08X, DLC %d, Data \r\n", _rxFrame.MsgID, _rxFrame.FIR.B.DLC);
            for (int i = 0; i < _rxFrame.FIR.B.DLC; i++) {
                Serial.printf("0x%02X ", _rxFrame.data.u8[i]);
            }
            Serial.println("\n");
        }
    }
}


void MotorsPwrCAN::setPower([[maybe_unused]] float leftPower, [[maybe_unused]] float rightPower)
{
    _txFrame.FIR.B.FF   = CAN_frame_std;
    _txFrame.MsgID      = 0x001;
    _txFrame.FIR.B.DLC  = 8;
    // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
    _txFrame.data.u8[0] = 0x00;
    _txFrame.data.u8[1] = 0x01;
    _txFrame.data.u8[2] = 0x02;
    _txFrame.data.u8[3] = 0x03;
    _txFrame.data.u8[4] = 0x04;
    _txFrame.data.u8[5] = 0x05;
    _txFrame.data.u8[6] = 0x06;
    _txFrame.data.u8[7] = 0x07;
    // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    ESP32Can.CANWriteFrame(&_txFrame);
}

#endif
