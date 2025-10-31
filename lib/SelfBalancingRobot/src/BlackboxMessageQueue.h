#pragma once

#include <AHRS.h>
#include <BlackboxMessageQueueBase.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <queue.h>
#endif
#endif


class BlackboxMessageQueue : public BlackboxMessageQueueBase {
public:
    const AHRS::imu_data_t& getQueueItem() const { return _queueItem; };
#if defined(FRAMEWORK_USE_FREERTOS)
    BlackboxMessageQueue()
        : _queue(xQueueCreateStatic(QUEUE_LENGTH, sizeof(_queueItem), &_queueStorageArea[0], &_queueStatic))
    {}
    // Blackbox Task calls WAIT_IF_EMPTY and then `update` when wait completes
    virtual int32_t WAIT_IF_EMPTY(uint32_t& timeMicroseconds) const override {
        const int32_t ret = xQueueReceive(_queue, &_queueItem, portMAX_DELAY);
        if (ret) {
            timeMicroseconds = _queueItem.timeMicroseconds;
        }
        return ret;
    }
    inline int32_t RECEIVE() const { return xQueueReceive(_queue, &_queueItem, portMAX_DELAY); }
    inline void SEND(const AHRS::imu_data_t& queueItem) const { xQueueOverwrite(_queue, &queueItem); }
#else
    BlackboxMessageQueue() = default;
    virtual int32_t WAIT_IF_EMPTY(uint32_t& timeMicroseconds) const override { timeMicroseconds = 0; return 0; }
    inline int32_t RECEIVE() const { return 0; }
    inline void SEND(const AHRS::imu_data_t& queueItem) const { _queueItem = queueItem; }
    //inline bool SEND_IF_NOT_FULL(const AHRS::imu_data_t& queueItem) const { (void)queueItem; return false; } // cppcheck-suppress knownConditionTrueFalse
#endif // USE_FREERTOS
private:
    mutable AHRS::imu_data_t _queueItem {};
    enum { QUEUE_LENGTH = 1 };
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_queueItem)> _queueStorageArea {};
#if defined(FRAMEWORK_USE_FREERTOS)
    StaticQueue_t _queueStatic {};
    QueueHandle_t _queue {};
#endif
};
