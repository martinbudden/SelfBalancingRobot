#include "ReceiverTask.h"
#include "ReceiverBase.h"
#include "TimeMicroSeconds.h"

ReceiverTask::ReceiverTask(uint32_t taskIntervalMicroSeconds, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher) :
    TaskBase(taskIntervalMicroSeconds),
    _receiver(receiver),
    _receiverWatcher(receiverWatcher)
{
}

/*!
loop() function for when not using FREERTOS
*/
void ReceiverTask::loop()
{
    // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than taskIntervalTicks
#if defined(USE_FREERTOS)
    const TickType_t tickCount = xTaskGetTickCount();
#else
    const uint32_t tickCount = timeUs() / 1000;
    //const uint32_t timeMicroSeconds = timeUs();
    //_timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
    //_timeMicroSecondsPrevious = timeMicroSeconds;
#endif

    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;
    if (_tickCountDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
        const bool newPacketAvailable = _receiver.update(_tickCountDelta);
        if (newPacketAvailable && _receiverWatcher) {
            // if there is a new packet and a watcher, then let the watcher know.
            _receiverWatcher->newReceiverPacketAvailable();
        }
    }
}

/*!
Task function for the ReceiverTask. Sets up and runs the task loop() function.
*/
[[noreturn]] void ReceiverTask::task()
{
#if defined(USE_FREERTOS)
#if defined(RECEIVER_TASK_IS_INTERRUPT_DRIVEN)
    while (true) {
        _receiver.WAIT_FOR_DATA_RECEIVED();
        loop();
    }
#else
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    const uint32_t taskIntervalTicks = pdMS_TO_TICKS(_taskIntervalMicroSeconds / 1000);
    assert(taskIntervalTicks > 0 && "ReceiverTask taskIntervalTicks is zero.");
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
        loop();
    }
#endif // RECEIVER_TASK_IS_INTERRUPT_DRIVEN
#else
    while (true) {}
#endif // USE_FREERTOS
}

/*!
Wrapper function for ReceiverTask::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void ReceiverTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<ReceiverTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast}
}
