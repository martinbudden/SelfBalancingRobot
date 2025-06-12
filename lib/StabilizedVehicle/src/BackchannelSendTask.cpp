#include "BackchannelSendTask.h"
#include "BackchannelBase.h"
#include "TimeMicroSeconds.h"

/*!
loop() function for when not using FREERTOS
*/
void BackchannelSendTask::loop()
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
        _backchannel.sendTelemetryPacket();
    }
}

/*!
Task function for the BackchannelSendTask.
*/
[[noreturn]] void BackchannelSendTask::task()
{
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    const uint32_t taskIntervalTicks = pdMS_TO_TICKS(_taskIntervalMicroSeconds / 1000);
    assert(taskIntervalTicks > 0 && "BackchannelSendTask taskIntervalTicks is zero.");
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);

        _backchannel.sendTelemetryPacket();
    }
#else
    while (true) {}
#endif // USE_FREERTOS
}

/*!
Wrapper function for BackchannelSendTask::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void BackchannelSendTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<BackchannelSendTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast}
}
