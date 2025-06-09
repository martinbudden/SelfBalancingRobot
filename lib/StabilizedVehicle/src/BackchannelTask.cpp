#include "BackchannelTask.h"
#include "BackchannelBase.h"
#include "TimeMicroSeconds.h"

BackchannelTask::BackchannelTask(BackchannelBase& backchannel) :
    TaskBase(0),
    _backchannel(backchannel)
{
}

/*!
loop() function for when not using FREERTOS
*/
void BackchannelTask::loop()
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
        _backchannel.update();
    }
}

/*!
Task function for the BackchannelTask. Sets up and runs the task loop() function.
*/
[[noreturn]] void BackchannelTask::task()
{
#if defined(USE_FREERTOS)
    while (true) {
        _backchannel.WAIT_FOR_DATA_RECEIVED();
        loop();
    }
#else
    while (true) {}
#endif // USE_FREERTOS
}

/*!
Wrapper function for BackchannelTask::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void BackchannelTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<BackchannelTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast}
}
