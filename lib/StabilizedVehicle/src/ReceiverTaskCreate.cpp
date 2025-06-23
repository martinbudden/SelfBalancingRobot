#include "ReceiverBase.h"
#include "ReceiverTask.h"

#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif

#include <TaskBase.h>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#endif

ReceiverTask* ReceiverTask::createTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID)
{
#if defined(RECEIVER_TASK_IS_NOT_INTERRUPT_DRIVEN)
    assert(false && "Task interval not specified for ReceiverTask");
#endif
    return createTask(receiver, receiverWatcher, priority, coreID, 0);
}

ReceiverTask* ReceiverTask::createTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID)
{
#if defined(RECEIVER_TASK_IS_NOT_INTERRUPT_DRIVEN)
    assert(false && "Task interval not specified for ReceiverTask");
#endif
    return createTask(taskInfo, receiver, receiverWatcher, priority, coreID, 0);
}

ReceiverTask* ReceiverTask::createTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, receiver, receiverWatcher, priority, coreID, taskIntervalMicroSeconds);
}

ReceiverTask* ReceiverTask::createTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static ReceiverTask task(taskIntervalMicroSeconds, receiver, receiverWatcher);

#if defined(USE_FREERTOS)
#if defined(RECEIVER_TASK_IS_NOT_INTERRUPT_DRIVEN)
    Serial.printf("**** ReceiverTask,           core:%u, priority:%u, task interval:%ums\r\n\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
#else
    Serial.printf("**** ReceiverTask,           core:%u, priority:%u, task is interrupt driven\r\n", coreID, priority);
#endif
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
#if !defined(RECEIVER_TASK_STACK_DEPTH)
    enum { RECEIVER_TASK_STACK_DEPTH = 4096 };
#endif
    static std::array<StackType_t, RECEIVER_TASK_STACK_DEPTH> stack;
    static StaticTask_t taskBuffer;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "ReceiverTask", // max length 16, including zero terminator
        .stackDepth = RECEIVER_TASK_STACK_DEPTH,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
    };
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "ReceiverTask: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "ReceiverTask: priority too high");

    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        ReceiverTask::Task,
        taskInfo.name,
        taskInfo.stackDepth,
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create ReceiverTask.");
#else
    (void)taskInfo;
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &task;
}
