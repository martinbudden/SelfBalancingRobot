#include "AHRS.h"
#include "BackchannelBase.h"
#include "SV_Tasks.h"
#include "TaskBase.h"
#include "VehicleControllerBase.h"

#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif
#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/FreeRTOSConfig.h>
#endif

#if defined(USE_FREERTOS)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    assert(false && "stack overflow");
    Serial.printf("\r\n\r\n*********\r\n");
    Serial.printf("********Task '%s' stack overflow ********\r\n", pcTaskName);
    Serial.printf("*********\r\n\r\n");
}
#endif

/*!
The ESP32S3 is dual core containing a Protocol CPU (known as CPU 0 or PRO_CPU) and an Application CPU (known as CPU 1 or APP_CPU).

The core affinities, priorities, and tick intervals for the 3 application tasks (AHRS_TASK, VEHICLE_CONTROLLER_TASK, and MAIN_LOOP_TASK):
1. The AHRS_TASK must have a higher priority than the MAIN_LOOP_TASK.
2. The VEHICLE_CONTROLLER_TASK must have a higher priority than the MAIN_LOOP_TASK
3. For single-processors the AHRS_TASK and the VEHICLE_CONTROLLER_TASK must have the same priority.
4. For dual-core processors
    1. The AHRS_TASK runs on the Application CPU (CPU 1).
    2. The VEHICLE_CONTROLLER_TASK runs on the Protocol CPU (CPU 0).
5. The MAIN_LOOP_TASK runs on the Application CPU (CPU 1) with priority 1 (this is set by the ESP32 Arduino framework).

The AHRS_TASK and the VEHICLE_CONTROLLER_TASK are deliberately chosen to run on different cores on ESP32 dual-core processors. This is so
that a context switch between the AHRS_TASK and the VEHICLE_CONTROLLER_TASK does not require saving the FPU(Floating Point Unit) registers
(see https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32/api-guides/freertos-smp.html#floating-point-usage).

The Atom JoyStick transmits a packet every 10ms.
Updating the screen takes approximately 50 ticks.
*/


void SV_Tasks::reportMainTask()
{
#if defined(USE_ARDUINO_ESP32)
    // The main task is set up by the framework, so just print its details.
    // It has name "loopTask" and priority 1.
    const TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const UBaseType_t taskPriority = uxTaskPriorityGet(taskHandle);
    const char* taskName = pcTaskGetName(taskHandle);
    Serial.printf("\r\n\r\n**** Main loop task, name:'%s' priority:%u, tickRate:%uHz\r\n", taskName, taskPriority, configTICK_RATE_HZ);
#endif
}

AHRS_Task* SV_Tasks::createAHRS_Task(task_info_t& taskInfo, AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static AHRS_Task task(taskIntervalMicroSeconds, ahrs);
    ahrs.setTask(&task);

#if defined(USE_FREERTOS)
    Serial.printf("**** AHRS_Task,              core:%u, priority:%u, task interval:%ums\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 2048 };
    static StaticTask_t taskBuffer;
    static std::array <StackType_t, TASK_STACK_DEPTH> stack;
#if !defined(configCHECK_FOR_STACK_OVERFLOW)
    // fill the stack so we can do our own stack overflow detection
    stack.fill(a5);
#endif
    taskInfo = {
        .taskHandle = nullptr,
        .name = "AHRS_Task",
        .stackDepth = TASK_STACK_DEPTH,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
    };
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "AHRS_Task: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "AHRS_Task: priority too high");

    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        AHRS_Task::Task,
        taskInfo.name,
        taskInfo.stackDepth,
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    taskInfo.taskHandle = taskHandle;
    assert(taskHandle != nullptr && "Unable to create AHRS task.");
#else
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &task;
}

AHRS_Task* SV_Tasks::createAHRS_Task(AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static task_info_t taskInfo;
    return createAHRS_Task(taskInfo, ahrs, priority, coreID, taskIntervalMicroSeconds);
}

VehicleControllerTask* SV_Tasks::createVehicleControllerTask(task_info_t& taskInfo, VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static VehicleControllerTask task(taskIntervalMicroSeconds, vehicleController);
    vehicleController.setTask(&task);

#if defined(USE_FREERTOS)
    Serial.printf("**** MPC_Task,               core:%u, priority:%u, task interval:%ums\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 2048 };
    static std::array<StackType_t, TASK_STACK_DEPTH> stack;
    static StaticTask_t taskBuffer;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "MPC_Task", // max length 16, including zero terminator
        .stackDepth = TASK_STACK_DEPTH,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
    };
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "MPC_Task: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "MPC_Task: priority too high");

    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        VehicleControllerTask::Task,
        taskInfo.name,
        taskInfo.stackDepth,
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create MPC_Task.");
#else
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &task;
}

VehicleControllerTask* SV_Tasks::createVehicleControllerTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static task_info_t taskInfo;
    return createVehicleControllerTask(taskInfo, vehicleController, priority, coreID, taskIntervalMicroSeconds);
}

ReceiverTask* SV_Tasks::createReceiverTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID)
{
#if defined(RECEIVER_TASK_IS_NOT_INTERRUPT_DRIVEN)
    assert(false && "Task interval not specified for ReceiverTask");
#endif
    return createReceiverTask(receiver, receiverWatcher, priority, coreID, 0);
}

ReceiverTask* SV_Tasks::createReceiverTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID)
{
#if defined(RECEIVER_TASK_IS_NOT_INTERRUPT_DRIVEN)
    assert(false && "Task interval not specified for ReceiverTask");
#endif
    return createReceiverTask(taskInfo, receiver, receiverWatcher, priority, coreID, 0);
}

ReceiverTask* SV_Tasks::createReceiverTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    task_info_t taskInfo;
    return createReceiverTask(taskInfo, receiver, receiverWatcher, priority, coreID, taskIntervalMicroSeconds);
}

ReceiverTask* SV_Tasks::createReceiverTask(task_info_t& taskInfo, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
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
    enum { TASK_STACK_DEPTH = 1024 };
    static std::array<StackType_t, TASK_STACK_DEPTH> stack;
    static StaticTask_t taskBuffer;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "ReceiverTask", // max length 16, including zero terminator
        .stackDepth = TASK_STACK_DEPTH,
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
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &task;
}

BackchannelTask* SV_Tasks::createBackchannelTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static BackchannelTask task(taskIntervalMicroSeconds, backchannel);

#if defined(USE_FREERTOS)
    Serial.printf("**** BackchannelTask,        core:%u, priority:%u, task interval:%ums\r\n\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 4096 }; // 2048 probably sufficient when not using Serial.printf statements
    static StaticTask_t taskBuffer;
    static std::array <StackType_t, TASK_STACK_DEPTH> stack;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "Backchannel", // max length 16, including zero terminator
        .stackDepth = TASK_STACK_DEPTH,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
    };
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "Backchannel: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "Backchannel: priority too high");

    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        BackchannelTask::Task,
        taskInfo.name,
        taskInfo.stackDepth,
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create BackchannelTask.");
#if !defined(configCHECK_FOR_STACK_OVERFLOW)
    // fill the stack so we can do our own stack overflow detection
    stack.fill(a5);
#endif
#else
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &task;
}

BackchannelTask* SV_Tasks::createBackchannelTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static task_info_t taskInfo;
    return createBackchannelTask(taskInfo, backchannel, priority, coreID, taskIntervalMicroSeconds);
}
