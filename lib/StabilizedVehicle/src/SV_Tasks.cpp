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
#endif


/*!
The ESP32S3 is dual core containing a Protocol CPU (known as CPU 0 or PRO_CPU) and an Application CPU (known as CPU 1 or APP_CPU).

The core affinities, priorities, and tick intervals and priorities for the 3 application tasks (AHRS_TASK, VEHICLE_CONTROLLER_TASK, and MAIN_LOOP_TASK).
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
    std::array<char, 128> buf;
    sprintf(&buf[0], "\r\n\r\n**** Main loop task, name:'%s' priority:%u, tickRate:%uHz\r\n", taskName, taskPriority, configTICK_RATE_HZ);
    Serial.print(&buf[0]);
#endif
}

AHRS_Task* SV_Tasks::setupTask(AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static AHRS_Task task(taskIntervalMicroSeconds, ahrs);
    ahrs.setTask(&task);

#if defined(USE_FREERTOS)
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 4096 };
    static StaticTask_t taskBuffer;
    static std::array <StackType_t, TASK_STACK_DEPTH> stack;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        AHRS_Task::Task,
        "AHRS_Task",
        TASK_STACK_DEPTH,
        &taskParameters,
        priority,
        &stack[0],
        &taskBuffer,
        coreID
    );
    assert(taskHandle != nullptr && "Unable to create AHRS task.");
#if !defined(FRAMEWORK_ESPIDF)
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** AHRS_Task,      core:%u, priority:%u, task interval:%ums\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
    Serial.print(&buf[0]);
#endif
#else
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &task;
}

VehicleControllerTask* SV_Tasks::setupTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static VehicleControllerTask task(taskIntervalMicroSeconds, vehicleController);
    vehicleController.setTask(&task);

#if defined(USE_FREERTOS)
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 4096 };
    static StaticTask_t taskBuffer;
    static std::array<StackType_t, TASK_STACK_DEPTH> stack;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(VehicleControllerTask::Task,
        "MPC_Task",
        TASK_STACK_DEPTH,
        &taskParameters,
        priority,
        &stack[0],
        &taskBuffer,
        coreID
    );
    assert(taskHandle != nullptr && "Unable to create MotorPairController task.");
#if !defined(FRAMEWORK_ESPIDF)
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** MPC_Task,       core:%u, priority:%u, task interval:%ums\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
    Serial.print(&buf[0]);
#endif
#else
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &task;
}

ReceiverTask* SV_Tasks::setupTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static ReceiverTask task(taskIntervalMicroSeconds, receiver, receiverWatcher);

#if defined(USE_FREERTOS)
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 4096 };
    static std::array<StackType_t, TASK_STACK_DEPTH> stack;
    static StaticTask_t taskBuffer;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        ReceiverTask::Task,
        "Receiver_Task",
        TASK_STACK_DEPTH,
        &taskParameters,
        priority,
        &stack[0],
        &taskBuffer,
        coreID
    );
    assert(taskHandle != nullptr && "Unable to create ReceiverTask task.");
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** RECEIVER_Task,  core:%u, priority:%u, task interval:%ums\r\n\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
    Serial.print(&buf[0]);
#else
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &task;
}

BackchannelTask* SV_Tasks::setupTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    (void)taskIntervalMicroSeconds;
    static BackchannelTask task(backchannel);
    backchannel.setTask(&task);

#if defined(USE_FREERTOS)
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 4096 };
    static std::array<StackType_t, TASK_STACK_DEPTH> stack;
    static StaticTask_t taskBuffer;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        BackchannelTask::Task,
        "Backchannel_Task",
        TASK_STACK_DEPTH,
        &taskParameters,
        priority,
        &stack[0],
        &taskBuffer,
        coreID
    );
    assert(taskHandle != nullptr && "Unable to create ReceiverTask task.");
    std::array<char, 128> buf;
    Serial.printf(&buf[0], "**** BACKCHANNEL_Task,core:%u, priority:%u, task is interrupt driven\r\n", coreID, priority);

    Serial.print(&buf[0]);
#else
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &task;
}
