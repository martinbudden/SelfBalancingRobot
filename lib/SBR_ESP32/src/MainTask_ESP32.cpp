#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#include <freertos/FreeRTOS.h>

#include <WiFi.h>

#include "MainTask_ESP32.h"
#include "TelemetryScaleFactors.h"

#include <AHRS.h>
#include <ESPNOW_Backchannel.h>
#include <ESPNOW_Receiver.h>
#include <IMU_BMI270.h>
#include <IMU_BNO085.h>
#include <IMU_Filters.h>
#include <IMU_LSM303AGR.h>
#include <IMU_M5Stack.h>
#include <IMU_M5Unified.h>
#include <IMU_MPU6886.h>
#include <MotorPairBase.h>
#include <MotorPairController.h>
#include <SV_Preferences.h>
#include <SensorFusionFilter.h>
#include <cfloat>

/*!
The ESP32S3 is dual core containing a Protocol CPU (known as CPU 0 or PRO_CPU) and an Application CPU (known as CPU 1 or APP_CPU).

The core affinities, priorities, and tick intervals and priorities for the 3 application tasks (AHRS_TASK, MPC_TASK, and MAIN_LOOP_TASK).
1. The AHRS_TASK must have a higher priority than the MAIN_LOOP_TASK.
2. The MPC_TASK must have a higher priority than the MAIN_LOOP_TASK
3. For single-processors the AHRS_TASK and the MPC_TASK must have the same priority.
4. For dual-core processors
    1. The AHRS_TASK runs on the Application CPU (CPU 1).
    2. The MPC_TASK runs on the  Protocol CPU (CPU 0).
5. The MAIN_LOOP_TASK runs on the Application CPU (CPU 1) with priority 1 (this is set by the ESP32 Arduino framework).

The AHRS_TASK and the MPC_TASK are deliberately chosen to run on different cores on ESP32 dual-core processors. This is so
that a context switch between the AHRS_TASK and the MPC_TASK does not require saving the FPU(Floating Point Unit) registers
(see https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32/api-guides/freertos-smp.html#floating-point-usage).

The Atom JoyStick transmits a packet every 10ms, so the MAIN_LOOP_TASK must run at least every 10m to ensure no dropped packets.
Updating the screen takes approximately 50 ticks, so packets will be dropped if the screen is not in QRCODE mode.
*/

enum { MPC_TASK_PRIORITY = 4, AHRS_TASK_PRIORITY = 5 };

enum { MPC_TASK_CORE = PRO_CPU_NUM };
#if defined(APP_CPU_NUM) // The processor has two cores
    enum { AHRS_TASK_CORE = APP_CPU_NUM };
#else // single core processor
    enum { AHRS_TASK_CORE = PRO_CPU_NUM };
#endif

enum { MAIN_LOOP_TASK_TICK_INTERVAL_MILLISECONDS = 5 };


#if !defined(MPC_TASK_TICK_INTERVAL_MILLISECONDS)
#if defined(USE_IMU_MPU6886)
    enum { MPC_TASK_TICK_INTERVAL_MILLISECONDS = 5 };
#else
    enum { MPC_TASK_TICK_INTERVAL_MILLISECONDS = 10 }; // M5Stack IMU code blocks I2C bus for extended periods, so MPC_TAsK must be set to run slower.
#endif
#endif

#if !defined(AHRS_TASK_TICK_INTERVAL_MILLISECONDS)
enum { AHRS_TASK_TICK_INTERVAL_MILLISECONDS = 5 };
#endif


/*!
Setup for the main loop, motor control task, and AHRS(Attitude and Heading Reference System) task.
*/
void MainTask::setup()
{
    // This task has name "loopTask" and priority 1.
    const TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const UBaseType_t taskPriority = uxTaskPriorityGet(taskHandle);
    const char*  taskName = pcTaskGetName(taskHandle);
    Serial.printf("\r\n\r\n**** Main loop task, name:'%s' priority:%d, tickRate:%dHz\r\n\r\n", taskName, taskPriority, configTICK_RATE_HZ);

    void* i2cMutex = nullptr;

    setupAHRS(i2cMutex);

    // Set WiFi to station mode
    WiFi.mode(WIFI_STA);
    // Disconnect from Access Point if it was previously connected
    WiFi.disconnect();
    // get my MAC address
    uint8_t myMacAddress[ESP_NOW_ETH_ALEN];
    WiFi.macAddress(&myMacAddress[0]);

    // Statically allocate and setup the receiver.
    static Receiver receiver(&myMacAddress[0]);
    _receiver = &receiver;
#if !defined(JOYSTICK_CHANNEL)
    constexpr uint8_t JOYSTICK_CHANNEL {3};
#endif
    const esp_err_t err = receiver.setup(JOYSTICK_CHANNEL);
    Serial.printf("**** ESP-NOW Ready:%X\r\n\r\n", err);
    assert(err == ESP_OK && "Unable to setup receiver.");

    // Statically allocate the motorPairController.
    static MotorPairController motorPairController(*_ahrs, receiver, i2cMutex);
    _motorPairController = &motorPairController;
    _receiver->setMotorController(_motorPairController);
    _ahrs->setMotorController(_motorPairController);

    static SV_Preferences preferences;
    _preferences = &preferences;

    checkGyroCalibration();

    loadPreferences();

#if defined(BACKCHANNEL_MAC_ADDRESS)
    // Statically allocate the telemetry scale factors
    static TelemetryScaleFactors telemetryScaleFactors(_motorPairController->getControlMode());
    // Statically allocate the backchannel.
    constexpr uint8_t backchannelMacAddress[ESP_NOW_ETH_ALEN] BACKCHANNEL_MAC_ADDRESS;
    static Backchannel backchannel(receiver.getESPNOW_Transceiver(), backchannelMacAddress, *_motorPairController, *_ahrs, *this, *_receiver, telemetryScaleFactors, _preferences);
    _backchannel = &backchannel;
#endif
    // And finally set up the AHRS and MotorPairController tasks.
    setupTasks();
}

void MainTask::setupAHRS([[maybe_unused]] void* i2cMutex)
{
#if defined(USE_IMU_MPU6886)
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, IMU_SDA_PIN, IMU_SCL_PIN, i2cMutex); // NOLINT(misc-const-correctness) false positive
#elif defined(USE_IMU_BMI270)
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, IMU_SDA_PIN, IMU_SCL_PIN, i2cMutex);
#elif defined(USE_IMU_BNO085)
    static IMU_BNO085 imuSensor(IMU_AXIS_ORDER, IMU_SDA_PIN, IMU_SCL_PIN, i2cMutex); // NOLINT(misc-const-correctness) false positive
#elif defined(USE_IMU_LSM303AGR)
    static IMU_LSM303AGR imuSensor(IMU_AXIS_ORDER, IMU_SDA_PIN, IMU_SCL_PIN, i2cMutex);
#endif

    // Statically allocate the Sensor Fusion Filter and the AHRS object.
#if defined(USE_COMPLEMENTARY_FILTER)
    // approx 130 microseconds per update
    static ComplementaryFilter sensorFusionFilter;
#elif defined(USE_MAHONY_FILTER)
    // approx 10 microseconds per update
    static MahonyFilter sensorFusionFilter;
#else
    // approx 16 microseconds per update
    static MadgwickFilter sensorFusionFilter; // NOLINT(misc-const-correctness) false positive
#endif
    // statically allocate the IMU_Filters
    constexpr float cutoffFrequency = 100.0F;
    static IMU_Filters imuFilters(cutoffFrequency, static_cast<float>(AHRS_TASK_TICK_INTERVAL_MILLISECONDS) / 1000.0F); // NOLINT(misc-const-correctness) false positive

    static AHRS ahrs(sensorFusionFilter, imuSensor, imuFilters);
    _ahrs = &ahrs;
}

void MainTask::checkGyroCalibration()
{
}

/*!
Resets the PID preferences and the Balance Angle to FLT_MAX (which represents unset).
*/
void MainTask::resetPreferences()
{
    //_preferences->removeGyroOffset();
    //_preferences->removeAccOffset();
    for (int ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        const std::string pidName = _motorPairController->getPID_Name(static_cast<MotorPairController::pid_index_t>(ii));
        constexpr PIDF::PIDF_t pidFLT_MAX { FLT_MAX, FLT_MAX, FLT_MAX,FLT_MAX };
        _preferences->putPID(pidName, pidFLT_MAX);
    }
    _preferences->putFloat(_motorPairController->getBalanceAngleName(), FLT_MAX);
    Serial.printf("**** preferences reset\r\n");
}

/*!
Loads the PID settings for the MotorPairController. Must be called *after* the MPC is created.
*/
void MainTask::loadPreferences()
{
    assert(_motorPairController != nullptr); // loadPreferences must be called after the MPC is created

    // Set all the preferences to zero if they have not been set
    if (!_preferences->isSetPID()) {
        resetPreferences();
    }
    const float pitchBalanceAngleDegrees = _preferences->getFloat(_motorPairController->getBalanceAngleName());
    if (pitchBalanceAngleDegrees != FLT_MAX) {
        _motorPairController->setPitchBalanceAngleDegrees(pitchBalanceAngleDegrees);
        Serial.printf("**** pitch balance angle loaded from preferences:%f\r\n", pitchBalanceAngleDegrees);
    }

    // Load the PID constants from preferences, and if they are non-zero then use them to set the motorPairController PIDs.
    for (int ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        std::string pidName = _motorPairController->getPID_Name(static_cast<MotorPairController::pid_index_t>(ii));
        const PIDF::PIDF_t pid = _preferences->getPID(pidName);
        if (pid.kp != FLT_MAX) {
            _motorPairController->setPID_Constants(static_cast<MotorPairController::pid_index_t>(ii), pid);
            Serial.printf("**** %s PID loaded from preferences: P:%f, I:%f, D:%f, F:%f\r\n", pidName.c_str(), pid.kp, pid.ki, pid.kd, pid.kf);
        }
    }
}

void MainTask::setupTasks()
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static AHRS::TaskParameters ahrsTaskParameters { // NOLINT(misc-const-correctness) false positive
        .ahrs = _ahrs,
        .tickIntervalMilliSeconds = AHRS_TASK_TICK_INTERVAL_MILLISECONDS
    };
    enum { AHRS_TASK_STACK_DEPTH = 4096 };
    static StaticTask_t ahrsTaskBuffer;
    static StackType_t ahrsStack[AHRS_TASK_STACK_DEPTH];
    const TaskHandle_t ahrsTaskHandle = xTaskCreateStaticPinnedToCore(AHRS::Task, "AHRS_Task", AHRS_TASK_STACK_DEPTH, &ahrsTaskParameters, AHRS_TASK_PRIORITY, ahrsStack, &ahrsTaskBuffer, AHRS_TASK_CORE);
    assert(ahrsTaskHandle != nullptr && "Unable to create AHRS task.");
    Serial.printf("\r\n**** AHRS_Task, core:%d, priority:%d, tick interval:%dms\r\n", AHRS_TASK_CORE, AHRS_TASK_PRIORITY, AHRS_TASK_TICK_INTERVAL_MILLISECONDS);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static MotorPairController::TaskParameters mpcTaskParameters { // NOLINT(misc-const-correctness) false positive
        .motorPairController = _motorPairController,
        .tickIntervalMilliSeconds = MPC_TASK_TICK_INTERVAL_MILLISECONDS
    };
    enum { MPC_TASK_STACK_DEPTH = 4096 };
    static StaticTask_t mpcTaskBuffer;
    static StackType_t mpcStack[MPC_TASK_STACK_DEPTH];
    const TaskHandle_t mpcTaskHandle = xTaskCreateStaticPinnedToCore(MotorPairController::Task, "MPC_Task", MPC_TASK_STACK_DEPTH, &mpcTaskParameters, MPC_TASK_PRIORITY, mpcStack, &mpcTaskBuffer, MPC_TASK_CORE);
    assert(mpcTaskHandle != nullptr && "Unable to create MotorPairController task.");
    Serial.printf("**** MPC_Task,  core:%d, priority:%d, tick interval:%dms\r\n", MPC_TASK_CORE, MPC_TASK_PRIORITY, MPC_TASK_TICK_INTERVAL_MILLISECONDS);
}

/*!
The main loop handles:
1. Input from the receiver(joystick)
2. Failsafe when contact is lost with the transmitter.
3. Input from the backchannel(PID tuning).
4. Input from the buttons
5. Output to the backchannel(telemetry).
6. Output to the screen

The IMU(Inertial Measurement Unit) is read in the AHRS(Attitude and Heading Reference System) task.
The motors are controlled in the MotorPairController task.
*/
void MainTask::loop()
{
    // Delay task to yield to other tasks.
    // Most of the time this task does nothing, but when we get a packet from the receiver we want to process it immediately,
    // hence the short delay
    vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_TASK_TICK_INTERVAL_MILLISECONDS));

    const TickType_t tickCount = xTaskGetTickCount();
    // calculate _tickCountDelta for instrumentation
    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;

    const bool packetReceived = _receiver->update(_tickCountDelta);
    if (packetReceived) {
        _receiverInUse = true;
        _failSafeTickCount = tickCount;
    } else if ((tickCount - _failSafeTickCount > 1500) && _receiverInUse) {
        // _receiverInUse is initialized to false, so the motors won't turn off it the transmitter hasn't been turned on yet.
        // We've had 1500 ticks (1.5 seconds) without a packet, so we seem to have lost contact with the transmitter,
        // so switch off the motors to prevent the robot from doing a runaway.
        _motorPairController->motorsSwitchOff();
        _receiverInUse = false;
    }

#if defined(BACKCHANNEL_MAC_ADDRESS)
    (void)_backchannel->update();
#endif
}
