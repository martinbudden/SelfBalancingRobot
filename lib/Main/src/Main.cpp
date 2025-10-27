#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#include "ButtonsM5.h"
#include "Main.h"
#include "ScreenM5.h"

#include <AHRS.h>
#include <AHRS_Task.h>

#include <BackchannelSBR.h>
#include <BackchannelTask.h>

#if defined(USE_BLACKBOX)
#include <BlackboxTask.h>
#endif

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <HardwareSerial.h>
#endif

#include <MotorPairController.h>
#include <NonVolatileStorage.h>
#include <RadioController.h>

#include <ReceiverTask.h>
#include <TimeMicroseconds.h>
#include <VehicleControllerTask.h>

#if defined(FRAMEWORK_ESPIDF)
#include <esp_timer.h>
#endif

/*!
Setup for the main loop, motor control task, and AHRS(Attitude and Heading Reference System) task.
*/
void Main::setup()
{
    // Initialize the M5Stack object
#if defined(M5_UNIFIED)
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();
#if defined(MOTORS_BALA_C)
    // with additional battery, we need to increase charge current
    M5.Power.setChargeCurrent(360);
#endif
#elif defined(M5_STACK)
    //M5Stack::begin(bool LCDEnable, bool SDEnable, bool SerialEnable, bool I2CEnable)
    M5.begin(true, false, false, false);
    M5.IMU.Init();
    M5.Power.begin();
#endif

#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
    esp_timer_init();
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    Serial.begin(115200);
#endif

    static NonVolatileStorage nvs;
    nvs.init();
    const uint8_t currentPID_Profile = nvs.loadPidProfileIndex();
    nvs.setCurrentPidProfileIndex(currentPID_Profile);

    // Create a mutex to ensure there is no conflict between objects using the I2C bus, namely the motors and the AHRS.
    // The mutex is created statically, ie without dynamic memory allocation.
    // If the motors and the AHRS are on separate busses (for example the motors were on a CAN bus, or the AHRS was on an SPI bus),
    // then the mutex is not required and may be set to nullptr.
#if defined(I2C_MUTEX_REQUIRED)
    static StaticSemaphore_t i2cMutexBuffer;
    SemaphoreHandle_t i2cMutex = xSemaphoreCreateMutexStatic(&i2cMutexBuffer);
#else
    void* i2cMutex = nullptr;
#endif

    IMU_Base& imuSensor = createIMU(i2cMutex);
    const uint32_t imuSampleRateHz = imuSensor.getGyroSampleRateHz();
    const uint32_t AHRS_taskIntervalMicroSeconds = static_cast<uint32_t>(1000000.0F / static_cast<float>(imuSampleRateHz));

    AHRS& ahrs = createAHRS(imuSensor);

    // Statically allocate the motorPairController.
    static MotorPairController motorPairController(AHRS_taskIntervalMicroSeconds, OUTPUT_TO_MOTORS_DENOMINATOR, ahrs, MotorPairController::allocateMotors(), i2cMutex);
    ahrs.setVehicleController(&motorPairController); //!!TODO: remove this after checking

    ReceiverBase& receiver = createReceiver();
    static RadioController radioController(receiver, motorPairController); // NOLINT(misc-const-correctness)

#if defined(USE_BLACKBOX)
    Blackbox& blackbox = createBlackBox(ahrs, motorPairController, radioController);
#endif

#if defined(M5_STACK) || defined(M5_UNIFIED)
    // Holding BtnA down while switching on enters calibration mode.
    if (M5.BtnA.isPressed()) {
        calibrateIMU(nvs, ahrs, CALIBRATE_ACC_AND_GYRO);
    }
    checkIMU_Calibration(nvs, ahrs);

    // Holding BtnC down while switching on clears the preferences.
    if (M5.BtnC.isPressed()) {
        clearSettings(nvs, motorPairController, ahrs);
    }
    loadSettings(nvs, motorPairController);

    // Statically allocate the screen.
    static ScreenM5 screen(ahrs, motorPairController, receiver);
    ReceiverWatcher* const receiverWatcher =  &screen;
    _screen = &screen;
    _screen->updateTemplate(); // Update the template as soon as we can, to minimize the time the screen is blank

    // Statically allocate the buttons.
    static ButtonsM5 buttons(motorPairController, receiver, _screen);
    _buttons = &buttons;
#if defined(M5_ATOM)
    // The Atom has no BtnB, so it always broadcasts address for binding on startup.
    receiver.broadcastMyEUI();
#else
    // Holding BtnB down while switching on initiates binding.
    if (M5.BtnB.wasPressed()) {
        receiver.broadcastMyEUI();
    }
#endif

#else

    checkIMU_Calibration(nvs, ahrs);
    loadSettings(nvs, motorPairController);
    // no buttons defined, so always broadcast address for binding on startup
    receiver.broadcastMyEUI();
    ReceiverWatcher* const receiverWatcher =  nullptr; // no screen available

#endif // M5_STACK || M5_UNIFIED

    // Create the tasks
    static DashboardTask dashboardTask(DASHBOARD_TASK_INTERVAL_MICROSECONDS); // NOLINT(misc-const-correctness) false positive
    _tasks.dashboardTask = &dashboardTask;
    reportDashboardTask();
    _tasks.ahrsTask = AHRS_Task::createTask(_tasks.ahrsTaskInfo, ahrs, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, AHRS_taskIntervalMicroSeconds);
    _tasks.vehicleControllerTask = VehicleControllerTask::createTask(_tasks.vehicleControllerTaskInfo, motorPairController, MPC_TASK_PRIORITY, MPC_TASK_CORE);
    _tasks.receiverTask = ReceiverTask::createTask(_tasks.receiverTaskInfo, radioController, receiverWatcher, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_CORE);
#if defined(USE_BLACKBOX)
    _tasks.blackboxTask = BlackboxTask::createTask(blackbox, BLACKBOX_TASK_PRIORITY, BLACKBOX_TASK_CORE, BLACKBOX_TASK_INTERVAL_MICROSECONDS);
#endif

#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
    BackchannelBase& backchannel = createBackchannel(motorPairController, ahrs, receiver, &dashboardTask, nvs);
    _tasks.backchannelTask = BackchannelTask::createTask(_tasks.backchannelTaskInfo, backchannel, BACKCHANNEL_TASK_PRIORITY, BACKCHANNEL_TASK_CORE, BACKCHANNEL_TASK_INTERVAL_MICROSECONDS);
#endif
}

void Main::reportDashboardTask()
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

#if defined(FRAMEWORK_USE_FREERTOS)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    assert(false && "stack overflow");
    Serial.printf("\r\n\r\n*********\r\n");
    Serial.printf("********Task '%s' stack overflow ********\r\n", pcTaskName);
    Serial.printf("*********\r\n\r\n");
}
#endif


void Main::checkIMU_Calibration(NonVolatileStorage& nonVolatileStorage, AHRS& ahrs)
{
    // Set the gyro offsets from non-volatile storage.
#if defined(USE_IMU_M5_UNIFIED)
    // M5_UNIFIED directly uses NVS (non-volatile storage) to store the gyro offsets.
    if (!M5.Imu.loadOffsetFromNVS()) {
        calibrateIMU(nonVolatileStorage, ahrs, CALIBRATE_GYRO_ONLY);
    }
#else
    // For M5_STACK and USE_IMU_MPU6886, the gyro offsets are stored in preferences.
    IMU_Base::xyz_int32_t offset {};
    if (nonVolatileStorage.loadGyroOffset(offset.x, offset.y, offset.z)) {
        ahrs.setGyroOffset(offset);
#if defined(FRAMEWORK_ARDUINO)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** AHRS gyroOffsets loaded from preferences: gx:%5d, gy:%5d, gz:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
        Serial.print(&buf[0]);
#endif
        if (nonVolatileStorage.loadAccOffset(offset.x, offset.y, offset.z)) {
            ahrs.setAccOffset(offset);
#if defined(FRAMEWORK_ARDUINO)
            sprintf(&buf[0], "**** AHRS accOffsets loaded from preferences:  ax:%5d, ay:%5d, az:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
            Serial.print(&buf[0]);
#endif
        }
    } else {
        // a calibrate by default only calibrates the gyro, since the SBR may be in an odd orientation when initially switched on
        calibrateIMU(nonVolatileStorage, ahrs, CALIBRATE_GYRO_ONLY);
    }
#endif
}

/*!
Clears the settings stored in nonvolatile storage.
*/
void Main::clearSettings(NonVolatileStorage& nonVolatileStorage, MotorPairController& motorPairController, AHRS& ahrs)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    nonVolatileStorage.clear();
#else
    for (size_t ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        nonVolatileStorage.resetPID(ii);
    }
    nonVolatileStorage.storeBalanceAngle(DEFAULTS::balanceAngle);
    nonVolatileStorage.storeAccOffset(0, 0, 0);
    nonVolatileStorage.storeGyroOffset(0, 0, 0);
#endif
    const IMU_Base::xyz_int32_t offset {0, 0, 0};
    ahrs.setAccOffset(offset);
    ahrs.setGyroOffset(offset);
    motorPairController.setPitchBalanceAngleDegrees(DEFAULTS::balanceAngle);
#if defined(FRAMEWORK_ARDUINO)
    Serial.print("**** NVS clear");
#endif
}

/*!
Loads the PID settings for the MotorPairController. Must be called *after* the MPC is created.
*/
void Main::loadSettings(NonVolatileStorage& nonVolatileStorage, MotorPairController& motorPairController)
{
    const float pitchBalanceAngleDegrees =nonVolatileStorage.loadBalanceAngle();
    motorPairController.setPitchBalanceAngleDegrees(pitchBalanceAngleDegrees);
#if defined(FRAMEWORK_ARDUINO)
    Serial.print("**** BALANCE ANGLE loaded from preferences: ");
    Serial.println(pitchBalanceAngleDegrees);
#endif

    // Load the PID constants from preferences, and if they are non-zero then use them to set the motorPairController PIDs.
    for (size_t ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        const VehicleControllerBase::PIDF_uint16_t pid = nonVolatileStorage.loadPID(ii);
        motorPairController.setPID_Constants(static_cast<MotorPairController::pid_index_e>(ii), pid);
#if defined(FRAMEWORK_ARDUINO)
        std::string pidName = motorPairController.getPID_Name(static_cast<MotorPairController::pid_index_e>(ii));
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** %15s PID loaded from preferences: P:%4d, I:%4d, D:%4d, S:%4d K:%4d\r\n", pidName.c_str(), static_cast<int>(pid.kp), static_cast<int>(pid.ki), static_cast<int>(pid.kd), static_cast<int>(pid.ks), static_cast<int>(pid.kk));
        Serial.print(&buf[0]);
#endif
    }
}

void DashboardTask::loop()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const TickType_t tickCount = xTaskGetTickCount();
    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;
#endif // USE_FREERTOS
}

/*!
The main loop handles:
1. Input from the buttons
2. Output to the screen

The IMU(Inertial Measurement Unit) is read in the AHRS(Attitude and Heading Reference System) task.
The motors are controlled in the MotorPairController task.
The receiver (joystick) values are obtained in the Receiver task.
*/
void Main::loop() // NOLINT(readability-make-member-function-const)
{
#if defined(FRAMEWORK_USE_FREERTOS)

    vTaskDelay(pdMS_TO_TICKS(DASHBOARD_TASK_INTERVAL_MICROSECONDS / 1000));
    [[maybe_unused]] const TickType_t tickCount = xTaskGetTickCount();
    //checkStackUsage();

#else
    [[maybe_unused]] const uint32_t tickCount = timeUs() / 1000;
    // simple round-robbin scheduling
    _tasks.dashboardTask->loop();
    _tasks.ahrsTask->loop();
    _tasks.vehicleControllerTask->loop();
    _tasks.receiverTask->loop();

#endif

#if defined(USE_SCREEN)
    // screen and button update tick counts are coprime, so screen and buttons are not normally updated in same loop
    // update the screen every 101 ticks (0.1 seconds)
    if (_screenTickCount - tickCount > 101) {
        _screenTickCount = tickCount;
        _screen->update();
    }
#endif
#if defined(USE_BUTTONS)
    // update the buttons every 149 ticks (0.15 seconds)
    if (_buttonsTickCount - tickCount > 149) {
        _buttonsTickCount = tickCount;
        _buttons->update();
    }
#endif
}

void Main::checkStackUsage()
{
#if defined(INCLUDE_uxTaskGetStackHighWaterMark) && false
    static uint32_t ahrsStackUsedMax = 0;
    const UBaseType_t ahrsStackUsed =  _tasks.ahrsTaskInfo.stackDepthBytes -  uxTaskGetStackHighWaterMark(_tasks.ahrsTaskInfo.taskHandle);
    if (ahrsStackUsed > ahrsStackUsedMax) {
        ahrsStackUsedMax = ahrsStackUsed;
        Serial.printf("AHRS,                stack used:%d\r\n", ahrsStackUsed);
    }

    static uint32_t mpcStackUsedMax = 0;
    const UBaseType_t mpcStackUsed =  _tasks.vehicleControllerTaskInfo.stackDepthBytes -  uxTaskGetStackHighWaterMark(_tasks.vehicleControllerTaskInfo.taskHandle);
    if (mpcStackUsed > mpcStackUsedMax) {
        mpcStackUsedMax = mpcStackUsed;
        Serial.printf("MotorPairController, stack used:%d\r\n", mpcStackUsed);
    }

    static uint32_t receiverStackUsedMax = 0;
    const UBaseType_t receiverStackUsed =  _tasks.receiverTaskInfo.stackDepthBytes -  uxTaskGetStackHighWaterMark(_tasks.receiverTaskInfo.taskHandle);
    if (receiverStackUsed > receiverStackUsedMax) {
        receiverStackUsedMax = receiverStackUsed;
        Serial.printf("Receiver             stack used:%d\r\n", receiverStackUsed);
    }
    static uint32_t backchannelStackUsedMax = 0;
    const UBaseType_t backchannelStackUsed =  _tasks.backchannelTaskInfo.stackDepthBytes -  uxTaskGetStackHighWaterMark(_tasks.backchannelTaskInfo.taskHandle);
    if (backchannelStackUsed > backchannelStackUsedMax) {
        backchannelStackUsedMax = backchannelStackUsed;
        Serial.printf("backchannel,     stack used:%d\r\n", backchannelStackUsed);
    }
#endif
}
