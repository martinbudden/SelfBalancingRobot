#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#endif

#if defined(USE_ESPNOW)
#include <WiFi.h>
#endif

#include "ButtonsM5.h"
#include "Calibration.h"
#include "MainTask.h"
#include "ScreenM5.h"
#include "TelemetryScaleFactors.h"

#include <AHRS.h>
#include <AHRS_Task.h>

#if defined(USE_ESPNOW)
#include <ESPNOW_Backchannel.h>
#include <HardwareSerial.h>
#endif

#include <IMU_BMI270.h>
#include <IMU_BNO085.h>
#include <IMU_FiltersDefault.h>
#include <IMU_LSM6DS3TR_C.h>
#include <IMU_M5Stack.h>
#include <IMU_M5Unified.h>
#include <IMU_MPU6886.h>
#include <MotorPairControllerTask.h>
#if defined(USE_ESPNOW)
#include <ReceiverAtomJoyStick.h>
#endif
#include <ReceiverNull.h>
#include <ReceiverTask.h>
#include <SV_Preferences.h>
#include <SensorFusion.h>
#include <TimeMicroSeconds.h>

#if defined(FRAMEWORK_ESPIDF)
#include <esp_timer.h>
#endif

/*!
The ESP32S3 is dual core containing a Protocol CPU (known as CPU 0 or PRO_CPU) and an Application CPU (known as CPU 1 or APP_CPU).

The core affinities, priorities, and tick intervals and priorities for the 3 application tasks (AHRS_TASK, MPC_TASK, and MAIN_LOOP_TASK).
1. The AHRS_TASK must have a higher priority than the MAIN_LOOP_TASK.
2. The MPC_TASK must have a higher priority than the MAIN_LOOP_TASK
3. For single-processors the AHRS_TASK and the MPC_TASK must have the same priority.
4. For dual-core processors
    1. The AHRS_TASK runs on the Application CPU (CPU 1).
    2. The MPC_TASK runs on the Protocol CPU (CPU 0).
5. The MAIN_LOOP_TASK runs on the Application CPU (CPU 1) with priority 1 (this is set by the ESP32 Arduino framework).

The AHRS_TASK and the MPC_TASK are deliberately chosen to run on different cores on ESP32 dual-core processors. This is so
that a context switch between the AHRS_TASK and the MPC_TASK does not require saving the FPU(Floating Point Unit) registers
(see https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32/api-guides/freertos-smp.html#floating-point-usage).

The Atom JoyStick transmits a packet every 10ms, so the MAIN_LOOP_TASK must run at least every 10m to ensure no dropped packets.
Updating the screen takes approximately 50 ticks, so packets will be dropped if the screen is not in QRCODE mode.
*/


#if !defined(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS)
enum { MAIN_LOOP_TASK_INTERVAL_MICROSECONDS = 10000 };
#endif

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif

#if !defined(MPC_TASK_INTERVAL_MICROSECONDS)
#if defined(USE_IMU_M5_UNIFIED) || defined(USE_IMU_M5_STACK)
    enum { MPC_TASK_INTERVAL_MICROSECONDS = 10000 }; // M5Stack IMU code blocks I2C bus for extended periods, so MPC_TASK must be set to run slower.
#else
    enum { MPC_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif
#endif

#if !defined(RECEIVER_TASK_INTERVAL_MICROSECONDS)
enum { RECEIVER_TASK_INTERVAL_MICROSECONDS = 5000 };
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

    _ahrs = &setupAHRS(i2cMutex);

#if defined(USE_ESPNOW)
    // Set WiFi to station mode
    WiFi.mode(WIFI_STA);
    // Disconnect from Access Point if it was previously connected
    WiFi.disconnect();
    // get my MAC address
    uint8_t myMacAddress[ESP_NOW_ETH_ALEN];
    WiFi.macAddress(&myMacAddress[0]);

    // Statically allocate and setup the receiver.
    static ReceiverAtomJoyStick receiver(&myMacAddress[0]);
    _receiver = &receiver;
#if !defined(JOYSTICK_CHANNEL)
    constexpr uint8_t JOYSTICK_CHANNEL {3};
#endif
    const esp_err_t espErr = receiver.setup(JOYSTICK_CHANNEL);
    //delay(400); // delay to allow serial port to initialize before first print
    Serial.print("\r\n\r\n**** ESP-NOW Ready:");
    Serial.println(espErr);
    Serial.println();
    assert(espErr == ESP_OK && "Unable to setup receiver.");
#else
    static ReceiverNull receiver;
    _receiver = &receiver;
#endif // USE_ESPNOW

    // Statically allocate the motorPairController.
    static MotorPairController motorPairControllerStatic(*_ahrs, receiver, i2cMutex);
    _motorPairController = &motorPairControllerStatic;
    _ahrs->setVehicleController(_motorPairController);

    static SV_Preferences preferences;

#if defined(M5_STACK) || defined(M5_UNIFIED)
    // Holding BtnA down while switching on enters calibration mode.
    if (M5.BtnA.isPressed()) {
        calibrateGyro(*_ahrs, preferences, CALIBRATE_ACC_AND_GYRO);
    }
    checkGyroCalibration(preferences, *_ahrs);
    // Holding BtnC down while switching on resets the preferences.
    if (M5.BtnC.isPressed()) {
        resetPreferences(preferences, *_motorPairController);
    }
#else
    checkGyroCalibration(preferences, *_ahrs);
#endif
    loadPreferences(preferences, *_motorPairController);

#if defined(M5_STACK) || defined(M5_UNIFIED)
    // Statically allocate the screen.
    static ScreenM5 screen(*_ahrs, *_motorPairController, receiver);
    ReceiverWatcher* receiverWatcher =  &screen;
    _screen = &screen;
    _screen->updateFull(); // Update the as soon as we can, to minimize the time the screen is blank

    // Statically allocate the buttons.
    static ButtonsM5 buttons(*_motorPairController, receiver, _screen);
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

    // no buttons defined, so always broadcast address for binding on startup
    receiver.broadcastMyEUI();
    ReceiverWatcher* receiverWatcher =  nullptr; // no screen available
#endif // M5_STACK || M5_UNIFIED

    // And finally set up the AHRS and MotorPairController and Receiver tasks.
    _tasks = setupTasks(*_ahrs, *_motorPairController, receiver, receiverWatcher);

#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(USE_ESPNOW)
    // Statically allocate the telemetry scale factors
    static TelemetryScaleFactors telemetryScaleFactors(_motorPairController->getControlMode());
    // Statically allocate the backchannel.
    constexpr uint8_t backchannelMacAddress[ESP_NOW_ETH_ALEN] BACKCHANNEL_MAC_ADDRESS;
    static Backchannel backchannel(
        receiver.getESPNOW_Transceiver(),
        backchannelMacAddress,
        *_tasks.mpcTask,
        *_tasks.ahrsTask,
        *_tasks.mainTask,
        *_receiver,
        telemetryScaleFactors,
        preferences
    );
    _backchannel = &backchannel;
#endif
}

AHRS& Main::setupAHRS(void* i2cMutex)
{
    // Statically allocate the IMU according the the build flags
// NOLINTBEGIN(misc-const-correctness)
    [[maybe_unused]] static const uint32_t spiFrequency = 20000000;
#if defined(USE_IMU_MPU6886_I2C)
#if defined(M5_STACK)
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, pins);
#else
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, BUS_I2C::pins_t{.sda=static_cast<uint8_t>(M5.In_I2C.getSDA()), .scl=static_cast<uint8_t>(M5.In_I2C.getSCL()), .irq=BUS_I2C::IRQ_NOT_SET, .irqLevel=0});
#endif
#elif defined(USE_IMU_MPU6886_SPI)
    const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_BMI270_I2C)
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_BMI270_SPI)
    const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_BNO085_I2C)
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_BNO085 imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_BNO085_SPI)
    static IMU_BNO085 imuSensor(IMU_AXIS_ORDER, spiFrequency, IMU_SPI_CS_PIN);
#elif defined(USE_IMU_LSM6DS3TR_C_I2C) || defined(USE_IMU_ISM330DHCX_I2C) || defined(USE_LSM6DSOX_I2C)
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_LSM6DS3TR_C imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_LSM6DS3TR_C_SPI) || defined(USE_IMU_ISM330DHCX_SPI) || defined(USE_LSM6DSOX_SPI)
    const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    static IMU_LSM6DS3TR_C imuSensor(IMU_AXIS_ORDER, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_M5_STACK)
    static IMU_M5_STACK imuSensor(IMU_AXIS_ORDER);
#elif defined(USE_IMU_M5_UNIFIED)
    static IMU_M5_UNIFIED imuSensor(IMU_AXIS_ORDER);
#else
    static_assert(false);
#endif

    //static_cast<IMU_Base&>(imuSensor).init(1000000 / AHRS_TASK_INTERVAL_MICROSECONDS, i2cMutex);
    static_cast<IMU_Base&>(imuSensor).init(i2cMutex);

    // Statically allocate the Sensor Fusion Filter
    // Timings are for 240MHz ESP32-S3
#if defined(USE_COMPLEMENTARY_FILTER)
    // approx 130 microseconds per update
    static ComplementaryFilter sensorFusionFilter;
#elif defined(USE_MAHONY_FILTER)
    // approx 10 microseconds per update
    static MahonyFilter sensorFusionFilter;
#elif defined(USE_VQF)
    const float deltaT = static_cast<float>(AHRS_TASK_INTERVAL_MICROSECONDS) / 1000000.0F;
    static VQF sensorFusionFilter(deltaT, deltaT, deltaT, true, false, false);
#elif defined(USE_VQF_BASIC)
    static BasicVQF sensorFusionFilter(static_cast<float>(AHRS_TASK_INTERVAL_MICROSECONDS) / 1000000.0F);
#else
    // approx 16 microseconds per update
    static MadgwickFilter sensorFusionFilter;
#endif
    // statically allocate the IMU_Filters
    constexpr float cutoffFrequency = 100.0F;
    static IMU_FiltersDefault imuFilters(cutoffFrequency, static_cast<float>(AHRS_TASK_INTERVAL_MICROSECONDS) / 1000000.0F);
    //static IMU_FiltersDefault imuFilters;
// NOLINTEND(misc-const-correctness)

    // Statically allocate the AHRS object
    static AHRS ahrs(AHRS_TASK_INTERVAL_MICROSECONDS, sensorFusionFilter, imuSensor, imuFilters);
    return ahrs;
}

void Main::checkGyroCalibration(SV_Preferences& preferences, AHRS& ahrs)
{
    // Set the gyro offsets from non-volatile storage.
#if defined(USE_IMU_M5_UNIFIED)
    // M5_UNIFIED directly uses NVS (non-volatile storage) to store the gyro offsets.
    if (!M5.Imu.loadOffsetFromNVS()) {
        calibrateGyro(ahrs, preferences, CALIBRATE_JUST_GYRO);
    }
#else
    // For M5_STACK and USE_IMU_MPU6886, the gyro offsets are stored in preferences.
    IMU_Base::xyz_int32_t offset {};
    if (preferences.getGyroOffset(offset.x, offset.y, offset.z)) {
        ahrs.setGyroOffset(offset);
#if defined(FRAMEWORK_ARDUINO)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** AHRS gyroOffsets loaded from preferences: gx:%5d, gy:%5d, gz:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
        Serial.print(&buf[0]);
#endif
        if (preferences.getAccOffset(offset.x, offset.y, offset.z)) {
            ahrs.setAccOffset(offset);
#if defined(FRAMEWORK_ARDUINO)
            sprintf(&buf[0], "**** AHRS accOffsets loaded from preferences:  ax:%5d, ay:%5d, az:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
            Serial.print(&buf[0]);
#endif
        }
    } else {
        // when calibrateGyro called automatically on startup, just calibrate the gyroscope.
        calibrateGyro(ahrs, preferences, CALIBRATE_JUST_GYRO);
    }
#endif
}

/*!
Resets the PID preferences and the Balance Angle to SV_Preferences::NOT_SET (which represents unset).
*/
void Main::resetPreferences(SV_Preferences& preferences, MotorPairController& motorPairController)
{
    //_preferences->clear();
    //_preferences->removeGyroOffset();
    //_preferences->removeAccOffset();
    for (int ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        const std::string pidName = motorPairController.getPID_Name(static_cast<MotorPairController::pid_index_e>(ii));
        constexpr PIDF::PIDF_t pidNOT_SET { SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET };
        preferences.putPID(pidName, pidNOT_SET);
    }
    preferences.putFloat(motorPairController.getBalanceAngleName(), SV_Preferences::NOT_SET);
#if defined(FRAMEWORK_ARDUINO)
    Serial.print("**** preferences reset");
#endif
}

/*!
Loads the PID settings for the MotorPairController. Must be called *after* the MPC is created.
*/
void Main::loadPreferences(SV_Preferences& preferences, MotorPairController& motorPairController)
{
    // Set all the preferences to zero if they have not been set
    if (!preferences.isSetPID()) {
        resetPreferences(preferences, motorPairController);
    }
    const float pitchBalanceAngleDegrees = preferences.getFloat(motorPairController.getBalanceAngleName());
    if (pitchBalanceAngleDegrees != SV_Preferences::NOT_SET) {
        motorPairController.setPitchBalanceAngleDegrees(pitchBalanceAngleDegrees);
#if defined(FRAMEWORK_ARDUINO)
        Serial.print("**** pitch balance angle loaded from preferences:");
        Serial.print(pitchBalanceAngleDegrees);
#endif
    }

    // Load the PID constants from preferences, and if they are non-zero then use them to set the motorPairController PIDs.
    for (int ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        std::string pidName = motorPairController.getPID_Name(static_cast<MotorPairController::pid_index_e>(ii));
        const PIDF::PIDF_t pid = preferences.getPID(pidName);
        if (pid.kp != SV_Preferences::NOT_SET) {
            motorPairController.setPID_Constants(static_cast<MotorPairController::pid_index_e>(ii), pid);
#if defined(FRAMEWORK_ARDUINO)
            std::array<char, 128> buf;
            sprintf(&buf[0], "**** %s PID loaded from preferences: P:%f, I:%f, D:%f, F:%f\r\n", pidName.c_str(), static_cast<double>(pid.kp), static_cast<double>(pid.ki), static_cast<double>(pid.kd), static_cast<double>(pid.kf));
            Serial.print(&buf[0]);
#endif
        }
    }
}

Main::tasks_t Main::setupTasks(AHRS& ahrs, MotorPairController& motorPairController, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static MainTask mainTask(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS);
    static AHRS_Task ahrsTask(AHRS_TASK_INTERVAL_MICROSECONDS, ahrs);
    static MotorPairControllerTask mpcTask(MPC_TASK_INTERVAL_MICROSECONDS, motorPairController);
    static ReceiverTask receiverTask(RECEIVER_TASK_INTERVAL_MICROSECONDS, receiver, receiverWatcher);

    const tasks_t ret {
        .mainTask = &mainTask,
        .ahrsTask = &ahrsTask,
        .mpcTask = &mpcTask,
        .receiverTask = &receiverTask
    };

#if defined(USE_FREERTOS)
    enum { AHRS_TASK_PRIORITY = 5, MPC_TASK_PRIORITY = 4, RECEIVER_TASK_PRIORITY = 3, MSP_TASK_PRIORITY = 2 };

enum { MPC_TASK_CORE = PRO_CPU_NUM };
enum { RECEIVER_TASK_CORE = PRO_CPU_NUM };
#if defined(APP_CPU_NUM) // The processor has two cores
    enum { AHRS_TASK_CORE = APP_CPU_NUM }; // AHRS should be the only task running on the second core
#else // single core processor
    enum { AHRS_TASK_CORE = PRO_CPU_NUM };
#endif

#if !defined(FRAMEWORK_ESPIDF)
    std::array<char, 128> buf;
#endif

#if defined(USE_ARDUINO_ESP32)
    // The main task is set up by the framework, so just print its details.
    // It has name "loopTask" and priority 1.
    const TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const UBaseType_t taskPriority = uxTaskPriorityGet(taskHandle);
    const char* taskName = pcTaskGetName(taskHandle);
    sprintf(&buf[0], "\r\n\r\n**** Main loop task, name:'%s' priority:%d, tickRate:%dHz\r\n", taskName, taskPriority, configTICK_RATE_HZ);
    Serial.print(&buf[0]);
#endif

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t ahrsTaskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &ahrsTask,
    };
    enum { AHRS_TASK_STACK_DEPTH = 4096 };
    static StaticTask_t ahrsTaskBuffer;
    static std::array <StackType_t, AHRS_TASK_STACK_DEPTH> ahrsStack;
    const TaskHandle_t ahrsTaskHandle = xTaskCreateStaticPinnedToCore(
        AHRS_Task::Task,
        "AHRS_Task",
        AHRS_TASK_STACK_DEPTH,
        &ahrsTaskParameters,
        AHRS_TASK_PRIORITY,
        &ahrsStack[0],
        &ahrsTaskBuffer,
        AHRS_TASK_CORE
    );
    assert(ahrsTaskHandle != nullptr && "Unable to create AHRS task.");
#if !defined(FRAMEWORK_ESPIDF)
    sprintf(&buf[0], "**** AHRS_Task,      core:%d, priority:%d, task interval:%dms\r\n", AHRS_TASK_CORE, AHRS_TASK_PRIORITY, AHRS_TASK_INTERVAL_MICROSECONDS / 1000);
    Serial.print(&buf[0]);
#endif

    static TaskBase::parameters_t mpcTaskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &mpcTask,
    };
    enum { MPC_TASK_STACK_DEPTH = 4096 };
    static StaticTask_t mpcTaskBuffer;
    static std::array<StackType_t, MPC_TASK_STACK_DEPTH> mpcStack;
    const TaskHandle_t mpcTaskHandle = xTaskCreateStaticPinnedToCore(MotorPairControllerTask::Task,
        "MPC_Task",
        MPC_TASK_STACK_DEPTH,
        &mpcTaskParameters,
        MPC_TASK_PRIORITY,
        &mpcStack[0],
        &mpcTaskBuffer,
        MPC_TASK_CORE
    );
    assert(mpcTaskHandle != nullptr && "Unable to create MotorPairController task.");
#if !defined(FRAMEWORK_ESPIDF)
    sprintf(&buf[0], "**** MPC_Task,       core:%d, priority:%d, task interval:%dms\r\n", MPC_TASK_CORE, MPC_TASK_PRIORITY, MPC_TASK_INTERVAL_MICROSECONDS / 1000);
    Serial.print(&buf[0]);
#endif

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t receiverTaskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &receiverTask,
    };
    enum { RECEIVER_TASK_STACK_DEPTH = 4096 };
    static std::array<StackType_t, RECEIVER_TASK_STACK_DEPTH> receiverStack;
    static StaticTask_t receiverTaskBuffer;
    const TaskHandle_t receiverTaskHandle = xTaskCreateStaticPinnedToCore(
        ReceiverTask::Task,
        "Receiver_Task",
        RECEIVER_TASK_STACK_DEPTH,
        &receiverTaskParameters,
        RECEIVER_TASK_PRIORITY,
        &receiverStack[0],
        &receiverTaskBuffer,
        RECEIVER_TASK_CORE
    );
    assert(receiverTaskHandle != nullptr && "Unable to create ReceiverTask task.");
    sprintf(&buf[0], "**** RECEIVER_Task,  core:%d, priority:%d, task interval:%dms\r\n\r\n", RECEIVER_TASK_CORE, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_INTERVAL_MICROSECONDS / 1000);
    Serial.print(&buf[0]);

#endif // USE_FREERTOS
    return ret;
}

/*!
The main loop handles:
1. Input from the receiver(joystick)
2. Input from the backchannel(PID tuning).
3. Input from the buttons
4. Output to the backchannel(telemetry).
5. Output to the screen

The IMU(Inertial Measurement Unit) is read in the AHRS(Attitude and Heading Reference System) task.
The motors are controlled in the MotorPairController task.
*/
void MainTask::loop()
{
#if defined(USE_FREERTOS)
    const TickType_t tickCount = xTaskGetTickCount();
    _tickCountDelta = tickCount - _tickCountPrevious;
#else
    const uint32_t tickCount = timeUs() / 1000;
    _tickCountDelta = tickCount - _tickCountPrevious;
    if (_tickCountDelta < _taskIntervalMicroSeconds / 1000) {
        return;
    }
#endif // USE_FREERTOS
    _tickCountPrevious = tickCount;
}

void Main::loop()
{
#if !defined(USE_FREERTOS)
    // simple round-robbin scheduling
    _tasks.mainTask->loop();
    _tasks.ahrsTask->loop();
    _tasks.mpcTask->loop();
    _tasks.receiverTask->loop();
#endif
#if defined(BACKCHANNEL_MAC_ADDRESS)
    _backchannel->update();
#endif
#if defined(USE_SCREEN) || defined(USE_BUTTONS)
    const uint32_t tickCount = _tasks.mainTask->getTickCountPrevious();
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

#if defined(USE_FREERTOS)
    // Delay task to yield to other tasks.
    // Most of the time this task does nothing, but when we get a packet from the receiver we want to process it immediately,
    // hence the short delay
    vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS / 1000));
#endif
}
