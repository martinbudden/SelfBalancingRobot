#include "AHRS.h"
#include "IMU_FiltersBase.h"
#include "MotorControllerBase.h"

#include <IMU_BNO085.h>
#include <SensorFusionFilter.h>
#include <cmath>
#if defined(AHRS_IS_INTERRUPT_DRIVEN) || defined(USE_FREERTOS)
#include <driver/gpio.h>
#include <esp32-hal-gpio.h>
#endif

// Either the USE_AHRS_DATA_MUTEX or USE_AHRS_DATA_CRITICAL_SECTION build flag can be set (but not both).
// The critical section variant seems to give better performance.
#if defined(USE_AHRS_DATA_MUTEX) && defined(USE_AHRS_DATA_CRITICAL_SECTION)
static_assert(false);
#endif


AHRS* AHRS::ahrs {nullptr};

#if defined(USE_FREERTOS)
inline void YIELD_TASK() { taskYIELD(); }
#else
inline void YIELD_TASK() {}
#endif


/*!
Main AHRS task function. Reads the IMU and uses the sensor fusion filter to update the orientation quaternion.
Returns false if there was no new data to be read from the IMU.

NOTE: calls to YIELD_TASK have no effect on multi-core implementations, but are useful for single-core variants.
*/
bool AHRS::readIMUandUpdateOrientation(float deltaT)
{
#if defined(AHRS_RECORD_TIMES_CHECKS)
    _timeCheck0 = micros();
#endif

#if defined(USE_IMU_FIFO)
    // It uses the IMU FIFO. This ensures all IMU readings are processed, but it has the disadvantage that
    // reading the FIFO blocks the I2C bus, which in turn blocks the MPC_TASK.
    // I'm starting to come to the conclusion that, for M5Stack devices, better overall performance is obtained by not using the FIFO.

    _fifoCount = _IMU.readFIFO_ToBuffer();
    if (_fifoCount == 0) {
        YIELD_TASK();
        return false;
    }
    TIME_CHECK(0, _timeCheck0);
    TIME_CHECK(1);

    IMU_Base::gyroRPS_Acc_t gyroAcc {};
    xyz_t gyroRPS_Sum {0.0, 0.0, 0.0};
    xyz_t accSum {0.0, 0.0, 0.0};
    const float fifoCountReciprocal = 1.0F / static_cast<float>(_fifoCount);
    const float fifoDeltaT = deltaT * fifoCountReciprocal;
    // constexpr float dT {1.0 / 500.0}; // use fixed deltaT corresponding to the update rate of the FIFO

    for (auto ii = 0; ii < _fifoCount; ++ii) {
        gyroAcc = _IMU.readFIFO_Item(ii);
        _imuFilters.filter(gyroAcc.gyroRPS, gyroAcc.acc, fifoDeltaT);
        gyroRPS_Sum += gyroAcc.gyroRPS;
        accSum += gyroAcc.acc;
    }
    gyroAcc.gyroRPS = gyroRPS_Sum * fifoCountReciprocal;
    gyroAcc.acc = accSum * fifoCountReciprocal;
    TIME_CHECK(1);
    TIME_CHECK(2);

    const Quaternion orientation = _sensorFusionFilter.update(gyroAcc.gyroRPS, gyroAcc.acc, deltaT);
    TIME_CHECK(3);
    if (sensorFusionFilterIsInitializing()) {
        checkMadgwickConvergence(gyroAcc.acc, orientation);
    }

#else

#if defined(USE_IMU_BNO085)
    // BNO085 does the sensor fusion
    const xyz_t gyro = _IMU.readGyroRPS();
    const IMU_Base::gyroRPS_Acc_t gyroAcc = {
        .gyroRPS = gyro,
        .acc = {}
    };
    const Quaternion orientation = _IMU.readOrientation();
#else
    IMU_Base::gyroRPS_Acc_t gyroAcc = _IMU.readGyroRPS_Acc(); // NOLINT(misc-const-correctness) false positive
    TIME_CHECK(0, _timeCheck0);
    TIME_CHECK(1);

    _imuFilters.filter(gyroAcc.gyroRPS, gyroAcc.acc, deltaT); // 15us, 207us
    TIME_CHECK(2);

    const Quaternion orientation = _sensorFusionFilter.update(gyroAcc.gyroRPS, gyroAcc.acc, deltaT); // 15us, 140us
    TIME_CHECK(3);
    if (sensorFusionFilterIsInitializing()) {
        checkMadgwickConvergence(gyroAcc.acc, orientation);
    }
#endif

#endif // USE_IMU_FIFO

    // If _motorController is not nullptr, then things have been configured so that updateOutputsUsingPIDs
    // is called by the AHRS rather than the motor controller.
    if (_motorController != nullptr) {
        _motorController->updateOutputsUsingPIDs(gyroAcc.gyroRPS, gyroAcc.acc, orientation, deltaT); //25us, 900us
        TIME_CHECK(4);
    }

    LOCK_AHRS_DATA();
    _ahrsDataUpdatedSinceLastRead = true;
    _orientationUpdatedSinceLastRead = true;
    _orientation = orientation;
    _gyroRPS = gyroAcc.gyroRPS;
    _acc = gyroAcc.acc;
    UNLOCK_AHRS_DATA();

    return true;
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
[[noreturn]] void AHRS::Task([[maybe_unused]] const TaskParameters* taskParameters)
{
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    _tickIntervalTicks = pdMS_TO_TICKS(taskParameters->tickIntervalMilliSeconds);
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
        LOCK_IMU_DATA_READY(); // wait until the ISR unlocks data ready
        _imuDataReadyCount = 0;

        const uint32_t timeMicroSeconds = micros();
        const float deltaT = static_cast<float>(timeMicroSeconds - _timeMicroSecondsPrevious) * 0.000001F;
        _timeMicroSecondsPrevious = timeMicroSeconds;
        if (deltaT > 0.0F) {
            (void)readIMUandUpdateOrientation(deltaT);
        }
#else
        // delay until the end of the next tickIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, _tickIntervalTicks);
        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than _tickIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const uint32_t timeMicroSeconds = micros();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;

        if (_tickCountDelta > 0) { // guard against the case of the while loop executing twice on the same tick interval
            const float deltaT = pdTICKS_TO_MS(_tickCountDelta) * 0.001F;
            (void)readIMUandUpdateOrientation(deltaT);
        }
#endif // AHRS_IS_INTERRUPT_DRIVEN
    }
#else
    while (true) {}
#endif // USE_FREERTOS
}

/*!
Wrapper function for AHRS::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void AHRS::Task(void* arg)
{
    const AHRS::TaskParameters* taskParameters = static_cast<AHRS::TaskParameters*>(arg);

    taskParameters->ahrs->Task(taskParameters);
}

/*!
Read the raw gyro values. Used in calibration.
*/
void AHRS::readGyroRaw(int32_t& x, int32_t& y, int32_t& z) const
{
    const IMU_Base::xyz_int32_t gyro = _IMU.readGyroRaw();
    x = gyro.x;
    y = gyro.y;
    z = gyro.z;
}

/*!
Read the raw accelerometer values. Used in calibration.
*/
void AHRS::readAccRaw(int32_t& x, int32_t& y, int32_t& z) const
{
    const IMU_Base::xyz_int32_t acc = _IMU.readAccRaw();
    x = acc.x;
    y = acc.y;
    z = acc.z;
}

int32_t AHRS::getAccOneG_Raw() const
{
    return _IMU.getAccOneG_Raw();
}


IMU_Base::xyz_int32_t AHRS::getGyroOffset() const
{
    return _IMU.getGyroOffset();
}

/*!
Set the gyro offset. Used in calibration.
*/
void AHRS::setGyroOffset(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setGyroOffset(offset);
}

IMU_Base::xyz_int32_t AHRS::getAccOffset() const
{
    return _IMU.getAccOffset();
}

/*!
Set the accelerometer offset. Used in calibration.
*/
void AHRS::setAccOffset(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setAccOffset(offset);
}

Quaternion AHRS::getOrientationUsingLock(bool& updatedSinceLastRead) const
{
    LOCK_AHRS_DATA();
    updatedSinceLastRead = _orientationUpdatedSinceLastRead;
    _orientationUpdatedSinceLastRead = false;
    const Quaternion ret = _orientation;
    UNLOCK_AHRS_DATA();

    return ret;
}

/*!
Returns orientation without clearing _orientationUpdatedSinceLastRead.
*/
Quaternion AHRS::getOrientationForInstrumentationUsingLock() const
{
    LOCK_AHRS_DATA();
    const Quaternion ret = _orientation;
    UNLOCK_AHRS_DATA();

    return ret;
}

AHRS::data_t AHRS::getAhrsDataUsingLock(bool& updatedSinceLastRead) const
{
    LOCK_AHRS_DATA();
    updatedSinceLastRead = _ahrsDataUpdatedSinceLastRead;
    _ahrsDataUpdatedSinceLastRead = false;
    const data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRPS = _gyroRPS,
        .acc = _acc
    };
    UNLOCK_AHRS_DATA();

    return ret;
}

AHRS::data_t AHRS::getAhrsDataForInstrumentationUsingLock() const
{
    LOCK_AHRS_DATA();
    const data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRPS = _gyroRPS,
        .acc = _acc
    };
    UNLOCK_AHRS_DATA();

    return ret;
}

void AHRS::checkMadgwickConvergence(const xyz_t& acc, const Quaternion& orientation)
{
    constexpr float twoDegreesInRadians = 2.0F * Quaternion::degreesToRadians;

    // NOTE COORDINATE TRANSFORM: Madgwick filter uses Euler angles where roll is defined as rotation around the x-axis and pitch is rotation around the y-axis.
    // For the Self Balancing Robot, pitch is rotation around the x-axis and roll is rotation around the y-axis,
    // so SBR.roll = Madgwick.pitch and SPR.pitch = Madgwick.roll
    const float madgwickRollAngleRadians = orientation.calculateRollRadians();
    const float accPitchAngleRadians = std::atan2(acc.y, acc.z);
    //const float accRollAngleRadians = std::atan2(-acc.x, sqrtf(acc.y*acc.y + acc.z*acc.z));

    //Serial.printf("acc:P%5.1f mag:P%5.1f         diff:%5.1f\r\n", accPitchAngleRadians/Quaternion::degreesToRadians, madgwickRollAngleRadians/Quaternion::degreesToRadians, fabsf(accPitchAngleRadians - madgwickRollAngleRadians)/Quaternion::degreesToRadians);
    if (fabsf(accPitchAngleRadians - madgwickRollAngleRadians) < twoDegreesInRadians && accPitchAngleRadians != madgwickRollAngleRadians) {
        // the angles have converged to within 2 degrees, so set we can reduce the gain.
        setSensorFusionFilterInitializing(false);
        _sensorFusionFilter.setFreeParameters(0.615F, 0.0F); // corresponds to gyro measurement error of 15*2.7 degree/second, as discussed my Madgwick
    }
}

#if defined(AHRS_IS_INTERRUPT_DRIVEN)
IRAM_ATTR void AHRS::imuDataReadyInterruptServiceRoutine()
{
    //AHRS* ahrs = reinterpret_cast<AHRS*>(arg);
    ++ahrs->_imuDataReadyCount;
    ahrs->UNLOCK_IMU_DATA_READY();
}
#endif

/*!
Constructor: sets the sensor fusion filter, IMU, and IMU filters
*/
AHRS::AHRS(SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters) :
    _sensorFusionFilter(sensorFusionFilter),
    _IMU(imuSensor),
    _imuFilters(imuFilters)
#if defined(USE_IMU_DATA_READY_MUTEX)
    ,_imuDataReadyMutex(xSemaphoreCreateRecursiveMutexStatic(&_imuDataReadyMutexBuffer)) // statically allocate the imuDataMutex
#endif
#if defined(USE_AHRS_DATA_MUTEX)
    , _ahrsDataMutex(xSemaphoreCreateRecursiveMutexStatic(&_ahrsDataMutexBuffer)) // statically allocate the imuDataMutex
#endif

{
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    ahrs = this;
    attachInterrupt(IMU_INTERRUPT_PIN, imuDataReadyInterruptServiceRoutine, FALLING);
#endif
    setSensorFusionFilterInitializing(true);

#if defined(USE_FREERTOS)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#if defined(USE_IMU_DATA_READY_MUTEX)
    // ensure _imuDataReadyMutexBuffer declared before _imuDataReadyMutex
    static_assert(offsetof(AHRS, _imuDataReadyMutex) > offsetof(AHRS, _imuDataReadyMutexBuffer));
#endif
#if defined(USE_AHRS_DATA_MUTEX)
    // ensure _ahrsDataMutexBuffer declared before _ahrsDataMutex
    static_assert(offsetof(AHRS, _ahrsDataMutex) > offsetof(AHRS, _ahrsDataMutexBuffer));
#endif
#pragma GCC diagnostic pop

#elif defined(USE_PICO_BARE_METAL)

#if defined(USE_IMU_DATA_READY_MUTEX)
    mutex_init(&_imuDataReadyMutex);
#endif
#if defined(USE_AHRS_DATA_MUTEX)
    mutex_init(&_ahrsDataMutex);
#elif defined(USE_AHRS_DATA_CRITICAL_SECTION)
    critical_section_init(_ahrsDataCriticalSection);
#endif

#endif
}
