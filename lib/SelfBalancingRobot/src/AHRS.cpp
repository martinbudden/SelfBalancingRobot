#include "AHRS.h"
#include "IMU_Base.h"
#include "IMU_Filters.h"
#include <cmath>

// Either the USE_AHRS_DATA_MUTEX or USE_AHRS_DATA_CRITICAL_SECTION build flag can be set (but not both).
// The critical section variant seems to give better performance.
#if defined(USE_AHRS_DATA_MUTEX) && defined(USE_AHRS_DATA_CRITICAL_SECTION)
static_assert(false);
#endif



/*!
Main AHRS task function. Reads the IMU and uses the sensor fusion filter to update the orientation quaternion.
Returns false if there was no new data to be read from the IMU.
*/
bool AHRS::readIMUandUpdateOrientation(float deltaT)
{
    xyz_t gyroRadians; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    xyz_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

#if defined(USE_IMU_FIFO)
    // It uses the IMU FIFO. This ensures all IMU readings are processed, but it has the disadvantage that
    // reading the FIFO blocks the I2C bus, which in turn blocks the MPC_TASK.
    // I'm starting to come to the conclusion that, for M5Stack devices, better overall performance is obtained by not using the FIFO.

    constexpr float dT {1.0 / 500.0}; // use fixed deltaT corresponding to the update rate of the FIFO
    _fifoCount = _IMU.readFIFO_ToBuffer();
    if (_fifoCount == 0) {
        YIELD_TASK();
        return false;
    }

    xyz_t gyroRadiansSum {0.0, 0.0, 0.0};
    xyz_t accSum {0.0, 0.0, 0.0};
    const float fifoCountReciprocal = 1.0F / static_cast<float>(_fifoCount);
    const float fifoDeltaT = deltaT * fifoCountReciprocal;
    for (auto ii = 0; ii < _fifoCount; ++ii) {
        _IMU.readFIFO_Item(gyroRadians, acc, ii);
        _imuFilters->filter(gyroRadians, acc, fifoDeltaT);
        gyroRadiansSum += gyroRadians;
        accSum += acc;
    }
    gyroRadians = gyroRadiansSum * fifoCountReciprocal;
    acc = accSum * fifoCountReciprocal;
    const Quaternion orientation = _sensorFusionFilter.update(gyroRadians, acc, deltaT);
#else
    const bool dataRead = _IMU.readGyroRadiansAcc(gyroRadians, acc);
    if (dataRead == false) {
        YIELD_TASK();
        return false;
    }
    _imuFilters->filter(gyroRadians, acc, deltaT);
    const Quaternion orientation = _sensorFusionFilter.update(gyroRadians, acc, deltaT);
#endif

    if (sensorFusionFilterIsInitializing()) {
        checkMadgwickConvergence(acc, orientation);
    }

    LOCK();
    _ahrsDataUpdatedSinceLastRead = true;
    _orientationUpdatedSinceLastRead = true;
    _orientation = orientation;
    _gyroRadians = gyroRadians;
    _acc = acc;
    UNLOCK();
    return true;
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
void AHRS::Task(const TaskParameters* taskParameters)
{
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    _tickIntervalTicks = pdMS_TO_TICKS(taskParameters->tickIntervalMilliSeconds);
    _previousWakeTime = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next tickIntervalTicks
        vTaskDelayUntil(&_previousWakeTime, _tickIntervalTicks);

        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than _tickIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;

        if (_tickCountDelta > 0) { // guard against the case of the while loop executing twice on the same tick interval
            const float deltaT = pdTICKS_TO_MS(_tickCountDelta) * 0.001F;
            (void)readIMUandUpdateOrientation(deltaT);
        }
    }
#endif
}

/*!
Wrapper function for AHRS::Task with the correct signature to be used in xTaskCreate.
*/
void AHRS::Task(void* arg)
{
    const AHRS::TaskParameters* taskParameters = static_cast<AHRS::TaskParameters*>(arg);

    AHRS* ahrs = taskParameters->ahrs;
    ahrs->Task(taskParameters);
}

/*!
Read the raw gyro values. Used in calibration.
*/
xyz_int16_t AHRS::readGyroRaw() const
{
    return _IMU.readGyroRaw();
}

/*!
Read the raw accelerometer values. Used in calibration.
*/
xyz_int16_t AHRS::readAccRaw() const
{
    return _IMU.readAccRaw();
}

/*!
Set the gyro offset. Used in calibration.
*/
void AHRS::setGyroOffset(const xyz_int16_t& gyroOffset)
{
    _IMU.setGyroOffset(gyroOffset);
}

/*!
Set the accelerometer offset. Used in calibration.
*/
void AHRS::setAccOffset(const xyz_int16_t& accOffset)
{
    _IMU.setAccOffset(accOffset);
}

Quaternion AHRS::getOrientationUsingLock(bool& updatedSinceLastRead) const
{
    LOCK();
    updatedSinceLastRead = _orientationUpdatedSinceLastRead;
    _orientationUpdatedSinceLastRead = false;
    const Quaternion ret = _orientation;
    UNLOCK();

    return ret;
}

Quaternion AHRS::getOrientationForInstrumentationUsingLock() const
{
    LOCK();
    const Quaternion ret = _orientation;
    UNLOCK();

    return ret;
}

AHRS_Base::data_t AHRS::getAhrsDataUsingLock(bool& updatedSinceLastRead) const
{
    LOCK();
    updatedSinceLastRead = _ahrsDataUpdatedSinceLastRead;
    _ahrsDataUpdatedSinceLastRead = false;
    const AHRS_Base::data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRadians = _gyroRadians,
        .acc = _acc
    };
    UNLOCK();

    return ret;
}

AHRS_Base::data_t AHRS::getAhrsDataForInstrumentationUsingLock() const
{
    LOCK();
    const AHRS_Base::data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRadians = _gyroRadians,
        .acc = _acc
    };
    UNLOCK();

    return ret;
}

void AHRS::checkMadgwickConvergence(const xyz_t& acc, const Quaternion& orientation)
{
    constexpr float degreesToRadians {M_PI / 180.0};
    constexpr float twoDegreesInRadians = 2.0F * degreesToRadians;

    const float madgwickPitchAngleRadians = orientation.calculatePitchRadians();
    const float accPitchAngleRadians = std::atan2(acc.y, acc.z);
    if (fabs(accPitchAngleRadians - madgwickPitchAngleRadians) < twoDegreesInRadians && accPitchAngleRadians != madgwickPitchAngleRadians) {
        // the angles have converged to within 2 degrees, so set we can reduce the gain.
        setSensorFusionFilterInitializing(false);
        _sensorFusionFilter.setFreeParameters(0.1F, 0.0F);
    }
}

/*!
Constructor: set the sensor fusion filter and IMU to be used by the AHRS.
*/
AHRS::AHRS(SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, uint32_t tickIntervalMicroSeconds) :
    AHRS_Base(sensorFusionFilter),
    _IMU(imuSensor)
#if defined(USE_AHRS_DATA_MUTEX)
    , _imuDataMutex(xSemaphoreCreateRecursiveMutexStatic(&_imuDataMutexBuffer)) // statically allocate the imuDataMutex
#endif
{
    setSensorFusionFilterInitializing(true);
    // statically allocate the IMU_Filters
    constexpr float cutoffFrequency = 100.0F;
    static IMU_Filters imuFilters(cutoffFrequency, static_cast<float>(tickIntervalMicroSeconds) * 1.0e-6F);
    _imuFilters = &imuFilters;
#if defined(USE_AHRS_DATA_MUTEX)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
     // ensure _imuDataMutexBuffer declared before _imuDataMutex
    static_assert(offsetof(AHRS, _imuDataMutex) > offsetof(AHRS, _imuDataMutexBuffer));
#pragma GCC diagnostic pop
#endif
}
