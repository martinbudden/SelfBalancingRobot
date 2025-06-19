#pragma once

#include <IMU_Base.h>
#include <IMU_FiltersDefault.h>

#include <cassert>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#endif
#if defined(FRAMEWORK_RPI_PICO)
#include <pico/critical_section.h>
#include <pico/mutex.h>
#endif


class VehicleControllerBase;
class SensorFusionFilterBase;
class TaskBase;

/*!
The AHRS uses the ENU (East North Up) coordinate frame.
*/
class AHRS {
public:
    struct data_t {
        uint32_t tickCountDelta;
        xyz_t gyroRPS;
        xyz_t acc;
    };
    static constexpr int TIME_CHECKS_COUNT = 4;
    enum : uint32_t {
        IMU_AUTO_CALIBRATES = 0x01,
        IMU_PERFORMS_SENSOR_FUSION = 0x02,
        SENSOR_FUSION_REQUIRES_INITIALIZATION = 0x04
     };
public:
    AHRS(uint32_t taskIntervalMicroSeconds, SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters, uint32_t flags);
    AHRS(uint32_t taskIntervalMicroSeconds, SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters);
public:
    void setVehicleController(VehicleControllerBase* vehicleController) { _vehicleController = vehicleController; }
    bool configuredToUpdateOutputs() const { return (_vehicleController==nullptr) ? false : true; }
private:
    // class is not copyable or moveable
    AHRS(const AHRS&) = delete;
    AHRS& operator=(const AHRS&) = delete;
    AHRS(AHRS&&) = delete;
    AHRS& operator=(AHRS&&) = delete;
public:
    const IMU_Base& getIMU() const { return _IMU; };
    IMU_Base& getIMU() { return _IMU; };
    IMU_Base::xyz_int32_t getGyroOffset() const;
    void setGyroOffset(const IMU_Base::xyz_int32_t& offset);
    IMU_Base::xyz_int32_t getAccOffset() const;
    void setAccOffset(const IMU_Base::xyz_int32_t& offset);

    static IMU_Base::xyz_int32_t mapOffset(const IMU_Base::xyz_int32_t& offset, IMU_Base::axis_order_e axisOrder);
    IMU_Base::xyz_int32_t getGyroOffsetMapped() const;
    void setGyroOffsetMapped(const IMU_Base::xyz_int32_t& offset);
    IMU_Base::xyz_int32_t getAccOffsetMapped() const;
    void setAccOffsetMapped(const IMU_Base::xyz_int32_t& offset);

    void readGyroRaw(int32_t& x, int32_t& y, int32_t& z) const;
    void readAccRaw(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t getAccOneG_Raw() const;

    data_t getAhrsDataUsingLock(bool& updatedSinceLastRead) const;
    data_t getAhrsDataUsingLock() const;
    data_t getAhrsDataForInstrumentationUsingLock() const;
    Quaternion getOrientationUsingLock(bool& updatedSinceLastRead) const;
    Quaternion getOrientationUsingLock() const;
    Quaternion getOrientationForInstrumentationUsingLock() const;

    void checkFusionFilterConvergence(const xyz_t& acc, const Quaternion& orientation);
    inline bool sensorFusionFilterIsInitializing() const { return  (_flags & SENSOR_FUSION_REQUIRES_INITIALIZATION) && _sensorFusionInitializing; }
    inline void setSensorFusionInitializing(bool sensorFusionInitializing) { _sensorFusionInitializing = sensorFusionInitializing; }
    inline uint32_t getFlags() const { return _flags; }

    const IMU_FiltersBase::filters_t& getFilters() const { return _imuFilters.getFilters(); }
    void setFilters(const IMU_FiltersBase::filters_t& filters);
    inline uint32_t getTaskIntervalMicroSeconds() const { return _taskIntervalMicroSeconds; }
    inline uint32_t getFifoCount() const { return _fifoCount; } // for instrumentation
    inline uint32_t getTimeChecksMicroSeconds(size_t index) const { return _timeChecksMicroSeconds[index]; } //!< Instrumentation time checks
    inline const TaskBase* getTask() const { return _task; }
    inline void setTask(const TaskBase* task) { _task = task; }
private:
    static uint32_t flags(const SensorFusionFilterBase& sensorFusionFilter, const IMU_Base& imuSensor);
public:
    bool readIMUandUpdateOrientation(float deltaT, uint32_t tickCountDelta);
private:
    SensorFusionFilterBase& _sensorFusionFilter;
    IMU_Base& _IMU;
    IMU_FiltersBase& _imuFilters;
    VehicleControllerBase* _vehicleController {nullptr};
    const TaskBase* _task {nullptr};

    IMU_Base::accGyroRPS_t _accGyroRPS {};
    IMU_Base::accGyroRPS_t _accGyroRPS_Locked {};
    mutable int32_t _ahrsDataUpdatedSinceLastRead {false};

    Quaternion _orientation {};
    mutable int32_t _orientationUpdatedSinceLastRead {false};
    uint32_t _sensorFusionInitializing {true};
    const uint32_t _flags;
    uint32_t _taskIntervalMicroSeconds;
    float _taskIntervalSeconds;
    uint32_t _tickCountDelta {};

    // instrumentation member data
    uint32_t _fifoCount {0};
    std::array<uint32_t, TIME_CHECKS_COUNT + 1> _timeChecksMicroSeconds {};

#if defined(USE_FREERTOS)
    inline void YIELD_TASK() { taskYIELD(); }
#if defined(USE_AHRS_DATA_MUTEX)
    // option to use mutex rather than critical section
    StaticSemaphore_t _ahrsDataMutexBuffer {}; // _ahrsDataMutexBuffer must be declared before _ahrsDataMutex
    mutable SemaphoreHandle_t _ahrsDataMutex {};
    inline void LOCK_AHRS_DATA() const { xSemaphoreTake(_ahrsDataMutex, portMAX_DELAY); }
    inline void UNLOCK_AHRS_DATA() const { xSemaphoreGive(_ahrsDataMutex); }
#else // defaults to use critical section
    mutable portMUX_TYPE _ahrsDataSpinlock = portMUX_INITIALIZER_UNLOCKED;
    inline void LOCK_AHRS_DATA() const { taskENTER_CRITICAL(&_ahrsDataSpinlock); }
    inline void UNLOCK_AHRS_DATA() const { taskEXIT_CRITICAL(&_ahrsDataSpinlock); }
#endif
#elif defined(FRAMEWORK_RPI_PICO)
    inline void YIELD_TASK() {}
    mutable mutex_t _ahrsDataMutex {};
    inline void LOCK_AHRS_DATA() const { mutex_enter_blocking(&_ahrsDataMutex); }
    inline void UNLOCK_AHRS_DATA() const { mutex_exit(&_ahrsDataMutex); }
#else
    inline void YIELD_TASK() {}
    inline void LOCK_AHRS_DATA() const {}
    inline void UNLOCK_AHRS_DATA() const {}
#endif // USE_FREERTOS
};
