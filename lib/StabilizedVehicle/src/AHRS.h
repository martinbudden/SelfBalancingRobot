#pragma once

#include "TaskBase.h"
#include <IMU_Base.h>

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


class IMU_FiltersBase;
class VehicleControllerBase;
class SensorFusionFilterBase;

/*!
The AHRS uses the ENU (East North Up) coordinate frame.
*/
class AHRS : public TaskBase {
public:
    struct data_t {
        uint32_t tickCountDelta;
        xyz_t gyroRPS;
        xyz_t acc;
    };
    static constexpr int TIME_CHECKS_COUNT = 4;
    struct filters_t {
        uint16_t gyro_lpf1_static_hz;
    };
public:
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
    data_t getAhrsDataForInstrumentationUsingLock() const;
    Quaternion getOrientationUsingLock(bool& updatedSinceLastRead) const;
    Quaternion getOrientationForInstrumentationUsingLock() const;

    void checkFusionFilterConvergence(const xyz_t& acc, const Quaternion& orientation);
    inline bool sensorFusionFilterIsInitializing() const { return _sensorFusionFilterInitializing; }
    inline void setSensorFusionFilterInitializing(bool sensorFusionFilterInitializing) { _sensorFusionFilterInitializing = sensorFusionFilterInitializing; }

    const filters_t& getFilters() const { return _filters; }
    void setFilters(const filters_t& filters);
    inline uint32_t getFifoCount() const { return _fifoCount; } // for instrumentation
    inline uint32_t getTimeChecksMicroSeconds(size_t index) const { return _timeChecksMicroSeconds[index]; } //!< Instrumentation time checks
public:
    struct TaskParameters {
        AHRS* ahrs;
        uint32_t taskIntervalMicroSeconds;
    };
    [[noreturn]] static void Task(void* arg);
    bool readIMUandUpdateOrientation(float deltaT);
    void loop();
private:
    [[noreturn]] void Task(const TaskParameters* taskParameters);
private:
    SensorFusionFilterBase& _sensorFusionFilter;
    IMU_Base& _IMU;
    IMU_FiltersBase& _imuFilters;
    VehicleControllerBase* _vehicleController {nullptr};

    IMU_Base::accGyroRPS_t _accGyroRPS {};
    IMU_Base::accGyroRPS_t _accGyroRPS_Locked {};
    mutable int32_t _ahrsDataUpdatedSinceLastRead {false};

    uint32_t _sensorFusionFilterInitializing {true};
    Quaternion _orientation {};
    mutable int32_t _orientationUpdatedSinceLastRead {false};

    // instrumentation member data
    uint32_t _fifoCount {0};
    std::array<uint32_t, TIME_CHECKS_COUNT + 1> _timeChecksMicroSeconds {};
    filters_t _filters {};

#if defined(USE_AHRS_DATA_MUTEX)
#if defined(USE_FREERTOS)
    StaticSemaphore_t _ahrsDataMutexBuffer {}; // _ahrsDataMutexBuffer must be declared before _ahrsDataMutex
    mutable SemaphoreHandle_t _ahrsDataMutex {};
    inline void LOCK_AHRS_DATA() const { xSemaphoreTake(_ahrsDataMutex, portMAX_DELAY); }
    inline void UNLOCK_AHRS_DATA() const { xSemaphoreGive(_ahrsDataMutex); }
#elif defined(FRAMEWORK_RPI_PICO)
    mutable mutex_t _ahrsDataMutex {};
    inline void LOCK_AHRS_DATA() const { mutex_enter_blocking(&_ahrsDataMutex); }
    inline void UNLOCK_AHRS_DATA() const { mutex_exit(&_ahrsDataMutex); }
#else
    inline void LOCK_AHRS_DATA() const {}
    inline void UNLOCK_AHRS_DATA() const {}
#endif // USE_FREERTOS
#elif defined(USE_AHRS_DATA_CRITICAL_SECTION)
#if defined(USE_FREERTOS)
    mutable portMUX_TYPE _ahrsDataSpinlock = portMUX_INITIALIZER_UNLOCKED;
    inline void LOCK_AHRS_DATA() const { taskENTER_CRITICAL(&_ahrsDataSpinlock); }
    inline void UNLOCK_AHRS_DATA() const { taskEXIT_CRITICAL(&_ahrsDataSpinlock); }
#elif defined(FRAMEWORK_RPI_PICO)
    mutable critical_section_t _ahrsDataCriticalSection {};
    inline void LOCK_AHRS_DATA() const { critical_section_enter_blocking(&_ahrsDataCriticalSection); }
    inline void UNLOCK_AHRS_DATA() const { critical_section_exit(&_ahrsDataCriticalSection); }
#else
    inline void LOCK_AHRS_DATA() const {}
    inline void UNLOCK_AHRS_DATA() const {}
#endif // USE_FREERTOS
#else
    inline void LOCK_AHRS_DATA() const {}
    inline void UNLOCK_AHRS_DATA() const {}
#endif // USE_AHRS_DATA_MUTEX
};
