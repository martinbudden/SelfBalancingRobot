#pragma once

#include "TaskBase.h"
#include <Quaternion.h>
#include <xyz_int16_type.h>
#include <xyz_type.h>

class MotorPairController;
class SensorFusionFilterBase;

/*!
AHRS (Attitude and Heading Reference System) virtual base class.
*/
class AHRS_Base : public TaskBase {
public:
    struct data_t {
        uint32_t tickCountDelta;
        xyz_t gyroRadians;
        xyz_t acc;
    };
public:
    AHRS_Base(SensorFusionFilterBase& sensorFusionFilter) : _sensorFusionFilter(sensorFusionFilter) {}
public:
    void setMotorController(MotorPairController* motorController) { _motorController = motorController; }
    const MotorPairController* getMotorController() const { return _motorController; }
public:
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) = 0;
    virtual void setAccOffset(const xyz_int16_t& accOffset) = 0;
    virtual xyz_int16_t readGyroRaw() const = 0;
    virtual xyz_int16_t readAccRaw() const = 0;

    virtual data_t getAhrsDataUsingLock(bool& updatedSinceLastRead) const = 0;
    virtual data_t getAhrsDataForInstrumentationUsingLock() const = 0;
    virtual Quaternion getOrientationUsingLock(bool& updatedSinceLastRead) const = 0;
    virtual Quaternion getOrientationForInstrumentationUsingLock() const = 0;

    inline bool sensorFusionFilterIsInitializing() const { return _sensorFusionFilterInitializing; }
    inline void setSensorFusionFilterInitializing(bool sensorFusionFilterInitializing) { _sensorFusionFilterInitializing = sensorFusionFilterInitializing; }

    inline uint32_t getFifoCount() const { return _fifoCount; } // for instrumentation
    inline uint32_t getUpdateTimeIMU_ReadMicroSeconds() const { return _updateTimeIMU_ReadMicroSeconds; } // for instrumentation
    inline uint32_t getUpdateTimeFiltersMicroSeconds() const { return _updateTimeFiltersMicroSeconds; } // for instrumentation
    inline uint32_t getUpdateTimeSensorFusionMicroSeconds() const { return _updateTimeSensorFusionMicroSeconds; } // for instrumentation
    inline uint32_t getUpdateTimePID_MicroSeconds() const { return _updateTimePID_MicroSeconds; } // for instrumentation
protected:
    SensorFusionFilterBase& _sensorFusionFilter;
    Quaternion _orientation;
    MotorPairController* _motorController {nullptr};
    uint32_t _sensorFusionFilterInitializing {true};
    mutable int32_t _ahrsDataUpdatedSinceLastRead {false};
    mutable int32_t _orientationUpdatedSinceLastRead {false};
    // instrumentation member data
    uint32_t _fifoCount {0};
    uint32_t _updateTimeIMU_ReadMicroSeconds {0};
    uint32_t _updateTimeFiltersMicroSeconds {0};
    uint32_t _updateTimeSensorFusionMicroSeconds {0};
    uint32_t _updateTimePID_MicroSeconds {0};
};
