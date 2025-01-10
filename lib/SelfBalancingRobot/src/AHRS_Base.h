#pragma once

#include "SensorFusionFilter.h"
#include "TaskBase.h"
#include <Quaternion.h>
#include <xyz_int16_type.h>
#include <xyz_type.h>


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
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) = 0;
    virtual void setAccOffset(const xyz_int16_t& accOffset) = 0;
    virtual xyz_int16_t readGyroRaw() const = 0;
    virtual xyz_int16_t readAccRaw() const = 0;

    virtual data_t getAhrsDataUsingLock(bool& updatedSinceLastRead) const = 0;
    virtual data_t getAhrsDataForInstrumentationUsingLock() const = 0;
    virtual Quaternion getOrientationUsingLock(bool& updatedSinceLastRead) const = 0;
    virtual Quaternion getOrientationForInstrumentationUsingLock() const = 0;

    inline bool filterIsInitializing() const { return _filterInitializing != 0; }
    inline void setFilterInitializing(bool filterInitializing) { _filterInitializing = filterInitializing; }
    inline uint32_t getFifoCount() const { return _fifoCount; } // for instrumentation
protected:
    SensorFusionFilterBase& _sensorFusionFilter;
    Quaternion _orientation;
    uint32_t _filterInitializing {false};
    uint32_t _fifoCount {0};
    mutable int32_t _ahrsDataUpdatedSinceLastRead {false};
    mutable int32_t _orientationUpdatedSinceLastRead {false};
};
