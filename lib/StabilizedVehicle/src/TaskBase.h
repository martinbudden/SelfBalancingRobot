#pragma once

#include <cstdint>

class TaskBase {
public:
    struct parameters_t {
        TaskBase* task;
    };

public:
    explicit TaskBase(uint32_t taskIntervalMicroSeconds) : _taskIntervalMicroSeconds(taskIntervalMicroSeconds) {}
    TaskBase() : TaskBase(0) {}
public:
    inline void setTaskIntervalMicroSeconds(uint32_t taskIntervalMicroSeconds) { _taskIntervalMicroSeconds = taskIntervalMicroSeconds; }
    inline uint32_t getTaskIntervalMicroSeconds() { return _taskIntervalMicroSeconds; }
    inline uint32_t getTickCountDelta() const { return _tickCountDelta; }
    inline uint32_t getTimeMicroSecondDelta() const { return _timeMicroSecondsDelta; }
protected:
    uint32_t _taskIntervalMicroSeconds {0};
    uint32_t _previousWakeTimeTicks {0};
    uint32_t _tickCountDelta {0};
    uint32_t _tickCountPrevious {0};
    uint32_t _timeMicroSecondsDelta {0};
    uint32_t _timeMicroSecondsPrevious {0};
};
