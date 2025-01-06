#pragma once

#include <cstdint>

class TaskBase {
public:
    inline uint32_t getTickCountDelta() const { return _tickCountDelta; }
protected:
    uint32_t _tickCountDelta {0};
    uint32_t _tickCountPrevious {0};
    uint32_t _tickIntervalTicks {0};
    uint32_t _previousWakeTime {0};
};
