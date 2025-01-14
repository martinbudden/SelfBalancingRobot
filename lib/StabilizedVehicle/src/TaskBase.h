#pragma once

#include <cstdint>

class TaskBase {
public:
    inline uint32_t getTickCountDelta() const { return _tickCountDelta; }
    inline uint32_t getTimeMicroSecondDelta() const { return _timeMicroSecondDelta; }
protected:
    uint32_t _tickCountDelta {0};
    uint32_t _tickCountPrevious {0};
    uint32_t _tickIntervalTicks {0};
    uint32_t _previousWakeTimeTicks {0};
    uint32_t _timeMicroSecondDelta {0};
    uint32_t _timeMicroSecondPrevious {0};
};
