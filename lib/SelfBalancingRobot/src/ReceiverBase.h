#pragma once
#include <cstdint>

class ReceiverBase {
public:
    struct controls_t {
        int32_t throttleStickQ4dot12; //<! unscaled throttle value from receiver as Q4.12 fixed point integer, ie in range [-2048, 2047]
        int32_t rollStickQ4dot12;
        int32_t pitchStickQ4dot12;
        int32_t yawStickQ4dot12;
    };
public:
    virtual bool update(uint32_t tickCountDelta) = 0;
    virtual controls_t getControls() const = 0;
    virtual uint32_t getFlags() const = 0;
    inline int32_t getDroppedPacketCountDelta() const { return _droppedPacketCountDelta; }
    inline uint32_t getTickCountDelta() const { return _tickCountDelta; }
protected:
    int32_t _droppedPacketCountDelta {0};
    uint32_t _tickCountDelta {0};
};
