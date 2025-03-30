#pragma once

#include <array>
#include <cassert>
#include <cstdint>


/*!
Abstract Base Class defining a receiver.
*/
class ReceiverBase {
public:
    enum { STICK_COUNT = 4 };
public:
    struct EUI_48_t {
        uint8_t octet[6];
    }; //!< 48-bit extended unique identifier (often synonymous with MAC address)
    struct controls_t {
        int32_t throttleStickQ4dot12; //<! unscaled throttle value from receiver as Q4.12 fixed point integer, ie in range [-2048, 2047]
        int32_t rollStickQ4dot12;
        int32_t pitchStickQ4dot12;
        int32_t yawStickQ4dot12;
    };
public:
    virtual bool update(uint32_t tickCountDelta) = 0;
    virtual void getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const = 0;
    // 48-bit Extended Unique Identifiers, usually the MAC address if the receiver has one, but may be an alternative provided by the receiver.
    virtual EUI_48_t getMyEUI() const = 0;
    virtual EUI_48_t getPrimaryPeerEUI() const = 0;

    inline controls_t getControls() const { return _controls; }
    inline uint8_t getAux(size_t index) const { assert(index < 4); return _aux[index]; }
    inline uint32_t getSwitch(size_t index) const { return (_switches & (0b11U << (2*index))) >> (2*index); }
    inline void setSwitch(size_t index, uint8_t value) { _switches &= ~(0b11U << (2*index)); _switches |= (value & 0b11U) << (2*index); }
    inline uint32_t getSwitches() const { return _switches; }

    inline int32_t getDroppedPacketCountDelta() const { return _droppedPacketCountDelta; }
    inline uint32_t getTickCountDelta() const { return _tickCountDelta; }
    inline static float Q4dot12_to_float(int32_t q4dot12) { return static_cast<float>(q4dot12) * (1.0F / 2048.0F); } //<! convert Q4dot12 fixed point number to floating point
protected:
    int32_t _droppedPacketCountDelta {0};
    uint32_t _tickCountDelta {0};
    uint32_t _switches {0}; // 16 2 or 3 positions switches, each using 2-bits
    controls_t _controls {}; //!< the main 4 channels
    std::array<uint8_t, 4> _aux {}; //!< 4 8-bit auxiliary channels
};
