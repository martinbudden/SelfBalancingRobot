#pragma once

#include <cstddef>
#include <cstdint>

class AHRS;
class SV_Preferences;
class TaskBase;

/*!
Virtual base class for the backchannel transceiver.
Can be subclassed to provide transceivers over ESPNOW or UDP (for example).
*/
class BackchannelTransceiverBase {
public:
    virtual int sendData(const uint8_t* data, size_t len) const = 0;
    virtual void WAIT_FOR_DATA_RECEIVED() = 0;
    virtual size_t getReceivedDataLength() const = 0;
    virtual void setReceivedDataLengthToZero() = 0;
    virtual uint32_t getTickCountDeltaAndReset() = 0;
};

class BackchannelBase {
protected:
    BackchannelBase() = default;
public:
    void WAIT_FOR_DATA_RECEIVED() { _backchannelTransceiverPtr->WAIT_FOR_DATA_RECEIVED(); }
    int sendData(const uint8_t* data, size_t len) const { return _backchannelTransceiverPtr->sendData(data, len); }

    virtual bool processedReceivedPacket() = 0;
    virtual bool sendPacket(uint8_t subCommand) = 0;
    bool sendPacket() { return sendPacket(0); }

    inline const TaskBase* getTask() const { return _task; }
    inline void setTask(const TaskBase* task) { _task = task; }
protected:
    BackchannelTransceiverBase* _backchannelTransceiverPtr {};
    uint8_t* _transmitDataBufferPtr {};
    size_t _transmitDataBufferSize {};
    uint8_t* _receivedDataBufferPtr {};
    size_t _receivedDataBufferSize {};
    const TaskBase* _task {nullptr};
};
