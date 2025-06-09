#pragma once

#include <cstdint>

class AHRS;
class SV_Preferences;

class BackchannelBase {
protected:
    BackchannelBase(AHRS& ahrs, SV_Preferences& preferences) :
        _ahrs(ahrs),
        _preferences(preferences)
    {}
public:
    virtual void WAIT_FOR_DATA_RECEIVED() = 0;
    virtual bool update() = 0;
    virtual bool sendTelemetryPacket(uint8_t valueType) = 0;
    bool sendTelemetryPacket() { return sendTelemetryPacket(0); }
protected:
    AHRS& _ahrs;
    SV_Preferences& _preferences;
};
