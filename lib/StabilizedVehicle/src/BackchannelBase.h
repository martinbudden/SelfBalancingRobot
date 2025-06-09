#pragma once

#include <cstdint>

class AHRS;
class SV_Preferences;

class BackchannelBase {
protected:
    BackchannelBase(AHRS& ahrs, SV_Preferences& preferences);
public:
    virtual void WAIT_FOR_DATA_RECEIVED() = 0;
    virtual bool update() = 0;
    virtual bool sendTelemetryPacket() = 0;
protected:
    AHRS& _ahrs;
    SV_Preferences& _preferences;
};
