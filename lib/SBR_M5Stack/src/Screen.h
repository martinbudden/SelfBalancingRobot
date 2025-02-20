#pragma once

#include "SV_TelemetryData.h"

class AHRS;
class MotorPairController;
class Receiver;


class Screen {
public:
    enum screen_size_t {
        SIZE_128x128,
        SIZE_80x160,
        SIZE_135x240,
        SIZE_320x240,
        SIZE_200x200,
        SIZE_540x960
    };
    enum mode_t { MODE_NORMAL = 1, MODE_INVERTED = 3, MODE_QRCODE = 5 };
public:
    Screen(const AHRS& ahrs, const MotorPairController& motorPairController, const Receiver& receiver);
private:
    // Screen is not copyable or moveable
    Screen(const Screen&) = delete;
    Screen& operator=(const Screen&) = delete;
    Screen(Screen&&) = delete;
    Screen& operator=(Screen&&) = delete;
public:
    inline screen_size_t getScreenSize() const { return _screenSize; }
    inline mode_t getScreenMode() const { return _screenMode; }
    void setScreenMode(mode_t screenMode);
    void nextScreenMode();

    void updateTemplate() const;
    void update(bool packetReceived) const;
    inline void update() const { update(false); }
private:
    screen_size_t screenSize();

    void updateTemplate128x128() const;
    void updateTemplate80x160() const;
    void updateTemplate135x240() const;
    void updateTemplate320x240() const;

    void updateReceivedData() const;
    void updateReceivedData80x160() const;
    void updateReceivedData128x128() const;
    void updateReceivedData135x240() const;
    void updateReceivedData320x240() const;

    void update80x160(const TD_AHRS::Data& ahrsData) const; // M5StickC
    void update128x128(const TD_AHRS::Data& ahrsData) const; // M5Atom
    void update135x240(const TD_AHRS::Data& ahrsData) const; // M5StickC Plus
    void update320x240(const TD_AHRS::Data& ahrsData) const; // MCore

    void updateAHRS_Data() const;

    static void displayEUI(const char* prompt, const ReceiverBase::EUI_48_t& eui);
    static void displayEUI_Compact(const char* prompt, const ReceiverBase::EUI_48_t& eui);
private:
    screen_size_t _screenSize {SIZE_320x240};
    mode_t _screenMode {MODE_NORMAL};
    int _screenRotationOffset {0};

    const AHRS& _ahrs;
    const MotorPairController& _motorPairController;
    const Receiver& _receiver;
};
