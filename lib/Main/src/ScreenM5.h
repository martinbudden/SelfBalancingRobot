#pragma once

#include <ReceiverBase.h>
#include <SV_TelemetryData.h>
#include <ScreenBase.h>


class ScreenM5 : public ScreenBase {
public:
    enum screen_size_e {
        SIZE_128x128,
        SIZE_80x160,
        SIZE_135x240,
        SIZE_320x240,
        SIZE_200x200,
        SIZE_540x960
    };
    enum mode_e { MODE_NORMAL = 1, MODE_INVERTED = 3, MODE_QRCODE = 5 };
public:
    ScreenM5(const AHRS& ahrs, const MotorPairController& motorPairController, const ReceiverBase& receiver);
private:
    // ScreenM5 is not copyable or moveable
    ScreenM5(const ScreenM5&) = delete;
    ScreenM5& operator=(const ScreenM5&) = delete;
    ScreenM5(ScreenM5&&) = delete;
    ScreenM5& operator=(ScreenM5&&) = delete;
public:
    inline screen_size_e getScreenSize() const { return _screenSize; }

    virtual void nextScreenMode() override;
    virtual void update() override;
    virtual void updateTemplate() override;
private:
    void setScreenMode(mode_e screenMode);
    inline mode_e getScreenMode() const { return _screenMode; }
    screen_size_e screenSize();

    void updateTemplate_128x128() const;
    void updateReceivedData_128x128() const;
    void update_128x128(const TD_AHRS::data_t& ahrsData) const; // M5Atom

    void updateTemplate_80x160() const;
    void updateReceivedData_80x160() const;
    void update_80x160(const TD_AHRS::data_t& ahrsData) const; // M5StickC

    void updateTemplate_135x240() const;
    void updateReceivedData_135x240() const;
    void update_135x240(const TD_AHRS::data_t& ahrsData) const; // M5StickC Plus

    void updateTemplate_320x240() const;
    void updateReceivedData_320x240() const;
    void update_320x240(const TD_AHRS::data_t& ahrsData) const; // MCore

    void updateReceivedData();
    void updateAHRS_Data() const;

    static void displayEUI(const char* prompt, const ReceiverBase::EUI_48_t& eui);
    static void displayEUI_Compact(const char* prompt, const ReceiverBase::EUI_48_t& eui);
private:
    screen_size_e _screenSize {SIZE_320x240};
    mode_e _screenMode {MODE_NORMAL};
    int _screenRotationOffset {0};
};
