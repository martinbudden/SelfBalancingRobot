#if defined(M5_UNIFIED) || defined(M5_STACK)

#include "ScreenM5.h"

#include <AHRS.h>
#include <AHRS_MessageQueue.h>
#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <MotorPairController.h>
#include <ReceiverAtomJoyStick.h>
#include <SV_TelemetryData.h>


enum {
    SCREEN_WIDTH_M5_ATOM_S3 = 128,
    SCREEN_WIDTH_M5_STICK_C = 80,
    SCREEN_WIDTH_M5_STICK_C_PLUS = 135,
    SCREEN_WIDTH_M5_CORE = 320,
    SCREEN_WIDTH_M5_CORE_INK = 200,
    SCREEN_WIDTH_M5_PAPER=540
};

enum {
    SCREEN_HEIGHT_M5_ATOM_S3 = 128,
    SCREEN_HEIGHT_M5_STICK_C = 80,
    SCREEN_HEIGHT_M5_STICK_C_PLUS = 135,
    SCREEN_HEIGHT_M5_CORE = 240,
    SCREEN_HEIGHT_M5_CORE_INK = 200,
    SCREEN_HEIGHT_M5_PAPER=960
};

static constexpr float radiansToDegrees {180.0 / M_PI};

ScreenM5::ScreenM5(const AHRS& ahrs, const MotorPairController& motorPairController, const ReceiverBase& receiver) :
    ScreenBase(ahrs, motorPairController, receiver),
    _screenSize(screenSize()),
    _screenRotationOffset(
        (_screenSize == SIZE_80x160 || _screenSize == SIZE_135x240) ? 1 :
        _screenSize == SIZE_128x128 ? -1 :
         0)
{
    M5.Lcd.setRotation(_screenMode + _screenRotationOffset);
    M5.Lcd.setTextSize(_screenSize == ScreenM5::SIZE_128x128 || _screenSize == ScreenM5::SIZE_80x160 || _screenSize == ScreenM5::SIZE_135x240 ? 1 : 2);
    M5.Lcd.fillScreen(TFT_BLACK);
}

ScreenM5::screen_size_e ScreenM5::screenSize()
{
    M5.Lcd.setRotation(1); // set to default, to find screen size

    screen_size_e screenSize;
    switch (M5.Lcd.height()) {
    case SCREEN_HEIGHT_M5_ATOM_S3:
        screenSize = SIZE_128x128;
        _screenSizeX = 128;
        _screenSizeY = 128;
        break;
    case SCREEN_WIDTH_M5_STICK_C: // M5_STICK_C rotated by default
        screenSize = SIZE_80x160;
        _screenSizeX = 80;
        _screenSizeY = 160;
        break;
    case SCREEN_WIDTH_M5_STICK_C_PLUS: // M5_STICK_C_PLUS rotated by default
        screenSize = SIZE_135x240;
        _screenSizeX = 135;
        _screenSizeY = 240;
        break;
    case SCREEN_HEIGHT_M5_CORE:
        screenSize = SIZE_320x240;
        _screenSizeX = 320;
        _screenSizeY = 240;
        break;
    case SCREEN_HEIGHT_M5_CORE_INK:
        screenSize = SIZE_200x200;
        _screenSizeX = 200;
        _screenSizeY = 200;
        break;
    case SCREEN_HEIGHT_M5_PAPER:
        screenSize = SIZE_540x960;
        _screenSizeX = 5400;
        _screenSizeY = 9600;
        break;
    default:
        screenSize = SIZE_320x240;
        _screenSizeX = 320;
        _screenSizeY = 240;
        break;
    }
    return screenSize;
}

void ScreenM5::setScreenMode(ScreenM5::mode_e screenMode)
{
    _screenMode = screenMode;

    if (_screenMode == MODE_QRCODE) {
        M5.Lcd.setRotation(MODE_NORMAL + _screenRotationOffset);
        M5.Lcd.fillScreen(TFT_BLACK);
        if (M5.Lcd.width() > M5.Lcd.height()) {
            M5.Lcd.qrcode("https://github.com/martinbudden/SelfBalancingRobot", (M5.Lcd.width() - M5.Lcd.height())/2, 0, M5.Lcd.height(), 6);
        } else {
            M5.Lcd.qrcode("https://github.com/martinbudden/SelfBalancingRobot", 0, (M5.Lcd.height() - M5.Lcd.width())/2, M5.Lcd.width(), 6);
        }
    } else {
        M5.Lcd.setRotation(_screenMode + _screenRotationOffset);
        updateScreenAndTemplate();
    }
}

/*!
Cycles through the different screen modes.
*/
void ScreenM5::nextScreenMode()
{
    const mode_e screenMode =
        _screenMode == ScreenM5::MODE_NORMAL ? ScreenM5::MODE_INVERTED :
        _screenMode == ScreenM5::MODE_INVERTED ? ScreenM5::MODE_QRCODE :
        ScreenM5::MODE_NORMAL;
    setScreenMode(screenMode);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
/*!
Utility function to display a MAC address.
*/
void ScreenM5::displayEUI(const char* prompt, const ReceiverBase::EUI_48_t& eui)
{
    M5.Lcd.printf("%s%02X:%02X:%02X:%02X:%02X:%02X", prompt, eui.octets[0], eui.octets[1], eui.octets[2], eui.octets[3], eui.octets[4], eui.octets[5]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

/*!
Utility function to display a MAC address in a compact format, for devices with small screens.
*/
void ScreenM5::displayEUI_Compact(const char* prompt, const ReceiverBase::EUI_48_t& eui)
{
    M5.Lcd.printf("%s%02x%02x%02x:%02x%02x%02x", prompt, eui.octets[0], eui.octets[1], eui.octets[2], eui.octets[3], eui.octets[4], eui.octets[5]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

void ScreenM5::updateTemplate_128x128() const
{
    M5.Lcd.setCursor(0, 10);
    displayEUI("M:", _receiver.getMyEUI());

    int32_t yPos = 50;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Gyro");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Acc");

    yPos = 95;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("T:");
    M5.Lcd.setCursor(60, yPos);
    M5.Lcd.printf("R:");

    yPos += 10;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Y:");
    M5.Lcd.setCursor(60, yPos);
    M5.Lcd.printf("P:");

    M5.Lcd.setCursor(0, 118);
    M5.Lcd.printf("M:");

    M5.Lcd.setCursor(60, 118);
    M5.Lcd.printf("I:");
}

void ScreenM5::updateReceivedData_128x128() const
{
    if (!_remoteEUI_updated) {
        M5.Lcd.setCursor(0, 20);
        displayEUI("J:", _receiver.getPrimaryPeerEUI());
    }

    // M5StickC
    int32_t yPos = 95;
    const ReceiverBase::controls_t controls = _receiver.getControls();

    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%7.3f", controls.throttle);
    M5.Lcd.setCursor(72, yPos);
    M5.Lcd.printf("%7.3f", controls.roll);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%7.3f", controls.yaw);
    M5.Lcd.setCursor(72, yPos);
    M5.Lcd.printf("%7.3f", controls.pitch);
}

void ScreenM5::update_128x128(const TD_AHRS::data_t& ahrsData) const
{
    int32_t yPos = 35;

    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:%5.0f", ahrsData.pitch);

    M5.Lcd.setCursor(60, yPos);
    M5.Lcd.printf("ro:%5.0f", ahrsData.roll);

    yPos += 25;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("x:%4.0f y:%4.0f z:%4.0f", ahrsData.gyroRPS.x*radiansToDegrees, ahrsData.gyroRPS.y*radiansToDegrees, ahrsData.gyroRPS.z*radiansToDegrees);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("x:%4.1f y:%4.1f z:%4.1f", ahrsData.acc.x, ahrsData.acc.y, ahrsData.acc.z);

    M5.Lcd.setCursor(12, 118);
    M5.Lcd.print(_motorPairController.motorsIsOn() ? "ON " : "OFF");

    M5.Lcd.setCursor(72, 118);
    M5.Lcd.printf("%2d  ", static_cast<int>(_receiver.getTickCountDelta()));
}

void ScreenM5::updateTemplate_80x160() const
{
    M5.Lcd.setCursor(0, 10);
    displayEUI_Compact("", _receiver.getMyEUI());

    int32_t yPos = 90;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("T:");

    yPos += 10;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("R:");

    yPos += 10;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("P:");

    yPos += 10;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Y:");

    M5.Lcd.setCursor(0, 150);
    M5.Lcd.printf("M:");

    M5.Lcd.setCursor(40, 150);
    M5.Lcd.printf("I:");
}

void ScreenM5::updateReceivedData_80x160() const
{
    if (!_remoteEUI_updated) {
        M5.Lcd.setCursor(0, 20);
        displayEUI_Compact("", _receiver.getPrimaryPeerEUI());
    }
    // M5StickC
    int32_t yPos = 90;
    M5.Lcd.setCursor(12, yPos);
    const ReceiverBase::controls_t controls = _receiver.getControls();
    M5.Lcd.printf("%7.3f", controls.throttle);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%7.3f", controls.roll);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%7.3f", controls.pitch);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%7.3f", controls.yaw);

    yPos += 10;
    M5.Lcd.setCursor(0, yPos);
    const uint32_t flipButton =_receiver.getSwitch(ReceiverAtomJoyStick::MOTOR_ON_OFF_SWITCH);
    const uint32_t mode = _receiver.getSwitch(ReceiverAtomJoyStick::MODE_SWITCH);
    const uint32_t altMode = _receiver.getSwitch(ReceiverAtomJoyStick::ALT_MODE_SWITCH);
    M5.Lcd.printf("M%1d%s A%1d F%1d ", static_cast<int>(mode), mode == ReceiverAtomJoyStick::MODE_STABLE ? "ST" : "SP", static_cast<int>(altMode), static_cast<int>(flipButton));
}

void ScreenM5::update_80x160(const TD_AHRS::data_t& ahrsData) const
{
    int32_t yPos = 30;
    const bool displayAcc = true;

    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:%5.0f", static_cast<double>(ahrsData.pitch));

    yPos += 10;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ro:%5.0f", static_cast<double>(ahrsData.roll));

    yPos += 10;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ya:%5.0f", static_cast<double>(ahrsData.yaw));

    yPos += 10;
    if (displayAcc) {
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("ax:%5.2f", ahrsData.acc.x);

        yPos += 10;
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("ay:%5.2f", ahrsData.acc.y);

        yPos += 10;
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("az:%5.2f", ahrsData.acc.z);
    } else {
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("gx:%5.0f", ahrsData.gyroRPS.x*radiansToDegrees);

        yPos += 10;
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("gy:%5.0f", ahrsData.gyroRPS.y*radiansToDegrees);

        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("gz:%5.0f", ahrsData.gyroRPS.z*radiansToDegrees);
    }

    M5.Lcd.setCursor(12, 150);
    M5.Lcd.print(_motorPairController.motorsIsOn() ? "ON " : "OFF");

    M5.Lcd.setCursor(52, 150);
    M5.Lcd.printf("%2d  ", static_cast<int>(_receiver.getTickCountDelta()));
}

void ScreenM5::updateTemplate_135x240() const
{
    M5.Lcd.setCursor(0, 0);
    displayEUI("M:", _receiver.getMyEUI());

    int32_t yPos = 105;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ax:");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ay:");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("az:");

    yPos += 25;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("T:");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("R:");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("P:");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Y:");

    M5.Lcd.setCursor(0, 220);
    M5.Lcd.printf("M:");
}

void ScreenM5::updateReceivedData_135x240() const
{
    updateReceivedData_80x160();
}

void ScreenM5::update_135x240(const TD_AHRS::data_t& ahrsData) const
{
    int32_t yPos = 45;

    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:%5.0f", ahrsData.pitch);
    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ro:%5.0f", ahrsData.roll);
    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ya:%5.0f", ahrsData.yaw);

    yPos += 20;
    M5.Lcd.setCursor(36, yPos);
    M5.Lcd.printf("%5.2f", ahrsData.acc.x);
    yPos += 20;
    M5.Lcd.setCursor(36, yPos);
    M5.Lcd.printf("%5.2f", ahrsData.acc.y);
    yPos += 20;
    M5.Lcd.setCursor(36, yPos);
    M5.Lcd.printf("%5.2f", ahrsData.acc.z);

    M5.Lcd.setCursor(18, 220);
    M5.Lcd.printf(_motorPairController.motorsIsOn() ? "ON " : "OFF");
}

void ScreenM5::updateTemplate_320x240() const
{
    M5.Lcd.setCursor(0, 0);
    displayEUI("MAC:", _receiver.getMyEUI());

    int32_t yPos = 115;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("T:");
    M5.Lcd.setCursor(160, yPos);
    M5.Lcd.printf("R:");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Y:");
    M5.Lcd.setCursor(160, yPos);
    M5.Lcd.printf("P:");

    yPos += 20;
    M5.Lcd.setCursor(160, yPos);
    M5.Lcd.printf("Ticks");

#if defined(MOTORS_HAVE_ENCODERS)
    yPos = 180;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("meL:");
    M5.Lcd.setCursor(160, yPos);
    M5.Lcd.printf("meR:");

    yPos = 200;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("msL:");
    M5.Lcd.setCursor(160, yPos);
    M5.Lcd.printf("msR:");
    M5.Lcd.setCursor(160, yPos + 20);
    M5.Lcd.printf("RPM:");
#else
    yPos = 190;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("mpL:");
    M5.Lcd.setCursor(160, yPos);
    M5.Lcd.printf("mpR:");
#endif

    M5.Lcd.setCursor(0, 220);
    M5.Lcd.printf("Motors:");
}

void ScreenM5::updateReceivedData_320x240() const
{
    if (!_remoteEUI_updated) {
        M5.Lcd.setCursor(0, 20);
        displayEUI("REM:", _receiver.getPrimaryPeerEUI());
    }

    int32_t yPos = 115;

    M5.Lcd.setCursor(20, yPos);
    const ReceiverBase::controls_t controls = _receiver.getControls();
    M5.Lcd.printf("%8.4f", controls.throttle);
    M5.Lcd.setCursor(180, yPos);
    M5.Lcd.printf("%8.4f", controls.roll);

    yPos += 20;
    M5.Lcd.setCursor(20, yPos);
    M5.Lcd.printf("%8.4f", controls.yaw);
    M5.Lcd.setCursor(180, yPos);
    M5.Lcd.printf("%8.4f", controls.pitch);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    const uint32_t flipButton =_receiver.getSwitch(ReceiverAtomJoyStick::MOTOR_ON_OFF_SWITCH);
    const uint32_t mode = _receiver.getSwitch(ReceiverAtomJoyStick::MODE_SWITCH);
    const uint32_t altMode = _receiver.getSwitch(ReceiverAtomJoyStick::ALT_MODE_SWITCH);
    M5.Lcd.printf("M%1d %s A%1d F%1d ", static_cast<int>(mode), mode == ReceiverAtomJoyStick::MODE_STABLE ? "ST" : "SP", static_cast<int>(altMode), static_cast<int>(flipButton));

    M5.Lcd.setCursor(255, yPos);
    M5.Lcd.printf("%3d", static_cast<int>(static_cast<int>(_receiver.getTickCountDelta())));
}

void ScreenM5::update_320x240(const TD_AHRS::data_t& ahrsData) const
{
    int32_t yPos = 45;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:%5.0f ro:%5.0f ya:%5.0f", ahrsData.pitch, ahrsData.roll, ahrsData.yaw);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("gx:%5.0f gy:%5.0f gz:%5.0f", ahrsData.gyroRPS.x*radiansToDegrees, ahrsData.gyroRPS.y*radiansToDegrees, ahrsData.gyroRPS.z*radiansToDegrees);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ax:%5.2f ay:%5.2f az:%5.2f", ahrsData.acc.x, ahrsData.acc.y, ahrsData.acc.z);

    const motor_pair_controller_telemetry_t mpcTelemetry = _motorPairController.getTelemetryData();
#if defined(MOTORS_HAVE_ENCODERS)
    yPos = 180;
    M5.Lcd.setCursor(48, yPos);
    M5.Lcd.printf("%8d", static_cast<int>(mpcTelemetry.encoderLeft));
    M5.Lcd.setCursor(208, yPos);
    M5.Lcd.printf("%8d", static_cast<int>(mpcTelemetry.encoderRight));

    yPos = 200;
    M5.Lcd.setCursor(48, yPos);
    M5.Lcd.printf("%8.0f", static_cast<double>(mpcTelemetry.speedLeftDPS));
    M5.Lcd.setCursor(208, yPos);
    M5.Lcd.printf("%8.0f", static_cast<double>(mpcTelemetry.speedRightDPS));
    M5.Lcd.setCursor(208, yPos + 20);
    M5.Lcd.printf("%8.0f", static_cast<double>(std::roundf(mpcTelemetry.speedDPS_Filtered*(1.0F/6.0F))));
#else
    yPos = 190;
    M5.Lcd.setCursor(48, yPos);
    M5.Lcd.printf("%8.3f", mpcTelemetry.powerLeft);
    M5.Lcd.setCursor(208, yPos);
    M5.Lcd.printf("%8.3f", mpcTelemetry.powerRight);
#endif

    yPos = 220;
    M5.Lcd.setCursor(85, yPos);
    M5.Lcd.printf("%s", _motorPairController.motorsIsOn() ? "ON " : "OFF");
}
#pragma GCC diagnostic pop

void ScreenM5::updateTemplate()
{
    M5.Lcd.fillScreen(TFT_BLACK);

    switch (_screenSize) {
    case SIZE_128x128:
        updateTemplate_128x128();
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        updateTemplate_80x160();
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        updateTemplate_320x240();
    }
    _templateIsUpdated = true;
}

void ScreenM5::updateReceivedData()
{
    switch (_screenSize) {
    case SIZE_128x128:
        updateReceivedData_128x128();
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        updateReceivedData_80x160();
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        updateReceivedData_320x240();
    }
    _remoteEUI_updated = true;
}

void ScreenM5::updateAHRS_Data() const
{
    AHRS::ahrs_data_t ahrsData {};
    _motorPairController.getAHRS_MessageQueue().PEEK_AHRS_DATA(ahrsData);
    const Quaternion orientation = ahrsData.orientation;
    TD_AHRS::data_t tdAhrsData {
        .roll = _motorPairController.getRollAngleDegreesRaw(),
        .pitch = _motorPairController.getPitchAngleDegreesRaw(),
        .yaw = _motorPairController.getYawAngleDegreesRaw(),
        .gyroRPS = ahrsData.accGyroRPS.gyroRPS,
        .acc = ahrsData.accGyroRPS.acc,
        .gyroOffset = {},
        .accOffset = {}
    };
    if (tdAhrsData.roll == MotorPairController::NOT_SET) {
        tdAhrsData.roll = orientation.calculatePitchDegrees();
    }
    if (tdAhrsData.yaw == MotorPairController::NOT_SET) {
        tdAhrsData.yaw = orientation.calculateYawDegrees();
    }
    switch (_screenSize) {
    case SIZE_128x128:
        update_128x128(tdAhrsData);
        break;
    case SIZE_80x160:
        update_80x160(tdAhrsData);
        break;
    case SIZE_135x240:
        update_135x240(tdAhrsData);
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        update_320x240(tdAhrsData);
    }
}

/*!
Update the screen with data from the AHRS and the receiver.
*/
void ScreenM5::update()
{
    // update the screen with the AHRS data
    if (_screenMode != ScreenM5::MODE_QRCODE) {
        // update the screen template if it hasn't been updated
        if (_templateIsUpdated == false) {
            updateTemplate();
        }
        updateAHRS_Data();
        if (isNewReceiverPacketAvailable()) {
            clearNewReceiverPacketAvailable();
            // update the screen with data received from the receiver
            updateReceivedData();
        }
    }
}
#endif
