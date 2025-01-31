#include "Screen.h"

#include <AHRS.h>
#include <ESPNOW_Receiver.h>
#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <MotorPairController.h>
#include <MotorPairControllerTelemetry.h>
#include <cfloat>


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

constexpr float radiansToDegrees {180.0 / M_PI};

Screen::Screen(const AHRS& ahrs, const MotorPairController& motorPairController, const Receiver& receiver) :
    _screenSize(screenSize()),
    _screenRotationOffset(
        (_screenSize == SIZE_80x160 || _screenSize == SIZE_135x240) ? 1 :
        _screenSize == SIZE_128x128 ? -1 :
         0),
    _ahrs(ahrs),
    _motorPairController(motorPairController),
    _receiver(receiver)
{
    M5.Lcd.setRotation(_screenMode + _screenRotationOffset);
    M5.Lcd.setTextSize(_screenSize == Screen::SIZE_128x128 || _screenSize == Screen::SIZE_80x160 || _screenSize == Screen::SIZE_135x240 ? 1 : 2);
}

Screen::screen_size_t Screen::screenSize()
{
    M5.Lcd.setRotation(1); // set to default, to find screen size

    screen_size_t screenSize;
    switch (M5.Lcd.height()) {
    case SCREEN_HEIGHT_M5_ATOM_S3:
        screenSize = SIZE_128x128;
        break;
    case SCREEN_WIDTH_M5_STICK_C: // M5_STICK_C rotated by default
        screenSize = SIZE_80x160;
        break;
    case SCREEN_WIDTH_M5_STICK_C_PLUS: // M5_STICK_C_PLUSE rotated by default
        screenSize = SIZE_135x240;
        break;
    case SCREEN_HEIGHT_M5_CORE:
        screenSize = SIZE_320x240;
        break;
    case SCREEN_HEIGHT_M5_CORE_INK:
        screenSize = SIZE_200x200;
        break;
    case SCREEN_HEIGHT_M5_PAPER:
        screenSize = SIZE_540x960;
        break;
    default:
        screenSize = SIZE_320x240;
        break;
    }
    return screenSize;
}

void Screen::setScreenMode(Screen::mode_t screenMode)
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
        updateTemplate();
        update();
    }
}

/*!
Cycles through the different screen modes.
*/
void Screen::nextScreenMode()
{
    const mode_t screenMode =
        _screenMode == Screen::MODE_NORMAL ? Screen::MODE_INVERTED :
        _screenMode == Screen::MODE_INVERTED ? Screen::MODE_QRCODE :
        Screen::MODE_NORMAL;
    setScreenMode(screenMode);
}

/*!
Utility function to display a MAC address.
*/
void Screen::displayEUI(const char* prompt, const ReceiverBase::EUI_48_t& eui)
{
    M5.Lcd.printf("%s%02X:%02X:%02X:%02X:%02X:%02X", prompt, eui.octet[0], eui.octet[1], eui.octet[2], eui.octet[3], eui.octet[4], eui.octet[5]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

/*!
Utility function to display a MAC address in a compact format, for devices with small screens.
*/
void Screen::displayEUI_Compact(const char* prompt, const ReceiverBase::EUI_48_t& eui)
{
    M5.Lcd.printf("%s%02x%02x%02x:%02x%02x%02x", prompt, eui.octet[0], eui.octet[1], eui.octet[2], eui.octet[3], eui.octet[4], eui.octet[5]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

void Screen::updateTemplate128x128() const
{
    M5.Lcd.setCursor(0, 10);
    displayEUI("M:", _receiver.getMyEUI());
    M5.Lcd.setCursor(0, 20);
    displayEUI("J:", _receiver.getPrimaryPeerEUI());

    int yPos = 35;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:");

    yPos += 15;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Acc");

    yPos = 75;
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

    M5.Lcd.setCursor(0, 118);
    M5.Lcd.printf("M:");

    M5.Lcd.setCursor(60, 118);
    M5.Lcd.printf("D:");
}

void Screen::updateReceivedData128x128() const
{
    // M5StickC
    int yPos = 75;
    M5.Lcd.setCursor(12, yPos);
    const ReceiverBase::controls_t controls = _receiver.getControls();
    M5.Lcd.printf("%6d", controls.throttleStickQ4dot12);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%6d", controls.rollStickQ4dot12);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%6d", controls.pitchStickQ4dot12);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%6d", controls.yawStickQ4dot12);
}

void Screen::update128x128(const TD_AHRS::Data& ahrsData) const
{
    int yPos = 35;

    M5.Lcd.setCursor(18, yPos);
    M5.Lcd.printf("%5.0F", ahrsData.pitch);

    yPos += 25;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("x:%4.1f y:%4.1f z:%4.1f", ahrsData.acc.x, ahrsData.acc.y, ahrsData.acc.z);

    M5.Lcd.setCursor(12, 118);
    M5.Lcd.print(_motorPairController.motorsIsOn() ? "ON " : "OFF");

    M5.Lcd.setCursor(72, 118);
    M5.Lcd.printf("%2d  ", _receiver.getDroppedPacketCountDelta());
}

void Screen::updateTemplate80x160() const
{
    M5.Lcd.setCursor(0, 10);
    displayEUI_Compact("", _receiver.getMyEUI());
    M5.Lcd.setCursor(0, 20);
    displayEUI_Compact("", _receiver.getPrimaryPeerEUI());

    int yPos = 30;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:");

    yPos = 90;
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
    M5.Lcd.printf("D:");
}

void Screen::updateReceivedData80x160() const
{
    // M5StickC
    int yPos = 90;
    M5.Lcd.setCursor(12, yPos);
    const ReceiverBase::controls_t controls = _receiver.getControls();
    M5.Lcd.printf("%6d", controls.throttleStickQ4dot12);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%6d", controls.rollStickQ4dot12);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%6d", controls.pitchStickQ4dot12);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%6d", controls.yawStickQ4dot12);

    yPos += 10;
    M5.Lcd.setCursor(0, yPos); 
    const uint8_t mode = _receiver.getSwitch(0);
    const uint8_t altMode = _receiver.getSwitch(1);
    const uint8_t flipButton =_receiver.getSwitch(2);
    M5.Lcd.printf("M%1d%s A%1d F%1d ", mode, mode == AtomJoyStickReceiver::MODE_STABLE ? "ST" : "SP", altMode, flipButton);
}

void Screen::update80x160(const TD_AHRS::Data& ahrsData) const
{
    int yPos = 30;
    const bool displayAcc = true;

    M5.Lcd.setCursor(18, yPos);
    M5.Lcd.printf("%5.0F", ahrsData.pitch);

    yPos += 10;
    if (ahrsData.roll != FLT_MAX) {
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("ro:%5.0F", ahrsData.roll);
    }

    yPos += 10;
    if (ahrsData.yaw != FLT_MAX) {
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("ya:%5.0F", ahrsData.yaw);
    }

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
        M5.Lcd.printf("gx:%5.0F", ahrsData.gyroRPS.x * radiansToDegrees);

        yPos += 10;
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("gy:%5.0F", ahrsData.gyroRPS.y * radiansToDegrees);

        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("gz:%5.0F", ahrsData.gyroRPS.z * radiansToDegrees);
    }

    M5.Lcd.setCursor(12, 150);
    M5.Lcd.print(_motorPairController.motorsIsOn() ? "ON " : "OFF");

    M5.Lcd.setCursor(52, 150);
    M5.Lcd.printf("%2d  ", _receiver.getDroppedPacketCountDelta());
}

void Screen::updateTemplate135x240() const
{
    M5.Lcd.setCursor(0, 0);
    displayEUI("M:", _receiver.getMyEUI());
    M5.Lcd.setCursor(0, 20);
    displayEUI("J:", _receiver.getPrimaryPeerEUI());

    int yPos = 45;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:");

    yPos = 105;
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

void Screen::updateReceivedData135x240() const
{
    updateReceivedData80x160();
}

void Screen::update135x240(const TD_AHRS::Data& ahrsData) const
{
    int yPos = 45;

    M5.Lcd.setCursor(36, yPos);
    M5.Lcd.printf("%5.0F", ahrsData.pitch);
    yPos += 20;
    if (ahrsData.roll != FLT_MAX) {
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("ro:%5.0F", ahrsData.roll);
    }
    yPos += 20;
    if (ahrsData.yaw != FLT_MAX) {
        M5.Lcd.setCursor(0, yPos);
        M5.Lcd.printf("ya:%5.0F", ahrsData.yaw);
    }

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

void Screen::updateTemplate320x240() const
{
    M5.Lcd.setCursor(0, 0);
    displayEUI("MAC:", _receiver.getMyEUI());
    M5.Lcd.setCursor(0, 20);
    displayEUI("REM:", _receiver.getPrimaryPeerEUI());

    int yPos = 45;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("gx:");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ax:");

    yPos += 25;
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
    M5.Lcd.printf("Dropped:");

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

void Screen::updateReceivedData320x240() const
{
    int yPos = 110;

    M5.Lcd.setCursor(20, yPos);
    const ReceiverBase::controls_t controls = _receiver.getControls();
    M5.Lcd.printf("%6d", controls.throttleStickQ4dot12);
    M5.Lcd.setCursor(180, yPos);
    M5.Lcd.printf("%6d", controls.rollStickQ4dot12);

    yPos += 20;
    M5.Lcd.setCursor(20, yPos);
    M5.Lcd.printf("%6d", controls.yawStickQ4dot12);
    M5.Lcd.setCursor(180, yPos);
    M5.Lcd.printf("%6d", controls.pitchStickQ4dot12);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    const uint8_t mode = _receiver.getSwitch(0);
    const uint8_t altMode = _receiver.getSwitch(1);
    const uint8_t flipButton =_receiver.getSwitch(2);
    M5.Lcd.printf("M%1d%s A%1d F%1d ", mode, mode == AtomJoyStickReceiver::MODE_STABLE ? "ST" : "SP", altMode, flipButton);

    M5.Lcd.setCursor(255, yPos);
    M5.Lcd.printf("%3d", _receiver.getDroppedPacketCountDelta());
}

void Screen::update320x240(const TD_AHRS::Data& ahrsData) const
{
    int yPos = 45;
    M5.Lcd.setCursor(36, yPos);
    if (ahrsData.roll == FLT_MAX) {
        M5.Lcd.printf("%5.0F", ahrsData.pitch);
    } else {
        if (ahrsData.yaw == FLT_MAX) {
            M5.Lcd.printf("%5.0F ro:%5.0F", ahrsData.pitch, ahrsData.roll);
        } else {
            M5.Lcd.printf("%5.0F ro:%5.0F ya:%5.0F", ahrsData.pitch, ahrsData.roll, ahrsData.yaw);
        }
    }

    yPos += 20;
    M5.Lcd.setCursor(36, yPos);
    M5.Lcd.printf("%5.0F gy:%5.0F gz:%5.0F", ahrsData.gyroRPS.x * radiansToDegrees, ahrsData.gyroRPS.y * radiansToDegrees, ahrsData.gyroRPS.z * radiansToDegrees);

    yPos += 20;
    M5.Lcd.setCursor(36, yPos);
    M5.Lcd.printf("%5.2f ay:%5.2f az:%5.2f", ahrsData.acc.x, ahrsData.acc.y, ahrsData.acc.z);

    motor_pair_controller_telemetry_t mpcTelemetry;
    _motorPairController.getTelemetryData(mpcTelemetry);
#if defined(MOTORS_HAVE_ENCODERS)
    yPos = 180;
    M5.Lcd.setCursor(48, yPos);
    M5.Lcd.printf("%8d", mpcTelemetry.encoderLeft);
    M5.Lcd.setCursor(208, yPos);
    M5.Lcd.printf("%8d", mpcTelemetry.encoderRight);

    yPos = 200;
    M5.Lcd.setCursor(48, yPos);
    M5.Lcd.printf("%8.0F", mpcTelemetry.speedLeftDPS);
    M5.Lcd.setCursor(208, yPos);
    M5.Lcd.printf("%8.0F", mpcTelemetry.speedRightDPS);
    M5.Lcd.setCursor(208, yPos + 20);
    M5.Lcd.printf("%8.0F", round(mpcTelemetry.speedDPS_Filtered * (1.0F/6.0F)));
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

void Screen::updateTemplate() const
{
    M5.Lcd.fillScreen(TFT_BLACK);

    switch (_screenSize) {
    case SIZE_128x128:
        updateTemplate128x128();
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        updateTemplate80x160();
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        updateTemplate320x240();
    }
}

void Screen::updateReceivedData() const
{
    switch (_screenSize) {
    case SIZE_128x128:
        updateReceivedData128x128();
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        updateReceivedData80x160();
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        updateReceivedData320x240();
    }
}

void Screen::update(const TD_AHRS::Data& ahrsData) const
{
    switch (_screenSize) {
    case SIZE_128x128:
        update128x128(ahrsData);
        break;
    case SIZE_135x240:
        update135x240(ahrsData);
    case SIZE_80x160:
        update80x160(ahrsData);
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        update320x240(ahrsData);
    }
}

/*!
Update the screen with data from the AHRS and the receiver.
*/
void Screen::update(bool packetReceived) const
{
    const AHRS::data_t ahrsData = _ahrs.getAhrsDataForInstrumentationUsingLock();
    const TD_AHRS::Data tdAhrsData {
        .pitch = _motorPairController.getPitchAngleDegreesRaw(),
        .roll = _motorPairController.getRollAngleDegreesRaw(),
        .yaw = _motorPairController.getYawAngleDegreesRaw(),
        .gyroRPS = ahrsData.gyroRPS,
        .acc = ahrsData.acc
    };
    // update the screen with the AHRS data
    if (_screenMode != Screen::MODE_QRCODE) {
        update(tdAhrsData);
        if (packetReceived) {
            // update the screen with data received from the receiver
            updateReceivedData();
        }
    }
}
