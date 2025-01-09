#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#include "Buttons.h"


Buttons::Buttons(Screen& screen, MotorControllerBase& motorController, const Receiver& receiver) :
    _screen(screen),
    _motorController(motorController),
    _receiver(receiver)
{
    const Screen::screen_size_t screenSize = _screen.getScreenSize();
    _drawPosX =
        (screenSize == Screen::SIZE_128x128) ? 115 :
        (screenSize == Screen::SIZE_80x160)  ? 60 :
        (screenSize == Screen::SIZE_135x240) ? 115 :
        (screenSize == Screen::SIZE_320x240) ? 300 :
        0;
    _drawPosY =
        (screenSize == Screen::SIZE_128x128) ? 115 :
        (screenSize == Screen::SIZE_80x160)  ? 140 :
        (screenSize == Screen::SIZE_135x240) ? 220 :
        (screenSize == Screen::SIZE_320x240) ? 220 :
        0;
}

/*!
Handle any button presses.

1. BtnA turns the motors on or off.
2. BtnB initiates binding (pairing).
3. BtcC cycles through the different screen modes.
*/
void Buttons::update()
{
#if defined(M5_ATOM) // M5 Atom has only BtnA
    if (M5.BtnA.wasDoubleClicked()) {
        // Only one button
        // Use double click of BtnA for cycling through screen modes
        _screen.nextScreenMode();
    }
#endif
    if (M5.BtnA.wasPressed()) {
        // BtnA turns the motors on or off
        _motorController.motorsToggleOnOff();
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.print('A');
    } else if (M5.BtnA.wasReleased()) {
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.printf("  ");
    }

#if !defined(M5_ATOM) // M5 Atom has only BtnA

    if (M5.BtnB.wasPressed()) {
        // BtnB initiates binding
        _receiver.broadcastMyMacAddressForBinding();
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.print('B');
    } else if (M5.BtnB.wasReleased()) {
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.printf("  ");
    }
#if defined(M5_STACK)
    if (M5.BtnC.wasPressed()) {
#else
    if (M5.BtnC.wasPressed() || M5.BtnB.wasDoubleClicked()) {
#endif
        // BtnC cycles through the different screen modes
        _screen.nextScreenMode();
    }

#if !defined(M5_STACK)
    if (M5.BtnPWR.wasDoubleClicked()) {
        M5.Power.powerOff();
        /*switch (M5.getBoard()) {
        case m5::board_t::board_M5StickC:
        case m5::board_t::board_M5StickCPlus:
        case m5::board_t::board_M5StickCPlus2:
            // double click of BtnB switches off
            M5.Power.powerOff();
            break;
        default:
            // do nothing
            break;
        }*/
    }
#endif // !defined(M5_STACK)

#endif //!defined(M5_ATOM)
}
