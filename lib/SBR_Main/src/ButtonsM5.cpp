#if defined(M5_UNIFIED) || defined(M5_STACK)

#include "ButtonsM5.h"

#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <MotorPairController.h>
#include <ReceiverBase.h>
#include <ScreenBase.h>


ButtonsM5::ButtonsM5(MotorPairController& motorController, const ReceiverBase& receiver, ScreenBase* screen) :
    ButtonsBase(motorController, receiver, screen)
{
    const int screenSizeX = _screen->getScreenSizeX();
    _drawPosX = (screenSizeX == 128) ? 115 : screenSizeX - 20;
    const int screenSizeY = _screen->getScreenSizeY();
    _drawPosY = (screenSizeY == 128) ? 115 : screenSizeY - 20;
}

/*!
Handle any button presses.

1. BtnA turns the motors on or off.
2. BtnB initiates binding (pairing).
3. BtcC cycles through the different screen modes.
*/
void ButtonsM5::update()
{
    M5.update();
    if (M5.BtnA.wasPressed()) {
        // BtnA turns the motors on or off
        _motorController.motorsToggleOnOff();
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.print('A');
    } else if (M5.BtnA.wasReleased()) {
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.printf("  ");
    }

#if defined(M5_ATOM) // M5 Atom has only BtnA
    if (M5.BtnA.wasDoubleClicked()) {
        // M5 Atom has only one button
        // Use double click of BtnA for cycling through screen modes
        _screen->nextScreenMode();
    }
#else
    if (M5.BtnB.wasPressed()) {
        // BtnB initiates binding
        _receiver.broadcastMyEUI();
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.print('B');
    } else if (M5.BtnB.wasReleased()) {
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.printf("  ");
    }
#if defined(M5_STACK)
    if (M5.BtnC.wasPressed()) {
        // BtnC cycles through the different screen modes
        _screen->nextScreenMode();
    }
#else
    if (M5.BtnC.wasPressed() || M5.BtnB.wasDoubleClicked()) {
        // BtnC cycles through the different screen modes
        _screen->nextScreenMode();
    }
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
#endif // defined(M5_STACK)
#endif //defined(M5_ATOM)
}
#endif
