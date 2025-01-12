#pragma once

#include "ESPNOW_Receiver.h"
#include "MotorPairController.h"
#include "Screen.h"


class Buttons {
public:
    Buttons(Screen& screen, MotorControllerBase& motorController, const Receiver& receiver);
    void update();
private:
    // Buttons is not copyable or moveable
    Buttons(const Buttons&) = delete;
    Buttons& operator=(const Buttons&) = delete;
    Buttons(Buttons&&) = delete;
    Buttons& operator=(Buttons&&) = delete;
private:
    Screen& _screen;
    MotorControllerBase& _motorController;
    const Receiver& _receiver;
    int _drawPosX;
    int _drawPosY;
};