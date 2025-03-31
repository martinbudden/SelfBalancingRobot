#pragma once

class MotorPairController;
class ReceiverBase;
class Screen;


class Buttons {
public:
    Buttons(Screen& screen, MotorPairController& motorController, const ReceiverBase& receiver);
    void update();
private:
    // Buttons is not copyable or moveable
    Buttons(const Buttons&) = delete;
    Buttons& operator=(const Buttons&) = delete;
    Buttons(Buttons&&) = delete;
    Buttons& operator=(Buttons&&) = delete;
private:
    Screen& _screen;
    MotorPairController& _motorController;
    const ReceiverBase& _receiver;
    int _drawPosX;
    int _drawPosY;
};
