#pragma once

class MotorPairController;
class Receiver;
class Screen;


class Buttons {
public:
    Buttons(Screen& screen, MotorPairController& motorController, const Receiver& receiver);
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
    const Receiver& _receiver;
    int _drawPosX;
    int _drawPosY;
};
