#pragma once

#include "Button.h"
#include "Display.h"

class Menu
{
public:
    Menu(const Button &button, const Display &display) : _button(button), _display(display) {}
    ~Menu();
    void begin();
    void loop();

private:
    Button _button;
    Display _display;
};